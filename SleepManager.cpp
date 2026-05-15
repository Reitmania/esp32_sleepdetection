#include "SleepManager.h"
#include "TaskManager.h"

#define INF_QUEUE_LEN 50
#define INF_QUEUE_ITEM sizeof(SleepPkt)

#define DECISION_FRAME 5000 // 5s für fehlende Bewegungen
#define DECISION_SAMPLE 3 // n Messungen innerhalb von Frame
#define MPU_THRESHOLD 0.2f
#define CAM_THRESHOLD 0.8f

float cam_last_prob;
float mpu_last_prob;
float max_last_prob;
uint32_t cam_last_time;
uint32_t mpu_last_time;
uint32_t max_last_time;

bool ruhezustand = false;

SleepManager::SleepManager(){
  cam_last_prob = 0.0f;
  mpu_last_prob = 0.0f;
  cam_last_time = 0;
  mpu_last_time = 0;
  sleep_queue = xQueueCreate(INF_QUEUE_LEN, INF_QUEUE_ITEM);
}

void sendInferenceResult(std::string zustand, float result) {
  std::string payload = zustand + ": " + std::to_string(result);
  TaskManager::sendBLE(payload);
}

void SleepManager::begin(){}

// MPU Batch Funktionen
void insertInference(MpuBatch &batch, float prob, uint32_t timestamp){
    batch.values[batch.head] = prob;
    batch.timestamps[batch.head] = timestamp;
    batch.head = (batch.head + 1) % MPU_BATCHSIZE;
    if(batch.count < MPU_BATCHSIZE) batch.count++;
}

// old
float getAverageInference(MpuBatch &batch){
    if(batch.count == 0) return 0.0f;
    float sum = 0.0f;
    size_t valid_count = 0;
    uint32_t now = millis();
    for(size_t i = 0; i < batch.count; i++){
        size_t idx = (batch.head + 40 - batch.count + i) % 40;
        if(now - batch.timestamps[idx] <= DECISION_FRAME){ // Entscheidungsfenster
            sum += batch.values[idx];
            valid_count++;
        }
    }
    return valid_count ? sum / valid_count : 0.0f;
}

// Bewegung im Zeitfenster?
bool movementCountInLast(MpuBatch &batch, uint32_t now, uint32_t window_ms, int required){
    int count = 0;
    for(int i = 0; i < batch.count; i++){
        int idx = (batch.head - 1 - i + MPU_BATCHSIZE) % MPU_BATCHSIZE;
        if(now - batch.timestamps[idx] > window_ms){
            break;
        }
        if(batch.values[idx] >= 0.8f){
            count++;
            if(count >= required){
                return true;
            }
        }
    }
    return false;
}

void SleepManager::run(){
  SleepPkt res;
  MpuBatch mpuBatch;

  while (true) {
      // Warte auf neue Ergebnisse
      if(xQueueReceive(sleep_queue, &res, portMAX_DELAY)){

          if(res.source == SleepPkt::CAM){
            cam_last_prob = res.probability;
            cam_last_time = res.timestamp;
            Serial.printf("scores: [%.3f]\n", cam_last_prob);
            // Beispiel Logik: keine Bewegung + Kamera Schlaf erkannt
            if (ruhezustand && cam_last_prob > CAM_THRESHOLD) {
              Serial.println("Schlaf festgestellt!");
              sendInferenceResult("Schlaf", cam_last_prob);
            } else {
              //Serial.println("Kein Schlaf festgestellt!");
              //sendInferenceResult("kein Schlaf", cam_last_prob);
            }
          } 
          if(res.source == SleepPkt::MPU){
              insertInference(mpuBatch, res.probability, res.timestamp);
              //float avg = getAverageInference(mpuBatch);
              // Threshold-Logikbatch

              if(movementCountInLast(mpuBatch, millis(), DECISION_FRAME, DECISION_SAMPLE)){
                  // Bewegung erkannt
                  ruhezustand = false;
                  Serial.println("Bewegung");
                  sendInferenceResult("Bewegung", res.timestamp);
                  TaskManager::deactivateCAMTask(); // Kamera deaktiveren
              } else {
                  // Ruhe erkannt
                  ruhezustand = true;
                  TaskManager::activateCAMTask(); // Kamera aktivieren
                  sendInferenceResult("Ruhe", res.timestamp);
              }
              /*
              if(avg < MPU_THRESHOLD){
                ruhezustand = true;
                Serial.println("Ruhe festgestellt!");
                sendInferenceResult("Ruhe", avg);
                TaskManager::activateCAMTask(); // Kamera aktivieren
              } else {
                Serial.println("Bewegung festgestellt!");
                ruhezustand = false;
                TaskManager::deactivateCAMTask(); // Kamera deaktiveren
              }
              */
              //if(mpu_last_prob < 0.1){
              //  TaskManager::activateCAMTask(); // Kamera aktivieren
              //  Serial.printf("scores: [%.3f]\n", mpu_last_prob);
              //}
          }
          if(res.source == SleepPkt::MAX){
              max_last_prob = res.hf;
              max_last_time = res.timestamp;
              Serial.printf("scores: [%.3f]\n", max_last_prob);
          }



          // Weitere Logik: Kombinationen, Thresholds, Zeitfenster
      }
  }
}

// External Push Helper
void SleepManager::pushPacket(const SleepPkt& pkt) {
    if (sleep_queue) {
        xQueueSend(sleep_queue, &pkt, 0);
    }
}

