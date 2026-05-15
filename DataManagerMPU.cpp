#include "DataManagerMPU.h"
#include "SD_MMC.h"
#include "TaskManager.h"
#include "SDManager.h"
#include "SleepManager.h"

#include "MLP_int8.h"
#include "scaler_params.h"
#include "TensorFlowLite_ESP32.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// SD Card
#define SD_MMC_CMD 38   // CMD
#define SD_MMC_CLK 39   // CLK
#define SD_MMC_D0  40   // DAT0

String logFilename;
bool batchFull = false;

// Entscheidungs Boolean 1 - log to SD | 0 - Processing 
extern bool capture;
extern SDManager sdManager;
extern SleepManager sleepManager;

// TFLite Parameter
constexpr int kWindowSize = 100;      // 1000ms @ 100Hz
constexpr int kStride = 20;           // 200ms @ 100Hz
constexpr int kNumFeatures = 7; // ohne Energie
struct Sample6 {
  int16_t v[kNumFeatures];
};
Sample6 ring[kWindowSize];
int ringPos = 0;
int samplesSeen = 0;
int strideCounter = 0;
constexpr size_t kTensorArenaSize = 16 * 1024;  // 16 KB --> Modell 5844 Bytes
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
tflite::AllOpsResolver resolver;  // alle nötigen Operatoren
tflite::MicroInterpreter* interpreter = nullptr;
static tflite::MicroErrorReporter micro_error_reporter;

// Konstruktor
DataManagerMPU::DataManagerMPU() {
    mpu_queue = NULL;
}

// TFLite Prozeduren

void pushSample(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t energy) {
  ring[ringPos].v[0]=ax; ring[ringPos].v[1]=ay; ring[ringPos].v[2]=az;
  ring[ringPos].v[3]=gx; ring[ringPos].v[4]=gy; ring[ringPos].v[5]=gz;

  ringPos = (ringPos + 1) % kWindowSize;
  samplesSeen++;
  strideCounter++;
}

void init_tflite() {
    const tflite::Model* model = tflite::GetModel(MPU_MLP_int8_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
      Serial.println("Schema mismatch!");
    return;
    }
    static tflite::AllOpsResolver resolver;

    static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter
    );
    interpreter = &static_interpreter;

    TfLiteStatus status = interpreter->AllocateTensors();
    if (status != kTfLiteOk) {
      Serial.println("MPU AllocateTensors failed");
      return;
    }
    Serial.printf("MPU arena used: %u bytes\n", (unsigned)interpreter->arena_used_bytes());
}

static inline int8_t quantize_to_int8(float x, float scale, int zero_point) {
  int32_t q = (int32_t)lrintf(x / scale) + zero_point;
  if (q < -128) q = -128;
  if (q > 127)  q = 127;
  return (int8_t)q;
}

void runInference() {
    TfLiteTensor* input = interpreter->input(0);

  // Sanity checks 
  // Erwartet: input->type == kTfLiteUInt8, input->bytes == 600 und shape [1,600] oder [600]
  if (input->type != kTfLiteUInt8) {
    Serial.println("Input tensor is not int8!");
    return;
  }
  const float in_scale = input->params.scale;
  const int in_zp = input->params.zero_point;

  // oldest index im Ringbuffer ist ringPos
  int outIdx = 0;
  for (int i = 0; i < kWindowSize; i++) {
    int idx = (ringPos + i) % kWindowSize;
    for (int j = 0; j < kNumFeatures; j++) {
      // Normalisieren wie in Python: (raw - mean) / std --> Daten aus Python Scaler
      float x = ((float)ring[idx].v[j] - kMean[j]) / kStd[j];
      input->data.int8[outIdx++] = quantize_to_int8(x, in_scale, in_zp);
    }
  }

  //uint32_t start = micros();
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed");
    return;
  }
  //uint32_t end = micros();
  //uint32_t inference_time = end - start;

  //Serial.print("Inference time: ");
  //Serial.print(inference_time);

  TfLiteTensor* output = interpreter->output(0);
  if (output->type != kTfLiteUInt8) {
    Serial.println("Output tensor is not int8!");
    return;
  }

  // Output hat 2 Klassen 
  const float out_scale = output->params.scale;
  const int out_zp = output->params.zero_point;

  // Dequantisieren und argmax
  float s0 = (output->data.int8[0] - out_zp) * out_scale;
  float s1 = (output->data.int8[1] - out_zp) * out_scale;
  int pred = (s1 > s0) ? 1 : 0;

  //Serial.printf("scores: [%.3f, %.3f] pred=%d\n", s0, s1, pred);

  // An sleepmanager senden
  SleepPkt res;
  res.source = SleepPkt::MPU;
  res.probability = pred;
  res.timestamp = millis();
  sleepManager.pushPacket(res);
}

// Init Queue
void DataManagerMPU::begin() {
    mpu_queue = xQueueCreate(64, sizeof(SensorPacket));
    if (mpu_queue == NULL) {
        Serial.println("DataManager MPU Queue creation failed");
    } else {
        Serial.println("DataManager MPU Queue bereit");
    }
    // TFLite Modell init
    init_tflite();
}

// Core1 Main Loop
void DataManagerMPU::run() {
    SensorPacket pkt;

    while (true) {
        if (xQueueReceive(mpu_queue, &pkt, portMAX_DELAY)) {
            if(capture){
              batch[batchIndex++] = pkt;
              // Batch voll?
              if (batchIndex >= BATCH_SIZE) {
                  processBatch(); // weitere Verarbeitung des Batch
                  batchIndex = 0;
              }
            }
            if(!capture){
              // Ringbuffer für TFLite füllen
              pushSample(pkt.ax, pkt.ay, pkt.az, pkt.gx, pkt.gy, pkt.gz, pkt.energy);
              if (samplesSeen >= kWindowSize && strideCounter >= kStride) {
                  strideCounter = 0;
                  runInference();
              }
            }        
            //Serial.printf("Ax:%d Ay:%d Az:%d Gx:%d Gy:%d Gz:%d\n", pkt.ax, pkt.ay, pkt.az, pkt.gx, pkt.gy, pkt.gz);         
        }
        //vTaskDelay(pdMS_TO_TICKS(5)); //unnötig, da schon Delay im QueueReceive
    }
}

// External Push Helper
void DataManagerMPU::pushPacket(const SensorPacket& pkt) {
    if (mpu_queue) {
        xQueueSend(mpu_queue, &pkt, 0);
    }
}

// Batch Processing (Logging OR Data Proessing) -----------------------------------
void DataManagerMPU::processBatch() {

    if(capture) { 
        // Speicherung
        sdManager.writeIMUBatch(batch, batchIndex);
    }
    else {
      // Non EdgeAI Verwertung
      /*
      float imuEnergy = 0;
      float headTiltAvg = 0;
      int camCount = 0;

      // Example Features
      for (int i = 0; i < BATCH_SIZE; i++) {
          float g = batch[i].gx / 131.0;
          imuEnergy += g * g;         
          
      }
      Serial.printf("IMU Energy: %.2f | HeadTilt: %.2f\n", imuEnergy, headTiltAvg);
      */

    }    
}