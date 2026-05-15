#include "DataManagerMAX.h"
#include "SD_MMC.h"
#include "TaskManager.h"
#include "SDManager.h"
#include "SleepManager.h"

// Entscheidungs Boolean 1 - log to SD | 0 - Processing 
extern bool capture;
extern SDManager sdManager;
extern SleepManager sleepManager;

// Herzfrequenz-Variablen
uint32_t lastBeat = 0;
uint32_t lastIR = 0;
int threshold = 5;
int minInterval = 800; // ms

// Threshold
int32_t dc = 0;
int32_t env = 0;
bool    above = false;

float bpmBuffer[5];
int idx = 0;

// konstruktor
DataManagerMAX::DataManagerMAX() {
    max_queue = NULL;
}

void DataManagerMAX::flushBatchToSD() { }

// Init Queue
void DataManagerMAX::begin() {
    max_queue = xQueueCreate(10, sizeof(SensorPacketCam));

    if (max_queue == NULL) {
        Serial.println("DataManager MAX Queue creation failed");
    } else {
        Serial.println("DataManager MAX Queue bereit");
    }
}

void addBPM(float bpm, uint32_t time) {
  bpmBuffer[idx] = bpm;
  idx = (idx + 1) % 5;
  float sum = 0;
  for(int i=0;i<5;i++) sum += bpmBuffer[i];
  float avg = sum / 5;
  if(capture){
    // Speicherung
    sdManager.writeMAX(avg, time);
  } else {
    // Logik
    //Serial.printf("Avg BPM: %.1f\n", avg);
    // An sleepmanager senden
    SleepPkt res;
    res.source = SleepPkt::MAX;
    res.hf = avg;
    res.timestamp = millis();
    sleepManager.pushPacket(res);
  }
  
}

void detectBPM(uint32_t irValue, uint32_t timestamp) {
  if (irValue < 5000) return; // Finger nicht auf Sensor

  int diff = irValue - lastIR;
  if (diff > threshold) {
    uint32_t now = timestamp;
    uint32_t dt = now - lastBeat;
    if (dt > minInterval) {
      float bpm = 60000.0 / dt;
      addBPM(bpm, now);
      lastBeat = now;
    }
  }
  lastIR = irValue;  
}

// Core1 Main Loop
void DataManagerMAX::run() {
    SensorPacketMax pkt;

    while (true) {
        if (xQueueReceive(max_queue, &pkt, portMAX_DELAY)) {
          // detectBPM
          detectBPM(pkt.ir, pkt.timestamp);
        }
    }
}

// External Push Helper
void DataManagerMAX::pushPacket(const SensorPacketMax& pkt) {
    if (max_queue) {
        xQueueSend(max_queue, &pkt, 0);
    }
}