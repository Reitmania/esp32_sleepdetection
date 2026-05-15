#ifndef SLEEPMANAGER_H
#define SLEEPMANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MPU_BATCHSIZE 100

struct SleepPkt {
    enum Source { CAM, MPU, MAX } source;
    float probability;      // Output der Inferenz (0..1)
    float hf;
    uint32_t timestamp;     // millis() des Frames / Sensors
};

struct MpuBatch {
    float values[MPU_BATCHSIZE];     
    uint32_t timestamps[MPU_BATCHSIZE];
    size_t head = 0;        
    size_t count = 0;       
};

class SleepManager{
  private:
    QueueHandle_t sleep_queue;

  public:
    SleepManager();
    void begin();
    void run();
    void pushPacket(const SleepPkt& pkt);
};

#endif