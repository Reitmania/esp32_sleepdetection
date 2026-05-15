#ifndef DATAMANAGERMAX_H
#define DATAMANAGERMAX_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct SensorPacketMax {
    uint32_t timestamp;

    // MAX Feature Placeholder
    uint32_t ir;
    uint32_t red;
};

class DataManagerMAX {
private:
    QueueHandle_t max_queue;
    static const int BATCH_SIZE = 10; //abhängig machen von SD Writes
    SensorPacketMax batch[BATCH_SIZE];
    int batchIndex = 0;

public:
    DataManagerMAX();

    void begin();
    void pushPacket(const SensorPacketMax& pkt);

    String createNewLogFilename();
    void flushBatchToSD();
    void logToSD(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

    void run();   // Main loop Core1
    void processBatch();
};

#endif