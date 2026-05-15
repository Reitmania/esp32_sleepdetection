#ifndef DATAMANAGERCAM_H
#define DATAMANAGERCAM_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct SensorPacketCam {
    uint32_t timestamp;
    uint8_t* jpegData;
    uint32_t jpegLen;
    uint8_t* grayFrame;
    uint32_t grayLen;
};

class DataManagerCAM {
private:
    QueueHandle_t cam_queue;
    static const int BATCH_SIZE = 1; //abhängig machen von SD Writes
    SensorPacketCam batch[BATCH_SIZE];
    int batchIndex = 0;

public:
    DataManagerCAM();

    void begin();
    void pushPacket(const SensorPacketCam& pkt);
    void processFrame(SensorPacketCam& pkt);

    String createNewLogFilename();

    void run();   // Main loop Core1
    void processBatch();
};

#endif