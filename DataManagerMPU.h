#ifndef DATAMANAGERMPU_H
#define DATAMANAGERMPU_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Sensor Packet Definition
enum SensorType {
    SENSOR_IMU
};

struct SensorPacket {
    SensorType type;
    uint32_t timestamp;

    // IMU Data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int32_t energy;
};

class DataManagerMPU {
private:
    QueueHandle_t mpu_queue;
    static const int BATCH_SIZE = 50; //abhängig machen von SD Writes
    SensorPacket batch[BATCH_SIZE];
    int batchIndex = 0;

public:
    DataManagerMPU();

    void begin();
    void pushPacket(const SensorPacket& pkt);

    String createNewLogFilename();
    void logToSD(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

    void run();   // Main loop Core1
    void processBatch();
};

#endif