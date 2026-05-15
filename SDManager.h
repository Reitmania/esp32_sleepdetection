#ifndef SDMANAGER_H
#define SDMANAGER_H

#include "SD_MMC.h"
#include "DataManagerMPU.h"
#include "DataManagerCAM.h"
#include <freertos/semphr.h>

// SD Card
#define SD_MMC_CMD 38   // CMD
#define SD_MMC_CLK 39   // CLK
#define SD_MMC_D0  40   // DAT0

//bool batchFull = false;

class SDManager{

public:
  
  SDManager();
  void writeIMUBatch(SensorPacket* batch, int count);
  void writeMAX(float hf, uint32_t timestamp);
  void logJPEGtoSD(SensorPacketCam* pkt);
  String createNewLogFilename(const String& type);
  String createCamFolder(String baseName);
  void begin();
  void flushMPUBatchToSD(SensorPacket batch[], int batchIndex, int BATCH_SIZE);
  void MPUlogToSD(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
};

#endif