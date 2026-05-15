#include <Arduino.h>  
#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DataManagerMPU.h"
#include "DataManagerCAM.h"
#include "DataManagerMAX.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class TaskManager {
private:
  DataManagerMPU data_mgr;  // Member-Variable
  DataManagerCAM data_mgr_cam;
  DataManagerMAX data_mgr_max;
  static BLEServer* pServer;
  static BLECharacteristic* pCharacteristic;

public:

  static TaskHandle_t mpu_handle;
  static TaskHandle_t cam_handle;
  static void switchToMPU();    // MPU aktiv, Cam suspend
  static void switchToCam();    // Cam aktiv, MPU suspend
  static void activateCAMTask();
  static void deactivateCAMTask();
  static void resetI2CBus();

  static void initBLE();
  static void sendBLE(const std::string& data);
  static bool deviceConnected;

  static void initHardware();
  static void startCore0Tasks();  // Core 0 Tasks starten
  static void startCore1Tasks();  // Core 1 Tasks starten
  DataManagerMPU* getDataManager() { return &data_mgr; }  // Für Core 0 Zugriff
  DataManagerCAM* getDataManagerCAM()  { return &data_mgr_cam; }
  DataManagerMAX* getDataManagerMAX()  { return &data_mgr_max; }
};

#endif
