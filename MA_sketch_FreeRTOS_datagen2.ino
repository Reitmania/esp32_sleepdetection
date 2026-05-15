#include <Wire.h>
#include "TaskManager.h"
#include "MPU6050.h"
#include "MAX30102.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "CamOV2640.h" 
//#include "DataManager.h"
#include "DataManagerMPU.h"
#include "DataManagerCAM.h"
#include "DataManagerMAX.h"
#include "SDManager.h"
#include "SleepManager.h"

//TwoWire WireMPU = TwoWire(1);  // Zweiter I2C‑Port (I2C_NUM_1)!
//MPU6050 mpu(0x68, &WireMPU);  // globale MPU Instanz
MPU6050 mpu;  // globale MPU Instanz
CamOV2640 cam; // Cam Instanz
MAX30102 max30102; // MAX Instanz
//DataManager dataManager;  // Data Manager Instanz
DataManagerMPU dataManagerMPU;
DataManagerCAM dataManagerCAM;
DataManagerMAX dataManagerMAX;
SleepManager sleepManager;
SDManager sdManager;

bool capture = false;

void setup() {

  TaskManager::initHardware();
  delay(1000);
  if(cam.cameraInitialized){
    TaskManager::startCore0Tasks(); // Core 0: MPU & MAX & CAM
    delay(500);
    TaskManager::startCore1Tasks(); // Core 1: DataManager  
  }   
}

void loop() {
  //delay(1000);
}
