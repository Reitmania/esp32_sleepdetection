#include <Arduino.h>  
#include "TaskManager.h"
#include "MPU6050.h"
#include "MAX30102.h"
#define CAMERA_MODEL_ESP32S3_EYE
#include "CamOV2640.h"
#include "DataManagerMPU.h"
#include "DataManagerCAM.h"
#include "DataManagerMAX.h"
#include "SleepManager.h"
#include "SDManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "I2CGlobals.h"

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"

BLEServer* TaskManager::pServer = nullptr;
BLECharacteristic* TaskManager::pCharacteristic = nullptr;
bool TaskManager::deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        TaskManager::deviceConnected = true;
        Serial.println("Client connected");
    }

    void onDisconnect(BLEServer* pServer) override {
        TaskManager::deviceConnected = false;
        Serial.println("Client disconnected");
        BLEDevice::startAdvertising();
    }
};

extern DataManagerMPU dataManagerMPU;
extern DataManagerCAM dataManagerCAM;
extern DataManagerMAX dataManagerMAX;
extern SDManager sdManager;
extern SleepManager sleepManager;
extern MPU6050 mpu;  
extern CamOV2640 cam;
extern MAX30102 max30102;
extern bool capture;

TaskHandle_t TaskManager::mpu_handle = nullptr;
TaskHandle_t TaskManager::cam_handle = nullptr;

void DataTask(void* param) {
    dataManagerMPU.begin();
    dataManagerMPU.run();
}

void DataTaskCam(void* param) {
    dataManagerCAM.begin();
    dataManagerCAM.run();
}

void DataTaskMax(void* param) {
    dataManagerMAX.begin();
    dataManagerMAX.run();
}

void SleepTask(void* param){
    sleepManager.begin();
    sleepManager.run();
}

void TaskManager::activateCAMTask(){
    if(TaskManager::cam_handle){
        vTaskResume(TaskManager::cam_handle);
    }
}

void TaskManager::deactivateCAMTask(){
    if(TaskManager::cam_handle){
        vTaskSuspend(TaskManager::cam_handle);
    }
}

// BLE
void TaskManager::initBLE() {
    BLEDevice::init("ESP32-S3-SleepManager");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
}

void TaskManager::sendBLE(const std::string& data) {
    if (pCharacteristic != nullptr) {
        //Serial.println("BLE msg sent");
        pCharacteristic->setValue(data);
        pCharacteristic->notify();
    }
}

// Init
void TaskManager::initHardware(){
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  //if (capture) sdmmcInit();  // SD initialisieren
  sdManager.begin();

  // Mutex erstellen
  i2cMutex = xSemaphoreCreateMutex();

  cam.init();

  // MPU Wire Init
  Wire.begin(1, 2);
  Wire.setClock(400000);

  //xSemaphoreTake(i2cMutex, portMAX_DELAY);
  mpu.init();
  mpu.calibrateGyro(200);
  //xSemaphoreGive(i2cMutex);

  delay(200);

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  max30102.init();
  xSemaphoreGive(i2cMutex);
  delay(200);
  TaskManager::initBLE();
  delay(200);
}

void TaskManager::startCore0Tasks() {
  Serial.println("Core 0: MPU/MAX/CAM Tasks starten");
  xTaskCreatePinnedToCore(
    MPU6050::mpuTask, "MPU", 8192, &mpu, 2, NULL, 0  // Core 0
  );
  xTaskCreatePinnedToCore(
    MAX30102::maxTask, "MAX", 8192, &max30102, 2, NULL, 0  // Core 0
  );
  xTaskCreatePinnedToCore(
    CamOV2640::cameraTask, "CameraTask", 8192, &cam, 1, &TaskManager::cam_handle, 0  // Core 1
  );
  // Zum Start suspendieren, falls nicht Logging-Modus
  if(!capture){
      TaskManager::deactivateCAMTask();
      //xTaskNotifyGive(TaskManager::cam_handle);
  } else {
      xTaskNotifyGive(TaskManager::cam_handle);
  }
}

void TaskManager::startCore1Tasks() {
  Serial.println("Core 1: DataManager/AI Tasks starten");
  xTaskCreatePinnedToCore(
    DataTask, "DataTaskMPU", 16384, &dataManagerMPU, 2, NULL, 1  // Core 1
  );
  xTaskCreatePinnedToCore(
    DataTaskCam, "DataTaskCAM", 8192, &dataManagerCAM, 2, NULL, 1  // Core 1
  );
  xTaskCreatePinnedToCore(
    DataTaskMax, "DataTaskMAX", 8192, &dataManagerMAX, 2, NULL, 1  // Core 1
  );
  xTaskCreatePinnedToCore(
    SleepTask, "SleepTask", 4096, &sleepManager, 1, NULL, 1  // Core 1
  );
}
