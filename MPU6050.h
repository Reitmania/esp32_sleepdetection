#include <Arduino.h> 
#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include "freertos/FreeRTOS.h"

class MPU6050 {
private:
  uint8_t addr, sdaPin, sclPin;  
  float pitch = 0, roll = 0;     // Gefilterte Winkel
  //TwoWire* wire;  // ← NEU: Wire-Instanz
  

public:
  //MPU6050(uint8_t address = 0x68, TwoWire* w = &Wire);
  MPU6050(uint8_t address = 0x68, uint8_t sda = 8, uint8_t scl = 9);
  void init();  // Nur Init, keine Task!
  void calibrateGyro(int samples = 200);
  float lowPassFilter(float newVal, float oldVal, float alpha);
  String classifyGesture(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
  static void mpuTask(void* param);  // Task-Funktion (static!)
  static int16_t readWord(uint8_t reg, uint8_t address);
  static bool mpuInitialized;
};

#endif
