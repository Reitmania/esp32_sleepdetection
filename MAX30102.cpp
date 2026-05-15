#include <Arduino.h>  
#include "MAX30102.h"
#include "TaskManager.h"
#include "DataManagerMAX.h"
extern DataManagerMAX dataManagerMAX;
#include "I2CGlobals.h"

MAX30102::MAX30102(){

}

bool MAX30102::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(value);
  //Wire.endTransmission();
  return (Wire.endTransmission(true) == 0);
}

uint8_t MAX30102::readRegister(uint8_t reg) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30102_ADDR, (uint8_t)1) != 1) return false;
  uint8_t val = (uint8_t)Wire.read();
  return true;
}

void MAX30102::init(){
  writeRegister(REG_MODE_CONFIG, 0x40);
  delay(500);
  // FIFO Reset
  writeRegister(REG_FIFO_WR_PTR, 0);
  writeRegister(REG_OVF_COUNTER, 0);
  writeRegister(REG_FIFO_RD_PTR, 0);
  // FIFO Config: averaging + rollover enable
  //writeRegister(REG_FIFO_CONFIG, 0x4F);
  uint8_t fifo = readRegister(REG_FIFO_CONFIG);
  fifo |= 0x10;                                 // Bit4 setzen (Rollover)
  writeRegister(REG_FIFO_CONFIG, fifo);
  // SpO2 Mode aktivieren
  writeRegister(REG_MODE_CONFIG, 0x03);
  // SPO2 Config: SampleRate 100Hz, PulseWidth 411us
  writeRegister(REG_SPO2_CONFIG, 0x27);
  // LED Strom setzen
  writeRegister(REG_LED1_PA, 0x24); // RED
  writeRegister(REG_LED2_PA, 0x24); // IR
  Serial.println("MAX30102 ready!");
}

void MAX30102::detectBPM(uint32_t irValue) {
  if (irValue < 5000) return; // Finger nicht drauf
  int diff = irValue - lastIR;
  //Serial.println(diff);
  if (diff > threshold) {
    uint32_t now = millis();
    uint32_t dt = now - lastBeat;
    if (dt > minInterval) {
      float bpm = 60000.0 / dt;
      Serial.printf("BPM:%d\n",int(bpm));
      addBPM(bpm);
      //Serial.println((int)bpm);
      lastBeat = now;
    }
  }
  lastIR = irValue;
}

void MAX30102::addBPM(float bpm) {
  bpmBuffer[idx] = bpm;
  idx = (idx + 1) % 5;

  float sum = 0;
  for(int i=0;i<5;i++) sum += bpmBuffer[i];

  float avg = sum / 5;
  Serial.printf("Avg BPM: %.1f\n", avg);
}

void MAX30102::readSample(uint32_t &red, uint32_t &ir) {
  this->readRegister(REG_INTR_STATUS_1);

  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  Wire.endTransmission();                         

  Wire.requestFrom(MAX30102_ADDR, 6);

  uint32_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();

  red = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x3FFFF;
  ir  = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x3FFFF;

  // An DataManager senden
  SensorPacketMax pkt;
  pkt.timestamp = millis();
  pkt.red = red;
  pkt.ir = ir;
  dataManagerMAX.pushPacket(pkt);

  //Serial.println(ir);
}

// TASK LOOP -------------

void MAX30102::maxTask(void* parameter) {

  MAX30102* max30102 = (MAX30102*)parameter;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

  for (;;) {
    uint32_t red, ir;
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    //Serial.println("MAX misst");
    max30102->readSample(red, ir);
    xSemaphoreGive(i2cMutex);
    //max30102->detectBPM(ir);    
    //vTaskDelay(pdMS_TO_TICKS(200));
    vTaskDelayUntil(&lastWake, period);
  }
}