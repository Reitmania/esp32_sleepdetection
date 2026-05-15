#include <Arduino.h> 
#ifndef MAX30102_H
#define MAX30102_H

#include <Wire.h>
#include "freertos/FreeRTOS.h"

// Adresse
#ifndef MAX30102_ADDR
#define MAX30102_ADDR 0x57
#endif

// Register 
#define REG_INTR_STATUS_1 0x00
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D

class MAX30102{
  public:
    float bpmBuffer[5] = {0,0,0,0,0};
    int idx = 0;
    // Beat detection parameters/state
    uint32_t lastBeat = 0;
    uint32_t lastIR = 0;
    int threshold = 100;            // anpassen!
    uint32_t minInterval = 300;     // ms (=> max ~200 BPM)
    uint32_t fingerThreshold = 5000;
    static bool maxInitialized;

    // Basic register access
    uint8_t  readRegister(uint8_t reg);
    bool     writeRegister(uint8_t reg, uint8_t value);

    MAX30102();  
    static void maxTask(void* param);  // Task-Funktion (static!)
    void init();    
    void readSample(uint32_t &red, uint32_t &ir);
    void detectBPM(uint32_t irValue);
    void addBPM(float bpm);

};

#endif