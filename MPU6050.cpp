#include <Arduino.h>  
#include "MPU6050.h"
#include "TaskManager.h"
#include "DataManagerMPU.h"
extern DataManagerMPU dataManagerMPU;
#include "I2CGlobals.h"

#define MPU_ADDR 0x68

#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B  // Accel: 0x3B=Ax, 0x3D=Ay, 0x3F=Az
#define GYRO_XOUT_H  0x43  // Gyro:  0x43=Gx, 0x45=Gy, 0x47=Gz

// Filter-Parameter
#define ALPHA 0.98f      // Complementary Filter (Gyro 98%, Accel 2%)
#define DT 0.1f          // 100ms Update-Rate
#define ACC_ALPHA  0.95f
#define GYRO_ALPHA 0.97f

// Feature-Fenster
#define FEATURE_WINDOW_SAMPLES 10   // 10 × 100 ms = 1 s

// Bewegungsschwellen 
#define GYRO_MOVE_THRESHOLD  5e5f
#define ACC_MOVE_THRESHOLD   8e5f

// Gesture Thresholds 
#define GESTURE_THRESHOLD 5000
#define SHAKE_THRESHOLD 15000

float gyroX_offset = 0; 
float gyroY_offset = 0;  
float gyroZ_offset = 0;     // Gyro Bias
float accX_offset = 0; 
float accY_offset = 0;  
float accZ_offset = 0; 

// Konstruktor

//MPU6050::MPU6050(uint8_t address, TwoWire* w) 
//  : addr(address),  wire(w ? w : &Wire) {}
MPU6050::MPU6050(uint8_t address, uint8_t sda, uint8_t scl) 
  : addr(address), sdaPin(sda), sclPin(scl) {}

// INITIALISIERUNG

void MPU6050::init() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);      // Power Management
  Wire.write(0x00);      // Wake up!
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  // Sample Rate Divider (1 kHz / (1+4) = 200 Hz)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(4);
  Wire.endTransmission();

  Serial.println("MPU6050 ready!");
}

// KALIBIRIERUNG

void MPU6050::calibrateGyro(int samples) {
  long gx_sum = 0;
  long gy_sum = 0;
  long gz_sum = 0;
  long ax_sum = 0;
  long ay_sum = 0;
  long az_sum = 0;
  Serial.println("Gyro Kalibrierung...");
  for(int i = 0; i < samples; i++) {
    gx_sum += MPU6050::readWord(GYRO_XOUT_H, addr);
    gy_sum += MPU6050::readWord(GYRO_XOUT_H + 2, addr);
    gz_sum += MPU6050::readWord(GYRO_XOUT_H + 4, addr);
    ax_sum += readWord(ACCEL_XOUT_H, addr);
    ay_sum += readWord(ACCEL_XOUT_H + 2, addr);
    az_sum += readWord(ACCEL_XOUT_H + 4, addr);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  gyroX_offset = gx_sum / (float)samples;
  gyroY_offset = gy_sum / (float)samples;
  gyroZ_offset = gz_sum / (float)samples;
  accX_offset = ax_sum / (float)samples;
  accY_offset = ay_sum / (float)samples;
  accZ_offset = az_sum / (float)samples;
  Serial.printf("Gyro X Offset: %.0f\n", gyroX_offset);
  Serial.printf("Gyro Y Offset: %.0f\n", gyroY_offset);
  Serial.printf("Gyro Z Offset: %.0f\n", gyroZ_offset);
  Serial.printf("ACC X Offset: %.0f\n", accX_offset);
  Serial.printf("ACC Y Offset: %.0f\n", accY_offset);
  Serial.printf("ACC Z Offset: %.0f\n", accZ_offset);
}

int16_t MPU6050::readWord(uint8_t reg, uint8_t address) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(address); 
  Wire.write(reg); 
  Wire.endTransmission(false);
  Wire.requestFrom(address, 2);
  int16_t value = (Wire.read() << 8) | Wire.read();
  xSemaphoreGive(i2cMutex);
  return value;
  //return (int16_t)(Wire.read() << 8 | Wire.read());
}

// DATENERFASSUNG

void MPU6050::mpuTask(void* parameter) {

  MPU6050* mpu = (MPU6050*)parameter;

  static float ax_lp = 0, ay_lp = 0, az_lp = 0;
  static float gx_lp = 0, gy_lp = 0, gz_lp = 0;
  // Feature Accumulator
  static float gyro_energy_sum = 0;
  static float acc_energy_sum  = 0;
  static int sample_count    = 0;

  //mpu->calibrateGyro(200);  // Einmal kalibrieren

  float gyroX, accelPitch, accelRoll;

  Serial.printf("MPU Task auf Core %d\n", xPortGetCoreID());


  for(;;) {
    // ACCELEROMETER X,Y,Z auslesen
    int16_t ax = MPU6050::readWord(ACCEL_XOUT_H + 0, mpu->addr) - accX_offset;;
    int16_t ay = MPU6050::readWord(ACCEL_XOUT_H + 2, mpu->addr) - accY_offset;;
    int16_t az = MPU6050::readWord(ACCEL_XOUT_H + 4, mpu->addr) - accZ_offset;;
    
    // GYROSCOPE X,Y,Z auslesen
    int16_t gx = MPU6050::readWord(GYRO_XOUT_H + 0, mpu->addr) - gyroX_offset;
    int16_t gy = MPU6050::readWord(GYRO_XOUT_H + 2, mpu->addr) - gyroY_offset;
    int16_t gz = MPU6050::readWord(GYRO_XOUT_H + 4, mpu->addr) - gyroZ_offset;

    //Serial.printf("Ax:%d Ay:%d Az:%d Gx:%d Gy:%d Gz:%d\n", ax, ay, az, gx, gy, gz);

    // An DataManager senden
    SensorPacket pkt;
    pkt.type = SENSOR_IMU;
    pkt.timestamp = millis();

    pkt.ax = ax;
    pkt.ay = ay;
    pkt.az = az;
    pkt.gx = gx;
    pkt.gy = gy;
    pkt.gz = gz;
    pkt.energy = (int32_t)gx * gx + (int32_t)gy * gy + (int32_t)gz * gz;

    dataManagerMPU.pushPacket(pkt);
    //Serial.printf("Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\n", 
    //              ax, ay, az, gx, gy, gz);
    vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    
  }  


  
}

// FILTER

float MPU6050::lowPassFilter(float newVal, float oldVal, float alpha) {
  return (alpha * newVal) + ((1.0f - alpha) * oldVal);
}

// KLASSIFIKATION

String MPU6050::classifyGesture(int16_t ax, int16_t ay, int16_t az, 
                                int16_t gx, int16_t gy, int16_t gz) {
  int16_t accel_mag = sqrt(ax*ax + ay*ay + az*az);
  int16_t gyro_mag  = sqrt(gx*gx + gy*gy + gz*gz);
  
  if(accel_mag > SHAKE_THRESHOLD) return "SHAKE";
  if(gyro_mag > GESTURE_THRESHOLD && abs(gx) > abs(gy)) return "ROTATE_X";
  if(abs(ay) > 8000 && abs(az) < 12000) return "TILT_RIGHT";
  if(abs(ay) < 4000 && accel_mag > 14000) return "FLAT";
  
  return "IDLE";
}


