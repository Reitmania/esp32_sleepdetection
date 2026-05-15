#include <Arduino.h> 
#define CAMERA_MODEL_ESP32S3_EYE
#include "CamOV2640.h"
#include "MPU6050.h"
#include "DataManagerCAM.h"
extern DataManagerCAM dataManagerCAM;

bool CamOV2640::cameraInitialized = false;

#define CAM_FREQ 5000 // alle 5 s

#define IMG_SIZE 96
#define IMG_X 320
#define IMG_Y 240



// Entscheidungs Boolean 1 - log to SD | 0 - Processing 
extern bool capture;

unsigned long lastCapture = 0;
int photoIndex = 0;

bool CamOV2640::init(){

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.sccb_i2c_port = 0;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  //config.frame_size = FRAMESIZE_UXGA;
  //config.frame_size = FRAMESIZE_QVGA;
  //config.pixel_format = PIXFORMAT_JPEG;
  //config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  //config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 15;
  config.fb_count = 1;
  
  if (psramFound()) {
    Serial.print("PSRAM active\n");
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  if(capture){
      // JPEG Modus
      config.pixel_format = PIXFORMAT_JPEG;
      config.frame_size   = FRAMESIZE_QVGA; 
      //config.frame_size   = FRAMESIZE_96X96;
      config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    } else {
      // AI Modus
      config.pixel_format = PIXFORMAT_GRAYSCALE;
      //config.frame_size   = FRAMESIZE_96X96;
      config.frame_size   = FRAMESIZE_QVGA; // 320x240
      config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return 0;
  } else {
    cameraInitialized = true;
    Serial.printf("Camera ready!");
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);
  s->set_special_effect(s, 2);   // 2 = Grayscale
  return 1;
}

void CamOV2640::cameraTask(void *pvParameters) {

  CamOV2640* self = (CamOV2640*) pvParameters;
  Serial.printf("Camera Task läuft auf Core %d\n", xPortGetCoreID());
  
  lastCapture = 0;
  
  while (true) {
    unsigned long now = millis();

    if (now - lastCapture >= 2000) {
        lastCapture = now;
        
        camera_fb_t *fb = esp_camera_fb_get();
        if(!fb) {
          Serial.println("Capture failed!");
        }
        else {
          Serial.printf("Frame OK: %d bytes\n", fb->len);
          // An Datamanager senden
          SensorPacketCam pkt;
          // JPEG
          if(capture){            
            pkt.timestamp = millis();
            // JPEG Buffer kopieren
            pkt.jpegLen = fb->len;
            pkt.jpegData = (uint8_t*)malloc(pkt.jpegLen);
            if (pkt.jpegData) {
                memcpy(pkt.jpegData, fb->buf, pkt.jpegLen);
                dataManagerCAM.pushPacket(pkt);
            } else {
                Serial.println("malloc failed");
            } 
          }        
          // Grayscale Packet für CNN
          if(!capture){
              pkt.timestamp = millis();
              pkt.grayLen = IMG_X * IMG_Y; // 320 x 240 QVGA
              pkt.grayFrame = (uint8_t*) heap_caps_malloc(pkt.grayLen, MALLOC_CAP_SPIRAM);
              if(pkt.grayFrame){
                  memcpy(pkt.grayFrame, fb->buf, pkt.grayLen);
                  dataManagerCAM.pushPacket(pkt);
              } else {
                  Serial.println("malloc failed for gray frame");
              }
          }                  
          esp_camera_fb_return(fb);
        }
      }      
      vTaskDelay(pdMS_TO_TICKS(CAM_FREQ)); // alle 5s
  } 
}