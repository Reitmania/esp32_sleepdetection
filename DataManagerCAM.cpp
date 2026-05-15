#include "DataManagerCAM.h"
#include "SD_MMC.h"
#include "TaskManager.h"
#include "SDManager.h"
#include "SleepManager.h"
#include "CNN_int8.h" // Modellparameter
#include "TensorFlowLite_ESP32.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// Entscheidungs Boolean 1 - log to SD | 0 - Processing 
extern bool capture;
extern SDManager sdManager;
extern SleepManager sleepManager;

// Cropping Parameter für QVGA
#define IMG_X 320 
#define IMG_Y 240
// --- ROI definieren ---
const int ROI_W = 96;
const int ROI_H = 96;
const int ROI_X = 112; // zentriert im 320x240 Frame
const int ROI_Y = 72;

// TFLite Parameter
constexpr size_t kTensorArenaSize_cam = 512 * 1024; // 512kb für Kamera
uint8_t* tensor_arena_cam = (uint8_t*) heap_caps_malloc(
    kTensorArenaSize_cam,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
);
tflite::AllOpsResolver resolver_cam;  // alle nötigen Operatoren
tflite::MicroInterpreter* interpreter_cam = nullptr;
static tflite::MicroErrorReporter micro_error_reporter_cam;

// konstruktor
DataManagerCAM::DataManagerCAM() {
    cam_queue = NULL;
}

// TFLite
void init_tflite_cam() {
    
    const tflite::Model* model_cam = tflite::GetModel(cnn_int8_tflite);
    if (model_cam->version() != TFLITE_SCHEMA_VERSION) {
      Serial.println("Schema mismatch!");
    return;
    }
    static tflite::AllOpsResolver resolver_cam;

    static tflite::MicroInterpreter static_interpreter_cam(
        model_cam,
        resolver_cam,
        tensor_arena_cam,
        kTensorArenaSize_cam,
        &micro_error_reporter_cam
    );
    interpreter_cam = &static_interpreter_cam;  
    TfLiteStatus status_cam = interpreter_cam->AllocateTensors();
    if (status_cam != kTfLiteOk) {
      Serial.println("CAM AllocateTensors failed");
      return;
    }
    Serial.printf("CAM arena used: %u bytes\n", (unsigned)interpreter_cam->arena_used_bytes());
}

void DataManagerCAM::processFrame(SensorPacketCam& pkt) {
    if (!interpreter_cam || !pkt.grayFrame) return;
    TfLiteTensor* input_cam = interpreter_cam->input(0);

    // temporärer Buffer für Crop
    uint8_t* roi_frame = (uint8_t*) heap_caps_malloc(ROI_W * ROI_H, MALLOC_CAP_SPIRAM);
    if(!roi_frame){
        Serial.println("malloc failed for ROI");
        return;
    }
    // --- Crop durchführen ---
    for(int y = 0; y < ROI_H; y++){
        memcpy(&roi_frame[y*ROI_W],
               &pkt.grayFrame[(ROI_Y + y)*IMG_X + ROI_X],
               ROI_W);
    }
    // --- INT8 Quantisierung ---
    const float input_scale = 0.0034849231597036123f;
    const int input_zero_point = -128;

    for (int i = 0; i < ROI_W*ROI_H; i++) {
        float normalized = roi_frame[i] / 255.0f;            // 0..1
        int quantized = round(normalized / input_scale) + input_zero_point;

        // Clamp auf -128 .. 127
        if (quantized < -128) quantized = -128;
        if (quantized > 127) quantized = 127;

        input_cam->data.int8[i] = (int8_t) quantized;
    }
    // Inferenz starten
    uint32_t start = micros();
    TfLiteStatus status = interpreter_cam->Invoke();
    if (status != kTfLiteOk){
        Serial.println("CAM Invoke failed");
        free(roi_frame);
        return;
    }
    uint32_t end = micros();
    uint32_t inference_time = end - start;

    Serial.print("Inference time: ");
    Serial.print(inference_time);
    // Output auslesen
    int8_t raw_output = interpreter_cam->output(0)->data.int8[0];
    float probability = (raw_output + 128) / 255.0f;
    Serial.printf("CNN Output: %d -> %.2f\n", raw_output, probability);
    
    // An sleepmanager senden
    SleepPkt res;
    res.source = SleepPkt::CAM;
    res.probability = probability;
    res.timestamp = millis();
    sleepManager.pushPacket(res);
}

// Init Queue
void DataManagerCAM::begin() {
    cam_queue = xQueueCreate(1, sizeof(SensorPacketCam));

    if (cam_queue == NULL) {
        Serial.println("DataManager CAM Queue creation failed");
    } else {
        Serial.println("DataManager CAM Queue bereit");
    }
    // TFLite Modell init
    init_tflite_cam();
}

// Core1 Main Loop
void DataManagerCAM::run() {
    SensorPacketCam pkt;

    while (true) {
        if (xQueueReceive(cam_queue, &pkt, portMAX_DELAY)) {
            if(capture){
                // Speicherung
                Serial.printf("Frame erhalten: %lu bytes\n", pkt.jpegLen);
                sdManager.logJPEGtoSD(&pkt);
            } else {
                // Cam AI Processing
                Serial.printf("Frame erhalten: %lu bytes\n", pkt.grayLen);
                processFrame(pkt);  // CNN Inferenz
            }
            free(pkt.jpegData); 
            free(pkt.grayFrame);           
        }
        //vTaskDelay(pdMS_TO_TICKS(5)); //unnötig, da schon Delay im QueueReceive
    }
}

// External Push Helper
void DataManagerCAM::pushPacket(const SensorPacketCam& pkt) {
    if (cam_queue) {
        xQueueSend(cam_queue, &pkt, 0);
    }
}