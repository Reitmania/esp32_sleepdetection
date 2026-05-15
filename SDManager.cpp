#include "SDManager.h"
#include <FS.h>

extern bool capture;

SemaphoreHandle_t sdMutex = NULL;

String logfilename_mpu;
String logfilename_max;
String folder_cam;

SDManager::SDManager(){
}

void SDManager::begin(){

    if(capture){
        // SD Card init
        SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
        if (!SD_MMC.begin("/sdcard", true, true)) {  // 1‑Bit‑Mode
        Serial.println("Card Mount Failed");
        while (1);
        }
        Serial.println("SD ready.");
        folder_cam = createCamFolder("Log");
        delay(1000);
        // Logfiles
        logfilename_mpu = createNewLogFilename("mpu");
        Serial.print("Logfile MPU: ");
        Serial.println(logfilename_mpu);
        delay(1000);
        logfilename_max = createNewLogFilename("max");
        Serial.print("Logfile MAX: ");
        Serial.println(logfilename_max);
    }
    sdMutex = xSemaphoreCreateMutex();
}

void SDManager::writeIMUBatch(SensorPacket* batch, int count) {
    if(count == 0) return;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File file = SD_MMC.open(logfilename_mpu, FILE_APPEND);
    if(!file){
        Serial.println("SD write MPU failed");
        xSemaphoreGive(sdMutex);
        return;
    }
    for(int i=0; i<count; i++){
        // CSV Text Format
        file.printf("%lu;%d;%d;%d;%d;%d;%d;%d\n",
            batch[i].timestamp,
            batch[i].ax, batch[i].ay, batch[i].az,
            batch[i].gx, batch[i].gy, batch[i].gz,
            batch[i].energy
        );
    }
    file.flush();
    file.close();
    xSemaphoreGive(sdMutex);
    Serial.printf("SDManager: IMU Batch geschrieben (%d Punkte)\n", count);
}

void SDManager::writeMAX(float hf, uint32_t timestamp){
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File file = SD_MMC.open(logfilename_max, FILE_APPEND);
    if(!file){
        Serial.println("SD write MAX failed");
        xSemaphoreGive(sdMutex);
        return;
    }
    file.printf("%lu;%.2f\n", timestamp, hf);
    file.flush();
    file.close();
    xSemaphoreGive(sdMutex);
    Serial.printf("SDManager: MAX HF geschrieben\n");
}

void SDManager::logJPEGtoSD(SensorPacketCam* pkt){
    // Dateiname aus Timestamp erzeugen
    char filename[32];
    snprintf(filename, sizeof(filename), "/%s/frame_%lu.jpg", folder_cam, pkt->timestamp);
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File file = SD_MMC.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Konnte Datei nicht öffnen!");
        xSemaphoreGive(sdMutex);
        return;
    }
    size_t written = file.write(pkt->jpegData, pkt->jpegLen);
    file.close();
    if (written != pkt->jpegLen) {
        Serial.println("Fehler beim Schreiben der Datei");
        xSemaphoreGive(sdMutex);
        return;
    }
    xSemaphoreGive(sdMutex);
    Serial.printf("Frame gespeichert: %s (%u bytes)\n", filename, pkt->jpegLen);

}

String SDManager::createCamFolder(String baseName){
    uint16_t maxIndex = 0;

    File root = SD_MMC.open("/");
    if(!root){
        Serial.println("Fehler beim Öffnen des Root-Verzeichnisses");
        return "";
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            String fname = file.name(); // z.B. "/Cam001"
            if(fname.startsWith("/")){
                fname = fname.substring(1); // remove leading slash
            }

            if(fname.startsWith(baseName)){
                String numStr = fname.substring(baseName.length()); // z.B. "001"
                int num = numStr.toInt();
                if(num > maxIndex) maxIndex = num;
            }
        }
        file = root.openNextFile();
    }

    uint16_t nextIndex = maxIndex + 1;
    char newDir[32];
    snprintf(newDir, sizeof(newDir), "/%s%03u", baseName, nextIndex);

    Serial.printf("Creating Dir: %s\n", newDir);
    if(SD_MMC.mkdir(newDir)){
        Serial.println("Dir created");
        return String(newDir); // zurückgeben für weitere Nutzung
    } else {
        Serial.println("mkdir failed");
        return "";
    }
}

String SDManager::createNewLogFilename(const String& type) {
  int index = 1;
  char namebuf[32];

  // Prefix wählen
  const char* prefix = nullptr;
  if (type.equalsIgnoreCase("mpu")) {        // [web:97]
    prefix = "mpu";
  } else if (type.equalsIgnoreCase("max")) { // [web:97]
    prefix = "max";
  } else {
    // Fallback: unbekannter Typ
    prefix = "log";
  }

  while (true) {
    //snprintf(namebuf, sizeof(namebuf), "/mpu_%04d.csv", index);
    snprintf(namebuf, sizeof(namebuf), "/%s/%s_%04d.csv", folder_cam, prefix, index);
    if (!SD_MMC.exists(namebuf)) {
      // freie nummer
      return String(namebuf);
    }
    index++;
    if (index > 9999) {
      // nicht ins Unendliche laufen
      snprintf(namebuf, sizeof(namebuf), "/%s/%s_%04d.csv", folder_cam, prefix, 9999); // [web:99]
      return String(namebuf);
      //return String("/mpu_9999.csv");
    }
  }
}

void SDManager::flushMPUBatchToSD(SensorPacket batch[], int batchIndex, int BATCH_SIZE) {

}


