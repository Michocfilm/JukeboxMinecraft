#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SD.h>
#include <driver/i2s.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

// ---------------- PIN ----------------
#define BTN_PIN 15

#define RFID_SCK  19  
#define RFID_MISO 15  
#define RFID_MOSI 22 
#define RFID_SS   23
#define RFID_RST  4

#define SD_SCK  17
#define SD_MISO 5
#define SD_MOSI 18 
#define SD_CS   2

#define I2S_BCLK 25
#define I2S_LRC  26
#define I2S_DIN  33

// ---------------- SPI BUS ----------------
SPIClass SPI_SD(VSPI);
SPIClass SPI_RFID(HSPI);
MFRC522 rfid(RFID_SS, RFID_RST, &SPI_RFID);

// ---------------- GLOBAL ----------------
unsigned long lastPress = 0;
volatile bool btnPressed = false;
volatile unsigned long btnPressTime = 0;
unsigned long pressStart = 0;
bool btnPrev = HIGH;
volatile int mode = 0; // 0 = RFID, 1 = Bluetooth


File file;
bool isPlaying = false;
byte lastUID[4] = {0,0,0,0};

uint8_t buffer[2048];
size_t bytes_written;

unsigned long lastSeen = 0;
const unsigned long CARD_TIMEOUT = 1500;

// ---------------- A2DP ----------------
I2SStream i2s_bt;
BluetoothA2DPSink a2dp_sink(i2s_bt);

// ---------------- UID Mapping ----------------
byte style[4]      = {0x55, 0x9C, 0xD1, 0x49};
byte shakeitoff[4] = {0xAE, 0x4F, 0x37, 0x06};
byte blankspace[4] = {0x9C, 0x4C, 0x3A, 0x06};

// ---------------- FUNC ----------------
bool isSameUID(byte *a, byte *b){
  for(int i=0;i<4;i++) if(a[i]!=b[i]) return false;
  return true;
}

void i2s_init_sd(){
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DIN,
    .data_in_num = -1
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void startBluetooth(){
  a2dp_sink.end(true); 
  auto cfg = i2s_bt.defaultConfig();
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws  = I2S_LRC;
  cfg.pin_data = I2S_DIN;
  i2s_bt.begin(cfg);
  a2dp_sink.start("MIFI");
}

void stopBluetooth(){
  a2dp_sink.end(true);
}

void openFileForUID(byte *uid){

  if(file) file.close();

  if(isSameUID(uid, style)){
    file = SD.open("/music/Taylor-Swift-style.wav");
    Serial.println("Taylor-Swift-style");
  }
  else if(isSameUID(uid, shakeitoff)){
    file = SD.open("/music/Taylor-Swift-shakeitoff.wav");
    Serial.println("Taylor-Swift-shakeitoff");
  }
  else if(isSameUID(uid, blankspace)){
    file = SD.open("/music/Taylor-Swift-blankSpace.wav");
    Serial.println("Taylor-Swift-blankSpace");
  }
  if(!file){
    Serial.println("❌ File open failed");
    return;
  }

  file.seek(44);
  memcpy(lastUID, uid, 4);
  isPlaying = true;
  Serial.println("🎵 New song selected");
}

// ---------------- BUTTON ISR ----------------
void IRAM_ATTR switchModeISR() {
  if (!btnPressed) {        // กดครั้งแรก
    btnPressed = true;
    btnPressTime = millis();
  }
}

// ---------------- TASK RFID ----------------
void TaskRFID(void *pv){
  while(1){
    if(mode == 1){ 
      vTaskDelay(100);
      continue;
    }

    if(rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()){
      lastSeen = millis();

      if(isSameUID(rfid.uid.uidByte, lastUID)){
        isPlaying = true;
        vTaskDelay(50);
        continue;
      }
      openFileForUID(rfid.uid.uidByte);
    }

    if(isPlaying && (millis()-lastSeen > CARD_TIMEOUT)){
      isPlaying = false;
    }

    vTaskDelay(50);
  }
}

// ---------------- TASK AUDIO ----------------
void TaskAudio(void *pv){
  while(1){

    // Bluetooth Mode
    if(mode == 1){
      isPlaying = false;
      vTaskDelay(50);
      continue;
    }

    // SD Player Mode
    if(!isPlaying){
      vTaskDelay(5);
      continue;
    }

    if(isPlaying && file){
      if(file.available()){
        size_t bytes_read = file.read(buffer, sizeof(buffer));
        i2s_write(I2S_NUM_0, buffer, bytes_read, &bytes_written, portMAX_DELAY);
      } else {
        file.seek(44);
      }
    }

    vTaskDelay(1);
  }
}

// =========================
// Play / Stop Audio Control
// =========================
void playAudio() {
  if (mode == 1) {
    Serial.println("BT Play Request");
  } else {
    if (file) file.seek(44);
    isPlaying = true;
  }
}

void stopAudio() {
  if (mode == 1) {
    Serial.println("BT Stop Request");
    a2dp_sink.stop();
  } else {
    isPlaying = false;
  }
}

// ---------------- SETUP ----------------
void setup(){
  Serial.begin(115200);

  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(BTN_PIN, switchModeISR, FALLING);

  // SD
  SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS, SPI_SD);

  // RFID
  SPI_RFID.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();

  // Start SD I2S
  i2s_init_sd();

  // Tasks
  xTaskCreatePinnedToCore(TaskRFID,  "RFID",  4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskAudio, "AUDIO", 4096, NULL, 1, NULL, 0);

  Serial.println("System Ready!");
}

void loop() {
  bool btnNow = digitalRead(BTN_PIN);

  // ตรวจว่ากดลง
  if (btnPrev == HIGH && btnNow == LOW) {
    pressStart = millis();
  }

  // ตรวจว่าปล่อยปุ่ม
  if (btnPrev == LOW && btnNow == HIGH) {
    unsigned long pressTime = millis() - pressStart;

    if (pressTime < 500) {
      // -------------------------
      //   Short Press (กดสั้น)
      // -------------------------
      if (isPlaying) {
        stopAudio();   // หยุดเพลง
        isPlaying = false;
        Serial.println("⏸️ Short Press: Stop");
      } else {
        playAudio();   // เล่นเพลง
        isPlaying = true;
        Serial.println("▶️ Short Press: Play");
      }

    } else {
      // -------------------------
      //   Long Press (กดค้าง)
      // -------------------------
      mode = !mode;
      Serial.print("🔄 Long Press: Mode changed to ");
      Serial.println(mode == 0 ? "💿RFID" : "🔊Bluetooth");
    }
  }

  btnPrev = btnNow;
}