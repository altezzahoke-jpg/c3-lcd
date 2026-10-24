#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino_GFX_Library.h>

// =========================================================================
// KONFIGURASI HARDWARE ESP32-C3 & TFT SPI (76x284)
// =========================================================================
// Definisikan pin SPI untuk ESP32-C3 (sesuaikan dengan wiring PCB Anda)
#define TFT_MOSI 6
#define TFT_SCLK 4
#define TFT_DC   2
#define TFT_CS   7
#define TFT_RST  3  // Perhatikan: jika GPIO 3 dipakai tombol, pindahkan salah satu pin RST/Tombol
#define PIN_BUTTON 5  // Pin Tombol (Internal Pullup) dipindah ke GPIO 5 agar tidak bentrok

// Konfigurasi Display 76x284 menggunakan Arduino_GFX
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *display = new Arduino_ST7789(bus, TFT_RST, 0, true, 76, 284, 35, 0, 35, 0);

// --- KONSTANTA & TIMEOUT SISTEM ---
const uint16_t MAX_RPM = 14000;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_DELAY = 3000;
const unsigned long CONNECTION_TIMEOUT = 1000;

// --- STRUKTUR DATA TELEMETRI & KOMANDO (PRESISI ECU) ---
struct __attribute__((packed)) TelemetryData {
  uint16_t header;     
  uint16_t rpm;
  uint8_t  tps;        
  int16_t  degree10;   
  int16_t  temp10;     
  uint16_t battVolt10; 
  uint8_t  mode;
  uint8_t  fuel;
  uint8_t  state;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandData {
  uint16_t header;     
  uint8_t requestedMode;
  uint16_t crc16;
};

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryData currentTelemetry = {0, 0, 0, 100, 300, 126, 0, 0, 0, 0};

volatile unsigned long lastRxTime = 0; 
bool connectionActive = false;

// --- VARIABEL NAVIGASI LAYAR ---
uint8_t activePage = 0; 

// --- VARIABEL TOMBOL ---
unsigned long buttonPressStartTime = 0;
bool buttonIsPressed = false;
bool longPressTriggered = false;
bool lastButtonState = HIGH; 
unsigned long lastDebounceTime = 0;

// --- HISTORI DATA GRAFIK (KURVA 76 PIKSEL SESUAI LEBAR LAYAR) ---
#define HISTORY_LEN 76
uint8_t advHistoryMapped[HISTORY_LEN] = {0};
int historyIdx = 0;

// Alamat MAC Target ECU Transmitter (Sesuaikan dengan ESP32-S3 Anda)
uint8_t targetEcuAddress[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; 
esp_now_peer_info_t peerInfo;

// =========================================================================
// FUNGSI BATAS REDLINE RPM DINAMIS PER MODE
// =========================================================================
uint16_t getRedlineRPM(uint8_t mode) {
  switch (mode) {
    case 0: return 5000;   // DAILY
    case 1: return 8000;   // RACE
    case 2: return 10000;  // EXTREME
    case 3: return 11500;  // CUSTOM
    default: return 5000;
  }
}

// =========================================================================
// KALKULASI CRC16 & UTILITAS
// =========================================================================
uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// =========================================================================
// ESP-NOW RECEIVE CALLBACK
// =========================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  const uint8_t *mac = esp_now_info->src_addr;
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (memcmp(mac, targetEcuAddress, 6) != 0) return;

  if (len == sizeof(TelemetryData)) {
    TelemetryData *packet = (TelemetryData*)incomingData;
    if (packet->header == 0xAA55) {
      uint16_t calcCrc = calculateCRC16(incomingData, sizeof(TelemetryData) - sizeof(uint16_t));
      if (packet->crc16 == calcCrc) {
        portENTER_CRITICAL(&dataMux);
        memcpy(&currentTelemetry, packet, sizeof(TelemetryData));
        portEXIT_CRITICAL(&dataMux);
        lastRxTime = millis();
      }
    }
  }
}

// =========================================================================
// LOGIKA TAMPILAN DASHBOARD (TFT VERTIKAL 76x284)
// =========================================================================

void showWelcomeLogo() {
  display->fillScreen(BLACK);
  display->drawRect(0, 0, 76, 284, WHITE);
  display->drawRect(2, 2, 72, 280, WHITE);

  display->setTextColor(WHITE);
  display->setTextSize(1);
  display->setCursor(8, 60);
  display->println("KUDUS");
  display->setCursor(14, 80);
  display->println("SATRIA");
  
  display->setCursor(14, 130);
  display->println("DASHBOARD");

  display->setCursor(14, 180);
  display->println("TFT v3.0");
  
  delay(2000);
}

void drawDashboardPage(const TelemetryData &data) {
  // 1. Header: Mode & Voltase
  const char* modeStr = "DAIL";
  if (data.mode == 1) modeStr = "RACE";
  else if (data.mode == 2) modeStr = "EXTR";
  else if (data.mode == 3) modeStr = "CUST";

  display->setTextSize(1);
  display->setTextColor(WHITE);
  display->setCursor(4, 4);
  display->printf("%s", modeStr);

  display->setCursor(40, 4);
  display->printf("%.1fV", data.battVolt10 / 10.0f);

  display->drawFastHLine(0, 16, 76, WHITE);

  // 2. RPM Utama (Vertikal Menengah)
  display->setTextSize(1);
  display->setCursor(18, 30);
  display->println("RPM");

  display->setTextSize(2);
  display->setCursor(4, 48);
  display->printf("%5d", data.rpm);

  // 3. Bar Gauge RPM Vertikal / Horizontal
  display->drawRect(4, 80, 68, 12, WHITE);
  int barWidth = map(constrain(data.rpm, 0, MAX_RPM), 0, MAX_RPM, 0, 64);
  if (barWidth > 0) {
    display->fillRect(6, 82, barWidth, 8, WHITE);
  }

  // 4. Parameter Tambahan Tersusun Vertikal di Bawah
  display->setTextSize(1);
  display->setCursor(6, 120);
  display->printf("Advance: %.1f*", data.degree10 / 10.0f);

  display->setCursor(6, 150);
  display->printf("TPS: %d %%", data.tps);

  display->setCursor(6, 180);
  display->printf("Temp: %.0f C", data.temp10 / 10.0f);
}

void drawCurvePage(const TelemetryData &data) {
  display->setTextSize(1);
  display->setTextColor(WHITE);
  display->setCursor(4, 4);
  display->println("ADV CURVE");
  display->drawFastHLine(0, 16, 76, WHITE);

  // Plot Grafik Advance Pengapian (Lebar 76 px)
  int prevX = 0;
  int prevY = 150; 

  for (int x = 0; x < HISTORY_LEN - 1; x++) {
    int readIdx = (historyIdx + x) % HISTORY_LEN;
    int currentY = 220 - advHistoryMapped[readIdx]; 
    if (x > 0) display->drawLine(prevX, prevY, x, currentY, WHITE);
    prevX = x;
    prevY = currentY;
  }

  display->drawFastHLine(0, 230, 76, WHITE);

  display->setCursor(4, 245);
  display->printf("RPM:%d", data.rpm);
  display->setCursor(4, 262);
  display->printf("TPS:%d%%", data.tps);
}

void drawShiftUpNotification() {
  display->fillRect(6, 100, 64, 50, WHITE);
  display->drawRect(8, 102, 60, 46, BLACK);
  
  display->setTextColor(BLACK);
  display->setTextSize(1);
  display->setCursor(14, 112);
  display->println("SHIFT");
  display->setCursor(20, 130);
  display->println("UP!");
}

void renderDisplay() {
  display->fillScreen(BLACK);

  TelemetryData localTelemetry;
  portENTER_CRITICAL(&dataMux);
  localTelemetry = currentTelemetry;
  portEXIT_CRITICAL(&dataMux);

  connectionActive = (millis() - lastRxTime <= CONNECTION_TIMEOUT);

  if (!connectionActive) {
    display->drawRect(0, 0, 76, 284, WHITE);
    display->setTextColor(WHITE);
    display->setTextSize(2);
    display->setCursor(10, 120);
    display->println("NO");
    display->setCursor(10, 145);
    display->println("SIG");
    return;
  }

  if (activePage == 0) {
    drawDashboardPage(localTelemetry);
  } else {
    drawCurvePage(localTelemetry);
  }

  if (localTelemetry.rpm > getRedlineRPM(localTelemetry.mode)) {
    drawShiftUpNotification();
  }
}

// =========================================================================
// LOGIKA TOMBOL & MAIN LOOP
// =========================================================================
void handleButtons() {
  int currentReading = digitalRead(PIN_BUTTON);
  
  if (currentReading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    bool currentButtonState = (currentReading == LOW);
    
    if (currentButtonState && !buttonIsPressed) {
      buttonIsPressed = true;
      buttonPressStartTime = millis();
      longPressTriggered = false;
    } 
    else if (currentButtonState && buttonIsPressed) {
      if (!longPressTriggered && (millis() - buttonPressStartTime >= LONG_PRESS_DELAY)) {
        longPressTriggered = true;
        
        portENTER_CRITICAL(&dataMux);
        uint8_t targetMode = (currentTelemetry.mode + 1) % 4;
        portEXIT_CRITICAL(&dataMux);
        
        CommandData cmd;
        cmd.header = 0xCC55;
        cmd.requestedMode = targetMode;
        cmd.crc16 = calculateCRC16((uint8_t*)&cmd, sizeof(CommandData) - sizeof(uint16_t));
        
        esp_now_send(targetEcuAddress, (uint8_t*)&cmd, sizeof(CommandData));
      }
    } 
    else if (!currentButtonState && buttonIsPressed) {
      buttonIsPressed = false;
      if (!longPressTriggered) {
        activePage = (activePage + 1) % 2; 
      }
    }
  }
  
  lastButtonState = currentReading;
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Inisialisasi TFT Display
  display->begin();
  display->fillScreen(BLACK);

  showWelcomeLogo();

  // Inisialisasi ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    memcpy(peerInfo.peer_addr, targetEcuAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

void loop() {
  handleButtons();

  static unsigned long lastDataLogTime = 0;
  if (millis() - lastDataLogTime > 40 && connectionActive) { 
    lastDataLogTime = millis();
    
    portENTER_CRITICAL(&dataMux);
    int16_t degVal = currentTelemetry.degree10;
    portEXIT_CRITICAL(&dataMux);

    advHistoryMapped[historyIdx] = (uint8_t)map(constrain(degVal, 0, 400), 0, 400, 0, 60); 
    historyIdx = (historyIdx + 1) % HISTORY_LEN;
  }

  static unsigned long lastRender = 0;
  if (millis() - lastRender > 33) { 
    lastRender = millis();
    renderDisplay();
  }
}
