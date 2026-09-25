#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <esp_now.h>

// =========================================================================
// KONFIGURASI HARDWARE ESP32-C3 & OLED 0.96" (I2C)
// =========================================================================
#define PIN_SDA 8         // Pin Data I2C
#define PIN_SCL 9         // Pin Clock I2C
#define PIN_BUTTON 3      // Pin Tombol (Internal Pullup)

#define SCREEN_WIDTH 128  // Lebar layar OLED dalam piksel
#define SCREEN_HEIGHT 64  // Tinggi layar OLED dalam piksel
#define OLED_RESET -1     // Reset pin (-1 jika berbagi dengan pin RESET ESP)
#define SCREEN_ADDRESS 0x3C // Alamat I2C umum untuk OLED 0.96"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

// --- HISTORI DATA GRAFIK (KURVA 128 PIKSEL) ---
#define HISTORY_LEN 128
uint8_t advHistoryMapped[HISTORY_LEN] = {0};
int historyIdx = 0;

// Alamat MAC Target ECU Transmitter
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
// LOGIKA TAMPILAN DASHBOARD (OLED 128x64)
// =========================================================================

void showWelcomeLogo() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawRect(2, 2, 124, 60, SSD1306_WHITE);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(12, 12);
  display.println("PATAS KUDUS CLUB");
  
  display.setCursor(18, 28);
  display.println("ECU DASHBOARD");

  display.setCursor(28, 44);
  display.println("OLED v3.0");
  
  display.display();
  delay(2000);
}

void drawDashboardPage(const TelemetryData &data) {
  // 1. Header Bar: Mode & Voltase Baterai
  const char* modeStr = "DAILY";
  if (data.mode == 1) modeStr = "RACE ";
  else if (data.mode == 2) modeStr = "EXTR ";
  else if (data.mode == 3) modeStr = "CUST ";

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("[%s]", modeStr);

  display.setCursor(76, 0);
  display.printf("%4.1fV", data.battVolt10 / 10.0f);

  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // 2. Angka RPM Utama (Ukuran Besar)
  display.setTextSize(3);
  display.setCursor(10, 14);
  display.printf("%5d", data.rpm);

  // 3. Bar Gauge RPM
  display.drawRect(0, 40, 128, 9, SSD1306_WHITE);
  int barWidth = map(constrain(data.rpm, 0, MAX_RPM), 0, MAX_RPM, 0, 124);
  if (barWidth > 0) {
    display.fillRect(2, 42, barWidth, 5, SSD1306_WHITE);
  }

  // 4. Informasi Parameter Bawah (Deg, TPS, Temp)
  display.setTextSize(1);
  display.setCursor(0, 53);
  display.printf("D:%2.0f*", data.degree10 / 10.0f);

  display.setCursor(46, 53);
  display.printf("T:%2d%%", data.tps);

  display.setCursor(84, 53);
  display.printf("C:%2.0fC", data.temp10 / 10.0f);
}

void drawCurvePage(const TelemetryData &data) {
  // Header Info
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("ADV:%.1f*", data.degree10 / 10.0f);
  
  display.setCursor(70, 0);
  display.printf("%5dRPM", data.rpm);

  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  // Plot Grafik Advance Pengapian
  int prevX = 0;
  int prevY = 52; 

  for (int x = 0; x < HISTORY_LEN - 1; x++) {
    int readIdx = (historyIdx + x) % HISTORY_LEN;
    int currentY = 52 - advHistoryMapped[readIdx]; 
    if (x > 0) display.drawLine(prevX, prevY, x, currentY, SSD1306_WHITE);
    prevX = x;
    prevY = currentY;
  }

  display.drawLine(0, 54, 128, 54, SSD1306_WHITE);

  // Footer Info
  display.setCursor(0, 56);
  display.printf("TPS:%d%%", data.tps);

  display.setCursor(76, 56);
  display.printf("TMP:%.0fC", data.temp10 / 10.0f);
}

void drawShiftUpNotification() {
  display.fillRect(4, 12, 120, 40, SSD1306_WHITE);
  display.drawRect(6, 14, 116, 36, SSD1306_BLACK);
  
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(12, 24);
  display.println("SHIFT UP!");
}

void renderDisplay() {
  display.clearDisplay();

  TelemetryData localTelemetry;
  portENTER_CRITICAL(&dataMux);
  localTelemetry = currentTelemetry;
  portEXIT_CRITICAL(&dataMux);

  connectionActive = (millis() - lastRxTime <= CONNECTION_TIMEOUT);

  // Tampilan ketika koneksi ESP-NOW terputus
  if (!connectionActive) {
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(12, 24);
    display.println("NO SIGNAL");
    display.display();
    return;
  }

  // Render halaman aktif
  if (activePage == 0) {
    drawDashboardPage(localTelemetry);
  } else {
    drawCurvePage(localTelemetry);
  }

  // Peringatan Shift-Up jika RPM melebihi batas Redline
  if (localTelemetry.rpm > getRedlineRPM(localTelemetry.mode)) {
    drawShiftUpNotification();
  }

  display.display();
}

// =========================================================================
// LOGIKA TOMBOL NAVIGASI & MAIN LOOP
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
      // Ditekan Lama (>= 3 Detik): Mengganti Mode ECU
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
      // Ditekan Singkat: Mengganti Halaman Tampilan
      if (!longPressTriggered) {
        activePage = (activePage + 1) % 2; 
      }
    }
  }
  
  lastButtonState = currentReading;
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Inisialisasi I2C dengan Pin Khusus (SDA=8, SCL=9)
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;); // Berhenti jika OLED gagal terdeteksi
  }

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

  // Logging Histori Grafik setiap 40 ms
  static unsigned long lastDataLogTime = 0;
  if (millis() - lastDataLogTime > 40 && connectionActive) { 
    lastDataLogTime = millis();
    
    portENTER_CRITICAL(&dataMux);
    int16_t degVal = currentTelemetry.degree10;
    portEXIT_CRITICAL(&dataMux);

    // Memetakan derajat pengapian (0 - 40.0 deg) ke tinggi grafik OLED (0 - 38 px)
    advHistoryMapped[historyIdx] = (uint8_t)map(constrain(degVal, 0, 400), 0, 400, 0, 38); 
    historyIdx = (historyIdx + 1) % HISTORY_LEN;
  }

  // Refresh Tampilan Layar (~30 FPS)
  static unsigned long lastRender = 0;
  if (millis() - lastRender > 33) { 
    lastRender = millis();
    renderDisplay();
  }
}
