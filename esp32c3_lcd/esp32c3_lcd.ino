#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_now.h>

// =========================================================================
// PANDUAN REKOMENDASI PINOUT ESP32-C3 (Atur di User_Setup.h milik TFT_eSPI)
// =========================================================================
// MOSI (SDA) : GPIO 6
// SCLK (SCL) : GPIO 4
// CS         : GPIO 7
// DC         : GPIO 2
// RST        : GPIO 1 (atau -1 jika disambung ke EN)
// =========================================================================

const uint8_t PIN_BUTTON = 3; // Gunakan GPIO 3 (Internal Pullup) untuk Tombol

// --- KONSTANTA SISTEM ---
const uint16_t MAX_RPM = 14000;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_DELAY = 3000;
const unsigned long CONNECTION_TIMEOUT = 1000;

TFT_eSPI tft = TFT_eSPI(); 
// Menggunakan SATU Canvas Utama (Full Frame Buffer 284x76) -> RAM ~43KB
TFT_eSprite sprCanvas = TFT_eSprite(&tft); 

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

// --- VARIABEL NAVIGASI & TRANSISI LAYAR ---
uint8_t activePage = 0; 
uint8_t targetPage = 0;
bool isPageTransitioning = false;
int pageSlideOffset = 0; 

// --- VARIABEL TRANSISI MODE (COLOR FADING) ---
uint8_t activeMode = 0;
uint8_t previousMode = 0;
float modeFadeProgress = 1.0f; 

// --- VARIABEL TRANSISI NOTIFIKASI SHIFT UP ---
float shiftUpAnimY = -50.0f; 

// Variabel Tombol
unsigned long buttonPressStartTime = 0;
bool buttonIsPressed = false;
bool longPressTriggered = false;
bool lastButtonState = HIGH; 
unsigned long lastDebounceTime = 0;

#define HISTORY_LEN 284
uint8_t tpsHistory[HISTORY_LEN] = {0};
uint8_t advHistoryMapped[HISTORY_LEN] = {0};
int historyIdx = 0;

// MAC Address Spesifik ECU Target (Ganti dengan MAC Address ECU Anda)
uint8_t targetEcuAddress[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; 
esp_now_peer_info_t peerInfo;


// =========================================================================
// FUNGSI PENGATUR RPM DINAMIS BERDASARKAN MODE
// =========================================================================

uint16_t getRedlineRPM(uint8_t mode) {
  switch (mode) {
    case 0: return 5000;   // Mode 0: DAILY
    case 1: return 8000;   // Mode 1: RACE
    case 2: return 10000;  // Mode 2: EXTREME
    case 3: return 11500;  // Mode 3: CUSTOM
    default: return 5000;
  }
}

uint16_t getYellowlineRPM(uint8_t mode) {
  switch (mode) {
    case 0: return 4000;   // Mode 0: DAILY
    case 1: return 6500;   // Mode 1: RACE
    case 2: return 8500;   // Mode 2: EXTREME
    case 3: return 9500;   // Mode 3: CUSTOM
    default: return 4000;
  }
}

// =========================================================================
// FUNGSI UTILITAS & WARNA
// =========================================================================

uint16_t blendColor(uint16_t color1, uint16_t color2, float alpha) {
  if (alpha <= 0.0f) return color1;
  if (alpha >= 1.0f) return color2;

  uint8_t r1 = (color1 >> 11) & 0x1F;
  uint8_t g1 = (color1 >> 5) & 0x3F;
  uint8_t b1 = color1 & 0x1F;

  uint8_t r2 = (color2 >> 11) & 0x1F;
  uint8_t g2 = (color2 >> 5) & 0x3F;
  uint8_t b2 = color2 & 0x1F;

  uint8_t r = r1 + (r2 - r1) * alpha;
  uint8_t g = g1 + (g2 - g1) * alpha;
  uint8_t b = b1 + (b2 - b1) * alpha;

  return (r << 11) | (g << 5) | b;
}

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

uint16_t getModeBaseColor(uint8_t mode) {
  switch (mode) {
    case 1: return 0x3800; // RACE (Merah gelap)
    case 2: return 0x3018; // EXTR (Oranye/Cokelat)
    case 3: return 0x0320; // CUST (Hijau gelap)
    default: return 0x18A3; // DAILY (Biru)
  }
}

uint16_t getModeAccentColor(uint8_t mode) {
  switch (mode) {
    case 1: return 0x5800;
    case 2: return 0x502A;
    case 3: return 0x0440;
    default: return 0x10A2;
  }
}

// =========================================================================
// ESP-NOW CALLBACK
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
// FUNGSI RENDER TAMPILAN
// =========================================================================

void showWelcomeLogo() {
  sprCanvas.fillSprite(0x18A3);
  sprCanvas.drawRect(8, 6, 268, 64, TFT_CYAN);
  sprCanvas.drawRect(10, 8, 264, 60, 0x10A2);

  sprCanvas.setTextColor(TFT_CYAN); 
  sprCanvas.setTextSize(2);
  sprCanvas.setCursor(42, 18);
  sprCanvas.println("KUDUS SATRIA CLUB");

  sprCanvas.setTextSize(1);
  sprCanvas.setTextColor(TFT_WHITE);
  sprCanvas.setCursor(82, 45);
  sprCanvas.println("PRO DASHBOARD v3.0");
  
  sprCanvas.pushSprite(0, 0);
  delay(2000); 
}

void drawDashboardContent(const TelemetryData &data, int offsetX, uint16_t bgCol, uint16_t accentCol) {
  sprCanvas.fillRect(offsetX, 0, 284, 76, bgCol);
  
  for (int x = 0; x < 284; x += 4) {
    sprCanvas.drawFastVLine(offsetX + x, 0, 76, accentCol); 
  }
  sprCanvas.drawRect(offsetX + 4, 4, 276, 68, TFT_DARKGREY);

  sprCanvas.setTextSize(1);
  sprCanvas.setTextColor(TFT_CYAN);
  sprCanvas.setCursor(offsetX + 140, 12);
  sprCanvas.println("RPM");

  sprCanvas.setTextColor(TFT_WHITE);
  sprCanvas.setTextSize(3);
  sprCanvas.setCursor(offsetX + 10, 10);
  sprCanvas.printf("%5d", data.rpm);

  String modeStr = "DAILY";
  if (data.mode == 1) modeStr = "RACE ";
  else if (data.mode == 2) modeStr = "EXTR ";
  else if (data.mode == 3) modeStr = "CUST ";
  
  sprCanvas.drawRect(offsetX + 138, 25, 60, 15, TFT_DARKGREY);
  sprCanvas.setTextColor(TFT_YELLOW);
  sprCanvas.setTextSize(1);
  sprCanvas.setCursor(offsetX + 144, 29);
  sprCanvas.printf("%s", modeStr.c_str());

  sprCanvas.drawRect(offsetX + 10, 43, 260, 6, TFT_BLACK); 
  
  // LOGIKA WARNA BAR RPM DINAMIS
  uint16_t redline = getRedlineRPM(data.mode);
  uint16_t yellowline = getYellowlineRPM(data.mode);
  int barWidth = map(constrain(data.rpm, 0, MAX_RPM), 0, MAX_RPM, 0, 256);
  uint16_t barColor = TFT_GREEN;
  
  if (data.rpm > yellowline && data.rpm <= redline) barColor = TFT_YELLOW;
  else if (data.rpm > redline) barColor = TFT_RED;

  if (barWidth > 0) sprCanvas.fillRect(offsetX + 11, 44, barWidth, 4, barColor); 

  sprCanvas.setTextSize(1);
  sprCanvas.setTextColor(TFT_GREEN);
  sprCanvas.setCursor(offsetX + 10, 56);
  sprCanvas.printf("DEG:%4.1f*", data.degree10 / 10.0f);

  sprCanvas.setTextColor(TFT_WHITE);
  sprCanvas.setCursor(offsetX + 82, 56);
  sprCanvas.printf("TPS:%3d%%", data.tps);

  sprCanvas.setTextColor(TFT_ORANGE);
  sprCanvas.setCursor(offsetX + 145, 56);
  sprCanvas.printf("TMP:%2.0fC", data.temp10 / 10.0f);

  sprCanvas.setTextColor(TFT_CYAN);
  sprCanvas.setCursor(offsetX + 215, 56);
  sprCanvas.printf("BAT:%4.1fV", data.battVolt10 / 10.0f);
}

void drawCurveContent(const TelemetryData &data, int offsetX, uint16_t bgCol, uint16_t accentCol) {
  sprCanvas.fillRect(offsetX, 0, 284, 76, bgCol);
  for (int x = 0; x < 284; x += 35) sprCanvas.drawFastVLine(offsetX + x, 0, 76, accentCol);
  for (int y = 0; y < 76; y += 20) sprCanvas.drawFastHLine(offsetX, y, 284, accentCol);

  // LOGIKA WARNA KURVA DINAMIS
  uint16_t lineColor = (data.rpm > getRedlineRPM(data.mode)) ? TFT_RED : TFT_CYAN;
  int prevX = 0;
  int prevY = 65; 

  for (int x = 0; x < HISTORY_LEN - 1; x++) {
    int readIdx = (historyIdx + x) % HISTORY_LEN;
    int currentY = 65 - advHistoryMapped[readIdx]; 
    if (x > 0) sprCanvas.drawLine(offsetX + prevX, prevY, offsetX + x, currentY, lineColor);
    prevX = x;
    prevY = currentY;
  }

  sprCanvas.setTextColor(TFT_YELLOW);
  sprCanvas.setTextSize(1);
  sprCanvas.setCursor(offsetX + 6, 6);
  sprCanvas.printf("REAL MAP | ADV: %.1f*", data.degree10 / 10.0f);

  sprCanvas.setTextColor(TFT_WHITE);
  sprCanvas.setCursor(offsetX + 215, 6);
  sprCanvas.printf("%d RPM", data.rpm);

  sprCanvas.setTextColor(TFT_SILVER);
  sprCanvas.setCursor(offsetX + 6, 62);
  sprCanvas.printf("TPS:%d%% | TMP:%.0fC", data.tps, data.temp10 / 10.0f);
}

void drawShiftUpNotification() {
  if (shiftUpAnimY > -48.0f) {
    int curY = (int)shiftUpAnimY;
    sprCanvas.fillRoundRect(12, curY, 260, 50, 8, TFT_RED);
    sprCanvas.drawRoundRect(12, curY, 260, 50, 8, TFT_WHITE);
    
    sprCanvas.setTextColor(TFT_WHITE);
    sprCanvas.setTextSize(3);
    sprCanvas.setCursor(62, curY + 14);
    sprCanvas.println("SHIFT UP!");
  }
}

void renderDisplay() {
  TelemetryData localTelemetry;
  portENTER_CRITICAL(&dataMux);
  localTelemetry = currentTelemetry;
  portEXIT_CRITICAL(&dataMux);

  connectionActive = (millis() - lastRxTime <= CONNECTION_TIMEOUT);

  if (!connectionActive) {
    sprCanvas.fillSprite(0x18A3);
    sprCanvas.setTextColor(TFT_RED);
    sprCanvas.setTextSize(2);
    sprCanvas.setCursor(80, 28);
    sprCanvas.println("NO SIGNAL");
    sprCanvas.pushSprite(0, 0);
    return;
  }

  // 1. UPDATE TRANSISI MODE (SMOOTH COLOR FADE)
  if (localTelemetry.mode != activeMode) {
    previousMode = activeMode;
    activeMode = localTelemetry.mode;
    modeFadeProgress = 0.0f; 
  }

  if (modeFadeProgress < 1.0f) {
    modeFadeProgress += 0.08f; 
    if (modeFadeProgress > 1.0f) modeFadeProgress = 1.0f;
  }

  uint16_t bgCol = blendColor(getModeBaseColor(previousMode), getModeBaseColor(activeMode), modeFadeProgress);
  uint16_t accentCol = blendColor(getModeAccentColor(previousMode), getModeAccentColor(activeMode), modeFadeProgress);

  // 2. UPDATE TRANSISI POPUP SHIFT-UP DINAMIS (EASING SLIDE)
  uint16_t currentRedline = getRedlineRPM(localTelemetry.mode);
  if (localTelemetry.rpm > currentRedline) {
    if (shiftUpAnimY < 12.0f) shiftUpAnimY += (12.0f - shiftUpAnimY) * 0.25f; // Meluncur Turun
  } else {
    if (shiftUpAnimY > -50.0f) shiftUpAnimY += (-50.0f - shiftUpAnimY) * 0.25f; // Meluncur Naik
  }

  // 3. UPDATE TRANSISI HALAMAN (SLIDE IN/OUT)
  if (isPageTransitioning) {
    pageSlideOffset += 32; // Kecepatan geser
    if (pageSlideOffset >= 284) {
      pageSlideOffset = 0;
      activePage = targetPage;
      isPageTransitioning = false;
    }
  }

  // 4. RENDER ELEMENT KE CANVAS
  if (!isPageTransitioning) {
    if (activePage == 0) drawDashboardContent(localTelemetry, 0, bgCol, accentCol);
    else if (activePage == 1) drawCurveContent(localTelemetry, 0, bgCol, accentCol);
  } else {
    int dir = (targetPage > activePage) ? 1 : -1;
    int currentX = -dir * pageSlideOffset;
    int nextX = dir * (284 - pageSlideOffset);

    if (activePage == 0) drawDashboardContent(localTelemetry, currentX, bgCol, accentCol);
    else if (activePage == 1) drawCurveContent(localTelemetry, currentX, bgCol, accentCol);

    if (targetPage == 0) drawDashboardContent(localTelemetry, nextX, bgCol, accentCol);
    else if (targetPage == 1) drawCurveContent(localTelemetry, nextX, bgCol, accentCol);
  }

  drawShiftUpNotification();

  // 5. PUSH KE LAYAR
  sprCanvas.pushSprite(0, 0);
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
      if (!longPressTriggered && !isPageTransitioning) {
        targetPage = (activePage + 1) % 2; 
        isPageTransitioning = true;
        pageSlideOffset = 0;
      }
    }
  }
  
  lastButtonState = currentReading;
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  tft.init();
  tft.setRotation(1); 
  
  if (!sprCanvas.createSprite(284, 76)) {
    while (1); // Berhenti jika gagal alokasi RAM
  }

  showWelcomeLogo();

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
    uint8_t tpsVal = currentTelemetry.tps;
    int16_t degVal = currentTelemetry.degree10;
    portEXIT_CRITICAL(&dataMux);

    tpsHistory[historyIdx] = tpsVal;
    advHistoryMapped[historyIdx] = (uint8_t)map(degVal, 0, 400, 0, 45); 
    historyIdx = (historyIdx + 1) % HISTORY_LEN;
  }

  static unsigned long lastRender = 0;
  if (millis() - lastRender > 20) { // Berjalan di ~50 FPS untuk transisi yang sangat mulus
    lastRender = millis();
    renderDisplay();
  }
}
