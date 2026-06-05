#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// =====================================================
// Omega X Wireless Field Scanner v1.1.1.1
// Improvements over v0.4:
// - Tracker bars stay visible during rescans.
// - Slower, calmer live signal tracking.
// - Smooth meter updates with previous signal retained.
// - Better radar page with sweeping sector, rings, pings,
//   signal labels, and smoother motion.
// =====================================================

// ---------------------
// Touch
// ---------------------
#define TOUCH_ADDR 0x38

// ---------------------
// Colors RGB565
// ---------------------
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_CYAN    0x07FF
#define C_YELLOW  0xFFE0
#define C_MAGENTA 0xF81F
#define C_ORANGE  0xFD20
#define C_GRAY    0x8410
#define C_DARK    0x2104
#define C_DARKER  0x1082

// ---------------------
// Display
// ---------------------
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS,
  LCD_SCLK,
  LCD_SDIO0,
  LCD_SDIO1,
  LCD_SDIO2,
  LCD_SDIO3
);

Arduino_SH8601 *gfx = new Arduino_SH8601(
  bus,
  GFX_NOT_DEFINED,
  0,
  LCD_WIDTH,
  LCD_HEIGHT
);

// ---------------------
// Pages
// ---------------------
enum Page {
  PAGE_MAIN,
  PAGE_WIFI_LIST,
  PAGE_WIFI_DETAIL,
  PAGE_WIFI_TRACK,
  PAGE_BLE_LIST,
  PAGE_BLE_DETAIL,
  PAGE_BLE_TRACK,
  PAGE_SYS,
  PAGE_RADAR,
  PAGE_WATCH,
  PAGE_FIELD_STATUS,
  PAGE_SETTINGS,
  PAGE_MESSAGE
};

Page currentPage = PAGE_MAIN;

// ---------------------
// Timing
// ---------------------
unsigned long lastTouchTime = 0;
unsigned long lastRadarFrame = 0;
unsigned long lastTrackUpdate = 0;
unsigned long lastTrackDraw = 0;
int radarFrame = 0;
float radarAngle = 0.0;
float oldRadarAngle = -999.0;
unsigned long lastRadarTelemetry = 0;
int radarSweepCount = 0;
bool radarWifiMode = true;   // true = Wi-Fi radar, false = BLE radar
bool radarScanInProgress = false;

// Live radar engine v1.3
// The ESP32-S3 cannot measure true distance. This uses RSSI as a
// proximity estimate: stronger signal = closer to center.
// v1.3 changes: the radar is now MODED. Wi-Fi scan shows only Wi-Fi
// live targets. BLE scan shows only BLE live targets. No mixed clutter.
bool radarLiveMode = false;
bool radarWifiAsyncRunning = false;
unsigned long lastWifiLiveScanStart = 0;
unsigned long lastBleLiveScan = 0;
unsigned long lastRadarTargetAging = 0;

#define MAX_LIVE_RADAR_TARGETS 14
struct LiveRadarTarget {
  bool active;
  bool isWifi;
  String id;
  String label;
  int rssi;
  int lastRssi;
  float angle;
  float radius;
  float targetRadius;
  unsigned long lastSeen;
  int sourceIndex;
};

LiveRadarTarget liveRadarTargets[MAX_LIVE_RADAR_TARGETS];

// Settings - kept in RAM for now. No SD card needed.
int bleScanSeconds = 7;       // 5, 7, 10
int maxRadarTargets = 4;      // 3, 4, 6
int brightnessLevel = 255;    // 90, 160, 255

// RAM-only watchlist. No SD card needed.
#define MAX_WATCH_WIFI 5
#define MAX_WATCH_BLE 5
struct WatchWifiItem { String ssid; String bssid; int channel; wifi_auth_mode_t auth; };
struct WatchBleItem { String name; String address; String manufacturerHint; String serviceHint; };
WatchWifiItem watchWifi[MAX_WATCH_WIFI];
WatchBleItem watchBle[MAX_WATCH_BLE];
int watchWifiCount = 0;
int watchBleCount = 0;

struct RadarHit { int x; int y; int radius; bool isWifi; int index; };
#define MAX_RADAR_HITS 8
RadarHit radarHits[MAX_RADAR_HITS];
int radarHitCount = 0;

// ---------------------
// Wi-Fi Storage
// ---------------------
#define MAX_WIFI_ITEMS 10

struct WifiItem {
  String ssid;
  String bssid;
  int rssi;
  int channel;
  wifi_auth_mode_t auth;
  bool hidden;
};

WifiItem wifiItems[MAX_WIFI_ITEMS];
int wifiCountStored = 0;
int selectedWifiIndex = -1;
int wifiTrackRssi = -999;
int wifiTrackDisplayedRssi = -999;
bool wifiTrackFound = false;
bool wifiTrackFirstDraw = true;

// ---------------------
// BLE Storage
// ---------------------
#define MAX_BLE_ITEMS 10

struct BleItem {
  String name;
  String address;
  int rssi;
  String type;
  String manufacturerHint;
  String serviceHint;
  String rawCompanyId;
  String rawMfgPrefix;
  String nameSource;
  int seenCount;
};

BleItem bleItems[MAX_BLE_ITEMS];
int bleCountStored = 0;
int selectedBleIndex = -1;
int bleTrackRssi = -999;
int bleTrackDisplayedRssi = -999;
bool bleTrackFound = false;
bool bleTrackFirstDraw = true;
bool bleStarted = false;

// =====================================================
// General helpers
// =====================================================
String shortenText(String s, int maxLen) {
  if (s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 3) + "...";
}

bool inBox(int tx, int ty, int x, int y, int w, int h) {
  return tx >= x && tx <= x + w && ty >= y && ty <= y + h;
}

void drawCenteredText(const char *text, int y, int size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);

  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(text, 0, y, &x1, &y1, &w, &h);

  int x = (LCD_WIDTH - w) / 2;
  gfx->setCursor(x, y);
  gfx->print(text);
}

uint16_t rssiColor(int rssi) {
  if (rssi > -55) return C_GREEN;
  if (rssi > -70) return C_YELLOW;
  return C_RED;
}

String signalLabel(int rssi) {
  if (rssi == -999) return "NOT FOUND";
  if (rssi > -50) return "VERY CLOSE";
  if (rssi > -62) return "CLOSE";
  if (rssi > -75) return "MID RANGE";
  if (rssi > -88) return "FAR";
  return "VERY WEAK";
}

int rssiToBar(int rssi, int maxWidth) {
  if (rssi == -999) return 1;
  int w = map(rssi, -95, -30, 2, maxWidth);
  if (w < 2) w = 2;
  if (w > maxWidth) w = maxWidth;
  return w;
}

int stepToward(int current, int target, int step) {
  if (current == -999) return target;
  if (target == -999) return current;
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}


String strongestWifiLabel() {
  if (wifiCountStored <= 0) return "None";
  int best = 0;
  for (int i = 1; i < wifiCountStored; i++) if (wifiItems[i].rssi > wifiItems[best].rssi) best = i;
  return shortenText(wifiItems[best].ssid, 18) + " " + String(wifiItems[best].rssi) + "dBm";
}

String strongestBleLabel() {
  if (bleCountStored <= 0) return "None";
  int best = 0;
  for (int i = 1; i < bleCountStored; i++) if (bleItems[i].rssi > bleItems[best].rssi) best = i;
  return shortenText(bleItems[best].name, 18) + " " + String(bleItems[best].rssi) + "dBm";
}

// =====================================================
// Touch
// =====================================================
bool readTouch(int &x, int &y) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t data[5];
  int readCount = Wire.requestFrom(TOUCH_ADDR, 5);
  if (readCount < 5) return false;

  for (int i = 0; i < 5; i++) data[i] = Wire.read();

  uint8_t touches = data[0] & 0x0F;
  if (touches == 0) return false;

  x = ((data[1] & 0x0F) << 8) | data[2];
  y = ((data[3] & 0x0F) << 8) | data[4];

  return true;
}

// =====================================================
// Common UI
// =====================================================
void drawBackButton() {
  gfx->drawRoundRect(270, 14, 80, 34, 8, C_CYAN);
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(292, 26);
  gfx->print("BACK");
}

void drawTrackButton() {
  gfx->drawRoundRect(24, 304, 320, 34, 8, C_GREEN);
  gfx->setTextColor(C_GREEN);
  gfx->setTextSize(1);
  gfx->setCursor(126, 317);
  gfx->print("TRACK SIGNAL");
}

void drawWatchButton() {
  gfx->drawRoundRect(24, 344, 320, 34, 8, C_YELLOW);
  gfx->setTextColor(C_YELLOW);
  gfx->setTextSize(1);
  gfx->setCursor(124, 357);
  gfx->print("ADD TO WATCH");
}

void drawFutureButton() {
  gfx->drawRoundRect(24, 384, 320, 34, 8, C_MAGENTA);
  gfx->setTextColor(C_MAGENTA);
  gfx->setTextSize(1);
  gfx->setCursor(118, 397);
  gfx->print("FUTURE ACTION");
}

void drawMiniStatusBar(const char *text) {
  gfx->fillRect(0, 426, LCD_WIDTH, 22, C_DARK);
  gfx->setTextColor(C_GRAY);
  gfx->setTextSize(1);
  gfx->setCursor(12, 433);
  gfx->print(text);
}

void setStatusText(const char *text, uint16_t color = C_GRAY) {
  gfx->fillRect(8, 405, 352, 16, C_BLACK);
  gfx->setTextColor(color);
  gfx->setTextSize(1);
  gfx->setCursor(16, 408);
  gfx->print(text);
}

void drawMessagePage(const char *title, const char *line1, const char *line2) {
  currentPage = PAGE_MESSAGE;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print(title);

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(32, 130);
  gfx->print(line1);

  gfx->setCursor(32, 155);
  gfx->print(line2);

  drawMiniStatusBar("Tap BACK to return");
}

// Meter now redraws only its own fixed area.
// It does not erase the whole tracker body.
void drawSignalMeterStable(int x, int y, int rssi, bool found, const char *label) {
  gfx->fillRect(x - 2, y - 4, 300, 92, C_BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(x, y - 20);
  gfx->print(label);

  gfx->drawRoundRect(x, y, 260, 26, 6, C_GRAY);
  gfx->fillRect(x + 3, y + 3, 254, 20, C_DARKER);

  int bar = found ? rssiToBar(rssi, 254) : 3;

  // Draw tick marks behind the live bar
  for (int i = 0; i <= 5; i++) {
    int tx = x + 3 + (i * 50);
    gfx->drawFastVLine(tx, y + 5, 16, C_DARK);
  }

  gfx->fillRect(x + 3, y + 3, bar, 20, found ? rssiColor(rssi) : C_RED);

  gfx->setTextSize(1);
  gfx->setTextColor(found ? rssiColor(rssi) : C_RED);
  gfx->setCursor(x, y + 38);

  if (found) {
    gfx->print(rssi);
    gfx->print(" dBm  ");
    gfx->print(signalLabel(rssi));
  } else {
    gfx->print("Last signal held - target not found");
  }

  gfx->setTextColor(C_GRAY);
  gfx->setCursor(x, y + 58);
  gfx->print("Move around slowly and watch strength change.");
}

// =====================================================
// Boot + Main
// =====================================================
void bootScreen() {
  gfx->fillScreen(C_BLACK);
  delay(200);

  drawCenteredText("OMEGA X", 52, 4, C_CYAN);
  drawCenteredText("WIRELESS FIELD SCANNER", 105, 1, C_WHITE);

  gfx->drawRoundRect(32, 155, 304, 78, 14, C_CYAN);
  drawCenteredText("FIELD MODE ONLINE", 180, 2, C_GREEN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY);
  gfx->setCursor(56, 255);
  gfx->println("Wi-Fi / BLE / Details / Tracking");

  gfx->setCursor(56, 273);
  gfx->println("watchlist + field status + radar target taps");

  gfx->drawRect(54, 325, 260, 18, C_WHITE);
  for (int w = 0; w <= 256; w += 8) {
    gfx->fillRect(56, 327, w, 14, C_GREEN);
    delay(18);
  }

  delay(500);
}

void drawMenuBox(int x, int y, const char *title, const char *subtitle, uint16_t borderColor) {
  gfx->drawRoundRect(x, y, 148, 78, 12, borderColor);

  gfx->setTextColor(C_GREEN);
  gfx->setTextSize(2);
  gfx->setCursor(x + 22, y + 20);
  gfx->print(title);

  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(x + 22, y + 48);
  gfx->print(subtitle);
}

void drawMainScreen() {
  currentPage = PAGE_MAIN;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("OMEGA X FIELD");

  gfx->drawFastHLine(18, 48, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 64);
  gfx->print("Wireless Field Scanner v1.1.1");

  gfx->setCursor(18, 82);
  gfx->print("Tap WIFI, BLE, WATCH, or RADAR");

  drawMenuBox(20, 112, "WIFI", "AP Scanner", C_CYAN);
  drawMenuBox(200, 112, "BLE", "BLE Scanner", C_CYAN);
  drawMenuBox(20, 212, "SYS", "Status", C_CYAN);
  drawMenuBox(200, 212, "WATCH", "Targets", C_MAGENTA);

  gfx->drawRoundRect(20, 322, 328, 96, 12, C_BLUE);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(36, 338);
  gfx->print("Tap this area for Enhanced Radar");

  gfx->drawCircle(184, 374, 30, C_GREEN);
  gfx->drawCircle(184, 374, 52, C_GREEN);
  gfx->drawFastHLine(124, 374, 120, C_GREEN);
  gfx->drawFastVLine(184, 314, 120, C_GREEN);

  drawMiniStatusBar("Passive scan tool. Use only on authorized networks/devices.");
}

// =====================================================
// Wi-Fi
// =====================================================
String securityName(wifi_auth_mode_t type) {
  switch (type) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
    default: return "UNK";
  }
}

void drawWifiScanningScreen() {
  currentPage = PAGE_WIFI_LIST;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("WIFI SCANNER");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 82);
  gfx->print("Scanning nearby access points...");

  gfx->drawRect(54, 180, 260, 18, C_WHITE);
  for (int w = 0; w <= 256; w += 16) {
    gfx->fillRect(56, 182, w, 14, C_GREEN);
    delay(40);
  }

  drawMiniStatusBar("Scanning...");
}

void drawWifiList() {
  currentPage = PAGE_WIFI_LIST;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("WIFI RESULTS");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 72);
  gfx->print("Tap AP for details. Found: ");
  gfx->print(wifiCountStored);

  int y = 102;

  for (int i = 0; i < wifiCountStored; i++) {
    WifiItem item = wifiItems[i];

    gfx->drawRoundRect(14, y - 6, 340, 30, 6, C_DARK);

    gfx->setTextColor(C_WHITE);
    gfx->setCursor(22, y);
    gfx->print(i + 1);
    gfx->print(". ");
    gfx->print(shortenText(item.ssid, 18));

    gfx->setTextColor(rssiColor(item.rssi));
    gfx->setCursor(230, y);
    gfx->print(item.rssi);
    gfx->print("dBm");

    gfx->setTextColor(C_GRAY);
    gfx->setCursor(300, y);
    gfx->print("CH");
    gfx->print(item.channel);

    gfx->setTextColor(C_CYAN);
    gfx->setCursor(22, y + 13);
    gfx->print(securityName(item.auth));

    int barWidth = rssiToBar(item.rssi, 90);
    gfx->drawRect(100, y + 13, 92, 7, C_GRAY);
    gfx->fillRect(101, y + 14, barWidth, 5, rssiColor(item.rssi));

    y += 34;
  }

  drawMiniStatusBar("BACK=menu | tap network=details");
}

void scanWifi() {
  drawWifiScanningScreen();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(400);

  int count = WiFi.scanNetworks();

  wifiCountStored = 0;
  int maxStore = count;
  if (maxStore > MAX_WIFI_ITEMS) maxStore = MAX_WIFI_ITEMS;

  for (int i = 0; i < maxStore; i++) {
    String ssid = WiFi.SSID(i);

    wifiItems[i].ssid = ssid.length() == 0 ? "<hidden>" : ssid;
    wifiItems[i].hidden = ssid.length() == 0;
    wifiItems[i].bssid = WiFi.BSSIDstr(i);
    wifiItems[i].rssi = WiFi.RSSI(i);
    wifiItems[i].channel = WiFi.channel(i);
    wifiItems[i].auth = WiFi.encryptionType(i);

    wifiCountStored++;
  }

  WiFi.scanDelete();
  drawWifiList();
}

void drawWifiDetail(int index) {
  if (index < 0 || index >= wifiCountStored) return;

  selectedWifiIndex = index;
  currentPage = PAGE_WIFI_DETAIL;

  WifiItem item = wifiItems[index];

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("AP DETAILS");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);

  int y = 82;

  gfx->setCursor(24, y);
  gfx->print("SSID:");
  y += 18;

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(24, y);
  gfx->print(shortenText(item.ssid, 34));
  y += 30;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("BSSID/MAC:");
  y += 18;

  gfx->setTextColor(C_CYAN);
  gfx->setCursor(24, y);
  gfx->print(item.bssid);
  y += 30;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("RSSI: ");
  gfx->setTextColor(rssiColor(item.rssi));
  gfx->print(item.rssi);
  gfx->print(" dBm");
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Channel: ");
  gfx->setTextColor(C_YELLOW);
  gfx->print(item.channel);
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Security: ");
  gfx->setTextColor(C_CYAN);
  gfx->print(securityName(item.auth));
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Hidden: ");
  gfx->setTextColor(item.hidden ? C_YELLOW : C_GREEN);
  gfx->print(item.hidden ? "YES" : "NO");

  drawTrackButton();
  drawWatchButton();
  drawFutureButton();

  drawMiniStatusBar("BACK=list | TRACK=live signal | FUTURE=reserved");
}

void startWifiTracking() {
  if (selectedWifiIndex < 0 || selectedWifiIndex >= wifiCountStored) return;

  currentPage = PAGE_WIFI_TRACK;
  wifiTrackRssi = wifiItems[selectedWifiIndex].rssi;
  wifiTrackDisplayedRssi = wifiTrackRssi;
  wifiTrackFound = true;
  wifiTrackFirstDraw = true;
  lastTrackUpdate = 0;
  lastTrackDraw = 0;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("WIFI TRACKER");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, 82);
  gfx->print("Tracking selected AP:");

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(24, 104);
  gfx->print(shortenText(wifiItems[selectedWifiIndex].ssid, 34));

  gfx->setTextColor(C_CYAN);
  gfx->setCursor(24, 126);
  gfx->print(wifiItems[selectedWifiIndex].bssid);

  drawSignalMeterStable(24, 190, wifiTrackDisplayedRssi, wifiTrackFound, "Current Signal:");
  setStatusText("Holding last reading. Next scan soon...", C_GRAY);
  drawMiniStatusBar("Updates every 5s. Bar stays visible. BACK=details.");
}

void updateWifiTracker() {
  if (currentPage != PAGE_WIFI_TRACK) return;

  // Smooth visual draw every 220ms.
  if (millis() - lastTrackDraw > 220) {
    lastTrackDraw = millis();

    if (wifiTrackFound && wifiTrackRssi != -999) {
      wifiTrackDisplayedRssi = stepToward(wifiTrackDisplayedRssi, wifiTrackRssi, 2);
    }

    drawSignalMeterStable(24, 190, wifiTrackDisplayedRssi, wifiTrackFound, "Current Signal:");
  }

  // Slower rescan so user has time to read the meter.
  if (millis() - lastTrackUpdate < 5000) return;
  lastTrackUpdate = millis();

  WifiItem target = wifiItems[selectedWifiIndex];

  setStatusText("Rescanning... keeping previous signal on screen", C_YELLOW);

  int count = WiFi.scanNetworks();
  bool foundNow = false;
  int newRssi = wifiTrackRssi;

  for (int i = 0; i < count; i++) {
    String bssid = WiFi.BSSIDstr(i);
    if (bssid == target.bssid) {
      foundNow = true;
      newRssi = WiFi.RSSI(i);
      break;
    }
  }

  WiFi.scanDelete();

  wifiTrackFound = foundNow;
  if (foundNow) {
    wifiTrackRssi = newRssi;
    setStatusText("Updated. Watching for signal drift...", C_GREEN);
  } else {
    setStatusText("Target not found. Holding last known signal.", C_RED);
  }
}

// =====================================================
// BLE
// =====================================================

String shortBleAddress(String address) {
  if (address.length() >= 5) {
    return address.substring(address.length() - 5);
  }
  return address;
}

String companyIdToName(uint16_t companyId) {
  switch (companyId) {
    case 0x004C: return "Apple";
    case 0x0006: return "Microsoft";
    case 0x0075: return "Samsung";
    case 0x0059: return "Nordic";
    case 0x00E0: return "Google";
    case 0x0131: return "Sony";
    case 0x000F: return "Broadcom";
    case 0x005D: return "Realtek";
    case 0x000D: return "TexasInst";
    case 0x000A: return "Qualcomm";
    case 0x02E5: return "Tile";
    default: return "MFG";
  }
}


String bytesToHexPrefix(String data, int maxBytes) {
  String out = "";
  int n = data.length();
  if (n > maxBytes) n = maxBytes;

  for (int i = 0; i < n; i++) {
    uint8_t b = (uint8_t)data.charAt(i);
    if (b < 16) out += "0";
    out += String(b, HEX);
    if (i < n - 1) out += " ";
  }

  out.toUpperCase();
  if (out.length() == 0) return "None";
  return out;
}

String appleHeuristicLabel(String rawMfgPrefix, String shortAddr, int rssi) {
  if (rawMfgPrefix.indexOf("4C 00 07") >= 0 || rawMfgPrefix.indexOf("4C 00 19") >= 0) {
    return "Apple-Audio?-" + shortAddr;
  }

  if (rawMfgPrefix.indexOf("4C 00 10") >= 0) {
    return "Apple-Nearby-" + shortAddr;
  }

  if (rssi > -60) {
    return "Apple-Close-" + shortAddr;
  }

  return "Apple-" + shortAddr;
}

String microsoftHeuristicLabel(String rawMfgPrefix, String shortAddr) {
  return "Microsoft-" + shortAddr;
}

String serviceUuidHint(String uuid) {
  uuid.toLowerCase();

  if (uuid.indexOf("180f") >= 0) return "Battery";
  if (uuid.indexOf("180a") >= 0) return "DeviceInfo";
  if (uuid.indexOf("1812") >= 0) return "HID";
  if (uuid.indexOf("180d") >= 0) return "HeartRate";
  if (uuid.indexOf("181a") >= 0) return "EnvSensor";
  if (uuid.indexOf("1809") >= 0) return "HealthThermo";
  if (uuid.indexOf("feaa") >= 0) return "Eddystone";
  if (uuid.indexOf("fda5") >= 0) return "FastPair";
  if (uuid.indexOf("feed") >= 0) return "Beacon/Custom";

  if (uuid.length() > 0) return "Service";
  return "None";
}

String getBleManufacturerHint(BLEAdvertisedDevice device, String &rawCompanyId) {
  rawCompanyId = "None";

  if (!device.haveManufacturerData()) {
    return "None";
  }

  String data = device.getManufacturerData();

  if (data.length() < 2) {
    return "MFG";
  }

  uint8_t b0 = (uint8_t)data.charAt(0);
  uint8_t b1 = (uint8_t)data.charAt(1);
  uint16_t companyId = b0 | (b1 << 8);

  char idBuf[8];
  sprintf(idBuf, "0x%04X", companyId);
  rawCompanyId = String(idBuf);

  return companyIdToName(companyId);
}


String getBleRawMfgPrefix(BLEAdvertisedDevice device) {
  if (!device.haveManufacturerData()) {
    return "None";
  }

  String data = device.getManufacturerData();
  return bytesToHexPrefix(data, 8);
}

String getBleServiceHint(BLEAdvertisedDevice device) {
  if (!device.haveServiceUUID()) {
    return "None";
  }

  String uuid = device.getServiceUUID().toString().c_str();
  return serviceUuidHint(uuid);
}

String getBleDisplayName(BLEAdvertisedDevice device, String manufacturerHint, String serviceHint, String rawMfgPrefix) {
  String name = device.getName().c_str();

  if (name.length() > 0) {
    return name;
  }

  String address = device.getAddress().toString().c_str();
  String shortAddr = shortBleAddress(address);
  int rssi = device.getRSSI();

  if (manufacturerHint == "Apple") {
    return appleHeuristicLabel(rawMfgPrefix, shortAddr, rssi);
  }

  if (manufacturerHint == "Microsoft") {
    return microsoftHeuristicLabel(rawMfgPrefix, shortAddr);
  }

  if (manufacturerHint != "None" && manufacturerHint != "MFG") {
    return manufacturerHint + "-" + shortAddr;
  }

  if (serviceHint != "None" && serviceHint != "Service") {
    return serviceHint + "-" + shortAddr;
  }

  if (manufacturerHint == "MFG") {
    return "MFG-" + shortAddr;
  }

  if (serviceHint == "Service") {
    return "SERVICE-" + shortAddr;
  }

  return "BLE-" + shortAddr;
}

String getBleNameSource(BLEAdvertisedDevice device, String manufacturerHint, String serviceHint) {
  String realName = device.getName().c_str();

  if (realName.length() > 0) return "Advertised Name";
  if (manufacturerHint != "None") return "Manufacturer Heuristic";
  if (serviceHint != "None") return "Service Heuristic";
  return "Address Fallback";
}


void ensureBleStarted() {
  if (!bleStarted) {
    BLEDevice::init("OmegaX_Panel");
    bleStarted = true;
  }
}

void drawBleScanningScreen() {
  currentPage = PAGE_BLE_LIST;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("BLE SCANNER");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 82);
  gfx->print("Scanning BLE adv + scan response...");

  gfx->drawRect(54, 180, 260, 18, C_WHITE);
  for (int w = 0; w <= 256; w += 16) {
    gfx->fillRect(56, 182, w, 14, C_GREEN);
    delay(40);
  }

  drawMiniStatusBar("Scanning BLE advertisements...");
}

void drawBleList() {
  currentPage = PAGE_BLE_LIST;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("BLE RESULTS");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 72);
  gfx->print("Tap BLE device. Found: ");
  gfx->print(bleCountStored);

  int y = 102;

  for (int i = 0; i < bleCountStored; i++) {
    BleItem item = bleItems[i];

    gfx->drawRoundRect(14, y - 6, 340, 30, 6, C_DARK);

    gfx->setTextColor(C_WHITE);
    gfx->setCursor(22, y);
    gfx->print(i + 1);
    gfx->print(". ");
    gfx->print(shortenText(item.name, 20));

    gfx->setTextColor(rssiColor(item.rssi));
    gfx->setCursor(240, y);
    gfx->print(item.rssi);
    gfx->print("dBm");

    gfx->setTextColor(C_CYAN);
    gfx->setCursor(22, y + 13);
    gfx->print(shortenText(item.address, 22));

    int barWidth = rssiToBar(item.rssi, 90);
    gfx->drawRect(210, y + 13, 92, 7, C_GRAY);
    gfx->fillRect(211, y + 14, barWidth, 5, rssiColor(item.rssi));

    y += 34;
  }

  drawMiniStatusBar("BACK=menu | tap device=details");
}


void storeBleDevice(BLEAdvertisedDevice device) {
  String rawCompanyId = "None";
  String manufacturerHint = getBleManufacturerHint(device, rawCompanyId);
  String serviceHint = getBleServiceHint(device);
  String rawMfgPrefix = getBleRawMfgPrefix(device);
  String name = getBleDisplayName(device, manufacturerHint, serviceHint, rawMfgPrefix);
  String nameSource = getBleNameSource(device, manufacturerHint, serviceHint);
  String address = device.getAddress().toString().c_str();
  int rssi = device.getRSSI();

  for (int i = 0; i < bleCountStored; i++) {
    if (bleItems[i].address == address) {
      bleItems[i].seenCount++;

      if (rssi > bleItems[i].rssi) bleItems[i].rssi = rssi;

      if (nameSource == "Advertised Name") {
        bleItems[i].name = name;
        bleItems[i].nameSource = nameSource;
      }

      if (bleItems[i].manufacturerHint == "None" && manufacturerHint != "None") {
        bleItems[i].manufacturerHint = manufacturerHint;
        bleItems[i].rawCompanyId = rawCompanyId;
        bleItems[i].rawMfgPrefix = rawMfgPrefix;
      }

      if (bleItems[i].serviceHint == "None" && serviceHint != "None") {
        bleItems[i].serviceHint = serviceHint;
      }

      return;
    }
  }

  if (bleCountStored >= MAX_BLE_ITEMS) {
    int weakest = 0;
    for (int i = 1; i < MAX_BLE_ITEMS; i++) {
      if (bleItems[i].rssi < bleItems[weakest].rssi) weakest = i;
    }

    if (rssi <= bleItems[weakest].rssi) return;

    bleItems[weakest].name = name;
    bleItems[weakest].address = address;
    bleItems[weakest].rssi = rssi;
    bleItems[weakest].type = "BLE";
    bleItems[weakest].manufacturerHint = manufacturerHint;
    bleItems[weakest].serviceHint = serviceHint;
    bleItems[weakest].rawCompanyId = rawCompanyId;
    bleItems[weakest].rawMfgPrefix = rawMfgPrefix;
    bleItems[weakest].nameSource = nameSource;
    bleItems[weakest].seenCount = 1;
    return;
  }

  bleItems[bleCountStored].name = name;
  bleItems[bleCountStored].address = address;
  bleItems[bleCountStored].rssi = rssi;
  bleItems[bleCountStored].type = "BLE";
  bleItems[bleCountStored].manufacturerHint = manufacturerHint;
  bleItems[bleCountStored].serviceHint = serviceHint;
  bleItems[bleCountStored].rawCompanyId = rawCompanyId;
  bleItems[bleCountStored].rawMfgPrefix = rawMfgPrefix;
  bleItems[bleCountStored].nameSource = nameSource;
  bleItems[bleCountStored].seenCount = 1;
  bleCountStored++;
}

void sortBleItemsByRssi() {
  for (int i = 0; i < bleCountStored - 1; i++) {
    for (int j = i + 1; j < bleCountStored; j++) {
      if (bleItems[j].rssi > bleItems[i].rssi) {
        BleItem temp = bleItems[i];
        bleItems[i] = bleItems[j];
        bleItems[j] = temp;
      }
    }
  }
}

void scanBle() {
  drawBleScanningScreen();

  bleCountStored = 0;
  ensureBleStarted();

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  BLEScanResults *foundDevices = pBLEScan->start(bleScanSeconds, false);

  int count = foundDevices->getCount();

  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    storeBleDevice(device);
  }

  sortBleItemsByRssi();

  pBLEScan->clearResults();
  drawBleList();
}

void drawBleDetail(int index) {
  if (index < 0 || index >= bleCountStored) return;

  selectedBleIndex = index;
  currentPage = PAGE_BLE_DETAIL;

  BleItem item = bleItems[index];

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("BLE DETAILS");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);

  int y = 84;

  gfx->setCursor(24, y);
  gfx->print("Name:");
  y += 18;

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(24, y);
  gfx->print(shortenText(item.name, 34));
  y += 30;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Address:");
  y += 18;

  gfx->setTextColor(C_CYAN);
  gfx->setCursor(24, y);
  gfx->print(item.address);
  y += 30;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("RSSI: ");
  gfx->setTextColor(rssiColor(item.rssi));
  gfx->print(item.rssi);
  gfx->print(" dBm");
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Type: ");
  gfx->setTextColor(C_YELLOW);
  gfx->print(item.type);
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("MFG: ");
  gfx->setTextColor(C_MAGENTA);
  gfx->print(item.manufacturerHint);
  gfx->setTextColor(C_GRAY);
  gfx->print(" ");
  gfx->print(item.rawCompanyId);
  y += 24;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Service: ");
  gfx->setTextColor(C_CYAN);
  gfx->print(item.serviceHint);
  y += 22;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Source: ");
  gfx->setTextColor(C_GREEN);
  gfx->print(item.nameSource);
  y += 22;

  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, y);
  gfx->print("Seen: ");
  gfx->setTextColor(C_YELLOW);
  gfx->print(item.seenCount);
  gfx->print("x");
  y += 18;

  gfx->setTextColor(C_GRAY);
  gfx->setCursor(24, y + 4);
  gfx->print(shortenText(item.rawMfgPrefix, 34));

  drawTrackButton();
  drawWatchButton();
  drawFutureButton();

  drawMiniStatusBar("BACK=list | TRACK=live signal | FUTURE=reserved");
}

void startBleTracking() {
  if (selectedBleIndex < 0 || selectedBleIndex >= bleCountStored) return;

  currentPage = PAGE_BLE_TRACK;
  bleTrackRssi = bleItems[selectedBleIndex].rssi;
  bleTrackDisplayedRssi = bleTrackRssi;
  bleTrackFound = true;
  bleTrackFirstDraw = true;
  lastTrackUpdate = 0;
  lastTrackDraw = 0;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("BLE TRACKER");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, 82);
  gfx->print("Tracking selected BLE:");

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(24, 104);
  gfx->print(shortenText(bleItems[selectedBleIndex].name, 34));

  gfx->setTextColor(C_CYAN);
  gfx->setCursor(24, 126);
  gfx->print(bleItems[selectedBleIndex].address);

  drawSignalMeterStable(24, 190, bleTrackDisplayedRssi, bleTrackFound, "Current Signal:");
  setStatusText("Holding last reading. Next scan soon...", C_GRAY);
  drawMiniStatusBar("Updates every 4s. Bar stays visible. BACK=details.");
}

void updateBleTracker() {
  if (currentPage != PAGE_BLE_TRACK) return;

  // Smooth visual draw every 220ms.
  if (millis() - lastTrackDraw > 220) {
    lastTrackDraw = millis();

    if (bleTrackFound && bleTrackRssi != -999) {
      bleTrackDisplayedRssi = stepToward(bleTrackDisplayedRssi, bleTrackRssi, 2);
    }

    drawSignalMeterStable(24, 190, bleTrackDisplayedRssi, bleTrackFound, "Current Signal:");
  }

  // Slower BLE scan than v0.4. Keeps UI readable.
  if (millis() - lastTrackUpdate < 4000) return;
  lastTrackUpdate = millis();

  BleItem target = bleItems[selectedBleIndex];

  setStatusText("Rescanning BLE... keeping previous signal", C_YELLOW);

  ensureBleStarted();

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  BLEScanResults *foundDevices = pBLEScan->start(3, false);

  int count = foundDevices->getCount();
  bool foundNow = false;
  int newRssi = bleTrackRssi;

  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    String addr = device.getAddress().toString().c_str();

    if (addr == target.address) {
      foundNow = true;
      newRssi = device.getRSSI();
      break;
    }
  }

  pBLEScan->clearResults();

  bleTrackFound = foundNow;
  if (foundNow) {
    bleTrackRssi = newRssi;
    setStatusText("Updated. Watching for signal drift...", C_GREEN);
  } else {
    setStatusText("Target not found. Holding last known signal.", C_RED);
  }
}

// =====================================================
// System
// =====================================================
void drawSystemPage() {
  currentPage = PAGE_SYS;
  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("SYSTEM INFO");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);

  int y = 86;

  gfx->setCursor(24, y);
  gfx->print("Chip: ");
  gfx->print(ESP.getChipModel());
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("Revision: ");
  gfx->print(ESP.getChipRevision());
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("CPU: ");
  gfx->print(ESP.getCpuFreqMHz());
  gfx->print(" MHz");
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("Flash: ");
  gfx->print(ESP.getFlashChipSize() / 1024 / 1024);
  gfx->print(" MB");
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("PSRAM: ");
  gfx->print(ESP.getPsramSize() / 1024 / 1024);
  gfx->print(" MB");
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("Free Heap: ");
  gfx->print(ESP.getFreeHeap());
  y += 24;

  gfx->setCursor(24, y);
  gfx->print("Free PSRAM: ");
  gfx->print(ESP.getFreePsram());
  y += 40;

  gfx->drawRoundRect(24, y, 320, 70, 12, C_GREEN);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN);
  gfx->setCursor(58, y + 24);
  gfx->print("OMEGA X READY");

  drawMiniStatusBar("BACK=menu");
}


// =====================================================
// Live Radar Engine v1.3
// =====================================================
float hashStringToAngle(String s) {
  uint32_t h = 2166136261UL;
  for (int i = 0; i < s.length(); i++) {
    h ^= (uint8_t)s[i];
    h *= 16777619UL;
  }
  return ((h % 6283) / 1000.0); // 0..6.283 radians
}

float radarRadiusFromRssiSmooth(int rssi) {
  // RSSI proximity map. Stronger = closer to center.
  if (rssi < -100) rssi = -100;
  if (rssi > -35) rssi = -35;
  return map(rssi, -100, -35, 102, 24);
}


void clearLiveRadarTargets() {
  for (int i = 0; i < MAX_LIVE_RADAR_TARGETS; i++) {
    liveRadarTargets[i].active = false;
    liveRadarTargets[i].id = "";
    liveRadarTargets[i].label = "";
    liveRadarTargets[i].rssi = -999;
    liveRadarTargets[i].lastRssi = -999;
    liveRadarTargets[i].sourceIndex = -1;
  }
  radarHitCount = 0;
}

void stopWifiLiveRadarScan() {
  if (radarWifiAsyncRunning) {
    WiFi.scanDelete();
    radarWifiAsyncRunning = false;
  }
}

void upsertLiveRadarTarget(bool isWifi, String id, String label, int rssi, int sourceIndex);

void seedWifiLiveRadarTargets() {
  for (int i = 0; i < wifiCountStored; i++) {
    upsertLiveRadarTarget(true, wifiItems[i].bssid, wifiItems[i].ssid, wifiItems[i].rssi, i);
  }
}

void seedBleLiveRadarTargets() {
  for (int i = 0; i < bleCountStored; i++) {
    String label = bleItems[i].name;
    if (label == "<unknown>") label = bleItems[i].manufacturerHint;
    if (label.length() == 0 || label == "Unknown") label = bleItems[i].address.substring(12);
    upsertLiveRadarTarget(false, bleItems[i].address, label, bleItems[i].rssi, i);
  }
}

int findLiveRadarTarget(String id, bool isWifi) {
  for (int i = 0; i < MAX_LIVE_RADAR_TARGETS; i++) {
    if (liveRadarTargets[i].active && liveRadarTargets[i].isWifi == isWifi && liveRadarTargets[i].id == id) return i;
  }
  return -1;
}

int findLiveRadarSlot() {
  for (int i = 0; i < MAX_LIVE_RADAR_TARGETS; i++) {
    if (!liveRadarTargets[i].active) return i;
  }

  // Replace the stalest weak target if all slots are full.
  int oldest = 0;
  for (int i = 1; i < MAX_LIVE_RADAR_TARGETS; i++) {
    if (liveRadarTargets[i].lastSeen < liveRadarTargets[oldest].lastSeen) oldest = i;
  }
  return oldest;
}

void upsertLiveRadarTarget(bool isWifi, String id, String label, int rssi, int sourceIndex) {
  if (id.length() == 0) return;

  int slot = findLiveRadarTarget(id, isWifi);
  if (slot < 0) {
    slot = findLiveRadarSlot();
    liveRadarTargets[slot].active = true;
    liveRadarTargets[slot].isWifi = isWifi;
    liveRadarTargets[slot].id = id;
    liveRadarTargets[slot].label = label;
    liveRadarTargets[slot].rssi = rssi;
    liveRadarTargets[slot].lastRssi = rssi;
    liveRadarTargets[slot].angle = hashStringToAngle(id);
    liveRadarTargets[slot].radius = radarRadiusFromRssiSmooth(rssi);
    liveRadarTargets[slot].targetRadius = liveRadarTargets[slot].radius;
    liveRadarTargets[slot].sourceIndex = sourceIndex;
  } else {
    liveRadarTargets[slot].lastRssi = liveRadarTargets[slot].rssi;
    liveRadarTargets[slot].rssi = rssi;
    liveRadarTargets[slot].targetRadius = radarRadiusFromRssiSmooth(rssi);
    liveRadarTargets[slot].label = label;
    liveRadarTargets[slot].sourceIndex = sourceIndex;
  }

  liveRadarTargets[slot].lastSeen = millis();
}

void ageLiveRadarTargets() {
  if (millis() - lastRadarTargetAging < 500) return;
  lastRadarTargetAging = millis();

  for (int i = 0; i < MAX_LIVE_RADAR_TARGETS; i++) {
    if (!liveRadarTargets[i].active) continue;

    unsigned long age = millis() - liveRadarTargets[i].lastSeen;

    // Smooth movement toward the new RSSI-derived radius.
    liveRadarTargets[i].radius += (liveRadarTargets[i].targetRadius - liveRadarTargets[i].radius) * 0.18;

    // If not seen recently, drift outward before disappearing.
    if (age > 7000) {
      liveRadarTargets[i].targetRadius = 108;
    }
    if (age > 14000) {
      liveRadarTargets[i].active = false;
    }
  }
}

void applyWifiLiveRadarResults(int count) {
  wifiCountStored = 0;
  int maxStore = count;
  if (maxStore > MAX_WIFI_ITEMS) maxStore = MAX_WIFI_ITEMS;

  for (int i = 0; i < maxStore; i++) {
    String ssid = WiFi.SSID(i);
    wifiItems[i].ssid = ssid.length() == 0 ? "<hidden>" : ssid;
    wifiItems[i].hidden = ssid.length() == 0;
    wifiItems[i].bssid = WiFi.BSSIDstr(i);
    wifiItems[i].rssi = WiFi.RSSI(i);
    wifiItems[i].channel = WiFi.channel(i);
    wifiItems[i].auth = WiFi.encryptionType(i);
    wifiCountStored++;

    upsertLiveRadarTarget(true, wifiItems[i].bssid, wifiItems[i].ssid, wifiItems[i].rssi, i);
  }
}

void liveRadarStartWifiScan() {
  if (radarWifiAsyncRunning) return;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true); // async, include hidden
  radarWifiAsyncRunning = true;
  lastWifiLiveScanStart = millis();
}

void liveRadarPollWifiScan() {
  if (!radarWifiAsyncRunning) return;

  int result = WiFi.scanComplete();
  if (result == WIFI_SCAN_RUNNING) {
    if (millis() - lastWifiLiveScanStart > 9000) {
      WiFi.scanDelete();
      radarWifiAsyncRunning = false;
    }
    return;
  }

  if (result >= 0) {
    applyWifiLiveRadarResults(result);
  }

  WiFi.scanDelete();
  radarWifiAsyncRunning = false;
}

void liveRadarRunBleScan() {
  if (millis() - lastBleLiveScan < 3500) return;
  lastBleLiveScan = millis();

  bleCountStored = 0;
  ensureBleStarted();

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(80);
  pBLEScan->setWindow(60);

  // Short scan so the radar animation only hiccups briefly.
  BLEScanResults *foundDevices = pBLEScan->start(1, false);
  int count = foundDevices->getCount();

  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    storeBleDevice(device);
  }

  sortBleItemsByRssi();

  int maxStore = bleCountStored;
  if (maxStore > MAX_BLE_ITEMS) maxStore = MAX_BLE_ITEMS;
  for (int i = 0; i < maxStore; i++) {
    String label = bleItems[i].name;
    if (label == "<unknown>") label = bleItems[i].manufacturerHint;
    if (label.length() == 0 || label == "Unknown") label = bleItems[i].address.substring(12);
    upsertLiveRadarTarget(false, bleItems[i].address, label, bleItems[i].rssi, i);
  }

  pBLEScan->clearResults();
}

void updateLiveRadarScanner() {
  if (!radarLiveMode || currentPage != PAGE_RADAR || radarScanInProgress) return;

  // v1.3: Only the selected layer scans live.
  // Scan Wi-Fi = Wi-Fi-only live radar. Scan BLE = BLE-only live radar.
  if (radarWifiMode) {
    if (!radarWifiAsyncRunning && millis() - lastWifiLiveScanStart > 4200) liveRadarStartWifiScan();
    liveRadarPollWifiScan();
  } else {
    stopWifiLiveRadarScan();
    liveRadarRunBleScan();
  }

  ageLiveRadarTargets();
}

void drawLiveRadarTargets() {
  const int cx = 184;
  const int cy = 252;
  radarHitCount = 0;

  int drawn = 0;
  for (int i = 0; i < MAX_LIVE_RADAR_TARGETS; i++) {
    if (!liveRadarTargets[i].active) continue;
    if (liveRadarTargets[i].isWifi != radarWifiMode) continue;
    if (drawn >= maxRadarTargets + 4) break;

    float a = liveRadarTargets[i].angle;
    float r = liveRadarTargets[i].radius;
    int x = cx + cos(a) * r;
    int y = cy + sin(a) * r;

    uint16_t color = liveRadarTargets[i].isWifi ? C_CYAN : C_MAGENTA;
    if (liveRadarTargets[i].rssi > -55) color = C_GREEN;
    else if (liveRadarTargets[i].rssi < -78) color = C_RED;

    unsigned long age = millis() - liveRadarTargets[i].lastSeen;
    if (age > 7000) color = C_GRAY;

    int pulse = 6 + ((radarFrame + i * 3) % 8);
    gfx->fillCircle(x, y, 3, color);
    gfx->drawCircle(x, y, pulse, color);

    String tag = liveRadarTargets[i].isWifi ? "W:" : "B:";
    drawTargetLabel(x, y, tag + liveRadarTargets[i].label, color);

    addRadarHit(x, y, liveRadarTargets[i].isWifi, liveRadarTargets[i].sourceIndex);
    drawn++;
  }

  if (drawn == 0) {
    gfx->setTextSize(1);
    gfx->setTextColor(C_GRAY);
    gfx->setCursor(58, 238);
    gfx->print("Live radar warming up...");
    gfx->setCursor(52, 256);
    gfx->print(radarWifiMode ? "Wi-Fi-only scanning..." : "BLE-only scanning...");
  }
}

// =====================================================
// Radar Control Center v1.3
// =====================================================
// Radar now has top mode tabs and bottom scan buttons:
// - Top: WIFI AP / BLE layer switch
// - Bottom: SCAN WIFI starts Wi-Fi-only live radar
// - Bottom: SCAN BLE starts BLE-only live radar
// This keeps the screen clean and lets one radio layer own the radar.

void drawRadarModeTabs() {
  // Wi-Fi tab
  gfx->drawRoundRect(42, 86, 120, 28, 7, radarWifiMode ? C_GREEN : C_DARK);
  gfx->setTextSize(1);
  gfx->setTextColor(radarWifiMode ? C_GREEN : C_GRAY);
  gfx->setCursor(74, 96);
  gfx->print("WIFI AP");

  // BLE tab
  gfx->drawRoundRect(206, 86, 120, 28, 7, !radarWifiMode ? C_GREEN : C_DARK);
  gfx->setTextColor(!radarWifiMode ? C_GREEN : C_GRAY);
  gfx->setCursor(252, 96);
  gfx->print("BLE");
}

void drawRadarBottomButtons() {
  // Bottom controls sit above the mini status bar.
  gfx->fillRect(16, 386, 338, 38, C_BLACK);

  gfx->drawRoundRect(24, 390, 150, 30, 8, C_CYAN);
  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(70, 401);
  gfx->print("SCAN WIFI");

  gfx->drawRoundRect(194, 390, 150, 30, 8, C_MAGENTA);
  gfx->setTextColor(C_MAGENTA);
  gfx->setCursor(245, 401);
  gfx->print("SCAN BLE");
}

void drawRadarGrid() {
  const int cx = 184;
  const int cy = 252;
  const int outer = 104;

  // Radar body frame
  gfx->drawRoundRect(24, 122, 320, 256, 16, C_DARK);

  // Soft inner rings
  gfx->drawCircle(cx, cy, 30, C_DARK);
  gfx->drawCircle(cx, cy, 60, C_DARK);
  gfx->drawCircle(cx, cy, 82, C_GREEN);
  gfx->drawCircle(cx, cy, outer, C_GREEN);

  // Crosshair
  gfx->drawFastHLine(cx - outer, cy, outer * 2, C_DARK);
  gfx->drawFastVLine(cx, cy - outer, outer * 2, C_DARK);

  // Center node
  gfx->fillCircle(cx, cy, 4, C_CYAN);
  gfx->drawCircle(cx, cy, 8, C_GREEN);
}

int radarRadiusFromRssi(int rssi) {
  // Strong signal closer to center, weak signal farther out.
  if (rssi > -50) return 30;
  if (rssi > -62) return 52;
  if (rssi > -74) return 76;
  return 98;
}

void drawTargetLabel(int x, int y, String name, uint16_t color) {
  name = shortenText(name, 9);

  int boxW = 8 + (name.length() * 6);
  int boxX = x + 8;
  int boxY = y - 8;

  if (boxX + boxW > 342) boxX = x - boxW - 8;
  if (boxY < 124) boxY = y + 8;
  if (boxY > 352) boxY = 352;

  gfx->fillRect(boxX - 2, boxY - 2, boxW, 12, C_BLACK);
  gfx->drawRect(boxX - 2, boxY - 2, boxW, 12, C_DARK);
  gfx->setTextSize(1);
  gfx->setTextColor(color);
  gfx->setCursor(boxX + 2, boxY);
  gfx->print(name);
}

void addRadarHit(int x, int y, bool isWifi, int index) {
  if (radarHitCount >= MAX_RADAR_HITS) return;
  radarHits[radarHitCount].x = x;
  radarHits[radarHitCount].y = y;
  radarHits[radarHitCount].radius = 22;
  radarHits[radarHitCount].isWifi = isWifi;
  radarHits[radarHitCount].index = index;
  radarHitCount++;
}

void drawNamedWifiTargets() {
  const int cx = 184;
  const int cy = 252;
  int maxTargets = wifiCountStored;
  if (maxTargets > maxRadarTargets) maxTargets = maxRadarTargets;

  if (maxTargets == 0) {
    gfx->setTextSize(1);
    gfx->setTextColor(C_GRAY);
    gfx->setCursor(62, 238);
    gfx->print("No Wi-Fi data.");
    gfx->setCursor(54, 256);
    gfx->print("Tap SCAN WIFI below.");
    return;
  }

  for (int i = 0; i < maxTargets; i++) {
    float a = (-1.2) + (i * (2.4 / max(1, maxTargets - 1)));
    int r = radarRadiusFromRssi(wifiItems[i].rssi);
    int x = cx + cos(a) * r;
    int y = cy + sin(a) * r;

    uint16_t color = rssiColor(wifiItems[i].rssi);

    gfx->fillCircle(x, y, 4, color);
    gfx->drawCircle(x, y, 8, color);
    drawTargetLabel(x, y, wifiItems[i].ssid, color);
    addRadarHit(x, y, true, i);
  }
}

void drawNamedBleTargets() {
  const int cx = 184;
  const int cy = 252;
  int maxTargets = bleCountStored;
  if (maxTargets > maxRadarTargets) maxTargets = maxRadarTargets;

  if (maxTargets == 0) {
    gfx->setTextSize(1);
    gfx->setTextColor(C_GRAY);
    gfx->setCursor(70, 238);
    gfx->print("No BLE data.");
    gfx->setCursor(60, 256);
    gfx->print("Tap SCAN BLE below.");
    return;
  }

  for (int i = 0; i < maxTargets; i++) {
    float a = (-1.0) + (i * (2.0 / max(1, maxTargets - 1))) + 3.14159;
    int r = radarRadiusFromRssi(bleItems[i].rssi);
    int x = cx + cos(a) * r;
    int y = cy + sin(a) * r;

    uint16_t color = rssiColor(bleItems[i].rssi);

    gfx->fillCircle(x, y, 4, color);
    gfx->drawCircle(x, y, 8, color);

    String label = bleItems[i].name;
    if (label == "<unknown>") {
      label = bleItems[i].address.substring(12);
    }

    drawTargetLabel(x, y, label, color);
    addRadarHit(x, y, false, i);
  }
}

void drawRadarTargets() {
  if (radarLiveMode) {
    drawLiveRadarTargets();
    return;
  }

  if (radarWifiMode) {
    drawNamedWifiTargets();
  } else {
    drawNamedBleTargets();
  }
}

void redrawRadarStaticLayer() {
  // Clear body, tabs, and controls, but leave header and mini status alone.
  gfx->fillRect(16, 82, 338, 342, C_BLACK);

  radarHitCount = 0;
  drawRadarModeTabs();
  drawRadarGrid();
  drawRadarTargets();
  drawRadarBottomButtons();

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY);
  gfx->setCursor(34, 364);

  if (radarLiveMode && radarWifiMode) {
    gfx->print("Wi-Fi live radar: AP dots move by RSSI");
  } else if (radarLiveMode && !radarWifiMode) {
    gfx->print("BLE live radar: device dots move by RSSI");
  } else if (radarWifiMode) {
    gfx->print("Wi-Fi layer ready. Tap SCAN WIFI.");
  } else {
    gfx->print("BLE layer ready. Tap SCAN BLE.");
  }
}

void drawRadarBeam(float angle, uint16_t colorMain, uint16_t colorTrail1, uint16_t colorTrail2) {
  const int cx = 184;
  const int cy = 252;
  const int outer = 104;

  float a2 = angle - 0.16;
  float a1 = angle - 0.08;

  int x2 = cx + cos(a2) * outer;
  int y2 = cy + sin(a2) * outer;
  gfx->drawLine(cx, cy, x2, y2, colorTrail2);

  x2 = cx + cos(a1) * outer;
  y2 = cy + sin(a1) * outer;
  gfx->drawLine(cx, cy, x2, y2, colorTrail1);

  x2 = cx + cos(angle) * outer;
  y2 = cy + sin(angle) * outer;
  gfx->drawLine(cx, cy, x2, y2, colorMain);
}

void drawRadarTelemetry(bool force) {
  if (!force && millis() - lastRadarTelemetry < 1000) return;
  lastRadarTelemetry = millis();

  gfx->fillRect(38, 370, 292, 14, C_BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(48, 373);
  gfx->print("SWEEP ");
  gfx->print(radarSweepCount);

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(132, 373);
  if (radarLiveMode) gfx->print(radarWifiMode ? " LIVE: WIFI" : " LIVE: BLE");
  else gfx->print(radarWifiMode ? " MODE: WIFI" : " MODE: BLE");

  gfx->setTextColor(C_GRAY);
  gfx->setCursor(270, 373);
  gfx->print("v1.3");
}

void drawRadarPage() {
  currentPage = PAGE_RADAR;
  radarLiveMode = false;
  radarFrame = 0;
  radarAngle = 0.0;
  oldRadarAngle = -999.0;
  radarSweepCount = 0;
  radarScanInProgress = false;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("FIELD RADAR");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(18, 68);
  gfx->print("Choose Wi-Fi or BLE for live proximity radar");

  redrawRadarStaticLayer();
  drawRadarTelemetry(true);
  drawMiniStatusBar("Tap SCAN WIFI or SCAN BLE to start live radar");

  lastRadarFrame = millis();
}

void setRadarMode(bool wifiMode) {
  radarLiveMode = false;
  stopWifiLiveRadarScan();
  radarWifiMode = wifiMode;
  radarAngle = 0.0;
  oldRadarAngle = -999.0;
  radarSweepCount = 0;
  redrawRadarStaticLayer();
  drawRadarTelemetry(true);
}

void drawRadarScanOverlay(const char *label, uint16_t color) {
  radarScanInProgress = true;

  // Pause/cover the body during scan so it looks intentional.
  gfx->fillRect(44, 178, 280, 120, C_BLACK);
  gfx->drawRoundRect(44, 178, 280, 120, 12, color);

  gfx->setTextSize(2);
  gfx->setTextColor(color);
  gfx->setCursor(82, 210);
  gfx->print(label);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(82, 242);
  gfx->print("Updating radar targets...");

  gfx->drawRect(82, 268, 204, 12, C_GRAY);
  for (int w = 0; w <= 200; w += 20) {
    gfx->fillRect(84, 270, w, 8, color);
    delay(35);
  }
}

void radarScanWifi() {
  radarLiveMode = false;
  stopWifiLiveRadarScan();
  clearLiveRadarTargets();
  radarWifiMode = true;
  drawRadarScanOverlay("SCAN WIFI", C_CYAN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(250);

  int count = WiFi.scanNetworks();

  wifiCountStored = 0;
  int maxStore = count;
  if (maxStore > MAX_WIFI_ITEMS) maxStore = MAX_WIFI_ITEMS;

  for (int i = 0; i < maxStore; i++) {
    String ssid = WiFi.SSID(i);

    wifiItems[i].ssid = ssid.length() == 0 ? "<hidden>" : ssid;
    wifiItems[i].hidden = ssid.length() == 0;
    wifiItems[i].bssid = WiFi.BSSIDstr(i);
    wifiItems[i].rssi = WiFi.RSSI(i);
    wifiItems[i].channel = WiFi.channel(i);
    wifiItems[i].auth = WiFi.encryptionType(i);

    wifiCountStored++;
  }

  WiFi.scanDelete();

  seedWifiLiveRadarTargets();
  radarLiveMode = true;
  radarScanInProgress = false;
  oldRadarAngle = -999.0;
  lastWifiLiveScanStart = 0;
  redrawRadarStaticLayer();
  drawRadarTelemetry(true);
  drawMiniStatusBar("LIVE Wi-Fi radar | tap SCAN BLE to switch");
}

void radarScanBle() {
  radarLiveMode = false;
  stopWifiLiveRadarScan();
  clearLiveRadarTargets();
  radarWifiMode = false;
  drawRadarScanOverlay("SCAN BLE", C_MAGENTA);

  bleCountStored = 0;
  ensureBleStarted();

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  BLEScanResults *foundDevices = pBLEScan->start(bleScanSeconds, false);

  int count = foundDevices->getCount();

  for (int i = 0; i < count; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    storeBleDevice(device);
  }

  sortBleItemsByRssi();

  pBLEScan->clearResults();

  seedBleLiveRadarTargets();
  radarLiveMode = true;
  radarScanInProgress = false;
  oldRadarAngle = -999.0;
  lastBleLiveScan = millis();
  redrawRadarStaticLayer();
  drawRadarTelemetry(true);
  drawMiniStatusBar("LIVE BLE radar | tap SCAN WIFI to switch");
}

void updateRadarPage() {
  if (currentPage != PAGE_RADAR) return;
  if (radarScanInProgress) return;

  updateLiveRadarScanner();

  if (millis() - lastRadarFrame < 70) return;
  lastRadarFrame = millis();

  // Erase previous beam by painting it black.
  if (oldRadarAngle > -900.0) {
    drawRadarBeam(oldRadarAngle, C_BLACK, C_BLACK, C_BLACK);
  }

  // Restore static items where the beam crossed.
  drawRadarGrid();
  drawRadarTargets();

  oldRadarAngle = radarAngle;
  radarAngle += 0.055;

  if (radarAngle > TWO_PI) {
    radarAngle -= TWO_PI;
    radarSweepCount++;
  }

  drawRadarBeam(radarAngle, C_CYAN, C_GREEN, C_DARK);

  // Keep center crisp.
  gfx->fillCircle(184, 252, 4, C_CYAN);
  gfx->drawCircle(184, 252, 8, C_GREEN);

  radarFrame++;
  drawRadarTelemetry(false);
}
// =====================================================
// Watchlist Page - RAM only
// =====================================================
bool addWifiToWatch(int index) {
  if (index < 0 || index >= wifiCountStored) return false;
  WifiItem item = wifiItems[index];
  for (int i = 0; i < watchWifiCount; i++) if (watchWifi[i].bssid == item.bssid) return true;
  if (watchWifiCount >= MAX_WATCH_WIFI) return false;
  watchWifi[watchWifiCount].ssid = item.ssid;
  watchWifi[watchWifiCount].bssid = item.bssid;
  watchWifi[watchWifiCount].channel = item.channel;
  watchWifi[watchWifiCount].auth = item.auth;
  watchWifiCount++;
  return true;
}

bool addBleToWatch(int index) {
  if (index < 0 || index >= bleCountStored) return false;
  BleItem item = bleItems[index];
  for (int i = 0; i < watchBleCount; i++) if (watchBle[i].address == item.address) return true;
  if (watchBleCount >= MAX_WATCH_BLE) return false;
  watchBle[watchBleCount].name = item.name;
  watchBle[watchBleCount].address = item.address;
  watchBle[watchBleCount].manufacturerHint = item.manufacturerHint;
  watchBle[watchBleCount].serviceHint = item.serviceHint;
  watchBleCount++;
  return true;
}

void drawWatchPage() {
  currentPage = PAGE_WATCH;
  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(C_CYAN); gfx->setCursor(18,18); gfx->print("WATCHLIST");
  drawBackButton(); gfx->drawFastHLine(18,56,332,C_CYAN);
  gfx->setTextSize(1); gfx->setTextColor(C_WHITE); gfx->setCursor(18,72); gfx->print("Tap target to track. RAM only.");
  int y=100;
  gfx->setTextColor(C_CYAN); gfx->setCursor(22,y); gfx->print("Wi-Fi Targets"); y+=18;
  if (watchWifiCount==0) { gfx->setTextColor(C_GRAY); gfx->setCursor(28,y); gfx->print("No saved Wi-Fi targets."); y+=28; }
  else for (int i=0;i<watchWifiCount;i++){ gfx->drawRoundRect(20,y-6,328,28,6,C_DARK); gfx->setTextColor(C_WHITE); gfx->setCursor(30,y); gfx->print("W"); gfx->print(i+1); gfx->print(" "); gfx->print(shortenText(watchWifi[i].ssid,21)); gfx->setTextColor(C_GRAY); gfx->setCursor(238,y); gfx->print("CH"); gfx->print(watchWifi[i].channel); y+=34; }
  y+=4; gfx->setTextColor(C_MAGENTA); gfx->setCursor(22,y); gfx->print("BLE Targets"); y+=18;
  if (watchBleCount==0) { gfx->setTextColor(C_GRAY); gfx->setCursor(28,y); gfx->print("No saved BLE targets."); y+=28; }
  else for (int i=0;i<watchBleCount;i++){ gfx->drawRoundRect(20,y-6,328,28,6,C_DARK); gfx->setTextColor(C_WHITE); gfx->setCursor(30,y); gfx->print("B"); gfx->print(i+1); gfx->print(" "); gfx->print(shortenText(watchBle[i].name,20)); gfx->setTextColor(C_GRAY); gfx->setCursor(230,y); gfx->print(shortBleAddress(watchBle[i].address)); y+=34; }
  gfx->drawRoundRect(24,386,150,32,8,C_RED); gfx->setTextColor(C_RED); gfx->setCursor(72,398); gfx->print("CLEAR");
  gfx->drawRoundRect(194,386,150,32,8,C_CYAN); gfx->setTextColor(C_CYAN); gfx->setCursor(246,398); gfx->print("STATUS");
  drawMiniStatusBar("BACK=menu | target tap=track | CLEAR removes list");
}

void clearWatchlist(){ watchWifiCount=0; watchBleCount=0; drawWatchPage(); }

void trackWatchWifi(int index) {
  if (index < 0 || index >= watchWifiCount) return;
  for (int i=0;i<wifiCountStored;i++) if (wifiItems[i].bssid==watchWifi[index].bssid) { selectedWifiIndex=i; startWifiTracking(); return; }
  if (wifiCountStored < MAX_WIFI_ITEMS) { int i=wifiCountStored; wifiItems[i].ssid=watchWifi[index].ssid; wifiItems[i].bssid=watchWifi[index].bssid; wifiItems[i].channel=watchWifi[index].channel; wifiItems[i].auth=watchWifi[index].auth; wifiItems[i].rssi=-75; wifiItems[i].hidden=false; wifiCountStored++; selectedWifiIndex=i; startWifiTracking(); return; }
  drawMessagePage("WATCH ERROR","Run a Wi-Fi scan first.","Then open the watch target.");
}

void trackWatchBle(int index) {
  if (index < 0 || index >= watchBleCount) return;
  for (int i=0;i<bleCountStored;i++) if (bleItems[i].address==watchBle[index].address) { selectedBleIndex=i; startBleTracking(); return; }
  if (bleCountStored < MAX_BLE_ITEMS) { int i=bleCountStored; bleItems[i].name=watchBle[index].name; bleItems[i].address=watchBle[index].address; bleItems[i].rssi=-75; bleItems[i].type="BLE"; bleItems[i].manufacturerHint=watchBle[index].manufacturerHint; bleItems[i].serviceHint=watchBle[index].serviceHint; bleItems[i].rawCompanyId="Watch"; bleItems[i].rawMfgPrefix="Watchlist"; bleItems[i].nameSource="Watchlist"; bleItems[i].seenCount=1; bleCountStored++; selectedBleIndex=i; startBleTracking(); return; }
  drawMessagePage("WATCH ERROR","Run a BLE scan first.","Then open the watch target.");
}

// =====================================================
// Field Status Page
// =====================================================
void drawFieldStatusPage() {
  currentPage = PAGE_FIELD_STATUS;
  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(C_CYAN); gfx->setCursor(18,18); gfx->print("FIELD STATUS");
  drawBackButton(); gfx->drawFastHLine(18,56,332,C_CYAN);
  gfx->setTextSize(1); gfx->setTextColor(C_WHITE);
  int y=84;
  gfx->setCursor(24,y); gfx->print("Wi-Fi APs: "); gfx->setTextColor(C_GREEN); gfx->print(wifiCountStored); y+=24;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("BLE Devices: "); gfx->setTextColor(C_GREEN); gfx->print(bleCountStored); y+=24;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("Strongest Wi-Fi:"); y+=18; gfx->setTextColor(C_CYAN); gfx->setCursor(36,y); gfx->print(strongestWifiLabel()); y+=28;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("Strongest BLE:"); y+=18; gfx->setTextColor(C_MAGENTA); gfx->setCursor(36,y); gfx->print(strongestBleLabel()); y+=28;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("Watch Wi-Fi / BLE: "); gfx->setTextColor(C_YELLOW); gfx->print(watchWifiCount); gfx->print(" / "); gfx->print(watchBleCount); y+=24;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("BLE scan time: "); gfx->setTextColor(C_GREEN); gfx->print(bleScanSeconds); gfx->print(" sec"); y+=24;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("Radar targets: "); gfx->setTextColor(C_GREEN); gfx->print(maxRadarTargets); y+=24;
  gfx->setTextColor(C_WHITE); gfx->setCursor(24,y); gfx->print("Free heap: "); gfx->setTextColor(C_GRAY); gfx->print(ESP.getFreeHeap());
  gfx->drawRoundRect(24,360,150,34,8,C_CYAN); gfx->setTextColor(C_CYAN); gfx->setCursor(72,373); gfx->print("SETTINGS");
  gfx->drawRoundRect(194,360,150,34,8,C_MAGENTA); gfx->setTextColor(C_MAGENTA); gfx->setCursor(246,373); gfx->print("WATCH");
  drawMiniStatusBar("BACK=menu | status summarizes scan memory");
}

// =====================================================
// Settings Page
// =====================================================
void drawSettingsPage() {
  currentPage = PAGE_SETTINGS;

  gfx->fillScreen(C_BLACK);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(18, 18);
  gfx->print("SETTINGS");

  drawBackButton();
  gfx->drawFastHLine(18, 56, 332, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(24, 78);
  gfx->print("Tap a row to cycle values.");

  gfx->drawRoundRect(24, 116, 320, 54, 10, C_CYAN);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(42, 132);
  gfx->print("BLE Scan Time");
  gfx->setTextColor(C_GREEN);
  gfx->setCursor(250, 132);
  gfx->print(bleScanSeconds);
  gfx->print(" sec");

  gfx->drawRoundRect(24, 190, 320, 54, 10, C_CYAN);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(42, 206);
  gfx->print("Max Radar Targets");
  gfx->setTextColor(C_GREEN);
  gfx->setCursor(270, 206);
  gfx->print(maxRadarTargets);

  gfx->drawRoundRect(24, 264, 320, 54, 10, C_CYAN);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(42, 280);
  gfx->print("Brightness");
  gfx->setTextColor(C_GREEN);
  gfx->setCursor(250, 280);
  gfx->print(brightnessLevel);

  gfx->setTextColor(C_GRAY);
  gfx->setCursor(32, 346);
  gfx->print("No SD logging in this build.");

  gfx->setCursor(32, 366);
  gfx->print("Settings reset after power cycle for now.");

  drawMiniStatusBar("BACK=menu | values apply immediately");
}

void cycleBleScanSeconds() {
  if (bleScanSeconds == 5) bleScanSeconds = 7;
  else if (bleScanSeconds == 7) bleScanSeconds = 10;
  else bleScanSeconds = 5;

  drawSettingsPage();
}

void cycleMaxRadarTargets() {
  if (maxRadarTargets == 3) maxRadarTargets = 4;
  else if (maxRadarTargets == 4) maxRadarTargets = 6;
  else maxRadarTargets = 3;

  drawSettingsPage();
}

void cycleBrightness() {
  if (brightnessLevel == 90) brightnessLevel = 160;
  else if (brightnessLevel == 160) brightnessLevel = 255;
  else brightnessLevel = 90;

  gfx->setBrightness(brightnessLevel);
  drawSettingsPage();
}

// =====================================================
// Future Action Hooks
// =====================================================
void runWifiFutureAction() {
  drawMessagePage("WIFI FUTURE", "Reserved Wi-Fi action slot.", "Your custom Wi-Fi feature goes here.");
}

void runBleFutureAction() {
  drawMessagePage("BLE FUTURE", "Reserved BLE action slot.", "Your custom BLE feature goes here.");
}

// =====================================================
// Touch Handler
// =====================================================
void handleTouch() {
  int tx, ty;

  if (!readTouch(tx, ty)) return;
  if (millis() - lastTouchTime < 250) return;

  lastTouchTime = millis();

  Serial.print("Touch X=");
  Serial.print(tx);
  Serial.print(" Y=");
  Serial.println(ty);

  // Back button
  if (currentPage != PAGE_MAIN && inBox(tx, ty, 270, 14, 80, 34)) {
    if (currentPage == PAGE_WIFI_DETAIL) {
      drawWifiList();
      return;
    }

    if (currentPage == PAGE_BLE_DETAIL) {
      drawBleList();
      return;
    }

    if (currentPage == PAGE_WIFI_TRACK) {
      drawWifiDetail(selectedWifiIndex);
      return;
    }

    if (currentPage == PAGE_BLE_TRACK) {
      drawBleDetail(selectedBleIndex);
      return;
    }

    drawMainScreen();
    return;
  }

  // Detail page buttons
  if (currentPage == PAGE_WIFI_DETAIL) {
    if (inBox(tx, ty, 24, 304, 320, 34)) { startWifiTracking(); return; }
    if (inBox(tx, ty, 24, 344, 320, 34)) { bool ok=addWifiToWatch(selectedWifiIndex); drawMessagePage(ok?"WATCHLIST":"WATCH FULL", ok?"Wi-Fi target saved.":"Wi-Fi watchlist is full.", "Tap BACK to return."); return; }
    if (inBox(tx, ty, 24, 384, 320, 34)) { runWifiFutureAction(); return; }
  }

  if (currentPage == PAGE_BLE_DETAIL) {
    if (inBox(tx, ty, 24, 304, 320, 34)) { startBleTracking(); return; }
    if (inBox(tx, ty, 24, 344, 320, 34)) { bool ok=addBleToWatch(selectedBleIndex); drawMessagePage(ok?"WATCHLIST":"WATCH FULL", ok?"BLE target saved.":"BLE watchlist is full.", "Tap BACK to return."); return; }
    if (inBox(tx, ty, 24, 384, 320, 34)) { runBleFutureAction(); return; }
  }

  // Radar mode switch tabs + radar scan buttons
  if (currentPage == PAGE_RADAR) {
    if (inBox(tx, ty, 42, 86, 120, 28)) {
      setRadarMode(true);
      return;
    }

    if (inBox(tx, ty, 206, 86, 120, 28)) {
      setRadarMode(false);
      return;
    }

    for (int i=0; i<radarHitCount; i++) {
      int dx=tx-radarHits[i].x; int dy=ty-radarHits[i].y;
      if ((dx*dx + dy*dy) <= (radarHits[i].radius * radarHits[i].radius)) {
        if (radarHits[i].isWifi) drawWifiDetail(radarHits[i].index); else drawBleDetail(radarHits[i].index);
        return;
      }
    }

    if (inBox(tx, ty, 24, 390, 150, 30)) { radarScanWifi(); return; }
    if (inBox(tx, ty, 194, 390, 150, 30)) { radarScanBle(); return; }
  }

  // Watchlist controls
  if (currentPage == PAGE_WATCH) {
    int y=118;
    if (watchWifiCount==0) y+=28; else { for (int i=0;i<watchWifiCount;i++){ if(inBox(tx,ty,20,y-6,328,28)){ trackWatchWifi(i); return; } y+=34; } }
    y+=22;
    if (watchBleCount==0) y+=28; else { for (int i=0;i<watchBleCount;i++){ if(inBox(tx,ty,20,y-6,328,28)){ trackWatchBle(i); return; } y+=34; } }
    if (inBox(tx,ty,24,386,150,32)) { clearWatchlist(); return; }
    if (inBox(tx,ty,194,386,150,32)) { drawFieldStatusPage(); return; }
  }

  // Field status controls
  if (currentPage == PAGE_FIELD_STATUS) {
    if (inBox(tx,ty,24,360,150,34)) { drawSettingsPage(); return; }
    if (inBox(tx,ty,194,360,150,34)) { drawWatchPage(); return; }
  }

  // Settings controls
  if (currentPage == PAGE_SETTINGS) {
    if (inBox(tx, ty, 24, 116, 320, 54)) {
      cycleBleScanSeconds();
      return;
    }

    if (inBox(tx, ty, 24, 190, 320, 54)) {
      cycleMaxRadarTargets();
      return;
    }

    if (inBox(tx, ty, 24, 264, 320, 54)) {
      cycleBrightness();
      return;
    }
  }

  // Main menu
  if (currentPage == PAGE_MAIN) {
    if (inBox(tx, ty, 20, 112, 148, 78)) {
      scanWifi();
      return;
    }

    if (inBox(tx, ty, 200, 112, 148, 78)) {
      scanBle();
      return;
    }

    if (inBox(tx, ty, 20, 212, 148, 78)) { drawFieldStatusPage(); return; }

    if (inBox(tx, ty, 200, 212, 148, 78)) { drawWatchPage(); return; }

    if (inBox(tx, ty, 20, 322, 328, 96)) {
      drawRadarPage();
      return;
    }
  }

  // Wi-Fi list item taps
  if (currentPage == PAGE_WIFI_LIST) {
    int y = 102;

    for (int i = 0; i < wifiCountStored; i++) {
      if (inBox(tx, ty, 14, y - 6, 340, 30)) {
        drawWifiDetail(i);
        return;
      }

      y += 34;
    }
  }

  // BLE list item taps
  if (currentPage == PAGE_BLE_LIST) {
    int y = 102;

    for (int i = 0; i < bleCountStored; i++) {
      if (inBox(tx, ty, 14, y - 6, 340, 30)) {
        drawBleDetail(i);
        return;
      }

      y += 34;
    }
  }
}

// =====================================================
// I2C Scan
// =====================================================
void scanI2CDevices() {
  Serial.println("Scanning I2C devices...");

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Omega X Wireless Field Scanner v1.1.1.1");

  Wire.begin(IIC_SDA, IIC_SCL);
  pinMode(TP_INT, INPUT);

  scanI2CDevices();

  if (!gfx->begin()) {
    Serial.println("Display begin failed");
    while (true) delay(1000);
  }

  gfx->setBrightness(brightnessLevel);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  bootScreen();
  drawMainScreen();
}

void loop() {
  handleTouch();

  updateRadarPage();
  updateWifiTracker();
  updateBleTracker();
}
