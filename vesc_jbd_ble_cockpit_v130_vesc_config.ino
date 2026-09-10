// V19 - scan/connect race fix + JBD RX checksum fix.
/*
  VESC + JBD BLE COCKPIT
  ESP32-2432S028 / TPM408-2.8 CYD

  BLE:
    VESC1 + VESC2: Nordic UART Service
    JBD: Xiaoxiang/JBD BLE

  IMPORTANT:
    - No touch calibration screen.
    - Settings are stored in ESP32 NVS.
    - BLE device selection is done from the panel.
    - The two VESCs can be selected independently.
    - Speed uses configurable pole pairs + wheel diameter.
    - TFT inversion is REQUIRED on this panel.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include <NimBLEDevice.h>
#include <math.h>

// ---------- TFT ----------
#define TFT_SCLK 14
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  -1
#define TFT_BL   21

// CYD ESP32-2432S028 RGB LED on rear side (active LOW)
#define RGB_LED_R 4
#define RGB_LED_G 16
#define RGB_LED_B 17

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, TFT_RST, 0, false);

// ---------- TOUCH ----------
#define TOUCH_CLK 25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CS 33
#define TOUCH_IRQ 36

SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

// Fixed touch mapping; no calibration UI.
static const int TOUCH_X_MIN = 250;
static const int TOUCH_X_MAX = 3850;
static const int TOUCH_Y_MIN = 250;
static const int TOUCH_Y_MAX = 3850;

// ---------- COLORS ----------
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_GREY    0x8410
#define C_DGREY   0x4208
#define C_CYAN    0x07FF
#define C_GREEN   0x07E0
#define C_ORANGE  0xFD20
#define C_RED     0xF800
#define C_BLUE    0x001F
#define C_YELLOW  0xFFE0

// ---------- BLE UUID ----------
static const char *NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NUS_RX      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NUS_TX      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

static const char *JBD_SERVICE_1 = "0000ffe0-0000-1000-8000-00805f9b34fb";
static const char *JBD_WRITE_1   = "0000ffe1-0000-1000-8000-00805f9b34fb";
static const char *JBD_NOTIFY_1  = "0000ffe2-0000-1000-8000-00805f9b34fb";

static const char *JBD_SERVICE_2 = "0000ff00-0000-1000-8000-00805f9b34fb";
static const char *JBD_NOTIFY_2  = "0000ff01-0000-1000-8000-00805f9b34fb";
static const char *JBD_WRITE_2   = "0000ff02-0000-1000-8000-00805f9b34fb";

// ---------- SETTINGS ----------
struct Config {
  uint8_t polePairs = 7;
  float wheelDiameter = 0.254f;
  uint8_t batterySeries = 13;
  float battMin = 39.0f;
  float battMax = 54.6f;
  float battCapacity = 20.0f;
  float maxSpeed = 55.0f;
  float maxPower = 8.0f;
  float maxBatteryCurrent = 100.0f;
  float maxMotorCurrent = 200.0f;
  uint16_t vescPollMs = 250;
  uint16_t jbdPollMs = 700;
  bool smoothDisplay = true;
  bool autoReconnect = false;
  uint16_t bleConnectTimeoutMs = 2500;
  uint8_t vesc1AddrType = BLE_ADDR_PUBLIC;
  uint8_t vesc2AddrType = BLE_ADDR_PUBLIC;
  uint8_t jbdAddrType = BLE_ADDR_PUBLIC;
  char vesc1Addr[18] = "";
  char vesc2Addr[18] = "";
  char jbdAddr[18] = "";
} cfg;

Preferences prefs;

// ---------- TELEMETRY ----------
struct VescData {
  bool valid = false;
  float fetTemp = 0;
  float motorTemp = 0;
  float motorCurrent = 0;
  float inputCurrent = 0;
  float id = 0;
  float iq = 0;
  float duty = 0;
  float erpm = 0;
  float vin = 0;
  float ah = 0;
  float ahCharged = 0;
  float wh = 0;
  float whCharged = 0;
  int32_t tach = 0;
  int32_t tachAbs = 0;
  uint8_t fault = 0;
  uint32_t lastMs = 0;
};

struct JbdData {
  bool valid = false;
  float voltage = 0;
  float current = 0;
  float remainingAh = 0;
  float capacityAh = 0;
  float soc = 0;
  uint16_t protection = 0;
  uint8_t cellCount = 0;
  uint8_t ntcCount = 0;
  float temp1 = NAN;
  float temp2 = NAN;
  float minCell = 0;
  float maxCell = 0;
  float delta = 0;
  float cells[32] = {};
  uint32_t lastMs = 0;
};

struct VescConfigData {
  bool valid=false; uint32_t signature=0;
  uint8_t pwmMode=0, commMode=0, motorType=0, sensorMode=0;
  float motorCurrentMax=0, motorCurrentMin=0, batteryCurrentMax=0, batteryCurrentMin=0;
  float absCurrentMax=0, minErpm=0, maxErpm=0, erpmStart=0, minDuty=0, maxDuty=0, wattMax=0, wattMin=0;
  uint32_t lastMs=0; uint16_t payloadLen=0;
};
VescConfigData vescCfg[2];
uint32_t lastConfigRequestMs[2]={0,0};
int configPageLast=-1;

VescData vesc1, vesc2;
JbdData jbd;

// ---------- LONG-TERM BMS CELL BALANCE ----------
// Per-cell exponentially stable running average of absolute deviation from
// the pack cell-voltage average.  Stored in NVS periodically so the result
// survives power cycles and really represents long-term behaviour.
float cellDevAvg[32] = {};
uint32_t cellDevSamples[32] = {};
float longTermMaxDelta = 0.0f;
float longTermMaxCellDev = 0.0f;
uint8_t longTermWorstCell = 0;
uint32_t cellStatsLastSaveMs = 0;
bool cellStatsDirty = false;
const uint32_t CELL_STATS_SAVE_MS = 600000UL; // 10 min

void saveLongTermCellStats() {
  bool opened = prefs.begin("cockpit", false);
  if (!opened) return;
  prefs.putBytes("cdevavg", cellDevAvg, sizeof(cellDevAvg));
  prefs.putBytes("cdevn", cellDevSamples, sizeof(cellDevSamples));
  prefs.putFloat("ltmaxd", longTermMaxDelta);
  prefs.putFloat("ltmaxdev", longTermMaxCellDev);
  prefs.putUChar("ltworst", longTermWorstCell);
  prefs.end();
  cellStatsDirty = false;
  cellStatsLastSaveMs = millis();
}

void loadLongTermCellStats() {
  bool opened = prefs.begin("cockpit", true);
  if (!opened) return;
  if (prefs.getBytesLength("cdevavg") == sizeof(cellDevAvg))
    prefs.getBytes("cdevavg", cellDevAvg, sizeof(cellDevAvg));
  if (prefs.getBytesLength("cdevn") == sizeof(cellDevSamples))
    prefs.getBytes("cdevn", cellDevSamples, sizeof(cellDevSamples));
  longTermMaxDelta = prefs.getFloat("ltmaxd", 0.0f);
  longTermMaxCellDev = prefs.getFloat("ltmaxdev", 0.0f);
  longTermWorstCell = prefs.getUChar("ltworst", 0);
  prefs.end();
}

void updateLongTermCellStats() {
  if (!jbd.cellCount) return;
  uint8_t n = min((uint8_t)32, jbd.cellCount);
  float avg = 0.0f;
  uint8_t valid = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (jbd.cells[i] > 0.1f) { avg += jbd.cells[i]; valid++; }
  }
  if (!valid) return;
  avg /= valid;

  bool changed = false;
  if (jbd.delta > longTermMaxDelta) {
    longTermMaxDelta = jbd.delta;
    changed = true;
  }

  for (uint8_t i = 0; i < n; i++) {
    if (jbd.cells[i] <= 0.1f) continue;
    float dev = fabsf(jbd.cells[i] - avg);
    uint32_t count = cellDevSamples[i];
    if (count < 0xFFFFFFFFUL) {
      count++;
      cellDevAvg[i] += (dev - cellDevAvg[i]) / (float)count;
      cellDevSamples[i] = count;
    }
    if (cellDevAvg[i] > longTermMaxCellDev) {
      longTermMaxCellDev = cellDevAvg[i];
      longTermWorstCell = i;
      changed = true;
    }
  }
  // Re-evaluate in case an old record cell changed after loading.
  float best = 0.0f;
  uint8_t bestCell = longTermWorstCell;
  for (uint8_t i = 0; i < n; i++) {
    if (cellDevSamples[i] && cellDevAvg[i] > best) {
      best = cellDevAvg[i];
      bestCell = i;
    }
  }
  if (best > 0.0f && (bestCell != longTermWorstCell || best > longTermMaxCellDev)) {
    longTermWorstCell = bestCell;
    longTermMaxCellDev = best;
    changed = true;
  }
  if (changed || valid) cellStatsDirty = true;
}

// ---------- BLE OBJECTS ----------
NimBLEClient *vescClient[2] = {nullptr, nullptr};
NimBLERemoteCharacteristic *vescRx[2] = {nullptr, nullptr};
NimBLERemoteCharacteristic *vescTx[2] = {nullptr, nullptr};

NimBLEClient *jbdClient = nullptr;
NimBLERemoteCharacteristic *jbdWrite = nullptr;
NimBLERemoteCharacteristic *jbdNotify = nullptr;

bool vescConnected[2] = {false, false};
bool jbdConnected = false;

uint32_t lastReconnectAttempt[3] = {0, 0, 0};
float tripWh = 0.0f;
float tripKm = 0.0f;
float lastEnergyMs = 0.0f;
float lastSpeedSample = 0.0f;
uint32_t lastEnergyTick = 0;

// ---------- SCAN LIST ----------
struct FoundDevice {
  String name;
  String addr;
  int rssi = 0;
  bool vesc = false;
  bool jbd = false;
  uint8_t addrType = BLE_ADDR_PUBLIC;
};

FoundDevice found[40];
int foundCount = 0;
int foundScroll = 0;
int selectedFound = -1;
bool scanning = false;

// ---------- UI ----------
enum UiMode {
  UI_DASH,
  UI_MENU,
  UI_BLE,
  UI_SETTINGS
};

UiMode uiMode = UI_DASH;
int page = 0;
static const int PAGE_COUNT = 23;
int settingsItem = 0;
int bleSlot = 0; // 0 VESC1, 1 VESC2, 2 JBD

// ---------- ON-SCREEN DIAGNOSTIC LOG ----------
// forceDraw is defined later with the rest of the UI state.
extern bool forceDraw;
static const uint8_t DIAG_LINES = 18;
String diagLog[DIAG_LINES];
uint8_t diagHead = 0;
uint8_t diagCount = 0;
int diagScroll = 0;

// ---------- AUTOMATIC SCAN + RECONNECT OF SAVED DEVICES ----------
// BLE does not need classic "pairing" here.  We scan for the saved MAC
// addresses and reconnect to VESC1, VESC2 and JBD when they are visible.
bool autoConnectRunning = false;
bool autoScanActive = false;
uint8_t autoConnectSlot = 0;
uint32_t autoConnectNextMs = 0;

const uint32_t AUTO_CONNECT_INTERVAL_MS = 30000UL;
const uint32_t AUTO_CONNECT_START_DELAY_MS = 250UL;
const uint32_t AUTO_CONNECT_RETRY_GAP_MS = 1500UL;

extern bool bleBusy;
extern volatile bool bleSetupRunning;
extern volatile int8_t bleSetupRequest;
extern bool vescConnected[2];
extern bool jbdConnected;

bool connectVescSlot(uint8_t slot, const char *addressText, uint8_t addrType);
bool connectJbd(const char *addressText, uint8_t addrType);
void startScan();

bool savedAddrForSlot(uint8_t slot) {
  if (slot == 0) return cfg.vesc1Addr[0] != 0;
  if (slot == 1) return cfg.vesc2Addr[0] != 0;
  if (slot == 2) return cfg.jbdAddr[0] != 0;
  return false;
}

bool slotAlreadyConnected(uint8_t slot) {
  if (slot < 2) return vescConnected[slot];
  return jbdConnected;
}

int findSavedDevice(uint8_t slot) {
  if (!savedAddrForSlot(slot)) return -1;

  const char *saved =
    slot == 0 ? cfg.vesc1Addr :
    slot == 1 ? cfg.vesc2Addr : cfg.jbdAddr;

  for (int i = 0; i < foundCount; i++) {
    if (found[i].addr.equalsIgnoreCase(saved)) return i;
  }
  return -1;
}

void autoConnectTick() {
  if (!cfg.autoReconnect) {
    autoConnectRunning = false;
    autoScanActive = false;
    return;
  }

  uint32_t now = millis();
  if ((int32_t)(now - autoConnectNextMs) < 0) return;

  // Start a real 5-second scan.  This is deliberately scan-first: after
  // boot or a lost link we do not try an old address blindly.
  if (!autoConnectRunning) {
    if (bleBusy || bleSetupRunning || bleSetupRequest >= 0) {
      autoConnectNextMs = now + 250;
      return;
    }

    autoConnectRunning = true;
    autoScanActive = true;
    autoConnectSlot = 0;
    diagAdd("AUTO SCAN START");

    startScan();
    if (!scanning) {
      autoScanActive = false;
      autoConnectRunning = false;
      autoConnectNextMs = now + 5000;
      diagAdd("AUTO SCAN FAILED");
    }
    return;
  }

  // Let the asynchronous scanner finish and populate found[].
  if (autoScanActive) {
    if (scanning) return;
    autoScanActive = false;
    autoConnectSlot = 0;
    diagAdd(String("AUTO FOUND ") + String(foundCount));
  }

  // Never overlap GATT discovery/connection operations.
  if (bleBusy || bleSetupRunning || bleSetupRequest >= 0) {
    autoConnectNextMs = now + 250;
    return;
  }

  while (autoConnectSlot < 3) {
    uint8_t s = autoConnectSlot++;

    if (!savedAddrForSlot(s)) continue;
    if (slotAlreadyConnected(s)) continue;

    int idx = findSavedDevice(s);
    if (idx < 0) {
      diagAdd(String("AUTO ") + (s < 2 ? String("V") + String(s + 1) : String("JBD")) + " NOT FOUND");
      continue;
    }

    uint8_t at = found[idx].addrType;

    if (s < 2) {
      diagAdd(String("AUTO V") + String(s + 1) + " CONNECT");
      connectVescSlot(s, found[idx].addr.c_str(), at);
    } else {
      diagAdd("AUTO JBD CONNECT");
      connectJbd(found[idx].addr.c_str(), at);
    }

    // Async connect/discovery continues in the BLE task.  Give it time
    // before looking at the next saved device.
    autoConnectNextMs = now + AUTO_CONNECT_RETRY_GAP_MS;
    return;
  }

  // Nothing else to connect right now.  Scan again in 30 s if something
  // was out of range.  A disconnect also resets this state immediately.
  autoConnectRunning = false;
  autoConnectNextMs = now + AUTO_CONNECT_INTERVAL_MS;
  diagAdd("AUTO SCAN WAIT 30s");
}

void diagAdd(const String &msg) {
  char line[54];
  String s = msg;
  if (s.length() > 49) s = s.substring(0, 49);
  snprintf(line, sizeof(line), "%7.1fs %s", millis() / 1000.0f, s.c_str());

  diagLog[diagHead] = String(line);
  diagHead = (diagHead + 1) % DIAG_LINES;
  if (diagCount < DIAG_LINES) diagCount++;
  diagScroll = 0;
  forceDraw = true;

  Serial.printf("[DIAG] %s\n", line);
}

String diagAt(int newestOffset) {
  if (newestOffset < 0 || newestOffset >= diagCount) return "";
  int idx = (int)diagHead - 1 - newestOffset;
  while (idx < 0) idx += DIAG_LINES;
  return diagLog[idx];
}

uint32_t lastDraw = 0;
uint32_t lastTouch = 0;
uint32_t lastVescPoll = 0;
uint32_t lastJbdPoll = 0;
bool forceDraw = true;
float displaySpeed = 0.0f;
float displayPower = 0.0f;
uint32_t loopCounter = 0;
uint32_t loopWindowStart = 0;
float loopHz = 0.0f;
uint32_t bootMs = 0;
String bleStatus[3] = {"OFFLINE", "OFFLINE", "OFFLINE"};

// Connection activity is deliberately serialized.  A BLE connect/discovery can
// block the NimBLE client for a while, so never start more than one from loop().
bool bleBusy = false;
int8_t bleBusySlot = -1;
uint32_t bleBusySince = 0;
uint32_t lastBleErrorMs[3] = {0,0,0};
bool bleNeedsSetup[3] = {false, false, false};
int bleLastReason[3] = {0, 0, 0};
uint32_t bleConnectStarted[3] = {0, 0, 0};

// BLE service discovery runs outside the Arduino loop. NimBLE getService() can
// perform a synchronous GATT discovery; doing that in loop() made the touchscreen
// appear frozen and prevented DISCONNECT/BACK from being processed.
volatile int8_t bleSetupRequest = -1;
volatile bool bleSetupRunning = false;
volatile bool bleSetupCancel = false;
TaskHandle_t bleSetupTaskHandle = nullptr;

// Session statistics. They are intentionally RAM-only and reset by RESET STATS.
float maxSpeedSession = 0.0f;
float maxPowerSession = 0.0f;
float maxBatteryCurrentSession = 0.0f;
float maxMotorTempSession = 0.0f;
float maxCellDeltaSession = 0.0f;

void setBleStatus(uint8_t slot, const char *s) {
  if (slot < 3) {
    bleStatus[slot] = s;
    forceDraw = true;
  }
}


// ---------- UTIL ----------
static uint16_t rd16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}
static int16_t rdS16(const uint8_t *p) {
  return (int16_t)rd16(p);
}
static int32_t rdS32(const uint8_t *p) {
  return ((int32_t)p[0] << 24) | ((int32_t)p[1] << 16) |
         ((int32_t)p[2] << 8) | p[3];
}

static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

float getFloat32Auto(const uint8_t *p, int32_t &idx, size_t n) {
  if (idx+4>(int32_t)n) return NAN;
  uint32_t r=((uint32_t)p[idx]<<24)|((uint32_t)p[idx+1]<<16)|((uint32_t)p[idx+2]<<8)|p[idx+3]; idx+=4;
  int e=(r>>23)&0xFF; uint32_t si=r&0x7FFFFF; float sig=0.0f;
  if(e!=0 || si!=0){sig=(float)si/(8388608.0f*2.0f)+0.5f; e-=126;}
  if(r&0x80000000UL) sig=-sig; return ldexpf(sig,e);
}
uint32_t getU32BE(const uint8_t *p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
float getFloat16(const uint8_t *p,int32_t &idx,size_t n,float scale){
  if(idx+2>(int32_t)n)return NAN; int16_t v=(int16_t)(((uint16_t)p[idx]<<8)|p[idx+1]); idx+=2; return (float)v/scale;
}
void parseVescConfig(uint8_t slot,const uint8_t *p,size_t n){
  if(slot>1 || n<9) return; VescConfigData &c=vescCfg[slot]; int32_t i=0;
  c.signature=getU32BE(p); i=4; c.pwmMode=p[i++]; c.commMode=p[i++]; c.motorType=p[i++]; c.sensorMode=p[i++];
  c.motorCurrentMax=getFloat32Auto(p,i,n); c.motorCurrentMin=getFloat32Auto(p,i,n);
  c.batteryCurrentMax=getFloat32Auto(p,i,n); c.batteryCurrentMin=getFloat32Auto(p,i,n);
  if(i+2<=(int32_t)n)i+=2; c.absCurrentMax=getFloat32Auto(p,i,n); c.minErpm=getFloat32Auto(p,i,n); c.maxErpm=getFloat32Auto(p,i,n); c.erpmStart=getFloat32Auto(p,i,n);
  // FW 6.x: brake ERPM x2, VIN x2, battery cut x4.
  for(int k=0;k<8;k++)(void)getFloat32Auto(p,i,n);
  if(i<(int32_t)n)i++; // slow absolute-current limit flag
  // Four thermal bytes followed by float16 accel-dec, min duty and max duty.
  if(i+4<=(int32_t)n)i+=4;
  (void)getFloat16(p,i,n,10000.0f);
  c.minDuty=getFloat16(p,i,n,10000.0f);
  c.maxDuty=getFloat16(p,i,n,10000.0f);
  c.wattMax=getFloat32Auto(p,i,n); c.wattMin=getFloat32Auto(p,i,n);
  c.valid=true;c.lastMs=millis();c.payloadLen=n;
}
void requestVescConfig(uint8_t slot){
  if(slot>1||!vescConnected[slot]||!vescRx[slot])return;
  uint8_t p1[1]={13};uint16_t c1=crc16(p1,1);uint8_t q1[6]={2,1,13,(uint8_t)(c1>>8),(uint8_t)c1,3};vescRx[slot]->writeValue(q1,6,false);
  uint8_t p2[1]={16};uint16_t c2=crc16(p2,1);uint8_t q2[6]={2,1,16,(uint8_t)(c2>>8),(uint8_t)c2,3};vescRx[slot]->writeValue(q2,6,false);
  lastConfigRequestMs[slot]=millis();
}

float speedFromErpm(float erpm) {
  float pp = max(1.0f, (float)cfg.polePairs);
  float rpm = fabsf(erpm) / pp;
  return rpm * PI * cfg.wheelDiameter * 60.0f / 1000.0f;
}

float batteryVoltage() {
  if (jbd.valid) return jbd.voltage;
  if (vesc1.valid) return vesc1.vin;
  if (vesc2.valid) return vesc2.vin;
  return 0;
}

float batteryCurrent() {
  if (jbd.valid) return jbd.current;
  if (vesc1.valid) return vesc1.inputCurrent;
  if (vesc2.valid) return vesc2.inputCurrent;
  return 0;
}

float combinedPowerW() {
  float v = batteryVoltage();
  float a = batteryCurrent();
  if (v == 0) return 0;
  return v * a;
}

// ---------- SETTINGS AUTO-PERSIST ----------
// Previously settings (including "Auto reconnect") only reached flash if the
// user pressed the explicit SAVE row in Settings. Forgetting that step meant
// every value - including whether auto-reconnect was even on - reverted to
// firmware defaults after a reboot/power cycle. configDirty + the check in
// loop() now debounce-save any change automatically a moment after the user
// stops touching a value, so nothing needs a manual SAVE anymore.
bool configDirty = false;
uint32_t configDirtySinceMs = 0;
const uint32_t CONFIG_AUTOSAVE_DEBOUNCE_MS = 800;

void markConfigDirty() {
  configDirty = true;
  configDirtySinceMs = millis();
}

void saveConfig() {
  bool opened = prefs.begin("cockpit", false);
  if (!opened) {
    diagAdd("NVS SAVE FAILED (begin)");
    configDirty = false; // don't spin retrying every loop on a hard NVS failure
    return;
  }
  prefs.putUChar("poles", cfg.polePairs);
  prefs.putFloat("wheel", cfg.wheelDiameter);
  prefs.putUChar("series", cfg.batterySeries);
  prefs.putFloat("bmin", cfg.battMin);
  prefs.putFloat("bmax", cfg.battMax);
  prefs.putFloat("cap", cfg.battCapacity);
  prefs.putFloat("speed", cfg.maxSpeed);
  prefs.putFloat("power", cfg.maxPower);
  prefs.putFloat("bcur", cfg.maxBatteryCurrent);
  prefs.putFloat("mcur", cfg.maxMotorCurrent);
  prefs.putUShort("vpoll", cfg.vescPollMs);
  prefs.putUShort("jpoll", cfg.jbdPollMs);
  prefs.putBool("smooth", cfg.smoothDisplay);
  prefs.putBool("autorec", cfg.autoReconnect);
  prefs.putUShort("bleto", cfg.bleConnectTimeoutMs);
  prefs.putUChar("v1type", cfg.vesc1AddrType);
  prefs.putUChar("v2type", cfg.vesc2AddrType);
  prefs.putUChar("jtype", cfg.jbdAddrType);
  prefs.putString("v1", cfg.vesc1Addr);
  prefs.putString("v2", cfg.vesc2Addr);
  prefs.putString("jbd", cfg.jbdAddr);
  prefs.end();
  configDirty = false;
  diagAdd("SETTINGS SAVED TO NVS");
}

void loadConfig() {
  prefs.begin("cockpit", true);
  cfg.polePairs = prefs.getUChar("poles", 7);
  cfg.wheelDiameter = prefs.getFloat("wheel", 0.254f);
  cfg.batterySeries = prefs.getUChar("series", 13);
  cfg.battMin = prefs.getFloat("bmin", 39.0f);
  cfg.battMax = prefs.getFloat("bmax", 54.6f);
  cfg.battCapacity = prefs.getFloat("cap", 20.0f);
  cfg.maxSpeed = prefs.getFloat("speed", 55.0f);
  cfg.maxPower = prefs.getFloat("power", 8.0f);
  cfg.maxBatteryCurrent = prefs.getFloat("bcur", 100.0f);
  cfg.maxMotorCurrent = prefs.getFloat("mcur", 200.0f);
  cfg.vescPollMs = prefs.getUShort("vpoll", 250);
  cfg.jbdPollMs = prefs.getUShort("jpoll", 700);
  cfg.smoothDisplay = prefs.getBool("smooth", true);
  cfg.autoReconnect = prefs.getBool("autorec", false);
  cfg.bleConnectTimeoutMs = prefs.getUShort("bleto", 2500);
  cfg.vesc1AddrType = prefs.getUChar("v1type", BLE_ADDR_PUBLIC);
  cfg.vesc2AddrType = prefs.getUChar("v2type", BLE_ADDR_PUBLIC);
  cfg.jbdAddrType = prefs.getUChar("jtype", BLE_ADDR_PUBLIC);
  String s;
  s = prefs.getString("v1", ""); strncpy(cfg.vesc1Addr, s.c_str(), sizeof(cfg.vesc1Addr)); cfg.vesc1Addr[17] = 0;
  s = prefs.getString("v2", ""); strncpy(cfg.vesc2Addr, s.c_str(), sizeof(cfg.vesc2Addr)); cfg.vesc2Addr[17] = 0;
  s = prefs.getString("jbd", ""); strncpy(cfg.jbdAddr, s.c_str(), sizeof(cfg.jbdAddr)); cfg.jbdAddr[17] = 0;
  prefs.end();
  loadLongTermCellStats();
}

// ---------- VESC PARSER ----------
void parseVesc(VescData &d, const uint8_t *p, size_t n) {
  if (n < 54 || p[0] != 4) return;

  d.fetTemp      = rdS16(p + 1) / 10.0f;
  d.motorTemp    = rdS16(p + 3) / 10.0f;
  d.motorCurrent = rdS32(p + 5) / 100.0f;
  d.inputCurrent = rdS32(p + 9) / 100.0f;
  d.id           = rdS32(p + 13) / 100.0f;
  d.iq           = rdS32(p + 17) / 100.0f;
  d.duty         = rdS16(p + 21) / 1000.0f;
  d.erpm         = rdS32(p + 23);
  d.vin          = rdS16(p + 27) / 10.0f;
  d.ah           = rdS32(p + 29) / 10000.0f;
  d.ahCharged    = rdS32(p + 33) / 10000.0f;
  d.wh           = rdS32(p + 37) / 10000.0f;
  d.whCharged    = rdS32(p + 41) / 10000.0f;
  d.tach         = rdS32(p + 45);
  d.tachAbs      = rdS32(p + 49);
  d.fault        = p[53];
  d.valid        = true;
  d.lastMs       = millis();
}

struct VescParser {
  uint8_t buf[160];
  size_t len = 0;

  void feed(uint8_t slot,VescData &d, const uint8_t *data, size_t n) {
    for (size_t i = 0; i < n; i++) {
      if (len < sizeof(buf)) buf[len++] = data[i];
      else len = 0;

      while (len >= 5) {
        if (buf[0] != 2) {
          memmove(buf, buf + 1, --len);
          continue;
        }

        size_t payloadLen = buf[1];
        size_t total = payloadLen + 5;
        if (total > sizeof(buf)) {
          len = 0;
          break;
        }
        if (len < total) break;
        if (buf[total - 1] != 3) {
          memmove(buf, buf + 1, --len);
          continue;
        }

        uint16_t got = ((uint16_t)buf[total - 3] << 8) | buf[total - 2];
        uint16_t calc = crc16(buf + 2, payloadLen);
        if (got == calc) { parseVesc(d, buf + 2, payloadLen); if(payloadLen>=5 && buf[2]==13) parseVescConfig(slot,buf+3,payloadLen-1); }

        size_t remain = len - total;
        if (remain) memmove(buf, buf + total, remain);
        len = remain;
      }
    }
  }
};

VescParser vescParser[2];

void vescNotify1(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  vescParser[0].feed(0,vesc1,data,len);
}
void vescNotify2(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  vescParser[1].feed(1,vesc2,data,len);
}

class CockpitClientCallbacks : public NimBLEClientCallbacks {
  int slotFor(NimBLEClient *c) {
    for (int i=0;i<2;i++) if (vescClient[i] == c) return i;
    if (jbdClient == c) return 2;
    return -1;
  }
  void onConnect(NimBLEClient *c) override {
    int s = slotFor(c);
    if (s < 0) return;
    bleLastReason[s] = 0;
    bleStatus[s] = "LINK OK";
    bleNeedsSetup[s] = true;
    bleSetupCancel = false;
    bleSetupRequest = s;
    forceDraw = true;
    Serial.printf("[BLE] %d connected\n", s);
    diagAdd(String("V") + String(s + 1) + " LINK OK");
  }
  void onConnectFail(NimBLEClient *c, int reason) override {
    int s = slotFor(c);
    if (s < 0) return;
    bleLastReason[s] = reason;
    bleNeedsSetup[s] = false;
    bleBusy = false; bleBusySlot = -1;
    bleStatus[s] = "CONNECT FAIL";
    lastBleErrorMs[s] = millis();
    forceDraw = true;
    Serial.printf("[BLE] %d connect FAIL reason=%d\n", s, reason);
    diagAdd(String("V") + String(s + 1) + " CONNECT FAIL r=" + String(reason));
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    int s = slotFor(c);
    if (s < 0) return;
    bleLastReason[s] = reason;
    bleNeedsSetup[s] = false;
    if (bleSetupRequest == s) bleSetupRequest = -1;
    bleSetupCancel = true;
    if (s < 2) {
      vescConnected[s] = false;
      vescRx[s] = nullptr; vescTx[s] = nullptr;
      vesc1.valid = (s==0) ? false : vesc1.valid;
      vesc2.valid = (s==1) ? false : vesc2.valid;
    } else {
      jbdConnected = false;
      jbdWrite = nullptr; jbdNotify = nullptr;
      jbd.valid = false;
    }
    bleBusy = false; bleBusySlot = -1;
    bleStatus[s] = "DISCONNECTED";
    if (cfg.autoReconnect && savedAddrForSlot(s)) {
      autoConnectRunning = false;
      autoScanActive = false;
      autoConnectNextMs = millis() + AUTO_CONNECT_START_DELAY_MS;
      diagAdd(String("AUTO V") + String(s + 1) + " RETRY 1.5s");
    }
    forceDraw = true;
    Serial.printf("[BLE] %d disconnect reason=%d\n", s, reason);
  }
};

CockpitClientCallbacks bleCallbacks[3];

bool connectVescSlot(uint8_t slot, const char *addressText, uint8_t addrType) {
  if (slot > 1 || addressText == nullptr || strlen(addressText) < 10) return false;
  if (bleBusy) return false;

  // A scan and a connection attempt must never overlap.
  // IMPORTANT: stop() does NOT call onScanEnd() in NimBLE 2.x, so update
  // our own scanning flag immediately. The old code returned here, leaving
  // the user stuck in SCANNING and never starting the connection.
  if (scanning) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
    scanning = false;
    forceDraw = true;
    delay(30);
  }

  bleBusy = true; bleBusySlot = slot; bleBusySince = millis();
  bleConnectStarted[slot] = millis();
  bleStatus[slot] = "CONNECTING"; forceDraw = true;
  bleNeedsSetup[slot] = false;
  vescConnected[slot] = false;
  vescRx[slot] = nullptr; vescTx[slot] = nullptr;

  if (vescClient[slot]) {
    if (vescClient[slot]->isConnected()) vescClient[slot]->disconnect();
    else vescClient[slot]->cancelConnect();
    NimBLEDevice::deleteClient(vescClient[slot]);
    vescClient[slot] = nullptr;
  }

  NimBLEAddress addr(std::string(addressText), addrType);
  vescClient[slot] = NimBLEDevice::createClient();
  if (!vescClient[slot]) {
    bleBusy=false; bleBusySlot=-1; bleStatus[slot]="NO CLIENT"; return false;
  }
  vescClient[slot]->setClientCallbacks(&bleCallbacks[slot], false);
  vescClient[slot]->setConnectTimeout(cfg.bleConnectTimeoutMs);
  vescClient[slot]->setConnectRetries(0);

  // TRUE = asynchronous: the touchscreen/main loop is never held in connect().
  bool ok = vescClient[slot]->connect(addr, true, true, false);
  if (!ok) {
    bleBusy = false; bleBusySlot = -1;
    bleStatus[slot] = "CONNECT FAIL";
    lastBleErrorMs[slot] = millis();
    diagAdd(String("V") + String(slot + 1) + " CONNECT FAIL (local)");
    return false;
  }
  return true;
}

void setupVescAfterConnect(uint8_t slot) {
  if (slot > 1 || !vescClient[slot] || !vescClient[slot]->isConnected()) return;
  if (!bleNeedsSetup[slot]) return;

  bleNeedsSetup[slot] = false;
  NimBLERemoteService *svc = vescClient[slot]->getService(NimBLEUUID(NUS_SERVICE));
  if (!svc) {
    vescClient[slot]->disconnect();
    bleStatus[slot] = "NO NUS";
    bleBusy=false; bleBusySlot=-1;
    lastBleErrorMs[slot]=millis();
    diagAdd(String("V") + String(slot + 1) + " NO NUS SERVICE");
    forceDraw=true; return;
  }
  if (bleSetupCancel) {
    vescClient[slot]->disconnect();
    bleNeedsSetup[slot] = false;
    return;
  }
  vescRx[slot] = svc->getCharacteristic(NimBLEUUID(NUS_RX));
  vescTx[slot] = svc->getCharacteristic(NimBLEUUID(NUS_TX));
  if (!vescRx[slot] || !vescTx[slot]) {
    vescClient[slot]->disconnect();
    bleStatus[slot] = "NO NUS CH";
    diagAdd(String("V") + String(slot + 1) + " NUS CH MISSING");
    bleBusy=false; bleBusySlot=-1;
    lastBleErrorMs[slot]=millis(); forceDraw=true; return;
  }
  if (bleSetupCancel) {
    vescClient[slot]->disconnect();
    bleNeedsSetup[slot] = false;
    return;
  }
  bool sub = (slot==0) ? vescTx[slot]->subscribe(true, vescNotify1)
                       : vescTx[slot]->subscribe(true, vescNotify2);
  if (!sub) {
    vescClient[slot]->disconnect();
    bleStatus[slot] = "NO NOTIFY";
    diagAdd(String("V") + String(slot + 1) + " NOTIFY FAIL");
    bleBusy=false; bleBusySlot=-1;
    lastBleErrorMs[slot]=millis(); forceDraw=true; return;
  }
  vescConnected[slot] = true;
  bleStatus[slot] = "CONNECTED";
  bleBusy=false; bleBusySlot=-1;
  forceDraw=true;
  Serial.printf("[BLE] VESC%d NUS ready\n", slot+1);
  diagAdd(String("V") + String(slot + 1) + " NUS READY");
}

void sendVescGetValues(uint8_t slot) {
  if (slot > 1 || !vescConnected[slot] || !vescRx[slot]) return;
  uint8_t payload[1] = {4};
  uint16_t c = crc16(payload, 1);
  uint8_t packet[6] = {0x02, 0x01, 0x04, (uint8_t)(c >> 8), (uint8_t)c, 0x03};
  vescRx[slot]->writeValue(packet, sizeof(packet), false);
}

// ---------- JBD ----------
void parseJbdFrame(const uint8_t *p, size_t n) {
  if (n < 7 || p[0] != 0xDD || p[n - 1] != 0x77) return;
  uint8_t cmd = p[1];
  uint8_t status = p[2];
  uint8_t len = p[3];
  if ((size_t)(len + 7) != n) return;

  // JBD RX checksum covers STATUS + LENGTH + DATA.
  // The previous version summed DATA only, so every valid BMS frame was
  // rejected and the UI stayed at WAITING FOR BMS DATA.
  uint16_t sum = (uint16_t)p[2] + (uint16_t)p[3];
  for (size_t i = 4; i < 4 + len; i++) sum += p[i];
  uint16_t expected = (uint16_t)(0x10000 - sum);
  uint16_t received = ((uint16_t)p[4 + len] << 8) | p[5 + len];
  if (expected != received) return;

  const uint8_t *d = p + 4;

  if (cmd == 0x03 && len >= 21) {
    jbd.voltage = rd16(d) / 100.0f;
    jbd.current = (int16_t)rd16(d + 2) / 100.0f;
    jbd.remainingAh = rd16(d + 4) / 100.0f;
    jbd.capacityAh = rd16(d + 6) / 100.0f;

    // Standard JBD basic-info layout:
    // voltage, current, remaining capacity, nominal capacity,
    // cycle count, production date, balance, protection,
    // version, SOC, FET status, cell count, NTC count.
    if (len >= 16) jbd.protection = rd16(d + 14);
    if (len >= 18) jbd.soc = d[17];
    if (len >= 20) jbd.cellCount = d[19];
    if (len >= 21) jbd.ntcCount = d[20];

    if (jbd.ntcCount > 0 && len >= 23)
      jbd.temp1 = rd16(d + 21) / 10.0f - 273.15f;
    if (jbd.ntcCount > 1 && len >= 25)
      jbd.temp2 = rd16(d + 23) / 10.0f - 273.15f;

    if (jbd.capacityAh > 0.01f && jbd.soc <= 100.0f)
      jbd.soc = constrain(jbd.soc, 0.0f, 100.0f);

    jbd.valid = true;
    jbd.lastMs = millis();
  }
  else if (cmd == 0x04) {
    uint8_t count = len / 2;
    if (count > 32) count = 32;
    jbd.cellCount = count;
    jbd.minCell = 100.0f;
    jbd.maxCell = 0.0f;

    for (uint8_t i = 0; i < count; i++) {
      jbd.cells[i] = rd16(d + i * 2) / 1000.0f;
      jbd.minCell = min(jbd.minCell, jbd.cells[i]);
      jbd.maxCell = max(jbd.maxCell, jbd.cells[i]);
    }

    if (count) jbd.delta = jbd.maxCell - jbd.minCell;
    updateLongTermCellStats();
    jbd.valid = true;
    jbd.lastMs = millis();
  }
}

uint8_t jbdRxBuf[128];
size_t jbdRxLen = 0;

void feedJbd(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (jbdRxLen >= sizeof(jbdRxBuf)) jbdRxLen = 0;
    jbdRxBuf[jbdRxLen++] = data[i];

    while (jbdRxLen >= 7) {
      if (jbdRxBuf[0] != 0xDD) {
        memmove(jbdRxBuf, jbdRxBuf + 1, --jbdRxLen);
        continue;
      }

      size_t frameLen = (size_t)jbdRxBuf[3] + 7;
      if (frameLen > sizeof(jbdRxBuf)) {
        jbdRxLen = 0;
        break;
      }
      if (jbdRxLen < frameLen) break;

      if (jbdRxBuf[frameLen - 1] != 0x77) {
        memmove(jbdRxBuf, jbdRxBuf + 1, --jbdRxLen);
        continue;
      }

      parseJbdFrame(jbdRxBuf, frameLen);
      size_t remain = jbdRxLen - frameLen;
      if (remain) memmove(jbdRxBuf, jbdRxBuf + frameLen, remain);
      jbdRxLen = remain;
    }
  }
}

void jbdNotifyCallback(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  feedJbd(data, len);
}

bool connectJbd(const char *addressText, uint8_t addrType) {
  if (!addressText || strlen(addressText) < 10) return false;
  if (bleBusy) return false;
  // stop() does not invoke onScanEnd() in NimBLE 2.x. Clear our flag here
  // so a CONNECT pressed while scanning can continue immediately.
  if (scanning) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
    scanning = false;
    forceDraw = true;
    delay(30);
  }

  bleBusy = true; bleBusySlot = 2; bleBusySince = millis();
  bleConnectStarted[2] = millis();
  bleStatus[2] = "CONNECTING"; forceDraw = true;
  bleNeedsSetup[2] = false;
  jbdConnected = false; jbdWrite = nullptr; jbdNotify = nullptr; jbdRxLen=0; jbd.valid=false;

  if (jbdClient) {
    if (jbdClient->isConnected()) jbdClient->disconnect();
    else jbdClient->cancelConnect();
    NimBLEDevice::deleteClient(jbdClient);
    jbdClient = nullptr;
  }

  NimBLEAddress addr(std::string(addressText), addrType);
  jbdClient = NimBLEDevice::createClient();
  if (!jbdClient) { bleBusy=false; bleBusySlot=-1; bleStatus[2]="NO CLIENT"; return false; }
  jbdClient->setClientCallbacks(&bleCallbacks[2], false);
  jbdClient->setConnectTimeout(cfg.bleConnectTimeoutMs);
  jbdClient->setConnectRetries(0);

  bool ok = jbdClient->connect(addr, true, true, false);
  if (!ok) {
    bleBusy=false; bleBusySlot=-1; bleStatus[2]="CONNECT FAIL"; lastBleErrorMs[2]=millis();
    diagAdd("JBD CONNECT FAIL (local)");
    return false;
  }
  return true;
}

void setupJbdAfterConnect() {
  if (!jbdClient || !jbdClient->isConnected() || !bleNeedsSetup[2]) return;
  bleNeedsSetup[2]=false;

  NimBLERemoteService *svc = jbdClient->getService(NimBLEUUID(JBD_SERVICE_2));
  if (svc) {
    jbdNotify = svc->getCharacteristic(NimBLEUUID(JBD_NOTIFY_2));
    jbdWrite  = svc->getCharacteristic(NimBLEUUID(JBD_WRITE_2));
  }
  if (!jbdWrite) {
    svc = jbdClient->getService(NimBLEUUID(JBD_SERVICE_1));
    if (svc) {
      jbdWrite  = svc->getCharacteristic(NimBLEUUID(JBD_WRITE_1));
      jbdNotify = svc->getCharacteristic(NimBLEUUID(JBD_NOTIFY_1));
    }
  }

  if (bleSetupCancel) {
    jbdClient->disconnect();
    return;
  }

  if (!jbdWrite) {
    jbdClient->disconnect(); bleStatus[2]="NO JBD CH"; bleBusy=false; bleBusySlot=-1; forceDraw=true; return;
  }

  // Some JBD BLE modules use one characteristic (FFE1) for BOTH write and notify.
  NimBLERemoteCharacteristic *notifyChr = jbdNotify;
  if (!notifyChr || (!notifyChr->canNotify() && !notifyChr->canIndicate())) notifyChr = jbdWrite;

  if (bleSetupCancel) {
    jbdClient->disconnect();
    return;
  }

  bool sub = true;
  if (notifyChr && notifyChr->canNotify()) sub = notifyChr->subscribe(true, jbdNotifyCallback);
  else if (notifyChr && notifyChr->canIndicate()) sub = notifyChr->subscribe(false, jbdNotifyCallback);
  else sub = false;

  if (!sub) {
    jbdClient->disconnect(); bleStatus[2]="NO NOTIFY"; bleBusy=false; bleBusySlot=-1; forceDraw=true; return;
  }

  jbdConnected=true;
  bleStatus[2]="CONNECTED";
  bleBusy=false; bleBusySlot=-1;
  jbdRxLen=0; jbd.valid=false;
  forceDraw=true;
  Serial.printf("[BLE] JBD ready: service=%s notify=%s write=%s\n",
                svc ? svc->getUUID().toString().c_str() : "?",
                notifyChr ? notifyChr->getUUID().toString().c_str() : "?",
                jbdWrite ? jbdWrite->getUUID().toString().c_str() : "?");
}

// ---------- SCANNING ----------
bool hasText(const String &s, const char *needle) {
  String a = s; a.toUpperCase();
  String b = needle; b.toUpperCase();
  return a.indexOf(b) >= 0;
}

void clearFound() {
  foundCount = 0;
  selectedFound = -1;
}

void addFound(NimBLEAdvertisedDevice *d) {
  if (foundCount >= 40) return;
  String addr = d->getAddress().toString().c_str();

  for (int i = 0; i < foundCount; i++)
    if (found[i].addr.equalsIgnoreCase(addr)) return;

  FoundDevice &f = found[foundCount++];
  f.name = d->getName().c_str();
  f.addr = addr;
  f.rssi = d->getRSSI();
  f.addrType = d->getAddressType();

  f.vesc = d->isAdvertisingService(NimBLEUUID(NUS_SERVICE));
  f.jbd = d->isAdvertisingService(NimBLEUUID(JBD_SERVICE_1)) ||
          d->isAdvertisingService(NimBLEUUID(JBD_SERVICE_2));

  if (hasText(f.name, "JBD") || hasText(f.name, "BMS") ||
      hasText(f.name, "XIAOXIANG") || hasText(f.name, "SP13") ||
      hasText(f.name, "SP14") || hasText(f.name, "SP17"))
    f.jbd = true;
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    NimBLEAdvertisedDevice *d = const_cast<NimBLEAdvertisedDevice *>(dev);
    addFound(d);
    Serial.printf("[BLE] FOUND: %s  %s  RSSI=%d\n",
                  d->getName().c_str(),
                  d->getAddress().toString().c_str(),
                  d->getRSSI());
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    scanning = false;
    forceDraw = true;
    Serial.printf("[BLE] SCAN END reason=%d devices=%d stored=%d\n",
                  reason, results.getCount(), foundCount);
  }
};

ScanCallbacks scanCallbacks;

void startScan() {
  // NimBLE-Arduino 2.x: start() uses milliseconds and is asynchronous.
  // The old V13 code used start(10,false), which is only 10 ms, then
  // immediately stopped the scan. That effectively searched for nothing.
  if (selectedFound >= 0 && selectedFound < foundCount && !scanning) {
    gfx->setTextColor(C_YELLOW);
    gfx->setCursor(8, 108);
    gfx->printf("SEL: %s  RSSI %d", found[selectedFound].name.length() ? found[selectedFound].name.c_str() : "UNKNOWN", found[selectedFound].rssi);
  }

  if (scanning) {
    Serial.println("[BLE] SCAN already running");
    return;
  }

  clearFound();
  foundScroll = 0;
  scanning = true;
  forceDraw = true;

  NimBLEScan *scan = NimBLEDevice::getScan();

  if (scan->isScanning()) {
    scan->stop();
    delay(20);
  }

  scan->clearResults();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  scan->setMaxResults(40);

  Serial.println("[BLE] START SCAN 5000 ms");
  bool ok = scan->start(5000, false, true);

  if (!ok) {
    scanning = false;
    forceDraw = true;
    Serial.println("[BLE] SCAN START FAILED");
  } else {
    Serial.println("[BLE] SCAN RUNNING");
  }
}

// ---------- GRAPHICS ----------
void centerText(const String &s, int y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(s.c_str(), 0, y, &x1, &y1, &w, &h);
  gfx->setCursor((320 - (int)w) / 2, y);
  gfx->print(s);
}

void header(const String &title) {
  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(8, 7);
  gfx->print(title);
  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(252, 10);
  gfx->printf("%02d/%d", page + 1, PAGE_COUNT);
  gfx->drawFastHLine(0, 28, 320, C_DGREY);
}

bool statusDotsLast[3] = {false,false,false};
bool statusDotsInit = false;
void statusDots() {
  bool cur[3] = {vescConnected[0], vescConnected[1], jbdConnected};
  if (statusDotsInit && cur[0]==statusDotsLast[0] && cur[1]==statusDotsLast[1] && cur[2]==statusDotsLast[2]) return;
  gfx->fillCircle(300, 22, 4, cur[0] ? C_GREEN : C_RED);
  gfx->fillCircle(310, 22, 4, cur[1] ? C_GREEN : C_RED);
  gfx->fillCircle(320, 22, 4, cur[2] ? C_GREEN : C_RED);
  statusDotsLast[0]=cur[0]; statusDotsLast[1]=cur[1]; statusDotsLast[2]=cur[2];
  statusDotsInit = true;
}

void drawGauge(int cx, int cy, int r, float value, float maxValue,
                const String &label, const String &unit, uint16_t color) {
  // Clean automotive-style semicircular gauge. Static scale is drawn once;
  // the live value is large and easy to read from a riding position.
  const int segs = 36;
  float frac = maxValue > 0 ? constrain(value / maxValue, 0.0f, 1.0f) : 0.0f;

  gfx->drawCircle(cx, cy, r, C_DGREY);

  for (int i = 0; i < segs; i++) {
    float a0 = (-140.0f + i * (280.0f / segs)) * (PI / 180.0f);
    float a1 = (-140.0f + (i + 1) * (280.0f / segs) - 2.0f) * (PI / 180.0f);
    uint16_t c = (i < (int)roundf(frac * segs)) ? color : C_DGREY;
    int x0 = cx + (int)(cosf(a0) * (r - 4));
    int y0 = cy + (int)(sinf(a0) * (r - 4));
    int x1 = cx + (int)(cosf(a1) * (r - 4));
    int y1 = cy + (int)(sinf(a1) * (r - 4));
    gfx->drawLine(x0, y0, x1, y1, c);
    gfx->drawLine(x0, y0 + 1, x1, y1 + 1, c);
  }

  String v;
  if (unit == "kW") v = String(value, 1);
  else v = String((int)roundf(value));

  gfx->setTextSize(4);
  gfx->setTextColor(C_WHITE);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(v.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(cx - (int)tw / 2, cy - 15);
  gfx->print(v);

  gfx->setTextSize(1);
  gfx->setTextColor(color);
  gfx->getTextBounds(unit.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(cx - (int)tw / 2, cy + 14);
  gfx->print(unit);

  gfx->setTextColor(C_GREY);
  gfx->getTextBounds(label.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(cx - (int)tw / 2, cy + r + 5);
  gfx->print(label);
}

void drawMain() {
  gfx->fillScreen(C_BLACK);
  header("VESC COCKPIT");
  statusDots();

  float spd = speedFromErpm(vesc1.valid ? vesc1.erpm : vesc2.erpm);
  float power = fabsf(combinedPowerW()) / 1000.0f;
  float soc = batterySoc();

  // Two large gauges.  The central speed readout is intentionally the
  // dominant element, because it is the primary riding value.
  drawGauge(65, 105, 52, spd, max(20.0f, cfg.maxSpeed), "SPEED", "km/h", C_CYAN);
  drawGauge(255, 105, 52, power, max(1.0f, cfg.maxPower), "POWER", "kW",
            power > cfg.maxPower * 0.85f ? C_RED : C_ORANGE);

  // Big central speed.
  String speedText = String((int)roundf(spd));
  gfx->setTextSize(6);
  gfx->setTextColor(C_WHITE);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(speedText.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(160 - (int)tw / 2, 68);
  gfx->print(speedText);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(143, 124);
  gfx->print("km/h");

  // Three large, evenly spaced bottom values.
  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(12, 164);  gfx->print("BATTERY");
  gfx->setCursor(135, 164); gfx->print("SOC");
  gfx->setCursor(255, 164); gfx->print("CURRENT");

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(10, 177); gfx->printf("%.1f V", batteryVoltage());

  gfx->setTextColor(C_GREEN);
  gfx->setCursor(135, 177); gfx->printf("%.0f%%", soc);

  float amps = batteryCurrent();
  gfx->setTextColor(amps < 0 ? C_GREEN : C_WHITE);
  gfx->setCursor(248, 177); gfx->printf("%.1f A", amps);

  // Small connection/status strip at the very bottom.
  gfx->setTextSize(1);
  gfx->setCursor(8, 210);
  gfx->setTextColor(vescConnected[0] ? C_GREEN : C_RED);
  gfx->print("V1 ");
  gfx->print(vescConnected[0] ? "OK" : "--");

  gfx->setCursor(70, 210);
  gfx->setTextColor(vescConnected[1] ? C_GREEN : C_RED);
  gfx->print("V2 ");
  gfx->print(vescConnected[1] ? "OK" : "--");

  gfx->setCursor(132, 210);
  gfx->setTextColor(jbdConnected ? C_GREEN : C_RED);
  gfx->print("BMS ");
  gfx->print(jbdConnected ? "OK" : "--");

  gfx->setTextColor(C_GREY);
  gfx->setCursor(250, 210);
  gfx->print("MENU");
}

void simplePage(const String &title, const String &a, const String &b,
                const String &c, const String &d) {
  gfx->fillScreen(C_BLACK);
  header(title);
  centerText(a, 55, 3, C_WHITE);
  centerText(b, 105, 2, C_CYAN);
  centerText(c, 150, 2, C_ORANGE);
  centerText(d, 195, 2, C_GREY);
  statusDots();
}

void drawCells() {
  gfx->fillScreen(C_BLACK);
  header("CELLS");
  if (!jbd.valid || !jbd.cellCount) {
    centerText("NO DATA", 100, 3, C_RED);
    return;
  }
  for (uint8_t i = 0; i < jbd.cellCount; i++) {
    int col = i % 4, row = i / 4;
    int x = 8 + col * 78, y = 42 + row * 26;
    gfx->setTextSize(1);
    gfx->setTextColor(jbd.cells[i] == jbd.minCell ? C_RED : C_WHITE);
    gfx->setCursor(x, y);
    gfx->printf("C%02u %.3f", i + 1, jbd.cells[i]);
  }
}

void bleButton(int x, int y, int w, int h, const String &txt, bool active=false) {
  gfx->drawRect(x, y, w, h, active ? C_YELLOW : C_DGREY);
  gfx->setTextSize(1);
  gfx->setTextColor(active ? C_YELLOW : C_WHITE);
  int16_t x1,y1; uint16_t tw,th;
  gfx->getTextBounds(txt.c_str(),0,y,&x1,&y1,&tw,&th);
  gfx->setCursor(x + (w-(int)tw)/2, y + (h-(int)th)/2);
  gfx->print(txt);
}

String slotAddress(uint8_t slot) {
  if (slot == 0) return String(cfg.vesc1Addr);
  if (slot == 1) return String(cfg.vesc2Addr);
  return String(cfg.jbdAddr);
}

String slotName(uint8_t slot) {
  if (slot == 0) return "VESC1";
  if (slot == 1) return "VESC2";
  return "JBD";
}

void disconnectSlot(uint8_t slot) {
  if (slot > 2) return;

  // Never delete a client while the BLE worker may be inside GATT discovery.
  // The worker/callback will finish cleanup safely after the discovery returns.
  requestBleDisconnect(slot);

  if (bleSetupRunning) {
    bleBusy = true;
    bleBusySlot = slot;
    forceDraw = true;
    return;
  }

  if (slot < 2) {
    if (vescClient[slot]) {
      NimBLEDevice::deleteClient(vescClient[slot]);
      vescClient[slot] = nullptr;
    }
    bleBusy = false;
    bleBusySlot = -1;
    bleStatus[slot] = "OFFLINE";
  } else {
    if (jbdClient) {
      NimBLEDevice::deleteClient(jbdClient);
      jbdClient = nullptr;
    }
    bleBusy = false;
    bleBusySlot = -1;
    bleStatus[2] = "OFFLINE";
  }
  forceDraw = true;
}

void clearSlotAddress(uint8_t slot) {
  if (slot == 0) cfg.vesc1Addr[0] = 0;
  else if (slot == 1) cfg.vesc2Addr[0] = 0;
  else cfg.jbdAddr[0] = 0;
  disconnectSlot(slot);
  saveConfig();
}

void drawBluetooth(bool fresh) {
  if (!fresh) return; // beze zmeny stavu/scanu neni co prekreslovat
  gfx->fillScreen(C_BLACK);
  header("BLUETOOTH");
  bleButton(0, 0, 55, 24, "BACK");
  statusDotsInit = false;
  statusDots();

  // Slot selector
  bleButton(5, 34, 98, 24, "VESC1", bleSlot == 0);
  bleButton(111,34,98,24,"VESC2", bleSlot == 1);
  bleButton(217,34,98,24,"JBD",   bleSlot == 2);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(8,65);
  gfx->print("Saved: ");
  gfx->setTextColor(C_WHITE);
  String sa = slotAddress(bleSlot);
  gfx->print(sa.length() ? sa : "none");
  gfx->setTextColor(C_GREY); gfx->setCursor(235,65); gfx->printf("%s", bleStatus[bleSlot].c_str());

  // Actions
  bleButton(5, 75, 92, 25, "SCAN");
  bleButton(101,75,92,25,"CONNECT");
  bleButton(197,75,92,25,"DISCONNECT");
  bleButton(293,75,22,25,"X");

  gfx->setTextColor(C_GREY);
  gfx->setCursor(8,107);
  gfx->print("Select device, then CONNECT to assign it.");

  const int visible = 6;
  int maxScroll = max(0, foundCount - visible);
  foundScroll = constrain(foundScroll, 0, maxScroll);

  for (int row = 0; row < visible; row++) {
    int i = foundScroll + row;
    if (i >= foundCount) break;
    int y = 116 + row * 18;
    // Compatibility is only an indication. Some VESC/JBD versions do not
    // advertise their service UUID, so every discovered BLE device remains selectable.
    bool compatible = true;
    bool selected = (i == selectedFound);
    gfx->drawRect(2, y, 316, 16, selected ? C_YELLOW : ((bleSlot < 2 ? found[i].vesc : found[i].jbd) ? C_DGREY : 0x2104));

    String nm = found[i].name.length() ? found[i].name : "UNKNOWN";
    if (nm.length() > 16) nm = nm.substring(0,16);
    gfx->setTextColor(compatible ? C_WHITE : C_DGREY);
    gfx->setTextSize(1);
    gfx->setCursor(6, y+4);
    gfx->print(nm);
    gfx->setCursor(148, y+4);
    gfx->printf("%d", found[i].rssi);
    gfx->setCursor(180, y+4);
    gfx->print(found[i].addr.substring(9));
    gfx->setCursor(282, y+4);
    gfx->setTextColor(found[i].vesc ? C_CYAN : found[i].jbd ? C_ORANGE : C_GREY);
    gfx->print(found[i].vesc ? "V" : found[i].jbd ? "J" : "?");
  }

  if (foundCount > visible) {
    bleButton(245, 226, 34, 14, "UP");
    bleButton(283, 226, 34, 14, "DN");
  }
  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(105, 230);
  gfx->printf("%d/%d", foundCount ? foundScroll + 1 : 0, foundCount);

  if (selectedFound >= 0 && selectedFound < foundCount && !scanning) {
    gfx->setTextColor(C_YELLOW);
    gfx->setCursor(8, 108);
    gfx->printf("SEL: %s  RSSI %d", found[selectedFound].name.length() ? found[selectedFound].name.c_str() : "UNKNOWN", found[selectedFound].rssi);
  }

  if (scanning) {
    centerText("SCANNING...", 135, 2, C_YELLOW);
    gfx->setTextSize(1);
    gfx->setTextColor(C_YELLOW);
    gfx->setCursor(8, 230);
    gfx->printf("BLE scan active - %d found", foundCount);
  }
  else if (!foundCount) centerText("NO DEVICES", 140, 2, C_GREY);
  else {
    gfx->setTextColor(C_GREY);
    gfx->setCursor(8,229);
    gfx->print("V=VESC J=JBD ?=other");
  }
}

String settingName(int idx) {
  switch (idx) {
    case 0: return "Motor pole pairs";
    case 1: return "Wheel diameter mm";
    case 2: return "Battery series";
    case 3: return "Battery MIN V";
    case 4: return "Battery MAX V";
    case 5: return "Battery capacity Ah";
    case 6: return "Max speed km/h";
    case 7: return "Max power kW";
    case 8: return "Max battery A";
    case 9: return "Max motor A";
    case 10: return "VESC poll ms";
    case 11: return "JBD poll ms";
    case 12: return "Smooth gauges";
    case 13: return "Auto reconnect";
    case 14: return "BLE timeout ms";
    case 15: return "Reset trip";
    case 16: return "Save settings";
    case 17: return "Factory reset";
  }
  return "";
}

float settingValue(int idx) {
  switch (idx) {
    case 0: return cfg.polePairs;
    case 1: return cfg.wheelDiameter * 1000.0f;
    case 2: return cfg.batterySeries;
    case 3: return cfg.battMin;
    case 4: return cfg.battMax;
    case 5: return cfg.battCapacity;
    case 6: return cfg.maxSpeed;
    case 7: return cfg.maxPower;
    case 8: return cfg.maxBatteryCurrent;
    case 9: return cfg.maxMotorCurrent;
    case 10: return cfg.vescPollMs;
    case 11: return cfg.jbdPollMs;
    case 12: return cfg.smoothDisplay ? 1 : 0;
    case 13: return cfg.autoReconnect ? 1 : 0;
    case 14: return cfg.bleConnectTimeoutMs;
  }
  return 0;
}

void resetTrip() {
  tripWh = 0.0f;
  tripKm = 0.0f;
  lastEnergyTick = millis();
  maxSpeedSession = 0.0f;
  maxPowerSession = 0.0f;
  maxBatteryCurrentSession = 0.0f;
  maxMotorTempSession = 0.0f;
  maxCellDeltaSession = 0.0f;
}

void changeSetting(int idx, int dir) {
  switch (idx) {
    case 0: cfg.polePairs = constrain((int)cfg.polePairs + dir, 1, 20); markConfigDirty(); break;
    case 1: cfg.wheelDiameter = constrain(cfg.wheelDiameter * 1000.0f + dir * 1.0f, 100.0f, 500.0f) / 1000.0f; markConfigDirty(); break;
    case 2: cfg.batterySeries = constrain((int)cfg.batterySeries + dir, 1, 24); markConfigDirty(); break;
    case 3: cfg.battMin = constrain(cfg.battMin + dir * 0.1f, 1.0f, 100.0f); markConfigDirty(); break;
    case 4: cfg.battMax = constrain(cfg.battMax + dir * 0.1f, 1.0f, 120.0f); markConfigDirty(); break;
    case 5: cfg.battCapacity = constrain(cfg.battCapacity + dir * 0.5f, 0.5f, 500.0f); markConfigDirty(); break;
    case 6: cfg.maxSpeed = constrain(cfg.maxSpeed + dir * 1.0f, 1.0f, 200.0f); markConfigDirty(); break;
    case 7: cfg.maxPower = constrain(cfg.maxPower + dir * 0.1f, 0.1f, 50.0f); markConfigDirty(); break;
    case 8: cfg.maxBatteryCurrent = constrain(cfg.maxBatteryCurrent + dir * 1.0f, 1.0f, 500.0f); markConfigDirty(); break;
    case 9: cfg.maxMotorCurrent = constrain(cfg.maxMotorCurrent + dir * 5.0f, 5.0f, 1000.0f); markConfigDirty(); break;
    case 10: cfg.vescPollMs = constrain((int)cfg.vescPollMs + dir * 10, 50, 2000); markConfigDirty(); break;
    case 11: cfg.jbdPollMs = constrain((int)cfg.jbdPollMs + dir * 50, 100, 5000); markConfigDirty(); break;
    case 12: if (dir != 0) { cfg.smoothDisplay = !cfg.smoothDisplay; markConfigDirty(); } break;
    case 13:
      if (dir != 0) {
        cfg.autoReconnect = !cfg.autoReconnect;
        // Save this one immediately rather than waiting for the debounce -
        // it's an on/off toggle the user expects to "stick" right away, and
        // it directly gates whether autoConnectTick() does anything at all.
        saveConfig();
        diagAdd(String("AUTO RECONNECT ") + (cfg.autoReconnect ? "ON (saved)" : "OFF (saved)"));
      }
      break;
    case 14: cfg.bleConnectTimeoutMs = constrain((int)cfg.bleConnectTimeoutMs + dir * 500, 2000, 15000); markConfigDirty(); break;
    case 15: resetTrip(); break;
    case 16: saveConfig(); break;
    case 17:
      cfg = Config();
      resetTrip();
      saveConfig();
      break;
  }
}

void drawSettings(bool fresh) {
  if (!fresh) return; // meni se jen po dotyku (uprava hodnoty, vyber radku)
  gfx->fillScreen(C_BLACK);
  header("SETTINGS");
  bleButton(0, 0, 55, 24, "BACK");

  const int SET_COUNT = 18;
  int start = constrain(settingsItem - 3, 0, SET_COUNT - 7);

  for (int i = start; i < start + 7 && i < SET_COUNT; i++) {
    int y = 36 + (i - start) * 25;
    bool sel = i == settingsItem;
    gfx->fillRect(3,y-2,314,22,sel ? C_DGREY : C_BLACK);
    gfx->setTextSize(1);
    gfx->setTextColor(sel ? C_YELLOW : C_WHITE);
    gfx->setCursor(8,y+5);
    gfx->print(settingName(i));

    if (i <= 14) {
      gfx->setTextColor(C_WHITE);
      gfx->setCursor(205,y+5);
      if (i == 1 || i == 14) gfx->printf("%.0f", settingValue(i));
      else if (i == 3 || i == 4 || i == 5 || i == 6 || i == 7) gfx->printf("%.1f", settingValue(i));
      else if (i == 12) gfx->print(cfg.smoothDisplay ? "ON" : "OFF");
      else if (i == 13) gfx->print(cfg.autoReconnect ? "ON" : "OFF");
      else gfx->printf("%.0f", settingValue(i));

      if (i != 15) {
        gfx->drawRect(250,y,27,18,C_DGREY);
        gfx->drawRect(282,y,27,18,C_DGREY);
        gfx->setCursor(259,y+5); gfx->print("-");
        gfx->setCursor(291,y+5); gfx->print("+");
      }
    } else if (i == 15) {
      gfx->setTextColor(C_ORANGE); gfx->setCursor(238,y+5); gfx->print("RESET");
    } else if (i == 16) {
      gfx->setTextColor(C_GREEN); gfx->setCursor(240,y+5); gfx->print("SAVE");
    } else if (i == 17) {
      gfx->setTextColor(C_RED); gfx->setCursor(220,y+5); gfx->print("RESET ALL");
    }
  }

  gfx->setTextColor(C_GREY);
  gfx->setCursor(8,218); gfx->print("LEFT/RIGHT = value   ROW = select");
  gfx->setCursor(8,230); gfx->printf("%d/%d  NVS persistent", settingsItem+1, SET_COUNT);
}

void drawMenu(bool fresh) {
  if (!fresh) return; // uplne staticka obrazovka, neni co prekreslovat
  gfx->fillScreen(C_BLACK);
  header("MAIN MENU");
  bleButton(0, 0, 55, 24, "DASH");
  const char *items[] = {"DASHBOARD", "BLE SEARCH / PAIR", "SETTINGS", "BLE STATUS"};
  for (int i = 0; i < 4; i++) {
    int y = 55 + i * 38;
    gfx->drawRect(20, y, 280, 30, C_DGREY);
    gfx->setTextSize(2);
    gfx->setTextColor(C_WHITE);
    gfx->setCursor(35, y + 7);
    gfx->print(items[i]);
  }
}

float batterySoc() {
  if (jbd.valid) return constrain(jbd.soc, 0.0f, 100.0f);
  if (cfg.battMax <= cfg.battMin) return 0;
  return constrain((batteryVoltage() - cfg.battMin) * 100.0f /
                   (cfg.battMax - cfg.battMin), 0.0f, 100.0f);
}

float estimatedRangeKm() {
  float soc = batterySoc();
  float availableWh = cfg.battCapacity * batteryVoltage() * soc / 100.0f;
  if (tripKm > 0.5f && tripWh > 10.0f) {
    float whkm = tripWh / tripKm;
    return availableWh / whkm;
  }
  return 0.0f;
}

void updateTripEnergy() {
  uint32_t now = millis();
  if (!lastEnergyTick) {
    lastEnergyTick = now;
    return;
  }
  float dt = (now - lastEnergyTick) / 3600000.0f;
  if (dt <= 0 || dt > 0.1f) {
    lastEnergyTick = now;
    return;
  }

  float p = fabsf(combinedPowerW());
  float kmh = speedFromErpm(vesc1.valid ? vesc1.erpm : vesc2.erpm);

  tripWh += p * dt;
  tripKm += kmh * dt;
  maxSpeedSession = max(maxSpeedSession, fabsf(kmh));
  maxPowerSession = max(maxPowerSession, fabsf(p));
  maxBatteryCurrentSession = max(maxBatteryCurrentSession, fabsf(batteryCurrent()));
  maxMotorTempSession = max(maxMotorTempSession, max(vesc1.motorTemp, vesc2.motorTemp));
  maxCellDeltaSession = max(maxCellDeltaSession, jbd.delta);
  lastEnergyTick = now;
}


// ---------- AUDI VIRTUAL COCKPIT ----------
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769f
#endif

void ringGauge(int cx, int cy, int r, float value, float vmin, float vmax,
               const char *unit, bool redline=false) {
  gfx->drawCircle(cx, cy, r, C_DGREY);
  gfx->drawCircle(cx, cy, r-1, C_DGREY);
  gfx->drawCircle(cx, cy, r-4, 0x2104);

  float f = constrain((value-vmin)/(vmax-vmin), 0.0f, 1.0f);
  int segs = 32;
  int lit = (int)roundf(f * segs);

  for (int i=0;i<segs;i++) {
    float a0 = -140.0f + i * (280.0f/segs);
    float a1 = -140.0f + (i+1) * (280.0f/segs) - 2.0f;
    uint16_t col = (i < lit) ? (redline && i > 27 ? C_RED : C_CYAN) : 0x2104;
    for (int rr=r-7; rr<=r-5; rr++) {
      int x0 = cx + (int)(cosf(a0*DEG_TO_RAD)*rr);
      int y0 = cy + (int)(sinf(a0*DEG_TO_RAD)*rr);
      int x1 = cx + (int)(cosf(a1*DEG_TO_RAD)*rr);
      int y1 = cy + (int)(sinf(a1*DEG_TO_RAD)*rr);
      gfx->drawLine(x0,y0,x1,y1,col);
    }
  }

  float ang = (-140.0f + f*280.0f) * DEG_TO_RAD;
  gfx->drawLine(cx,cy,
                cx + (int)(cosf(ang)*(r-15)),
                cy + (int)(sinf(ang)*(r-15)), C_WHITE);
  gfx->fillCircle(cx,cy,4,C_WHITE);

  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(3);
  String vs = String((int)roundf(value));
  int16_t x1,y1; uint16_t tw,th;
  gfx->getTextBounds(vs.c_str(),0,0,&x1,&y1,&tw,&th);
  gfx->setCursor(cx-(int)tw/2,cy-16);
  gfx->print(vs);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->getTextBounds(unit,0,0,&x1,&y1,&tw,&th);
  gfx->setCursor(cx-(int)tw/2,cy+14);
  gfx->print(unit);
}

void miniValue(int x,int y,const char *label,const String &value,uint16_t color=C_WHITE) {
  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(x,y);
  gfx->print(label);
  gfx->setTextColor(color);
  gfx->setCursor(x,y+11);
  gfx->setTextSize(2);
  gfx->print(value);
}

bool audiDashboardInit = false;
float audiOldSpeed = -1000.0f;
float audiOldPower = -1000.0f;
int audiOldSpeedInt = -99999;
String audiOldBottomStrip = "";
String audiOldStatusLine = "";

void drawAudiNeedle(int cx, int cy, int r, float value, float vmin, float vmax, uint16_t col) {
  float f = constrain((value-vmin)/(vmax-vmin), 0.0f, 1.0f);
  float ang = (-140.0f + f*280.0f) * DEG_TO_RAD;
  gfx->drawLine(cx,cy,
                cx + (int)(cosf(ang)*(r-15)),
                cy + (int)(sinf(ang)*(r-15)), col);
}

void drawAudiDashboard() {
  float rpm = vesc1.valid ? vesc1.erpm : vesc2.erpm;
  float kmh = speedFromErpm(rpm);
  float pkw = fabsf(combinedPowerW()) / 1000.0f;
  float volts = batteryVoltage();
  float amps = jbd.valid ? jbd.current : (vesc1.valid ? vesc1.inputCurrent : 0);
  float soc = batterySoc();

  if (cfg.smoothDisplay) {
    if (displaySpeed == 0.0f && displayPower == 0.0f) {
      displaySpeed = kmh;
      displayPower = pkw;
    }
    displaySpeed += (kmh - displaySpeed) * 0.18f;
    displayPower += (pkw - displayPower) * 0.18f;
    kmh = displaySpeed;
    pkw = displayPower;
  } else {
    displaySpeed = kmh;
    displayPower = pkw;
  }

  // Main riding dashboard: exactly three gauges.
  // SPEED / POWER / BATTERY VOLTAGE.  No duplicate central speed value.
  if (!audiDashboardInit) {
    gfx->fillScreen(C_BLACK);
    header("VESC COCKPIT");
    statusDotsInit = false;
    statusDots();

    // Three equal gauges across the screen.
    drawGauge(59, 88, 59, 0, max(20.0f, cfg.maxSpeed),
              "SPEED", "km/h", C_CYAN);
    drawGauge(160, 88, 59, 0, max(1.0f, cfg.maxPower),
              "POWER", "kW", C_ORANGE);
    drawGauge(261, 88, 59, 0, max(60.0f, cfg.battMax),
              "BATTERY", "V", C_GREEN);

    // Bottom telemetry / navigation area.
    gfx->setTextSize(1);
    gfx->setTextColor(C_GREY);
    gfx->setCursor(10, 167);  gfx->print("CURRENT");
    gfx->setCursor(112, 167); gfx->print("SOC");
    gfx->setCursor(218, 167); gfx->print("ERPM");

    gfx->setCursor(8, 223);  gfx->print("V1");
    gfx->setCursor(75, 223); gfx->print("V2");
    gfx->setCursor(142, 223); gfx->print("BMS");
    gfx->setCursor(218, 223); gfx->print("MENU");
    gfx->setCursor(8, 234); gfx->print("<");
    gfx->setCursor(158, 234); gfx->print("PAGE");
    gfx->setCursor(307, 234); gfx->print(">");

    audiDashboardInit = true;
    audiOldSpeed = -1000;
    audiOldPower = -1000;
    audiOldSpeedInt = -99999;
    audiOldBottomStrip = "";
    audiOldStatusLine = "";
  }

  // Repaint the gauge area only when values actually changed.
  bool speedMoved = fabsf(kmh - audiOldSpeed) > 0.05f || audiOldSpeed < -999;
  bool powerMoved = fabsf(pkw - audiOldPower) > 0.05f || audiOldPower < -999;
  static float oldVolts = -1000.0f;
  bool voltsMoved = fabsf(volts - oldVolts) > 0.05f || oldVolts < -999;

  if (speedMoved || powerMoved || voltsMoved) {
    gfx->fillRect(0, 29, 320, 127, C_BLACK);

    drawGauge(59, 88, 59, kmh, max(20.0f, cfg.maxSpeed),
              "SPEED", "km/h", C_CYAN);
    drawGauge(160, 88, 59, pkw, max(1.0f, cfg.maxPower),
              "POWER", "kW",
              pkw > cfg.maxPower * 0.85f ? C_RED : C_ORANGE);

    // Voltage scale is based on the configured battery maximum.
    // 0 V is intentionally not shown as the normal operating scale.
    // drawGauge() uses 0..maxValue; use the configured battery maximum.
    drawGauge(261, 88, 59, volts, max(60.0f, cfg.battMax),
              "BATTERY", "V", C_GREEN);

    oldVolts = volts;
  }

  // The gauge itself now contains the only speed number.
  // Large lower values remain easy to read at a glance.
  String bottom = String(volts,1) + "|" + String(amps,1) + "|" +
                  String((int)soc) + "|" + String((int)fabsf(rpm));
  if (bottom != audiOldBottomStrip) {
    gfx->fillRect(0, 162, 320, 53, C_BLACK);

    gfx->setTextSize(1);
    gfx->setTextColor(C_GREY);
    gfx->setCursor(10, 167);  gfx->print("CURRENT");
    gfx->setCursor(112, 167); gfx->print("SOC");
    gfx->setCursor(218, 167); gfx->print("ERPM");

    gfx->setTextSize(3);
    gfx->setTextColor(amps < 0 ? C_GREEN : C_WHITE);
    gfx->setCursor(7, 178); gfx->printf("%.1f", amps);
    gfx->setTextColor(C_GREEN);
    gfx->setCursor(108, 178); gfx->printf("%.0f", soc);
    gfx->setTextColor(C_WHITE);
    gfx->setCursor(211, 178); gfx->printf("%d", (int)fabsf(rpm));

    gfx->setTextSize(1);
    gfx->setTextColor(C_GREY);
    gfx->setCursor(62, 201); gfx->print("A");
    gfx->setCursor(147, 201); gfx->print("%");
    gfx->setCursor(275, 201); gfx->print("ERPM");

    audiOldBottomStrip = bottom;
  }

  // Connection status only redraws when state changes.
  String statusLine = String(vescConnected[0]) + String(vescConnected[1]) +
                      String(jbdConnected) + String(vesc1.valid) +
                      String(vesc2.valid) + String(jbd.valid);

  if (statusLine != audiOldStatusLine) {
    gfx->fillRect(0, 216, 320, 15, C_BLACK);

    gfx->setTextSize(1);
    gfx->setCursor(8, 223);
    gfx->setTextColor(vescConnected[0] ? C_GREEN : C_RED);
    gfx->print("V1 "); gfx->print(vescConnected[0] ? "ONLINE" : "OFF");

    gfx->setCursor(75, 223);
    gfx->setTextColor(vescConnected[1] ? C_GREEN : C_RED);
    gfx->print("V2 "); gfx->print(vescConnected[1] ? "ONLINE" : "OFF");

    gfx->setCursor(142, 223);
    gfx->setTextColor(jbdConnected ? C_GREEN : C_RED);
    gfx->print("BMS "); gfx->print(jbdConnected ? "ONLINE" : "OFF");

    gfx->setTextColor(C_GREY);
    gfx->setCursor(218, 223);
    gfx->print("MENU");

    audiOldStatusLine = statusLine;
  }

  audiOldSpeed = kmh;
  audiOldPower = pkw;
}

void drawDualDashboard() {
  gfx->fillScreen(C_BLACK);
  header("DUAL VESC");
  statusDots();

  float s1=speedFromErpm(vesc1.erpm), s2=speedFromErpm(vesc2.erpm);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN); gfx->setCursor(10,45); gfx->print("VESC 1");
  gfx->setTextColor(C_ORANGE); gfx->setCursor(170,45); gfx->print("VESC 2");

  miniValue(10,78,"RPM",String((int)vesc1.erpm),C_WHITE);
  miniValue(10,115,"A",String(vesc1.inputCurrent,1),C_WHITE);
  miniValue(10,152,"TEMP",String(vesc1.motorTemp,1)+" C",C_WHITE);
  miniValue(170,78,"RPM",String((int)vesc2.erpm),C_WHITE);
  miniValue(170,115,"A",String(vesc2.inputCurrent,1),C_WHITE);
  miniValue(170,152,"TEMP",String(vesc2.motorTemp,1)+" C",C_WHITE);

  gfx->setTextSize(2); gfx->setTextColor(C_WHITE);
  gfx->setCursor(10,195); gfx->printf("%d km/h",(int)roundf(s1));
  gfx->setCursor(170,195); gfx->printf("%d km/h",(int)roundf(s2));

  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,222); gfx->printf("P1 %.2f kW",vesc1.inputCurrent*vesc1.vin/1000.0f);
  gfx->setCursor(170,222); gfx->printf("P2 %.2f kW",vesc2.inputCurrent*vesc2.vin/1000.0f);
}


void sendJbdCommand(uint8_t cmd) {
  if (!jbdWrite || !jbdConnected) return;
  // JBD read frame: DD A5 REG 00 CHECK_H CHECK_L 77
  uint16_t sum = (uint16_t)cmd;
  uint16_t crc = (uint16_t)(0x10000 - sum);
  uint8_t packet[7] = {0xDD, 0xA5, cmd, 0x00,
                       (uint8_t)(crc >> 8), (uint8_t)crc, 0x77};
  bool ok = jbdWrite->writeValue(packet, sizeof(packet), false);
  Serial.printf("[JBD] CMD %02X write=%s\n", cmd, ok ? "OK" : "FAIL");
}

// ---------- CONNECTION / DATA PANELS ----------
void drawConnectionStatus() {
  gfx->fillScreen(C_BLACK);
  header("CONNECTION STATUS");
  statusDots();

  miniValue(8,42,"VESC1",bleStatus[0],vescConnected[0]?C_GREEN:C_RED);
  miniValue(112,42,"VESC2",bleStatus[1],vescConnected[1]?C_GREEN:C_RED);
  miniValue(216,42,"JBD",bleStatus[2],jbdConnected?C_GREEN:C_RED);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(8,92); gfx->print("VESC1: "); gfx->setTextColor(C_WHITE); gfx->print(cfg.vesc1Addr[0]?cfg.vesc1Addr:"not assigned");
  gfx->setCursor(8,108); gfx->setTextColor(C_GREY); gfx->print("VESC2: "); gfx->setTextColor(C_WHITE); gfx->print(cfg.vesc2Addr[0]?cfg.vesc2Addr:"not assigned");
  gfx->setCursor(8,124); gfx->setTextColor(C_GREY); gfx->print("JBD:   "); gfx->setTextColor(C_WHITE); gfx->print(cfg.jbdAddr[0]?cfg.jbdAddr:"not assigned");

  gfx->setCursor(8,150); gfx->setTextColor(C_GREY);
  gfx->printf("V1 data: %s", vesc1.valid ? "OK" : "NO DATA");
  gfx->setCursor(108,150); gfx->printf("V2 data: %s", vesc2.valid ? "OK" : "NO DATA");
  gfx->setCursor(208,150); gfx->printf("JBD: %s", jbd.valid ? "OK" : "NO DATA");

  gfx->setCursor(8,177); gfx->setTextColor(C_GREY);
  if (bleBusy) gfx->printf("BLE BUSY: %s", bleBusySlot==2?"JBD":bleBusySlot==1?"VESC2":"VESC1");
  else gfx->print("BLE idle - no blocking reconnects");

  gfx->setCursor(8,197); gfx->printf("Heap %lu KB   Loop %.0f Hz", ESP.getFreeHeap()/1024UL, loopHz);
  gfx->setCursor(8,215); gfx->printf("Next: VESC1 -> VESC2 -> DASH -> JBD -> CONNECT");
  gfx->setCursor(8,232); gfx->print("Use BLE panel for connect / disconnect");
}

void drawVescPanel(uint8_t slot) {
  VescData &d = slot ? vesc2 : vesc1;
  gfx->fillScreen(C_BLACK);
  header(slot ? "VESC 2 DATA" : "VESC 1 DATA");
  statusDots();
  uint16_t col = slot ? C_ORANGE : C_CYAN;
  miniValue(8,42,"STATE",bleStatus[slot],vescConnected[slot]?C_GREEN:C_RED);
  miniValue(105,42,"ERPM",String((int)d.erpm),col);
  miniValue(210,42,"SPEED",String(speedFromErpm(d.erpm),1)+"",C_WHITE);
  miniValue(8,90,"BAT A",String(d.inputCurrent,1),C_WHITE);
  miniValue(105,90,"MOTOR A",String(d.motorCurrent,1),C_WHITE);
  miniValue(210,90,"DUTY",String(d.duty*100.0f,1)+"%",C_WHITE);
  miniValue(8,138,"VIN",String(d.vin,1)+" V",C_CYAN);
  miniValue(105,138,"FET",String(d.fetTemp,1)+" C",d.fetTemp>80?C_RED:C_WHITE);
  miniValue(210,138,"MOTOR",String(d.motorTemp,1)+" C",d.motorTemp>100?C_RED:C_WHITE);
  miniValue(8,186,"Id",String(d.id,1)+" A",C_GREY);
  miniValue(105,186,"Iq",String(d.iq,1)+" A",C_GREY);
  miniValue(210,186,"FAULT",String(d.fault),d.fault?C_RED:C_GREEN);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,224); gfx->printf("Ah %.2f   Wh %.2f   last %lums",d.ah,d.wh,(unsigned long)(d.valid?millis()-d.lastMs:0));
}

void drawJbdPanel() {
  gfx->fillScreen(C_BLACK);
  header("JBD / BMS DATA"); statusDots();
  miniValue(8,42,"STATE",bleStatus[2],jbdConnected?C_GREEN:C_RED);
  miniValue(105,42,"VOLT",String(jbd.voltage,2)+" V",C_CYAN);
  miniValue(210,42,"SOC",String(jbd.soc,0)+" %",C_GREEN);
  miniValue(8,90,"CURRENT",String(jbd.current,2)+" A",jbd.current<0?C_GREEN:C_WHITE);
  miniValue(105,90,"REMAIN",String(jbd.remainingAh,2)+" Ah",C_WHITE);
  miniValue(210,90,"CAP",String(jbd.capacityAh,2)+" Ah",C_WHITE);
  miniValue(8,138,"CELLS",String(jbd.cellCount),C_WHITE);
  miniValue(105,138,"DELTA",String(jbd.delta*1000.0f,0)+" mV",jbd.delta>0.05?C_RED:C_GREEN);
  miniValue(210,138,"PROT",String(jbd.protection),jbd.protection?C_RED:C_GREEN);
  miniValue(8,186,"T1",isnan(jbd.temp1)?"--":String(jbd.temp1,1)+" C",C_WHITE);
  miniValue(105,186,"T2",isnan(jbd.temp2)?"--":String(jbd.temp2,1)+" C",C_WHITE);
  miniValue(210,186,"MIN/MAX",String(jbd.minCell,3)+"/"+String(jbd.maxCell,3),C_WHITE);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,224); gfx->printf("last data: %lums   cells frame: %s",(unsigned long)(jbd.valid?millis()-jbd.lastMs:0),jbd.cellCount?"OK":"NO");
}

void drawConnectionSettingsPanel() {
  gridBegin("CONNECTION SETTINGS");
  gridCell(0,8,42,"VESC POLL",String(cfg.vescPollMs)+" ms",C_CYAN);
  gridCell(1,110,42,"JBD POLL",String(cfg.jbdPollMs)+" ms",C_ORANGE);
  gridCell(2,212,42,"TIMEOUT",String(cfg.bleConnectTimeoutMs)+" ms",C_YELLOW);
  gridCell(3,8,90,"AUTO RECONNECT",cfg.autoReconnect?"ON":"OFF",cfg.autoReconnect?C_YELLOW:C_GREEN);
  gridCell(4,110,90,"VESC1",vescConnected[0]?"ONLINE":"OFFLINE",vescConnected[0]?C_GREEN:C_RED);
  gridCell(5,212,90,"VESC2",vescConnected[1]?"ONLINE":"OFFLINE",vescConnected[1]?C_GREEN:C_RED);
  gridCell(6,8,138,"JBD",jbdConnected?"ONLINE":"OFFLINE",jbdConnected?C_GREEN:C_RED);
  gridCell(7,110,138,"SCAN",scanning?"ACTIVE":"IDLE",scanning?C_YELLOW:C_GREY);
  gridCell(8,212,138,"FOUND",String(foundCount),C_WHITE);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,188); gfx->print("CONNECT/DISCONNECT only from BLE manager");
  gfx->setCursor(8,204); gfx->print("Automatic reconnect is OFF by default to prevent freezes.");
  gfx->setCursor(8,220); gfx->print("VESC/JBD polling is non-blocking in loop().");
}

// ---------- COMPACT INFORMATION PAGES ----------
void pageFrame(const char *title) {
  gfx->fillScreen(C_BLACK); header(title); statusDots();
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,232); gfx->print("<");
  gfx->setCursor(150,232); gfx->printf("%d/%d", page+1, PAGE_COUNT);
  gfx->setCursor(309,232); gfx->print(">");
}

// ------------------------------------------------------------
// FLICKER-FREE MINI-GRID STRANKY
// ------------------------------------------------------------
// Puvodne kazda "summary" stranka volala pageFrame(), ktera dela
// gfx->fillScreen() PRI KAZDEM prekresleni (10x/s) - proto to
// blika. Staticke prvky (pozadi, nadpis, popisky bunek) se ted
// kresli jen JEDNOU pri vstupu na stranku; kazda hodnota se
// prekresli jen kdyz se opravdu zmenila.
// ------------------------------------------------------------

int gridInitForPage = -1; // ktera "page" ma prave nakreslene staticke prvky
String gridLastValue[16];
bool gridLastValueSet[16];

bool gridBegin(const char *title) {
  bool firstTime = (gridInitForPage != page);
  if (firstTime) {
    gfx->fillScreen(C_BLACK);
    header(title);
    gfx->setTextSize(1); gfx->setTextColor(C_GREY);
    gfx->setCursor(8,232); gfx->print("<");
    gfx->setCursor(150,232); gfx->printf("%d/%d", page+1, PAGE_COUNT);
    gfx->setCursor(309,232); gfx->print(">");

    gridInitForPage = page;
    for (int i = 0; i < 16; i++) gridLastValueSet[i] = false;
    statusDotsInit = false; // fillScreen just wiped the dots, force a redraw
  }
  statusDots(); // dirty-checked now: cheap AND correct every cycle
  return firstTime;
}

// slot = index bunky 0..15, musi byt v ramci jedne stranky unikatni
void gridCell(int slot, int x, int y, const char *label, const String &value, uint16_t color) {
  bool firstTime = !gridLastValueSet[slot];
  if (firstTime) {
    gfx->setTextSize(1);
    gfx->setTextColor(C_GREY);
    gfx->setCursor(x, y);
    gfx->print(label);
  }

  if (firstTime || gridLastValue[slot] != value) {
    gfx->fillRect(x, y + 11, 100, 18, C_BLACK); // mazeme jen hodnotu, ne popisek
    gfx->setTextColor(color);
    gfx->setCursor(x, y + 11);
    gfx->setTextSize(2);
    gfx->print(value);

    gridLastValue[slot] = value;
    gridLastValueSet[slot] = true;
  }
}

// jednoduchy plny radek textu (misto label/hodnota dvojice) se
// stejnym dirty-checkem, sdili cache pole s gridCell - pouzij
// jine "slot" cislo nez gridCell() na stejne strance.
void textLine(int slot, int x, int y, int w, const String &text, uint16_t color) {
  if (gridLastValueSet[slot] && gridLastValue[slot] == text) return;
  gfx->fillRect(x, y, w, 14, C_BLACK);
  gfx->setTextSize(1);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
  gridLastValue[slot] = text;
  gridLastValueSet[slot] = true;
}

void bigNumber(int cx, int cy, const String &v, const char *unit, uint16_t col=C_WHITE) {
  gfx->setTextSize(4); gfx->setTextColor(col);
  int16_t x1,y1; uint16_t w,h; gfx->getTextBounds(v.c_str(),0,0,&x1,&y1,&w,&h);
  gfx->setCursor(cx-w/2,cy); gfx->print(v);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->getTextBounds(unit,0,0,&x1,&y1,&w,&h); gfx->setCursor(cx-w/2,cy+38); gfx->print(unit);
}

void drawMainDashboard() {
  float speed = speedFromErpm(vesc1.valid ? vesc1.erpm : vesc2.erpm);
  float power = fabsf(combinedPowerW())/1000.0f;
  float soc = batterySoc();
  float volts = batteryVoltage();
  float amps = batteryCurrent();
  pageFrame("MAIN DASHBOARD");
  ringGauge(78,112,62,speed,0,max(20.0f,cfg.maxSpeed),"km/h",false);
  ringGauge(242,112,62,power,0,max(1.0f,cfg.maxPower),"kW",true);
  gfx->fillRect(105,76,110,58,C_BLACK);
  bigNumber(160,78,String((int)roundf(speed)),"KM/H",C_WHITE);
  miniValue(8,176,"BAT",String(volts,1)+" V",C_CYAN);
  miniValue(83,176,"CURRENT",String(amps,1)+" A",amps<0?C_GREEN:C_WHITE);
  miniValue(173,176,"SOC",String((int)soc)+" %",C_GREEN);
  miniValue(245,176,"CELL",String(jbd.minCell,3),C_WHITE);
}

void drawDriveSummary() {
  gridBegin("DRIVE SUMMARY");
  float speed=speedFromErpm(vesc1.valid?vesc1.erpm:vesc2.erpm);
  gridCell(0,8,42,"SPEED",String(speed,1)+" km/h",C_CYAN);
  gridCell(1,110,42,"POWER",String(fabsf(combinedPowerW())/1000.0f,2)+" kW",C_ORANGE);
  gridCell(2,212,42,"DUTY",String((vesc1.valid?vesc1.duty:vesc2.duty)*100.0f,0)+" %",C_WHITE);
  gridCell(3,8,92,"BATTERY",String(batteryVoltage(),1)+" V",C_CYAN);
  gridCell(4,110,92,"CURRENT",String(batteryCurrent(),1)+" A",C_WHITE);
  gridCell(5,212,92,"SOC",String(batterySoc(),0)+" %",C_GREEN);
  gridCell(6,8,142,"TRIP",String(tripKm,1)+" km",C_WHITE);
  gridCell(7,110,142,"ENERGY",String(tripWh,1)+" Wh",C_WHITE);
  gridCell(8,212,142,"RANGE",String(estimatedRangeKm(),0)+" km",C_GREEN);
  gridCell(9,8,192,"V1",vescConnected[0]?"ONLINE":"OFFLINE",vescConnected[0]?C_GREEN:C_RED);
  gridCell(10,110,192,"V2",vescConnected[1]?"ONLINE":"OFFLINE",vescConnected[1]?C_GREEN:C_RED);
  gridCell(11,212,192,"BMS",jbd.valid?"DATA OK":"NO DATA",jbd.valid?C_GREEN:C_RED);
}

void drawVescSummary(uint8_t slot) {
  VescData &d=slot?vesc2:vesc1; gridBegin(slot?"VESC 2 SUMMARY":"VESC 1 SUMMARY");
  uint16_t c=slot?C_ORANGE:C_CYAN;
  gridCell(0,8,42,"STATE",bleStatus[slot],vescConnected[slot]?C_GREEN:C_RED);
  gridCell(1,110,42,"ERPM",String((int)d.erpm),c);
  gridCell(2,212,42,"SPEED",String(speedFromErpm(d.erpm),1),C_WHITE);
  gridCell(3,8,92,"BAT A",String(d.inputCurrent,1),C_WHITE);
  gridCell(4,110,92,"MOTOR A",String(d.motorCurrent,1),C_WHITE);
  gridCell(5,212,92,"DUTY",String(d.duty*100,1)+" %",C_WHITE);
  gridCell(6,8,142,"VIN",String(d.vin,1)+" V",C_CYAN);
  gridCell(7,110,142,"FET",String(d.fetTemp,1)+" C",d.fetTemp>80?C_RED:C_WHITE);
  gridCell(8,212,142,"MOTOR",String(d.motorTemp,1)+" C",d.motorTemp>100?C_RED:C_WHITE);
  gridCell(9,8,192,"Ah",String(d.ah,2),C_GREY);
  gridCell(10,110,192,"Wh",String(d.wh,1),C_GREY);
  gridCell(11,212,192,"FAULT",String(d.fault),d.fault?C_RED:C_GREEN);
}

void drawBmsSummary() {
  gridBegin("BMS SUMMARY");
  gridCell(0,8,42,"STATE",bleStatus[2],jbdConnected?C_GREEN:C_RED);
  gridCell(1,110,42,"VOLT",String(jbd.voltage,2)+" V",C_CYAN);
  gridCell(2,212,42,"SOC",String(jbd.soc,0)+" %",C_GREEN);
  gridCell(3,8,92,"CURRENT",String(jbd.current,2)+" A",jbd.current<0?C_GREEN:C_WHITE);
  gridCell(4,110,92,"REMAIN",String(jbd.remainingAh,2)+" Ah",C_WHITE);
  gridCell(5,212,92,"CAP",String(jbd.capacityAh,2)+" Ah",C_WHITE);
  gridCell(6,8,142,"CELLS",String(jbd.cellCount),C_WHITE);
  gridCell(7,110,142,"DELTA",String(jbd.delta*1000,0)+" mV",jbd.delta>0.05?C_RED:C_GREEN);
  gridCell(8,212,142,"PROT",String(jbd.protection,HEX),jbd.protection?C_RED:C_GREEN);
  gridCell(9,8,192,"T1",isnan(jbd.temp1)?"--":String(jbd.temp1,1)+" C",C_WHITE);
  gridCell(10,110,192,"T2",isnan(jbd.temp2)?"--":String(jbd.temp2,1)+" C",C_WHITE);
  gridCell(11,212,192,"DATA",jbd.valid?"OK":"WAIT",jbd.valid?C_GREEN:C_YELLOW);
}

float cellBarsLastV[24];
bool cellBarsHadData = true; // vynuti nesoulad -> plny redraw pri prvnim volani

void drawCellBars() {
  bool haveData = jbd.valid && jbd.cellCount > 0;
  bool fresh = gridBegin("CELL VOLTAGES");

  if (fresh || haveData != cellBarsHadData) {
    gfx->fillRect(0, 30, 320, 190, C_BLACK); // vycistit celou obsahovou plochu (zmena rezimu)
    for (int i = 0; i < 24; i++) cellBarsLastV[i] = -1;
    cellBarsHadData = haveData;
  }

  if (!haveData) {
    centerText("WAITING FOR BMS DATA", 125, 2, C_YELLOW);
    return;
  }

  uint8_t n = min((uint8_t)24, jbd.cellCount);
  float lo = max(2.5f, jbd.minCell - 0.05f), hi = max(lo + 0.1f, jbd.maxCell + 0.05f);

  for (uint8_t i = 0; i < n; i++) {
    if (cellBarsLastV[i] > 0 && fabs(jbd.cells[i] - cellBarsLastV[i]) < 0.001f) continue; // beze zmeny

    int col = i < 12 ? 0 : 160, row = i % 12; int y = 38 + row * 15;
    float f = constrain((jbd.cells[i] - lo) / (hi - lo), 0.0f, 1.0f);

    gfx->fillRect(col, y, 155, 13, C_BLACK); // vycisti jen tento radek
    gfx->setTextSize(1); gfx->setTextColor(C_GREY); gfx->setCursor(col, y + 2); gfx->printf("%02u", i + 1);
    int bx = col + 22, bw = 100; gfx->drawRect(bx, y, bw, 11, C_DGREY);
    uint16_t bc = (jbd.cells[i] <= jbd.minCell + 0.005f) ? C_RED : (jbd.cells[i] >= jbd.maxCell - 0.005f ? C_GREEN : C_CYAN);
    gfx->fillRect(bx + 2, y + 2, (int)((bw - 4) * f), 7, bc);
    gfx->setTextColor(C_WHITE); gfx->setCursor(col + 125, y + 2); gfx->printf("%.3f", jbd.cells[i]);

    cellBarsLastV[i] = jbd.cells[i];
  }

  char summary[64];
  snprintf(summary, sizeof(summary), "MIN %.3f   MAX %.3f   DELTA %.0f mV", jbd.minCell, jbd.maxCell, jbd.delta * 1000);
  textLine(15, 8, 222, 300, String(summary), C_GREY);
}

void drawStatistics() {
  gridBegin("STATISTICS / MAX");
  gridCell(0,8,42,"MAX SPEED",String(maxSpeedSession,1)+" km/h",C_CYAN);
  gridCell(1,110,42,"MAX POWER",String(maxPowerSession/1000.0f,2)+" kW",C_ORANGE);
  gridCell(2,212,42,"MAX BAT A",String(maxBatteryCurrentSession,1)+" A",C_WHITE);
  gridCell(3,8,92,"MAX MOTOR T",String(maxMotorTempSession,1)+" C",C_WHITE);
  gridCell(4,110,92,"MAX CELL D",String(maxCellDeltaSession*1000,0)+" mV",C_YELLOW);
  gridCell(5,212,92,"TRIP",String(tripKm,1)+" km",C_WHITE);
  gridCell(6,8,142,"ENERGY",String(tripWh,1)+" Wh",C_WHITE);
  gridCell(7,110,142,"WH/KM",tripKm>0.1f?String(tripWh/tripKm,1):"--",C_WHITE);
  gridCell(8,212,142,"RANGE",String(estimatedRangeKm(),0)+" km",C_GREEN);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY); gfx->setCursor(8,216); gfx->print("Stats are session-only. RESET TRIP clears them.");
}

void drawEnergy() {
  gridBegin("ENERGY / RANGE");
  float whkm=tripKm>0.1f?tripWh/tripKm:0;
  gridCell(0,8,42,"SOC",String(batterySoc(),0)+" %",C_GREEN);
  gridCell(1,110,42,"PACK",String(batteryVoltage(),1)+" V",C_CYAN);
  gridCell(2,212,42,"CURRENT",String(batteryCurrent(),1)+" A",C_WHITE);
  gridCell(3,8,92,"TRIP",String(tripKm,2)+" km",C_WHITE);
  gridCell(4,110,92,"USED",String(tripWh,1)+" Wh",C_WHITE);
  gridCell(5,212,92,"WH/KM",whkm?String(whkm,1):"--",C_WHITE);
  gridCell(6,8,142,"EST RANGE",String(estimatedRangeKm(),0)+" km",C_GREEN);
  gridCell(7,110,142,"CAPACITY",String(cfg.battCapacity,1)+" Ah",C_WHITE);
  gridCell(8,212,142,"BMS REM",String(jbd.remainingAh,1)+" Ah",C_WHITE);
}


void drawDiagnosticLog() {
  gridBegin("DIAGNOSTIC LOG");

  gfx->setTextSize(1);
  gfx->setTextColor(C_CYAN);
  gfx->setCursor(8, 31);
  gfx->print("BLE / VESC / JBD EVENT LOG");

  gfx->setTextColor(C_GREY);
  gfx->setCursor(248, 31);
  gfx->printf("%d/%d", diagCount, DIAG_LINES);

  const int firstY = 48;
  const int lineH = 10;
  const int visible = 17;

  if (diagCount == 0) {
    gfx->setTextColor(C_YELLOW);
    gfx->setCursor(8, 65);
    gfx->print("NO EVENTS YET");
  } else {
    int start = diagScroll;
    if (start > max(0, diagCount - visible)) start = max(0, diagCount - visible);

    for (int row = 0; row < visible; row++) {
      int off = start + row;
      if (off >= diagCount) break;

      String s = diagAt(off);
      gfx->setTextColor(row == 0 ? C_WHITE : C_GREY);
      gfx->setCursor(5, firstY + row * lineH);
      gfx->print(s);
    }
  }

  // Bottom controls. Touch uses broad zones below.
  gfx->drawRect(5, 222, 72, 16, C_GREY);
  gfx->drawRect(78, 222, 150, 16, C_GREY);
  gfx->drawRect(233, 222, 82, 16, C_GREY);
  gfx->setTextColor(C_WHITE);
  gfx->setCursor(15, 226); gfx->print("UP");
  gfx->setCursor(91, 226); gfx->print("DOWN");
  gfx->setCursor(244, 226); gfx->print("CLEAR");

  gfx->setTextColor(C_GREY);
  gfx->setCursor(160, 206);
  gfx->print("V1/V2/JBD errors stay here");
}

void drawBleStatusPage() {
  gridBegin("BLE STATUS");
  gridCell(0,8,42,"VESC1",bleStatus[0],vescConnected[0]?C_GREEN:C_RED);
  gridCell(1,110,42,"VESC2",bleStatus[1],vescConnected[1]?C_GREEN:C_RED);
  gridCell(2,212,42,"JBD",bleStatus[2],jbdConnected?C_GREEN:C_RED);

  textLine(3,8,96,300,String("V1 ")+(cfg.vesc1Addr[0]?cfg.vesc1Addr:"--"),C_GREY);
  textLine(4,8,114,300,String("V2 ")+(cfg.vesc2Addr[0]?cfg.vesc2Addr:"--"),C_GREY);
  textLine(5,8,132,300,String("JBD ")+(cfg.jbdAddr[0]?cfg.jbdAddr:"--"),C_GREY);

  char buf[64];
  snprintf(buf,sizeof(buf),"SCAN: %s   FOUND: %d",scanning?"ACTIVE":"IDLE",foundCount);
  textLine(6,8,160,300,String(buf),C_GREY);
  snprintf(buf,sizeof(buf),"LOOP %.0f Hz   HEAP %lu KB",loopHz,ESP.getFreeHeap()/1024UL);
  textLine(7,8,178,300,String(buf),C_GREY);
  snprintf(buf,sizeof(buf),"REASON V1:%d V2:%d JBD:%d",bleLastReason[0],bleLastReason[1],bleLastReason[2]);
  textLine(8,8,196,300,String(buf),C_GREY);

  gfx->setTextSize(1); gfx->setTextColor(C_GREY); gfx->setCursor(8,216); gfx->print("BLE button opens scanner / manual connect.");
}

// ---------- EXTRA PAGES (11-19) ----------
// All built on the same gridBegin()/gridCell() dirty-check pattern as the
// pages above, so they inherit the flicker fix automatically. They surface
// telemetry fields that were already being parsed but never shown anywhere:
// Id/Iq, Ah/Wh charged, tach/tachAbs, JBD protection bits, system health.

void drawVescAdvanced(uint8_t slot) {
  VescData &d = slot ? vesc2 : vesc1;
  gridBegin(slot ? "VESC 2 ADVANCED" : "VESC 1 ADVANCED");
  uint16_t c = slot ? C_ORANGE : C_CYAN;
  gridCell(0,8,42,"Id",String(d.id,1)+" A",C_WHITE);
  gridCell(1,110,42,"Iq",String(d.iq,1)+" A",C_WHITE);
  gridCell(2,212,42,"DUTY",String(d.duty*100.0f,1)+" %",c);
  gridCell(3,8,92,"AH USED",String(d.ah,2),C_WHITE);
  gridCell(4,110,92,"AH CHARGED",String(d.ahCharged,2),C_GREEN);
  gridCell(5,212,92,"WH USED",String(d.wh,1),C_WHITE);
  gridCell(6,8,142,"WH CHARGED",String(d.whCharged,1),C_GREEN);
  gridCell(7,110,142,"TACH",String(d.tach),C_GREY);
  gridCell(8,212,142,"TACH ABS",String(d.tachAbs),C_GREY);
  gridCell(9,8,192,"ERPM",String((int)d.erpm),c);
  gridCell(10,110,192,"FAULT",String(d.fault),d.fault?C_RED:C_GREEN);
  gridCell(11,212,192,"STATE",bleStatus[slot],vescConnected[slot]?C_GREEN:C_RED);
}

void drawDualCompare() {
  gridBegin("DUAL VESC COMPARE");
  gridCell(0,8,42,"V1 ERPM",String((int)vesc1.erpm),C_CYAN);
  gridCell(1,160,42,"V2 ERPM",String((int)vesc2.erpm),C_ORANGE);
  gridCell(2,8,92,"V1 A",String(vesc1.inputCurrent,1),C_CYAN);
  gridCell(3,160,92,"V2 A",String(vesc2.inputCurrent,1),C_ORANGE);
  gridCell(4,8,142,"V1 MOTOR C",String(vesc1.motorTemp,1),vesc1.motorTemp>100?C_RED:C_CYAN);
  gridCell(5,160,142,"V2 MOTOR C",String(vesc2.motorTemp,1),vesc2.motorTemp>100?C_RED:C_ORANGE);
  gridCell(6,8,192,"V1 KM/H",String(speedFromErpm(vesc1.erpm),1),C_CYAN);
  gridCell(7,160,192,"V2 KM/H",String(speedFromErpm(vesc2.erpm),1),C_ORANGE);
}

void drawTemperatures() {
  gridBegin("TEMPERATURES");
  gridCell(0,8,42,"V1 FET",String(vesc1.fetTemp,1)+" C",vesc1.fetTemp>80?C_RED:C_CYAN);
  gridCell(1,110,42,"V1 MOTOR",String(vesc1.motorTemp,1)+" C",vesc1.motorTemp>100?C_RED:C_CYAN);
  gridCell(2,212,42,"MAX SEEN",String(maxMotorTempSession,1)+" C",C_YELLOW);
  gridCell(3,8,92,"V2 FET",String(vesc2.fetTemp,1)+" C",vesc2.fetTemp>80?C_RED:C_ORANGE);
  gridCell(4,110,92,"V2 MOTOR",String(vesc2.motorTemp,1)+" C",vesc2.motorTemp>100?C_RED:C_ORANGE);
  gridCell(5,8,142,"JBD T1",isnan(jbd.temp1)?"--":String(jbd.temp1,1)+" C",C_WHITE);
  gridCell(6,110,142,"JBD T2",isnan(jbd.temp2)?"--":String(jbd.temp2,1)+" C",C_WHITE);
}

void drawTripOdometer() {
  gridBegin("TRIP / ODOMETER");
  float whkm = tripKm>0.1f ? tripWh/tripKm : 0;
  gridCell(0,8,42,"TRIP",String(tripKm,2)+" km",C_WHITE);
  gridCell(1,110,42,"ENERGY",String(tripWh,1)+" Wh",C_WHITE);
  gridCell(2,212,42,"WH/KM",whkm?String(whkm,1):"--",C_WHITE);
  gridCell(3,8,92,"EST RANGE",String(estimatedRangeKm(),0)+" km",C_GREEN);
  gridCell(4,8,142,"V1 TACH ABS",String(vesc1.tachAbs),C_CYAN);
  gridCell(5,160,142,"V2 TACH ABS",String(vesc2.tachAbs),C_ORANGE);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,192); gfx->print("Tach = raw motor counts, not distance-calibrated.");
}

String jbdProtectionText(uint16_t p) {
  if (!p) return "OK - NO FLAGS";
  String s;
  if (p & 0x0001) s += "CELL OV ";
  if (p & 0x0002) s += "CELL UV ";
  if (p & 0x0004) s += "PACK OV ";
  if (p & 0x0008) s += "PACK UV ";
  if (p & 0x0010) s += "CHG OT ";
  if (p & 0x0020) s += "CHG UT ";
  if (p & 0x0040) s += "DSG OT ";
  if (p & 0x0080) s += "DSG UT ";
  if (p & 0x0100) s += "CHG OC ";
  if (p & 0x0200) s += "DSG OC ";
  if (p & 0x0400) s += "SHORT ";
  if (p & 0x0800) s += "IC ERR ";
  if (p & 0x1000) s += "SW LOCK ";
  if (s.length() == 0) s = "FLAGS SET";
  return s;
}

void drawJbdProtection() {
  gridBegin("JBD PROTECTION");
  gridCell(0,8,42,"RAW",String(jbd.protection,HEX),jbd.protection?C_RED:C_GREEN);
  gridCell(1,110,42,"CELLS",String(jbd.cellCount),C_WHITE);
  gridCell(2,212,42,"NTC",String(jbd.ntcCount),C_WHITE);
  gridCell(3,8,92,"REMAIN",String(jbd.remainingAh,2)+" Ah",C_WHITE);
  gridCell(4,110,92,"CAPACITY",String(jbd.capacityAh,2)+" Ah",C_WHITE);
  gridCell(5,212,92,"DELTA",String(jbd.delta*1000,0)+" mV",jbd.delta>0.05?C_RED:C_GREEN);
  textLine(6,8,150,300,jbdProtectionText(jbd.protection),jbd.protection?C_RED:C_GREEN);
  gfx->setTextSize(1); gfx->setTextColor(C_GREY);
  gfx->setCursor(8,192); gfx->print("Bit layout per standard JBD/Xiaoxiang protocol.");
}

void drawSystemInfo() {
  gridBegin("SYSTEM INFO");
  uint32_t upSec = millis()/1000UL;
  gridCell(0,8,42,"HEAP",String(ESP.getFreeHeap()/1024UL)+" KB",C_WHITE);
  gridCell(1,110,42,"LOOP",String(loopHz,0)+" Hz",C_WHITE);
  gridCell(2,212,42,"UPTIME",String(upSec)+" s",C_WHITE);
  gridCell(3,8,92,"VESC POLL",String(cfg.vescPollMs)+" ms",C_CYAN);
  gridCell(4,110,92,"JBD POLL",String(cfg.jbdPollMs)+" ms",C_ORANGE);
  gridCell(5,212,92,"BLE TIMEOUT",String(cfg.bleConnectTimeoutMs)+" ms",C_YELLOW);
  gridCell(6,8,142,"AUTO RECONNECT",cfg.autoReconnect?"ON":"OFF",cfg.autoReconnect?C_YELLOW:C_GREY);
  gridCell(7,110,142,"SETTINGS",configDirty?"SAVING...":"SAVED",configDirty?C_YELLOW:C_GREEN);
}

void drawBleDiagnostics() {
  gridBegin("BLE / LINK DIAGNOSTICS");
  uint32_t now = millis();
  gridCell(0,8,42,"V1",bleStatus[0],vescConnected[0]?C_GREEN:C_RED);
  gridCell(1,110,42,"V2",bleStatus[1],vescConnected[1]?C_GREEN:C_RED);
  gridCell(2,212,42,"JBD",bleStatus[2],jbdConnected?C_GREEN:C_RED);
  char buf[48];
  snprintf(buf,sizeof(buf),"r=%d %lus ago",bleLastReason[0],(unsigned long)((now-lastBleErrorMs[0])/1000));
  textLine(3,8,92,300,String("V1 ")+buf,C_GREY);
  snprintf(buf,sizeof(buf),"r=%d %lus ago",bleLastReason[1],(unsigned long)((now-lastBleErrorMs[1])/1000));
  textLine(4,8,110,300,String("V2 ")+buf,C_GREY);
  snprintf(buf,sizeof(buf),"r=%d %lus ago",bleLastReason[2],(unsigned long)((now-lastBleErrorMs[2])/1000));
  textLine(5,8,128,300,String("JBD ")+buf,C_GREY);
  textLine(6,8,156,300,String("BUSY: ")+(bleBusy?(bleBusySlot==2?"JBD":(bleBusySlot==1?"VESC2":"VESC1")):"idle"),bleBusy?C_YELLOW:C_GREY);
  textLine(7,8,180,300,String("AUTO CONNECT: ")+(autoConnectRunning?"running":"waiting"),C_GREY);
}

void drawAboutPage() {
  gridBegin("ABOUT");
  gridCell(0,8,42,"BUILD",__DATE__,C_WHITE);
  gridCell(1,160,42,"PAGES",String(PAGE_COUNT),C_WHITE);
  textLine(2,8,92,300,"VESC + JBD BLE COCKPIT",C_CYAN);
  textLine(3,8,110,300,"ESP32-2432S028 / TPM408-2.8 CYD",C_GREY);
  textLine(4,8,128,300,"Flicker-free dirty-check rendering",C_GREY);
  textLine(5,8,146,300,"Settings auto-save to NVS on change",C_GREY);
  textLine(6,8,164,300,"Swipe LEFT/RIGHT at bottom to change page",C_GREY);
}

void drawBmsLongTermDiagnostics() {
  gridBegin("BMS DIAGNOSTICS");

  if (!jbd.valid || !jbd.cellCount) {
    textLine(0,8,45,304,"BMS: NO DATA",C_RED);
    textLine(1,8,70,304,"Waiting for cell-voltage frame...",C_GREY);
    return;
  }

  gridCell(0,8,42,"BMS",jbdConnected?"ONLINE":"DATA",jbdConnected?C_GREEN:C_YELLOW);
  gridCell(1,160,42,"CELLS",String(jbd.cellCount),C_WHITE);
  gridCell(2,8,82,"PACK",String(jbd.voltage,2)+" V",C_CYAN);
  gridCell(3,160,82,"SOC",String(jbd.soc,0)+" %",C_GREEN);

  gridCell(4,8,122,"LIVE DELTA",String(jbd.delta*1000.0f,0)+" mV",jbd.delta>0.050f?C_RED:C_GREEN);
  gridCell(5,160,122,"MAX DELTA",String(longTermMaxDelta*1000.0f,0)+" mV",longTermMaxDelta>0.050f?C_RED:C_ORANGE);

  uint8_t worst = longTermWorstCell;
  if (worst >= jbd.cellCount) worst = 0;
  float worstDev = (worst < 32) ? cellDevAvg[worst] : 0.0f;
  gridCell(6,8,162,"LT WORST",String("C")+String(worst+1),C_YELLOW);
  gridCell(7,160,162,"LT DEV",String(worstDev*1000.0f,1)+" mV",worstDev>0.025f?C_RED:C_ORANGE);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GREY);
  gfx->setCursor(8, 192);
  gfx->printf("C%02u now %.3f V  avg-dev %.1f mV", worst+1, jbd.cells[worst], worstDev*1000.0f);

  // Compact cell-voltage list. Historical worst cell is yellow; current
  // minimum is red and current maximum is green.
  uint8_t n = min((uint8_t)16, jbd.cellCount);
  for (uint8_t i = 0; i < n; i++) {
    int col = i % 4;
    int row = i / 4;
    int x = 8 + col * 78;
    int y = 207 + row * 11;
    uint16_t colr = C_WHITE;
    if (i == worst) colr = C_YELLOW;
    if (fabsf(jbd.cells[i] - jbd.minCell) < 0.0006f) colr = C_RED;
    if (fabsf(jbd.cells[i] - jbd.maxCell) < 0.0006f) colr = C_GREEN;
    gfx->setTextColor(colr);
    gfx->setCursor(x, y);
    gfx->printf("C%02u %.3f", i+1, jbd.cells[i]);
  }
}

void drawVescConfigPage(uint8_t slot){
  VescConfigData &c=vescCfg[slot]; gridBegin(slot?"VESC 2 CONFIG":"VESC 1 CONFIG");
  if(vescConnected[slot] && (configPageLast!=page || !c.valid || millis()-c.lastMs>3000)){requestVescConfig(slot);configPageLast=page;}
  if(!c.valid){textLine(0,8,45,300,vescConnected[slot]?"READING VESC CONFIG...":"VESC OFFLINE",vescConnected[slot]?C_YELLOW:C_RED);return;}
  gridCell(0,8,42,"MOTOR MAX",String(c.motorCurrentMax,1)+" A",C_CYAN);
  gridCell(1,110,42,"BATTERY MAX",String(c.batteryCurrentMax,1)+" A",C_ORANGE);
  gridCell(2,212,42,"MAX ERPM",String((int)c.maxErpm),C_WHITE);
  gridCell(3,8,92,"MOTOR MIN",String(c.motorCurrentMin,1)+" A",C_WHITE);
  gridCell(4,110,92,"BATTERY MIN",String(c.batteryCurrentMin,1)+" A",C_WHITE);
  gridCell(5,212,92,"DUTY MAX",String(c.maxDuty,3),C_WHITE);
  gridCell(6,8,142,"POWER MAX",String(c.wattMax,0)+" W",C_YELLOW);
  gridCell(7,110,142,"POWER MIN",String(c.wattMin,0)+" W",C_GREY);
  gridCell(8,212,142,"ABS MAX",String(c.absCurrentMax,1)+" A",C_WHITE);
  gridCell(9,8,192,"MOTOR TYPE",String(c.motorType),C_CYAN);
  gridCell(10,110,192,"SENSOR",String(c.sensorMode),C_CYAN);
  gridCell(11,212,192,"PWM",String(c.pwmMode),C_GREY);
  gfx->setTextSize(1);gfx->setTextColor(C_GREY);gfx->setCursor(8,216);gfx->printf("SIG %08lX LEN %u AGE %lums",(unsigned long)c.signature,c.payloadLen,(unsigned long)(millis()-c.lastMs));
}

UiMode lastUiMode = (UiMode)100; // neplatna hodnota -> vynuti prvni redraw

void drawPage(bool touchTriggered) {
  bool uiModeChanged = (uiMode != lastUiMode);
  static int lastDrawnPage = -1;
  if(uiMode==UI_DASH && lastDrawnPage!=page){
    gfx->fillScreen(C_BLACK); statusDotsInit=false; gridInitForPage=-1; audiDashboardInit=false; configPageLast=-1;
    for(int i=0;i<16;i++) gridLastValueSet[i]=false; lastDrawnPage=page;
  }
  lastUiMode = uiMode;

  if (page != 0) audiDashboardInit = false;
  if (uiMode != UI_DASH) { gridInitForPage = -1; audiDashboardInit = false; }

  // Menu/BLE/Settings nemaji plynule "streamovana" data jako dash stranky -
  // meni se jen kdyz uzivatel neco stiskne (nebo behem BLE skenovani prichazi
  // nove nalezene zarizeni). Bez tehle podminky se i tady delal fillScreen()
  // 10x/s uplne zbytecne.
  if (uiMode == UI_MENU) { drawMenu(uiModeChanged); return; }
  if (uiMode == UI_BLE) { drawBluetooth(uiModeChanged || touchTriggered || scanning); return; }
  if (uiMode == UI_SETTINGS) { drawSettings(uiModeChanged || touchTriggered); return; }

  switch(page) {
    case 0: drawAudiDashboard(); break;
    case 1: drawDriveSummary(); break;
    case 2: drawVescSummary(0); break;
    case 3: drawVescSummary(1); break;
    case 4: drawBmsSummary(); break;
    case 5: drawCellBars(); break;
    case 6: drawStatistics(); break;
    case 7: drawEnergy(); break;
    case 8: drawBleStatusPage(); break;
    case 9: drawConnectionSettingsPanel(); break;
    case 10: drawDiagnosticLog(); break;
    case 11: drawVescAdvanced(0); break;
    case 12: drawVescAdvanced(1); break;
    case 13: drawDualCompare(); break;
    case 14: drawTemperatures(); break;
    case 15: drawTripOdometer(); break;
    case 16: drawJbdProtection(); break;
    case 17: drawSystemInfo(); break;
    case 18: drawBleDiagnostics(); break;
    case 19: drawAboutPage(); break;
    case 20: drawBmsLongTermDiagnostics(); break;
    case 21: drawVescConfigPage(0); break;
    case 22: drawVescConfigPage(1); break;
  }
}

// ---------- TOUCH ----------
bool touchReadXY(int &x, int &y) {
  if (!touch.touched()) return false;
  TS_Point p = touch.getPoint();

  // KEEP THE WORKING V9 FIXED MAPPING.
  x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, 320);
  y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 240);
  x = constrain(x, 0, 319);
  y = constrain(y, 0, 239);
  return true;
}

bool hit(int x, int y, int x0, int y0, int x1, int y1) {
  return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

void handleTouch() {
  int x, y;
  if (!touchReadXY(x, y)) return;

  uint32_t now = millis();
  if (now - lastTouch < 180) return;
  lastTouch = now;

  if (uiMode == UI_DASH) {
    // Bottom navigation has priority over the central MENU label.
    if (y >= 195 && x < 75) { page = (page + PAGE_COUNT - 1) % PAGE_COUNT; forceDraw = true; return; }
    if (y >= 195 && x > 245) { page = (page + 1) % PAGE_COUNT; forceDraw = true; return; }
    if (hit(x,y,0,0,80,38) || hit(x,y,90,180,230,194)) {
      uiMode = UI_MENU; forceDraw = true; return;
    }
    if (hit(x,y,225,0,319,55)) { uiMode = UI_BLE; forceDraw = true; return; }
    return;
  }


  if (uiMode == UI_DASH && page == 10) {
    if (hit(x,y,0,0,95,35)) {
      uiMode = UI_MENU; forceDraw = true; return;
    }

    if (hit(x,y,0,214,76,239)) {
      diagScroll = min(max(0, diagCount - 17), diagScroll + 1);
      forceDraw = true; return;
    }
    if (hit(x,y,77,214,232,239)) {
      diagScroll = max(0, diagScroll - 1);
      forceDraw = true; return;
    }
    if (hit(x,y,233,214,319,239)) {
      for (uint8_t i = 0; i < DIAG_LINES; i++) diagLog[i] = "";
      diagHead = 0;
      diagCount = 0;
      diagScroll = 0;
      diagAdd("LOG CLEARED");
      forceDraw = true; return;
    }
    return;
  }

  if (uiMode == UI_MENU) {
    if (hit(x,y,0,0,95,35)) {
      uiMode = UI_DASH; page = 0; forceDraw = true; return;
    }
    // Match the four drawn menu rectangles, with generous margins.
    if (hit(x,y,10,45,310,84)) {
      uiMode = UI_DASH; page = 0;
    } else if (hit(x,y,10,85,310,122)) {
      uiMode = UI_BLE;
    } else if (hit(x,y,10,123,310,160)) {
      uiMode = UI_SETTINGS;
    } else if (hit(x,y,10,161,310,218)) {
      uiMode = UI_DASH; page = 10;
    }
    forceDraw = true;
    return;
  }

  if (uiMode == UI_BLE) {
    // BLE TOUCH: use broad zones so the buttons remain usable even with
    // small touch-coordinate offsets.  The visible buttons are only visual;
    // the whole horizontal bands below are active.
    if (hit(x,y,0,0,105,35)) {
      uiMode = UI_MENU; forceDraw = true; return;
    }

    // Slot selector.
    if (hit(x,y,0,29,106,66)) { bleSlot = 0; forceDraw = true; return; }
    if (hit(x,y,107,29,213,66)) { bleSlot = 1; forceDraw = true; return; }
    if (hit(x,y,214,29,319,66)) { bleSlot = 2; forceDraw = true; return; }

    // ACTION BAND.  Very generous zones: SCAN / CONNECT / DISCONNECT / CLEAR.
    if (hit(x,y,0,67,79,114)) {
      Serial.printf("[TOUCH] BLE SCAN x=%d y=%d\n", x, y);
      if (!bleBusy) startScan(); forceDraw = true; return;
    }
    if (hit(x,y,80,67,159,114)) {
      if (bleBusy) { forceDraw = true; return; }
      String a = slotAddress(bleSlot);
      uint8_t type = BLE_ADDR_PUBLIC;
      if (selectedFound >= 0 && selectedFound < foundCount) {
        a = found[selectedFound].addr;
        type = found[selectedFound].addrType;
        if (bleSlot == 0) { strncpy(cfg.vesc1Addr, a.c_str(), sizeof(cfg.vesc1Addr)-1); cfg.vesc1Addr[17]=0; cfg.vesc1AddrType=type; }
        else if (bleSlot == 1) { strncpy(cfg.vesc2Addr, a.c_str(), sizeof(cfg.vesc2Addr)-1); cfg.vesc2Addr[17]=0; cfg.vesc2AddrType=type; }
        else { strncpy(cfg.jbdAddr, a.c_str(), sizeof(cfg.jbdAddr)-1); cfg.jbdAddr[17]=0; cfg.jbdAddrType=type; }
      } else {
        if (bleSlot == 0) type = cfg.vesc1AddrType;
        else if (bleSlot == 1) type = cfg.vesc2AddrType;
        else type = cfg.jbdAddrType;
      }
      if (a.length()) {
        bool ok = (bleSlot < 2) ? connectVescSlot(bleSlot, a.c_str(), type) : connectJbd(a.c_str(), type);
        if (ok) saveConfig();
      }
      forceDraw = true; return;
    }
    if (hit(x,y,160,67,239,114)) {
      disconnectSlot(bleSlot); forceDraw = true; return;
    }
    if (hit(x,y,240,67,319,114)) {
      clearSlotAddress(bleSlot); forceDraw = true; return;
    }

    // Found-device rows: deliberately accept the whole row, not only the
    // text. Six rows are visible; scrolling exposes the remaining devices.
    if (y >= 115 && y < 224) {
      int row = (y - 115) / 18;
      int idx = foundScroll + row;
      if (idx >= 0 && idx < foundCount && row < 6) {
        selectedFound = idx;
        forceDraw = true;
      }
      return;
    }

    if (hit(x,y,240,224,280,239)) {
      foundScroll = max(0, foundScroll - 1);
      forceDraw = true; return;
    }
    if (hit(x,y,281,224,319,239)) {
      foundScroll = min(max(0, foundCount - 6), foundScroll + 1);
      forceDraw = true; return;
    }
    return;
  }

  if (uiMode == UI_SETTINGS) {
    if (hit(x,y,0,0,95,35)) {
      uiMode = UI_MENU; forceDraw = true; return;
    }

    const int SET_COUNT = 18;
    int start = constrain(settingsItem - 3, 0, SET_COUNT - 7);
    if (y >= 32 && y < 210) {
      int row = (y - 32) / 25;
      if (row > 6) row = 6;
      int idx = start + row;
      if (idx >= 0 && idx < SET_COUNT) {
        settingsItem = idx;
        if (idx <= 14) {
          // Actual +/- buttons are at x=250..309; broad zones make touch reliable.
          if (x >= 275) changeSetting(idx, +1);
          else if (x <= 75) changeSetting(idx, -1);
          else if (idx == 12 || idx == 13) changeSetting(idx, +1);
        } else if (idx == 15) {
          resetTrip();
        } else if (idx == 16) {
          saveConfig();
        } else if (idx == 17) {
          cfg = Config();
          resetTrip();
          saveConfig();
        }
        forceDraw = true; return;
      }
    }

    if (y >= 205) {
      if (x < 150) settingsItem = max(settingsItem - 1, 0);
      else settingsItem = min(settingsItem + 1, SET_COUNT - 1);
      forceDraw = true;
    }
  }
}

// Complete service discovery only after the asynchronous link is established.
void bleSetupTask(void *param) {
  (void)param;
  for (;;) {
    int8_t s = bleSetupRequest;
    if (s >= 0 && s <= 2 && !bleSetupRunning) {
      bleSetupRequest = -1;
      bleSetupRunning = true;

      // Give the connection callback a moment to finish before GATT discovery.
      vTaskDelay(pdMS_TO_TICKS(30));

      if (!bleSetupCancel) {
        if (s < 2) setupVescAfterConnect((uint8_t)s);
        else setupJbdAfterConnect();
      }

      bleSetupRunning = false;
      bleSetupCancel = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void requestBleDisconnect(uint8_t slot) {
  bleSetupCancel = true;
  if (bleSetupRequest == (int8_t)slot) bleSetupRequest = -1;

  if (slot < 2) {
    vescConnected[slot] = false;
    vescRx[slot] = nullptr;
    vescTx[slot] = nullptr;
    vesc1.valid = (slot == 0) ? false : vesc1.valid;
    vesc2.valid = (slot == 1) ? false : vesc2.valid;
    bleStatus[slot] = "DISCONNECTING";
    if (vescClient[slot] && !bleSetupRunning) {
      if (vescClient[slot]->isConnected()) vescClient[slot]->disconnect();
      else vescClient[slot]->cancelConnect();
    }
  } else {
    jbdConnected = false;
    jbdWrite = nullptr;
    jbdNotify = nullptr;
    jbd.valid = false;
    bleStatus[2] = "DISCONNECTING";
    if (jbdClient && !bleSetupRunning) {
      if (jbdClient->isConnected()) jbdClient->disconnect();
      else jbdClient->cancelConnect();
    }
  }
  forceDraw = true;
}

// ---------- BLE HEALTH ----------
void updateBleHealth() {
  uint32_t now = millis();

  // Connection establishment itself is asynchronous. Abort a dead attempt here
  // without ever blocking the UI.
  if (bleBusy && bleBusySlot >= 0 && bleBusySlot <= 2 &&
      !bleSetupRunning && now - bleBusySince > (uint32_t)cfg.bleConnectTimeoutMs + 1500) {
    int s = bleBusySlot;
    bleSetupCancel = true;
    bleSetupRequest = -1;
    if (s < 2 && vescClient[s]) vescClient[s]->cancelConnect();
    if (s == 2 && jbdClient) jbdClient->cancelConnect();
    bleBusy = false;
    bleBusySlot = -1;
    bleStatus[s] = "TIMEOUT";
    lastBleErrorMs[s] = now;
    forceDraw = true;
  }
  if (vescConnected[0] && vescClient[0] && !vescClient[0]->isConnected()) {
    vescConnected[0] = false; vescRx[0] = nullptr; vescTx[0] = nullptr; bleStatus[0] = "DISCONNECTED"; forceDraw = true;
  }
  if (vescConnected[1] && vescClient[1] && !vescClient[1]->isConnected()) {
    vescConnected[1] = false; vescRx[1] = nullptr; vescTx[1] = nullptr; bleStatus[1] = "DISCONNECTED"; forceDraw = true;
  }
  if (jbdConnected && jbdClient && !jbdClient->isConnected()) {
    jbdConnected = false; jbdWrite = nullptr; jbdNotify = nullptr; bleStatus[2] = "DISCONNECTED"; forceDraw = true;
  }
  // Stale telemetry is not shown as current data.
  if (vesc1.valid && now - vesc1.lastMs > 2500) vesc1.valid = false;
  if (vesc2.valid && now - vesc2.lastMs > 2500) vesc2.valid = false;
  if (jbd.valid && now - jbd.lastMs > 3500) jbd.valid = false;
}

// ---------- SETUP/LOOP ----------
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);   // display backlight ON

  // Switch OFF the RGB LED on the rear of the CYD.
  // The CYD RGB channels are active LOW, so HIGH = OFF.
  pinMode(RGB_LED_R, OUTPUT);
  pinMode(RGB_LED_G, OUTPUT);
  pinMode(RGB_LED_B, OUTPUT);
  digitalWrite(RGB_LED_R, HIGH);
  digitalWrite(RGB_LED_G, HIGH);
  digitalWrite(RGB_LED_B, HIGH);

  gfx->begin();
  gfx->setRotation(1);      // TPM408-2.8: landscape 320x240
  gfx->invertDisplay(true); // REQUIRED for this TPM408 panel.
  gfx->fillScreen(C_BLACK);

  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  touch.begin(touchSPI);
  touch.setRotation(1);

  loadConfig();

  NimBLEDevice::init("ESP32 VESC Cockpit");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Keep all potentially blocking GATT discovery away from loop()/touch handling.
  xTaskCreatePinnedToCore(bleSetupTask, "bleSetup", 8192, nullptr, 1, &bleSetupTaskHandle, 0);

  bootMs = millis();
  loopWindowStart = bootMs;
  forceDraw = true;
  diagAdd("COCKPIT BOOT");
  diagAdd("AUTO CONNECT: " + String(cfg.autoReconnect ? "ON" : "OFF"));
  autoConnectNextMs = millis() + AUTO_CONNECT_START_DELAY_MS;
}

void loop() {

  autoConnectTick();
  uint32_t now = millis();

  updateBleHealth();
  handleTouch();

  // Auto-connect is intentionally handled as a non-blocking state machine.
  // A saved device is scanned/connected by the BLE manager rather than from
  // the fast UI loop itself.
  static bool autoLogDone = false;
  if (!autoLogDone) {
    autoLogDone = true;
    diagAdd(String("SAVED V1: ") + (cfg.vesc1Addr[0] ? "YES" : "NO"));
    diagAdd(String("SAVED V2: ") + (cfg.vesc2Addr[0] ? "YES" : "NO"));
    diagAdd(String("SAVED JBD: ") + (cfg.jbdAddr[0] ? "YES" : "NO"));
  }

  if (vescConnected[0] && now - lastVescPoll >= cfg.vescPollMs) {
    lastVescPoll = now;
    sendVescGetValues(0);
    if (vescConnected[1]) sendVescGetValues(1);
  }

  static uint8_t jbdCmdPhase = 0;
  static uint32_t jbdNextCmdMs = 0;
  if (jbdConnected && now - lastJbdPoll >= cfg.jbdPollMs) {
    lastJbdPoll = now;
    jbdCmdPhase = 0;
    jbdNextCmdMs = now;
  }
  if (jbdConnected && now >= jbdNextCmdMs) {
    sendJbdCommand(jbdCmdPhase == 0 ? 0x03 : 0x04);
    if (jbdCmdPhase == 0) { jbdCmdPhase = 1; jbdNextCmdMs = now + 25; }
    else jbdNextCmdMs = now + cfg.jbdPollMs;
  }

  updateTripEnergy();

  // Persist long-term BMS balance statistics only occasionally to avoid
  // unnecessary flash wear.
  if (cellStatsDirty && (now - cellStatsLastSaveMs >= CELL_STATS_SAVE_MS)) {
    saveLongTermCellStats();
  }

  // Debounced settings autosave: if a value changed and the user has left it
  // alone for a moment, persist it to NVS without needing the SAVE button.
  if (configDirty && now - configDirtySinceMs >= CONFIG_AUTOSAVE_DEBOUNCE_MS) {
    saveConfig();
  }

  loopCounter++;
  if (now - loopWindowStart >= 1000) {
    loopHz = loopCounter * 1000.0f / (now - loopWindowStart);
    loopCounter = 0;
    loopWindowStart = now;
  }

  if (forceDraw || now - lastDraw >= 100) {
    lastDraw = now;
    bool wasForced = forceDraw;
    forceDraw = false;
    drawPage(wasForced);
  }

  delay(2);
}
