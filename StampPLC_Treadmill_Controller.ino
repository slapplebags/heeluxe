/*
  Flux treadmill controller, firmware 11.3.4 (Walk/Run/Custom speed; quiet diagnostics).
  Hardware: M5 StampPLC or Waveshare ESP32-S3-Relay-1CH SKU 32152.
  Keep BoardConfig.h beside this sketch and select FLUX_BOARD there.
  Read README-v11-MOTOR.md first for wiring, build settings and commissioning.

  Waveshare: GPIO4 conditioned optical input, GPIO5 E-stop NO auxiliary,
  GPIO6 local Arm/Pause button; GPIO2/1 FRAM SDA/SCL. Inputs are 3.3V only.
  Onboard relay is a low-voltage dry RUN-PERMIT contact, closed only while
  ARMED/RUNNING. The independent NC E-stop contact stays in the treadmill
  stop loop. In serial motor mode relay 1 switches the 12 V interrupt permission through
  COM/NO and E-stop NC; legacy relay mode interrupts serial RX.
  Direct motor control replaces the console after commissioning. Never parallel two transmitters.
  This controller does not replace original safety devices. No direct treadmill mains/motor switching.

  Target completion stops and latches the count until Reset. Reset never
  arms. Hold/release the local Arm/Pause button (StampPLC B) for one second.
  Waveshare has no screen, light tower, buzzer or local preset buttons.

  RS-485 snapshot v3 (56 registers); FRAM format preserved. Update all network nodes.
  Configure preferred primary address 1; unique secondary addresses 2..8.
  Secondaries disable Wi-Fi; any node can take over as primary. Counts and
  stopping remain local. Bus partitions may each elect a primary.
  OTA updates only the hosting unit and checks a board-specific marker.
*/

#include <Arduino.h>
#include "CommandTypes.h"
#include "BoardConfig.h"
#include "CsafeLink.h"
#include "CybexMotorLink.h"
HardwareSerial csafeSerial(2); // RS485 remains on UART1
CsafeLink csafe;
CybexMotorLink motor;
// User-selected Walk/Run policy; supersedes the old prototype speed cap in BoardConfig.h.
static constexpr uint32_t WALK_SPEED_X100 = 280;
static constexpr uint32_t RUN_SPEED_X100 = 380;
static constexpr uint32_t OPERATING_MAX_X100 = 500;
uint32_t selectedSpeedX100 = WALK_SPEED_X100;
enum class LocalEditField : uint8_t { SPEED, TARGET };
LocalEditField localEditField=LocalEditField::SPEED;
bool localSpeedPage=false;
bool motorSession=false, motorStarted=false, motorStopping=false;
uint32_t motorStopAt=0;
uint8_t motorArmPhase=0; // 1 zero with permit open; 2 zero with permit closed; 0 settled
uint32_t motorArmFeedback=0; // require fresh zero feedback after restoring permission
uint8_t parkPhase=0; // 1 waiting for stop, 2 jogging, 3 final deceleration
uint32_t parkFeedback=0,parkJogAt=0;
// Speed settings and motion intent deliberately are not restored from FRAM.
uint16_t speedToRaw(uint32_t hundredths) { return uint16_t((hundredths*96+50)/100); }
bool motorCanStart();
void requestMotorStart();
void requestMotorSpeed(uint32_t hundredths);
void serviceMotor();
bool csafePermitStopped=true, csafeStartUsed=false, csafeRunRequested=false;
#if FLUX_BOARD == FLUX_STAMPLC
#include <M5StamPLC.h>
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <atomic>
#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_image_format.h>

// Cross-task handshake: only the hardware loop grants the maintenance lock.
// Once granted it is intentionally latched until reboot, including on failure.
std::atomic<uint8_t> otaGate{0}; // 0 normal, 1 requested, 2 locked, 3 refused
std::atomic<bool> otaReboot{false};
esp_ota_handle_t otaHandle = 0;
bool otaWriting = false, otaComplete = false;
size_t otaExpected = 0, otaReceived = 0;
uint32_t otaStarted = 0;
String otaError;
constexpr char FIRMWARE_VERSION[] = "11.3.4";
// Also returned by /api/firmware so the marker is retained in every binary.
// Identification only, not a signature or protection against malicious firmware.
#define OTA_PROJECT_MARKER BOARD_OTA_MARKER
size_t otaMarkerMatch=0;
bool otaMarkerFound=false;
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module for the StampPLC."
#endif
// Dedicated external bus; internal M5 bus stays on controller 0.
TwoWire framBus(1);
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ---------------------------------------------------------------------------
// Installation configuration -- verify these during bench commissioning.
// ---------------------------------------------------------------------------

// Optional factory defaults. Normally leave these blank and configure Wi-Fi
// from the fallback access point's web page. Saved settings override defaults.
constexpr char WIFI_SSID[]     = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char AP_PASSWORD[]   = "fluxcapacitor"; // minimum 8 characters
constexpr char WEB_PASSWORD[]  = "change-this-password";
constexpr bool WIRING_VERIFIED = BOARD_WIRING_VERIFIED;
constexpr uint32_t MAX_TARGET = 1000000000;

// Loaded from NVS at boot, immutable until restart. Configure via /bus.
// Standalone defaults avoid address collisions when several new units boot.
bool RS485_ENABLED = false;
bool PREFERRED_PRIMARY = false;
uint8_t NODE_ID = 0;
bool busConfigRestartPending = false; // hardware-loop owned
uint32_t busConfigRestartAt = 0;

constexpr uint8_t SENSOR_INPUT_CHANNEL = 0;
constexpr uint8_t STOP_AUX_INPUT_CHANNEL = 1;

// R1: serial RX permit; R2: green; R3: red; R4: brown buzzer lead.
// R1 COM/NO and E-stop NC interrupt the verified RX conductor in series.
// AC module supplies power; its switched-mains output remains unused and OFF.
constexpr uint8_t TREADMILL_STOP_RELAY = 0;
static_assert(TREADMILL_STOP_RELAY == 0, "Interrupt must use relay 1 (channel 0)");
constexpr uint8_t GREEN_RELAY          = 1;
constexpr uint8_t RED_RELAY            = 2;
constexpr uint8_t EXTERNAL_BUZZER_RELAY = 3;
constexpr uint32_t EXTERNAL_BEEP_MS = 1000;
#if FLUX_BOARD == FLUX_STAMPLC
M5StamPLC_AC acModule;
#endif

// Change these after observing the Input test example or serial diagnostics.
constexpr bool SENSOR_ACTIVE_LEVEL = BOARD_SENSOR_ACTIVE;
// Polarity verified on this unit after bench testing the mushroom input.
constexpr bool STOP_AUX_ACTIVE_LEVEL = BOARD_ESTOP_ACTIVE;

// False: stop = coil OFF/contact OPEN. Run permit = coil ON/contact CLOSED.
// Do NOT reuse the original G5V-1 NC pulse wiring without adapting this logic.
constexpr bool STOP_RELAY_ACTIVE_LEVEL = false;
constexpr bool INDICATOR_RELAY_ACTIVE_LEVEL = true;

constexpr uint32_t INPUT_SCAN_MS       = 2;      // 500 Hz scan via I2C expander
constexpr uint32_t SENSOR_DEBOUNCE_MS  = 5;
constexpr uint32_t STOP_DEBOUNCE_MS    = 30;
constexpr uint32_t STOP_PULSE_MS       = 3000;
constexpr uint32_t NO_PULSE_FAULT_MS   = 15000;
constexpr uint32_t ARM_WAIT_TIMEOUT_MS = 60000;
constexpr uint32_t DISPLAY_REFRESH_MS  = 250;
constexpr uint32_t WIFI_CONNECT_MS     = 15000;

constexpr uint64_t DEFAULT_TARGET = 100000;
// Legacy relay-only mode uses these targets. Direct motor mode uses the local settings editor.
constexpr uint64_t PRESETS[] = {10000, 50000, 100000};
constexpr size_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);
static_assert(PRESET_COUNT == 3, "Exactly three button presets required");
static_assert(PRESETS[0] > 0 && PRESETS[1] > 0 && PRESETS[2] > 0 &&
              PRESETS[0] <= MAX_TARGET && PRESETS[1] <= MAX_TARGET &&
              PRESETS[2] <= MAX_TARGET, "Invalid step target preset");

// ---------------------------------------------------------------------------

enum class RunState : uint8_t {
  IDLE,
  ARMED,
  RUNNING,
  PAUSED,
  TARGET_REACHED,
  STOPPED,
  FAULT
};

enum class StopReason : uint8_t {
  NONE,
  TARGET,
  LOCAL_BUTTON,
  WEB,
  MUSHROOM,
  NO_PULSES,
  POWER_RECOVERY,
  STORAGE,
  INPUT_IO,
  MOTOR_LINK
};

struct __attribute__((packed)) PersistentRecord {
  uint32_t magic;
  uint16_t formatVersion;
  uint16_t length;
  uint32_t sequence;
  uint64_t count;
  uint64_t target;
  uint8_t presetIndex;
  uint8_t state;
  uint8_t stopReason;
  uint8_t reserved[13];
  uint32_t crc;
};

static_assert(sizeof(PersistentRecord) <= 64, "FRAM record exceeds slot size");

constexpr uint32_t RECORD_MAGIC = 0x46584C58; // "FXLX"
constexpr uint16_t RECORD_VERSION = 1;
constexpr uint16_t FRAM_SLOT_SIZE = 64;
constexpr uint16_t FRAM_SLOT_A = 0;
constexpr uint16_t FRAM_SLOT_B = FRAM_SLOT_SIZE;

// All functions mentioning these types have explicit declarations so Arduino's
// automatic prototype generator cannot place them before their type definitions.
const char *stateName(RunState state);
const char *reasonName(StopReason reason);
bool readFramRecord(uint16_t address, PersistentRecord &record);
void requestTreadmillStop(StopReason reason, RunState finalState);
void setRelay(uint8_t channel, bool active, bool activeLevel = true);
void storageFault();
void setTreadmillStopped(bool stopped, bool keepMotorLink = false);
void startExternalBeep(uint32_t durationMs = EXTERNAL_BEEP_MS);
QueueHandle_t commandQueue;
QueueHandle_t busConfigReplyQueue;
SemaphoreHandle_t snapshotMutex;
// Sampler owns acquisition only; the main loop owns counting, storage and outputs.
struct InputSample { uint32_t at; bool sensor; bool stop; };
QueueHandle_t inputSamples=nullptr;
SemaphoreHandle_t plcIoMutex=nullptr;
std::atomic<uint32_t> sampleMaxGap{0}, sampleLastAt{0}, sampleDrops{0};
std::atomic<bool> sampleFailed{false}, sampleStarted{false};
bool countSavePending=false;
uint32_t countAcceptAfter=0;
constexpr uint32_t INPUT_STALE_MS=100;
constexpr unsigned INPUT_QUEUE_LENGTH=1024;
// Recursive because some board operations may call helpers under the same lock.
class PlcIoGuard {
 public:
  PlcIoGuard() { if(plcIoMutex)xSemaphoreTakeRecursive(plcIoMutex,portMAX_DELAY); }
  ~PlcIoGuard() { if(plcIoMutex)xSemaphoreGiveRecursive(plcIoMutex); }
};
String sharedStatus;
String networkAddress;
String sharedAddress;
String configuredWifiSsid;
String configuredWifiPassword;
bool networkRestartRequested = false;
uint32_t networkRestartAt = 0;
WebServer server(80);

RunState runState = RunState::IDLE;
StopReason stopReason = StopReason::NONE;
uint64_t stepCount = 0;
bool halfStepPending = false;
uint64_t stepTarget = DEFAULT_TARGET;
uint8_t presetIndex = 2;
uint32_t recordSequence = 0;
bool framAvailable = false;

bool sensorStable = false;
bool sensorCandidate = false;
uint32_t sensorCandidateSince = 0;

bool stopAuxStable = false;
bool stopAuxCandidate = false;
uint32_t stopAuxCandidateSince = 0;

bool stopPulseActive = false;
uint32_t stopPulseStarted = 0;
uint32_t lastPulseAt = 0;
uint32_t lastInputScanAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastWifiAttemptAt = 0;
uint32_t lastObservedEdge = 0;
bool seenInactive = false;
bool captureCoast = false;
bool completionLatched = false;
uint32_t buzzerStartedAt = 0, buzzerDurationMs = 0;
bool buzzerActive = false;
bool relayCommanded[4] = {}; // coil commands, NOT physical contact feedback
bool inputsHealthy = false;
bool opticalPulseSeen = false;
uint32_t opticalPulseAt = 0;
uint32_t maxScanGap = 0;
bool fallbackApStarted = false;
String deviceName;
String resetMessage;
uint32_t resetMessageAt = 0;
bool resetMessageError = false;

const char *stateName(RunState state) {
  switch (state) {
    case RunState::IDLE: return "IDLE";
    case RunState::ARMED: return "ARMED";
    case RunState::RUNNING: return "RUNNING";
    case RunState::PAUSED: return "PAUSED";
    case RunState::TARGET_REACHED: return "TARGET REACHED";
    case RunState::STOPPED: return "STOPPED";
    case RunState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char *reasonName(StopReason reason) {
  switch (reason) {
    case StopReason::MOTOR_LINK: return "Motor serial fault";
    case StopReason::NONE: return "None";
    case StopReason::TARGET: return "Target reached";
    case StopReason::LOCAL_BUTTON: return "Local stop";
    case StopReason::WEB: return "Web stop";
    case StopReason::MUSHROOM: return "Stop button";
    case StopReason::NO_PULSES: return "Pulse timeout";
    case StopReason::POWER_RECOVERY: return "Power interrupted while running";
    case StopReason::STORAGE: return "FRAM error";
    case StopReason::INPUT_IO: return "Input expander error";
  }
  return "Unknown";
}

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
    }
  }
  return ~crc;
}

bool readFramRecord(uint16_t address, PersistentRecord &record) {
  uint8_t *bytes = reinterpret_cast<uint8_t *>(&record);
  framBus.beginTransmission(0x50);
  framBus.write(uint8_t(address >> 8)); framBus.write(uint8_t(address));
  if (framBus.endTransmission(false) != 0) return false;
  if (framBus.requestFrom(uint8_t(0x50), uint8_t(sizeof(record))) != sizeof(record)) return false;
  for (size_t i = 0; i < sizeof(record); ++i) bytes[i] = framBus.read();
  if (record.magic != RECORD_MAGIC || record.formatVersion != RECORD_VERSION ||
      record.length != sizeof(PersistentRecord)) return false;
  const uint32_t expected = record.crc;
  record.crc = 0;
  const uint32_t actual = crc32(bytes, sizeof(record));
  record.crc = expected;
  return expected == actual && record.target > 0 && record.target <= MAX_TARGET
    && record.count <= MAX_TARGET + 100000ULL
    && record.state <= uint8_t(RunState::FAULT)
    && record.stopReason <= uint8_t(StopReason::MOTOR_LINK);
}

void writeFramRecord() {
  if (!framAvailable) return;

  PersistentRecord record{};
  record.magic = RECORD_MAGIC;
  record.formatVersion = RECORD_VERSION;
  record.length = sizeof(PersistentRecord);
  record.sequence = ++recordSequence;
  record.count = stepCount;
  record.target = stepTarget;
  record.presetIndex = presetIndex;
  record.state = static_cast<uint8_t>(runState);
  record.stopReason = static_cast<uint8_t>(stopReason);
  record.reserved[1] = halfStepPending;
  record.reserved[0] = completionLatched ? 1 : 0;
  record.crc = 0;
  record.crc = crc32(reinterpret_cast<const uint8_t *>(&record), sizeof(record));

  const uint16_t address = (record.sequence & 1U) ? FRAM_SLOT_A : FRAM_SLOT_B;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  // Invalidate destination first; the other slot remains valid during a write.
  framBus.beginTransmission(0x50);
  framBus.write(uint8_t(address >> 8)); framBus.write(uint8_t(address));
  framBus.write(uint8_t(0));
  if (framBus.endTransmission() != 0) { storageFault(); return; }
  framBus.beginTransmission(0x50);
  framBus.write(uint8_t(address >> 8)); framBus.write(uint8_t(address));
  framBus.write(bytes, sizeof(record));
  if (framBus.endTransmission() != 0) { storageFault(); return; }
  PersistentRecord check{};
  if (!readFramRecord(address, check) || memcmp(&check, &record, sizeof(record)))
    storageFault();
}

void loadFramState() {
  if (!framBus.begin(FRAM_SDA, FRAM_SCL, 400000)) {
    framAvailable = false;
    Serial.println("FRAM bus initialization failed");
    return;
  }
  framBus.setTimeOut(10);
  framBus.beginTransmission(0x50);
  framAvailable = framBus.endTransmission() == 0;
  if (!framAvailable) {
    Serial.println("WARNING: MB85RC256V FRAM not found at 0x50");
    return;
  }

  PersistentRecord a{}, b{};
  const bool validA = readFramRecord(FRAM_SLOT_A, a);
  const bool validB = readFramRecord(FRAM_SLOT_B, b);
  if (!validA && !validB) {
    Serial.println("FRAM has no valid record; using defaults");
    writeFramRecord();
    return;
  }

  const PersistentRecord &record =
      (!validB || (validA && static_cast<int32_t>(a.sequence - b.sequence) > 0)) ? a : b;
  recordSequence = record.sequence;
  stepCount = record.count;
  stepTarget = record.target ? record.target : DEFAULT_TARGET;
  presetIndex = record.presetIndex < PRESET_COUNT ? record.presetIndex : 0;
  runState = static_cast<RunState>(record.state);
  stopReason = static_cast<StopReason>(record.stopReason);
  halfStepPending = record.reserved[1] == 1;
  completionLatched = record.reserved[0] == 1 || runState == RunState::TARGET_REACHED || stepCount >= stepTarget;
  if (completionLatched && runState != RunState::FAULT) {
    runState = RunState::TARGET_REACHED;
    stopReason = StopReason::TARGET;
  }

  // Never silently resume a test after power loss.
  if (runState == RunState::ARMED || runState == RunState::RUNNING) {
    runState = RunState::FAULT;
    stopReason = StopReason::POWER_RECOVERY;
    writeFramRecord();
  }
}

void boardTone(unsigned frequency, unsigned duration) {
#if FLUX_BOARD == FLUX_STAMPLC
  PlcIoGuard guard;
  M5StamPLC.tone(frequency, duration);
#else
  (void)frequency; (void)duration;
#endif
}

// Return physical levels; polarity is applied by the common state machine.
bool readBoardInputs(bool &sensor, bool &stop) {
#if FLUX_BOARD == FLUX_WAVESHARE_RELAY1
  sensor = digitalRead(SENSOR_GPIO);
  stop = digitalRead(ESTOP_GPIO);
  return true; // GPIO reads do NOT diagnose disconnected wires or sensors.
#else
  PlcIoGuard guard;
  uint8_t ports[2];
  if (!M5.In_I2C.readRegister(0x59, 0x00, ports, 2, 400000)) return false;
  const uint8_t pins[8] = {4,5,6,7,12,13,14,15};
  const uint16_t bits = ports[0] | (uint16_t(ports[1]) << 8);
  sensor = bool(bits & (1U << pins[SENSOR_INPUT_CHANNEL]));
  stop = bool(bits & (1U << pins[STOP_AUX_INPUT_CHANNEL]));
  return true;
#endif
}

void setRelay(uint8_t channel, bool active, bool activeLevel) {
  const bool energized = active ? activeLevel : !activeLevel;
#if FLUX_BOARD == FLUX_WAVESHARE_RELAY1
  if (channel != TREADMILL_STOP_RELAY) return; // absent indicator outputs stay OFF
  digitalWrite(RELAY_GPIO, energized ? RELAY_GPIO_ON : !RELAY_GPIO_ON);
#else
  PlcIoGuard guard;
  M5StamPLC.writePlcRelay(channel, energized);
#endif
  if (channel < 4) relayCommanded[channel] = energized;
}

void setTreadmillStopped(bool stopped, bool keepMotorLink) {
  if (MOTOR_ENABLED && stopped) {
    motorArmPhase=0;
    parkPhase=0;
    // Preserve only an already healthy, confirmed-zero session after a normal stop.
    const bool keep = keepMotorLink && motorSession && motor.stopped(millis());
    motor.stop(millis());
    if (!keep) motor.disable();
    motorSession=keep;
    motorStarted=motorStopping=false;
  }
  // Serial mode: energized COM-NO supplies interrupt permission.
  // External E-stop NC is in series; releasing it never commands Start.
  setRelay(TREADMILL_STOP_RELAY, stopped, MOTOR_ENABLED ? false : STOP_RELAY_ACTIVE_LEVEL);
  // Relay first; serial communication can never delay opening the permit.
  if (stopped) {
    csafeRunRequested=false;
    if (!csafePermitStopped || csafe.startPending()) csafe.stop(millis());
  }
  csafePermitStopped=stopped;
}

void storageFault() {
  framAvailable = false;
  setTreadmillStopped(true);
  runState = RunState::FAULT;
  stopReason = StopReason::STORAGE;
}

void updateIndicators() {
  const bool green = runState == RunState::ARMED || runState == RunState::RUNNING;
  const bool red = !green;
  setRelay(GREEN_RELAY, green, INDICATOR_RELAY_ACTIVE_LEVEL);
  setRelay(RED_RELAY, red, INDICATOR_RELAY_ACTIVE_LEVEL);

  #if FLUX_BOARD == FLUX_STAMPLC
  PlcIoGuard guard;
  if (red) M5StamPLC.setStatusLight(255, 0, 0);
  else if (green) M5StamPLC.setStatusLight(0, 255, 0);
  else M5StamPLC.setStatusLight(0, 0, 32);
  #endif
}

void silenceExternalBeep() {
  setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  buzzerActive = false;
}
void startExternalBeep(uint32_t durationMs) {
  setRelay(EXTERNAL_BUZZER_RELAY, true, INDICATOR_RELAY_ACTIVE_LEVEL);
  buzzerStartedAt = millis();
  buzzerDurationMs = durationMs;
  buzzerActive = true;
}
void serviceExternalBeep() {
  if (buzzerActive && uint32_t(millis() - buzzerStartedAt) >= buzzerDurationMs)
    silenceExternalBeep();
}

void requestTreadmillStop(StopReason reason, RunState finalState) {
  motorArmPhase=0; // cancel re-arm before any stop/fault path
  const bool eligible=MOTOR_ENABLED && PARK_SLOT_LEVEL>=0 && motorStarted && !motorStopping &&
    !completionLatched && reason==StopReason::TARGET && finalState==RunState::TARGET_REACHED;
  parkPhase=eligible?1:0;
  if(eligible)parkFeedback=motor.feedbackSequence;
  if (finalState == RunState::TARGET_REACHED) completionLatched = true;
  captureCoast = !completionLatched && (captureCoast || runState == RunState::RUNNING);
  // Normal stops decelerate at zero, then open permission while retaining zero polling.
  const bool graceful=MOTOR_ENABLED && finalState!=RunState::FAULT &&
    reason!=StopReason::MUSHROOM && motorSession && motor.ready(millis()) && relayCommanded[0];
  if(graceful) {
    motor.stop(millis());
    if(!motorStopping)motorStopAt=millis();
    motorStopping=true;motorStarted=false;
  } else setTreadmillStopped(true);
  if(!MOTOR_ENABLED)csafe.stop(millis()); // legacy mode only
  stopPulseActive = true;
  stopPulseStarted = millis();
  // A subsequent web/local STOP must not clear an existing latched fault.
  if (runState != RunState::FAULT) {
    stopReason = completionLatched && finalState != RunState::FAULT ? StopReason::TARGET : reason;
    runState = completionLatched && finalState != RunState::FAULT ? RunState::TARGET_REACHED : finalState;
  }
  writeFramRecord();
  updateIndicators();
  startExternalBeep();
}

void serviceStopPulse() {
  // Held stop; released only by armTest after a local deliberate action.
}

void armTest() {
  // An Arm retry must not reset a live serial session or its pending Start.
  if (runState == RunState::ARMED || runState == RunState::RUNNING) {
    reportReset("Already armed/running; use Start or Stop", true);
    return;
  }
  if(MOTOR_ENABLED && (!MOTOR_COMMISSIONED || motorStopping)) {
    reportReset("Arm blocked: motor setup/stopping",true);return;
  }
  if (otaGate.load()==2 || busConfigRestartPending || completionLatched || !inputsHealthy || !WIRING_VERIFIED || !framAvailable || stopAuxStable || stopAuxCandidate ||
      stepTarget == 0 ||
      stepCount >= stepTarget || millis() - lastObservedEdge < 3000 ||
      runState == RunState::FAULT) {
    boardTone(220, 250);
    return;
  }
  countAcceptAfter=millis();
  stopReason = StopReason::NONE;
  runState = RunState::ARMED;
  lastPulseAt = millis();
  writeFramRecord();
  if (!framAvailable) return;
  captureCoast = false;
  stopPulseActive = false;
  csafeStartUsed=false;
  silenceExternalBeep();
  if(MOTOR_ENABLED) {
    // Establish zero while permission is OPEN, including after E-stop/reset.
    // Retain a healthy normal-stop session; faults still require initialization.
    setTreadmillStopped(true, true);
    const bool reuse = motorSession && motor.stopped(millis());
    if (!reuse) motor.startInitialization(millis());
    else motor.stop(millis());
    motorArmFeedback=motor.feedbackSequence;
    motorArmPhase=1;
    motorSession=true;motorStarted=false;
    reportReset("Arming: zero with interrupt disabled",false);
  } else setTreadmillStopped(false);
  updateIndicators();
  boardTone(880, 80);
}

// Called in the I/O loop, never directly from the network task.
bool csafeCanStart() {
  if(MOTOR_ENABLED)return motorCanStart();
  return CSAFE_MOTOR_CONTROL_ENABLED && CSAFE_ENABLED && csafe.online(millis()) && !(csafe.status & 0x30) &&
    (csafe.status & 15)!=0 && (csafe.status & 15)!=5 && !csafe.busy() &&
    !csafeStartUsed && !csafePermitStopped && runState==RunState::ARMED &&
    otaGate.load()!=1 && otaGate.load()!=2 && !busConfigRestartPending && !completionLatched &&
    inputsHealthy && WIRING_VERIFIED && framAvailable &&
    !stopAuxStable && !stopAuxCandidate && stepCount<stepTarget &&
    millis()-lastObservedEdge>=3000;
}
void requestCsafeStart() {
  if(MOTOR_ENABLED) {requestMotorStart();return;}
  if (!csafeCanStart() || !csafe.start(millis())) {
    reportReset("Start blocked: arm/check CSAFE",true); return;
  }
  csafeStartUsed=true; csafeRunRequested=true;
  reportReset("CSAFE Start queued; check belt",false);
}
void serviceCsafe() {
  if (!CSAFE_ENABLED) return;
  // Recheck immediately before the transport can transmit a queued start.
  if(csafe.startPending() && (csafePermitStopped || stopAuxStable || stopAuxCandidate ||
      !inputsHealthy || !framAvailable || completionLatched || otaGate.load()==1 || otaGate.load()==2 ||
      busConfigRestartPending || (runState!=RunState::ARMED && runState!=RunState::RUNNING)))
    setTreadmillStopped(true);
  csafe.tick(csafeSerial,millis());
  bool failed=csafe.takeStartFailure();
  if(csafeRunRequested && (failed || !csafe.online(millis()))) {
    requestTreadmillStop(StopReason::WEB,RunState::FAULT);
    reportReset(failed?"CSAFE start failed; reset":"CSAFE link lost; reset",true);
  }
}

bool motorCanStart() {
  return MOTOR_ENABLED && MOTOR_COMMISSIONED && motorSession && !motorArmPhase && !motorStopping && !motorStarted &&
    motor.stopped(millis()) && motor.feedbackSequence!=motorArmFeedback &&
    relayCommanded[TREADMILL_STOP_RELAY] && runState==RunState::ARMED &&
    otaGate.load()!=1 && otaGate.load()!=2 && !busConfigRestartPending && !completionLatched &&
    inputsHealthy && WIRING_VERIFIED && framAvailable && !stopAuxStable && !stopAuxCandidate &&
    stepCount<stepTarget && millis()-lastObservedEdge>=3000;
}
void requestMotorStart() {
  if(!motorCanStart() || !motor.setTarget(speedToRaw(selectedSpeedX100),speedToRaw(OPERATING_MAX_X100),millis())) {
    reportReset("Start blocked: wait for armed zero feedback/check link",true);return;
  }
  countAcceptAfter=millis();
  motorStarted=true;lastPulseAt=countAcceptAfter;
  reportReset("Start queued; verify belt motion",false);
}
void requestMotorSpeed(uint32_t hundredths) {
  if(MOTOR_ENABLED && !hundredths && motorStopping){requestTreadmillStop(StopReason::WEB,RunState::STOPPED);return;}
  if(!MOTOR_ENABLED || !MOTOR_COMMISSIONED || hundredths>OPERATING_MAX_X100 || motorStopping ||
     runState==RunState::FAULT || otaGate.load()==1 || otaGate.load()==2 || busConfigRestartPending) {
    reportReset("Speed change blocked",true);return;
  }
  if(!hundredths) {requestTreadmillStop(StopReason::WEB,RunState::STOPPED);return;}
  if(motorStarted) {
    if(!inputsHealthy || !framAvailable || stopAuxStable || stopAuxCandidate || completionLatched ||
       (runState!=RunState::ARMED && runState!=RunState::RUNNING) ||
       !motor.setTarget(speedToRaw(hundredths),speedToRaw(OPERATING_MAX_X100),millis())) {
      reportReset("Speed blocked: check motor link",true);return;
    }
  }
  selectedSpeedX100=hundredths; // staging a speed NEVER starts or arms the belt
  reportReset(motorStarted?"Speed change queued":"Speed staged; start separately",false);
}
// Called only after serviceMotor has checked interlocks and serviced the UART.
void serviceMotorArm() {
  if(!motorArmPhase || motorStopping || motorStarted || runState!=RunState::ARMED) return;
  if(!motor.stopped(millis()) || motor.feedbackSequence==motorArmFeedback) return;
  if(motorArmPhase==1) {
    // Zero has been acknowledged with the permit open. Restore permission,
    // invalidate any pending old reply, and demand another zero acknowledgement.
    setTreadmillStopped(false);
    motorArmFeedback=motor.feedbackSequence;
    motor.stop(millis());
    motorArmPhase=2;
  } else {
    motorArmPhase=0;
    reportReset("Armed: ready; hold B to start",false);
  }
}

void serviceMotor() {
  if(!MOTOR_ENABLED || !motorSession)return;
  if(!MOTOR_COMMISSIONED || !inputsHealthy || !framAvailable || stopAuxStable || stopAuxCandidate ||
     otaGate.load()==1 || otaGate.load()==2 || busConfigRestartPending) {
    requestTreadmillStop(StopReason::MOTOR_LINK,RunState::FAULT);return;
  }
  if(parkPhase==2) {
    if(millis()-parkJogAt>=PARK_JOG_LIMIT_MS) {
      requestTreadmillStop(StopReason::MOTOR_LINK,RunState::FAULT);
      reportReset("Park timeout; Interrupt disabled",true);return;
    }
    if(sensorCandidate!=bool(PARK_SLOT_LEVEL) || sensorStable!=bool(PARK_SLOT_LEVEL)) {
      parkPhase=3;parkFeedback=motor.feedbackSequence;motorStopAt=millis();motor.stop(millis());
    }
  }
  motor.tick(csafeSerial,millis());
  if(motor.phase==CybexMotorLink::Phase::FAULT) {
    requestTreadmillStop(StopReason::MOTOR_LINK,RunState::FAULT);
    reportReset("Motor link lost; motion cancelled",true);return;
  }
  serviceMotorArm();
  if(motorStopping) {
    if(parkPhase==2)return;
    if(motor.stopped(millis()) && (!parkPhase || parkFeedback!=motor.feedbackSequence)) {
      if(parkPhase==1 && sensorStable==bool(PARK_SLOT_LEVEL) && sensorCandidate==sensorStable &&
         runState==RunState::TARGET_REACHED && completionLatched && relayCommanded[0] &&
         motor.setTarget(speedToRaw(PARK_SPEED_X100),speedToRaw(OPERATING_MAX_X100),millis())) {
        parkPhase=2;parkJogAt=millis();reportReset("Parking: clearing slot",false);
      } else setTreadmillStopped(true, true);
    }
    else if(millis()-motorStopAt>=MOTOR_STOP_LIMIT_MS) {
      requestTreadmillStop(StopReason::MOTOR_LINK,RunState::FAULT);
      reportReset("Stop unconfirmed; Interrupt disabled",true);
    }
  } else if(motor.ready(millis()) && !motorStarted && (motor.actualRaw || motor.reportedTargetRaw)) {
    requestTreadmillStop(StopReason::MOTOR_LINK,RunState::FAULT);
    reportReset("Unexpected motor speed; Interrupt disabled",true);
  }
}

void togglePause() {
  if(MOTOR_ENABLED && motorStopping){requestTreadmillStop(StopReason::LOCAL_BUTTON,RunState::PAUSED);return;}
  // Default: B hold arms from stopped, then stops from ARMED/RUNNING.
  // Experimental CSAFE motor control is explicitly opt-in.
  if (MOTOR_ENABLED && runState==RunState::ARMED && !motorStarted) {
    requestMotorStart(); // not ready yet: report blocked, never silently turn Start into Pause
    return;
  }
  if (CSAFE_MOTOR_CONTROL_ENABLED && CSAFE_ENABLED && runState == RunState::ARMED && !csafeStartUsed) {
    requestCsafeStart();
    return;
  }
  if (runState == RunState::RUNNING || runState == RunState::ARMED) {
    requestTreadmillStop(StopReason::LOCAL_BUTTON, RunState::PAUSED);
    return;
  }
  armTest();
}

void reportReset(const char *message, bool error) {
  resetMessage = message;
  resetMessageAt = millis();
  resetMessageError = error;
  Serial.println(message);
  boardTone(error ? 220 : 660, error ? 250 : 100);
}

void resetState(bool clearFaultOnly) {
  if (clearFaultOnly && runState != RunState::FAULT) {
    reportReset("No fault to clear", false); return;
  }
  if ((otaGate.load()==1 || otaGate.load()==2) || busConfigRestartPending || motorStopping) {
    reportReset("Recovery blocked: maintenance", true); return;
  }
  if (clearFaultOnly && millis()-lastObservedEdge < 3000) {
    reportReset("Clear: wait for sensor to settle", true); return;
  }
  if (runState == RunState::RUNNING || runState == RunState::ARMED) {
    reportReset("Reset: stop run first", true);
    return;
  }
  countAcceptAfter=millis(); // discard historical pulse samples from before reset
  // Reset never grants permission. Preserve zero polling after target completion.
  setTreadmillStopped(true, runState != RunState::FAULT);
  bool sensorLevel, stopLevel;
  inputsHealthy = readBoardInputs(sensorLevel, stopLevel);
  if (!inputsHealthy) {
    requestTreadmillStop(StopReason::INPUT_IO, RunState::FAULT);
    reportReset("Reset: input IO fault", true);
    return;
  }
  const bool stopRaw = stopLevel == STOP_AUX_ACTIVE_LEVEL;
  if (stopRaw || stopAuxStable || stopAuxCandidate ||
      millis() - stopAuxCandidateSince < STOP_DEBOUNCE_MS) {
    reportReset("Reset: release E-stop", true);
    return;
  }
  if (!framAvailable) {
    // Reinitialize only the dedicated external FRAM bus, not the PLC IO bus.
    framBus.end();
    if (!framBus.begin(FRAM_SDA, FRAM_SCL, 400000)) {
      storageFault();
      reportReset("Reset: FRAM bus fault", true);
      return;
    }
    framBus.setTimeOut(10);
    framBus.beginTransmission(0x50);
    framAvailable = framBus.endTransmission() == 0;
    if (!framAvailable) {
      storageFault();
      reportReset("Reset: FRAM missing", true);
      return;
    }
    // Recover sequence only; never replace the live count with an older record.
    PersistentRecord a{}, b{};
    const bool validA = readFramRecord(FRAM_SLOT_A, a);
    const bool validB = readFramRecord(FRAM_SLOT_B, b);
    if (validA || validB) {
      recordSequence = (!validB || (validA && int32_t(a.sequence - b.sequence) > 0))
                       ? a.sequence : b.sequence;
    }
  }
  const bool previousHalf = halfStepPending;
  const uint64_t previousCount = stepCount;
  const bool previousCompletion = completionLatched;
  const bool previousCaptureCoast = captureCoast;
  captureCoast = false;
  if (!clearFaultOnly) {
    stepCount = 0;
    halfStepPending = false;
    completionLatched = false;
  } else if (stepCount >= stepTarget) {
    completionLatched = true;
  }
  runState = completionLatched ? RunState::TARGET_REACHED : RunState::IDLE;
  stopReason = StopReason::NONE;
  writeFramRecord();
  if (!framAvailable) {
    // Keep the displayed count if the reset could not be verified in storage.
    stepCount = previousCount;
    halfStepPending = previousHalf;
    completionLatched = previousCompletion;
    captureCoast = previousCaptureCoast;
    updateIndicators();
    reportReset("Reset: FRAM save failed", true);
    return;
  }
  stopPulseActive = false;
  seenInactive = false; // require a fresh inactive sensor level before counting
  silenceExternalBeep();
  updateIndicators();
  if(!sampleStarted.load() || millis()-sampleLastAt.load()>INPUT_STALE_MS) {
    requestTreadmillStop(StopReason::INPUT_IO,RunState::FAULT);
    reportReset("Recovery blocked: input sampler stale",true);return;
  }
  // Queue reset only while stopped; do not reset the sampler's cumulative counters.
  xQueueReset(inputSamples);
  sampleFailed.store(false);
  countSavePending=false;
  reportReset(clearFaultOnly ? "Fault cleared - count kept; stopped" : "Reset OK - stopped", false);
}
void resetCount() { resetState(false); }
void clearFault() { resetState(true); }

void processStepPulse() {
  if (completionLatched) return; // completed count remains frozen until reset
  if (runState != RunState::ARMED && runState != RunState::RUNNING && !captureCoast) return;
  // Serial mode requires an explicit accepted Start before counting motion.
  // Preserve post-stop coast counting and the relay-only pulse-to-run behavior.
  if (MOTOR_ENABLED && !motorStarted && !captureCoast) return;
  if (runState == RunState::ARMED) runState = RunState::RUNNING;
  halfStepPending = !halfStepPending;
  if (!halfStepPending) ++stepCount;
  lastPulseAt = millis();
  if (stepCount >= stepTarget && runState == RunState::RUNNING) {
    requestTreadmillStop(StopReason::TARGET, RunState::TARGET_REACHED);
  } else {
    countSavePending=true;
  }
}

void inputSamplerTask(void *) {
  TickType_t wake=xTaskGetTickCount();
  const TickType_t period=pdMS_TO_TICKS(INPUT_SCAN_MS)>0?pdMS_TO_TICKS(INPUT_SCAN_MS):1;
  uint32_t previous=0;
  for(;;) {
    InputSample sample{};
    const bool ok=readBoardInputs(sample.sensor,sample.stop);
    sample.at=millis();
    if(ok) {
      const uint32_t gap=previous?sample.at-previous:0;
      if(gap>sampleMaxGap.load())sampleMaxGap.store(gap);
      if(gap>INPUT_STALE_MS)sampleFailed.store(true);
      previous=sample.at;sampleLastAt.store(sample.at);sampleStarted.store(true);
      if(xQueueSend(inputSamples,&sample,0)!=pdTRUE) {
        sampleDrops.fetch_add(1);sampleFailed.store(true);
      }
    } else sampleFailed.store(true);
    vTaskDelayUntil(&wake,period);
    // No catch-up burst after an I2C stall.
    if(int32_t(xTaskGetTickCount()-wake)>int32_t(period))wake=xTaskGetTickCount();
  }
}

void scanInputs() {
  // Never silently continue a run after loss of acquisition or queue capacity.
  if(!sampleStarted.load() || sampleFailed.load() ||
     millis()-sampleLastAt.load()>INPUT_STALE_MS) {
    inputsHealthy=false;
    if(stopReason!=StopReason::INPUT_IO)
      requestTreadmillStop(StopReason::INPUT_IO,RunState::FAULT);
    return;
  }
  InputSample sample{};
  unsigned budget=INPUT_QUEUE_LENGTH;
  while(budget-- && xQueueReceive(inputSamples,&sample,0)==pdTRUE) {
    const uint32_t now=sample.at;
    const bool sensorLevel=sample.sensor, stopLevel=sample.stop;
    maxScanGap=sampleMaxGap.load();
    lastInputScanAt=now;
  inputsHealthy = true;
  const bool sensorRaw = sensorLevel == SENSOR_ACTIVE_LEVEL;
  if (sensorRaw != sensorCandidate) {
    sensorCandidate = sensorRaw;
    sensorCandidateSince = now;
  } else if (sensorStable != sensorCandidate && now - sensorCandidateSince >= SENSOR_DEBOUNCE_MS) {
    const bool old = sensorStable;
    sensorStable = sensorCandidate;
    lastObservedEdge = now;
    if (!sensorStable) seenInactive = true;
    if (!old && sensorStable && seenInactive) {
      opticalPulseSeen = true;
      opticalPulseAt = now; // also show sensor activity while counting is stopped
      if(int32_t(now-countAcceptAfter)>0) {
        processStepPulse();
        if(runState==RunState::RUNNING)lastPulseAt=now;
      }
    }
  }
  if (!sensorRaw && now - sensorCandidateSince >= SENSOR_DEBOUNCE_MS) seenInactive = true;

  const bool stopRaw = stopLevel == STOP_AUX_ACTIVE_LEVEL;
  if (stopRaw != stopAuxCandidate) {
    stopAuxCandidate = stopRaw;
    stopAuxCandidateSince = now;
  } else if (stopAuxStable != stopAuxCandidate && now - stopAuxCandidateSince >= STOP_DEBOUNCE_MS) {
    stopAuxStable = stopAuxCandidate;
    if (stopAuxStable) {
      // The primary NC mushroom contact has already stopped the treadmill.
      requestTreadmillStop(StopReason::MUSHROOM, RunState::STOPPED);
    }
  }

  }
  if(sampleFailed.load() || millis()-sampleLastAt.load()>INPUT_STALE_MS) {
    inputsHealthy=false;
    if(stopReason!=StopReason::INPUT_IO)
      requestTreadmillStop(StopReason::INPUT_IO,RunState::FAULT);
    return;
  }
  // One verified FRAM checkpoint per drained batch, not in the sampler task.
  if(countSavePending) {countSavePending=false;writeFramRecord();}
}

void servicePulseTimeout() {
  if (runState == RunState::RUNNING && millis() - lastPulseAt >= NO_PULSE_FAULT_MS) {
    requestTreadmillStop(StopReason::NO_PULSES, RunState::FAULT);
  }
  if (runState == RunState::ARMED && millis() - lastPulseAt >= ARM_WAIT_TIMEOUT_MS)
    requestTreadmillStop(StopReason::NO_PULSES, RunState::FAULT);
}

String u64(uint64_t value) {
  char text[24];
  snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(value));
  return String(text);
}

String jsonStatus() {
  String json;
  json.reserve(1600);
  json += "{\"device\":\"" + deviceName + "\",";
  json += "\"firmware\":\"11.3.4\",";
  json += "\"motor_enabled\":"+String(MOTOR_ENABLED?"true":"false")+",";
  json += "\"motor_commissioned\":"+String(MOTOR_COMMISSIONED?"true":"false")+",";
  json += "\"motor_arm_phase\":"+String(motorArmPhase)+",";
  json += "\"motor_ready\":"+String(motor.ready(millis())?"true":"false")+",";
  json += "\"motor_can_start\":"+String(motorCanStart()?"true":"false")+",";
  json += "\"motor_stopping\":"+String(motorStopping?"true":"false")+",";
  json += "\"motor_started\":"+String(motorStarted?"true":"false")+",";
  json += "\"motor_feedback_valid\":"+String(motor.feedbackValid?"true":"false")+",";
  json += "\"speed_mph\":"+String(selectedSpeedX100/100.0f,2)+",";
  json += "\"max_speed_mph\":"+String(OPERATING_MAX_X100/100.0f,2)+",";
  json += "\"actual_mph\":"+String(motor.actualRaw/96.0f,3)+",";
  json += "\"reported_target_mph\":"+String(motor.reportedTargetRaw/96.0f,3)+",";
  json += "\"motor_requested_raw\":"+String(motor.requestedRaw())+",";
  json += "\"motor_status_raw\":"+String(motor.speedStatus)+",";
  json += "\"motor_tx\":"+String(motor.txCount)+",\"motor_rx\":"+String(motor.rxCount)+",";
  json += "\"motor_bad_frames\":"+String(motor.badFrames)+",\"motor_timeouts\":"+String(motor.timeouts)+",";
  json += "\"motor_phase\":"+String(unsigned(motor.phase))+",";
  json += "\"csafe_motor_control\":"+String(CSAFE_MOTOR_CONTROL_ENABLED?"true":"false")+",";
  json += "\"csafe_enabled\":"+String(CSAFE_ENABLED?"true":"false")+",";
  json += "\"csafe_online\":"+String(csafe.online(millis())?"true":"false")+",";
  json += "\"csafe_can_start\":"+String(csafeCanStart()?"true":"false")+",";
  json += "\"csafe_status\":"+String(csafe.status)+",";
  json += "\"csafe_result\":"+String(unsigned(csafe.result))+",";
  json += "\"csafe_start_pending\":"+String(csafe.startPending()?"true":"false")+",";
  json += "\"csafe_tx\":"+String(csafe.txCount)+",\"csafe_rx\":"+String(csafe.rxCount)+",";
  json += "\"csafe_bad_frames\":"+String(csafe.badFrames)+",\"csafe_timeouts\":"+String(csafe.timeouts)+",";
  json += "\"board_code\":" + String(BOARD_CODE) + ",";
  json += "\"state\":\"" + String(stateName(runState)) + "\",";
  json += "\"reason\":\"" + String(reasonName(stopReason)) + "\",";
  json += "\"count\":" + u64(stepCount) + ",";
  json += "\"target\":" + u64(stepTarget) + ",";
  json += "\"sensor\":" + String(sensorStable ? "true" : "false") + ",";
  json += "\"stop_button\":" + String(stopAuxStable ? "true" : "false") + ",";
  json += "\"fram\":" + String(framAvailable ? "true" : "false") + ",";
  json += "\"reset_message\":\"" + resetMessage + "\",";
  json += "\"max_scan_gap_ms\":" + String(maxScanGap) + ",";
  json += "\"input_queue_overflows\":"+String(sampleDrops.load())+",";
  json += "\"input_sample_age_ms\":"+String(millis()-sampleLastAt.load())+",";
  json += "\"input_sampler_fault\":"+String(sampleFailed.load()?"true":"false")+",";
  json += "\"wiring_verified\":" + String(WIRING_VERIFIED ? "true" : "false");
  json += ",\"node\":" + String(NODE_ID) + ",\"online\":true,\"age_ms\":0,\"command_status\":0";
  json += ",\"inputs_healthy\":" + String(inputsHealthy ? "true" : "false");
  json += ",\"completed\":" + String(completionLatched ? "true" : "false");
  uint8_t relays=0; for(int i=0;i<4;++i) relays |= uint8_t(relayCommanded[i])<<i;
  json += ",\"relays\":"+String(relays)+",\"presets\":[";
  for(int i=0;i<3;++i) { if(i)json+=','; json+=u64(PRESETS[i]); }
  json += "],\"no_pulse_seconds\":"+String(NO_PULSE_FAULT_MS/1000)+
          ",\"arm_timeout_seconds\":"+String(ARM_WAIT_TIMEOUT_MS/1000)+"}";
  return json;
}

// Modbus RTU with election announcements. Every address 1..8 can lead.
// All UART operations live on their own task; only loop() touches PLC I/O/FRAM.
// Holding registers: 0..55 snapshot, 256..258 atomic command mailbox (FC16).
constexpr uint8_t BUS_LAST_NODE = 8;
constexpr uint16_t BUS_REG_COUNT = 56;
constexpr uint32_t BUS_BAUD = 19200;
constexpr uint32_t BUS_OFFLINE_MS = 6000;
constexpr uint32_t BUS_TIMEOUT_MS = 200;
HardwareSerial rs485(1);
struct BusPeer {
  bool seen = false;
  uint32_t lastSeen = 0;
  uint16_t regs[BUS_REG_COUNT] = {};
  uint8_t commandStatus = 0; // 0 none, 1 queued, 2 accepted, 3 rejected, 4 unknown/timeout
};
String peerJson(uint8_t id, const BusPeer &p);
bool decodeBusCommand(const uint8_t *p, size_t n, Command &c);
struct BusCommand { uint8_t node; Command command; uint32_t queuedAt; uint32_t epoch; };
BusPeer peers[BUS_LAST_NODE + 1];
uint16_t localRegs[BUS_REG_COUNT] = {};
QueueHandle_t busCommandQueue;
bool sharedPrimary=false, sharedCandidate=false;
uint8_t sharedLeader=0;
uint32_t sharedBusEpoch=0;
bool displayPrimary=false, displayCandidate=false;
bool sharedBusReady = false, sharedBusSeen = false;
uint32_t sharedBusLast = 0;
bool displayBusReady = false, displayBusOnline = false;
uint32_t busLastValid = 0;
bool busSeen = false;

uint16_t mbCrc(const uint8_t *data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
  }
  return crc;
}
uint16_t mbWord(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }
void mbPut(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
bool mbValid(const uint8_t *p, size_t n) {
  return n >= 4 && mbCrc(p, n - 2) == (uint16_t(p[n-1]) << 8 | p[n-2]);
}
void mbSend(uint8_t *p, size_t n) {
  const uint16_t crc = mbCrc(p, n);
  p[n++] = crc; p[n++] = crc >> 8;
  rs485.write(p, n);
  rs485.flush(true); // TX only; hardware RTS releases DE after the last stop bit
}

// Called only in loop(), then copied under the short snapshot lock.
void buildBusSnapshot(uint16_t *r) {
  memset(r, 0, BUS_REG_COUNT * sizeof(uint16_t));
  r[0] = 0x4658; r[1] = 3;
  uint64_t mac = ESP.getEfuseMac();
  for (int i=0; i<3; ++i) r[2+i] = mac >> (16*(2-i));
  r[5] = uint16_t(runState); r[6] = uint16_t(stopReason);
  for (int i=0; i<4; ++i) r[7+i] = stepCount >> (16*(3-i));
  r[11] = stepTarget >> 16; r[12] = stepTarget;
  r[13] = sensorStable | (stopAuxStable << 1) | (framAvailable << 2) |
          (inputsHealthy << 3) | (WIRING_VERIFIED << 4) | (completionLatched << 5);
  for (int i=0; i<4; ++i) r[14] |= uint16_t(relayCommanded[i]) << i;
  // v10 optional capabilities in previously unused high bits; legacy masks unchanged.
  r[13] |= (uint16_t(CSAFE_ENABLED)<<6) | (uint16_t(csafe.online(millis()))<<7) |
           (uint16_t(csafe.startPending())<<8) | (uint16_t(csafeCanStart())<<9) |
           (uint16_t(csafe.result)<<10);
  r[14] |= uint16_t(csafe.status)<<8;
  r[15] = maxScanGap >> 16; r[16] = maxScanGap;
  for (int i=0; i<3; ++i) { r[17+i*2] = PRESETS[i] >> 16; r[18+i*2] = PRESETS[i]; }
  r[23] = presetIndex;
  // Last operator result, up to 40 printable ASCII characters.
  for (size_t i=0; i<40 && i<resetMessage.length(); ++i)
    r[24+i/2] |= uint16_t(uint8_t(resetMessage[i])) << ((i%2) ? 0 : 8);
  r[44] = NO_PULSE_FAULT_MS / 1000;
  r[45] = ARM_WAIT_TIMEOUT_MS / 1000;
  r[46] = resetMessageError;
  r[48]=MOTOR_ENABLED | (MOTOR_COMMISSIONED<<1) | (motor.ready(millis())<<2) |
    (motorCanStart()<<3) | (motorStopping<<4) | (motorStarted<<5) | (motor.feedbackValid<<6);
  r[49]=selectedSpeedX100;r[50]=OPERATING_MAX_X100;r[51]=motor.actualRaw;
  r[52]=motor.reportedTargetRaw;r[53]=motor.requestedRaw();r[54]=motor.speedStatus;r[55]=unsigned(motor.phase);
  r[47] = BOARD_CODE; // 0 in legacy firmware, 1 StampPLC, 2 Waveshare
}

// Pure validation before a remote command reaches the hardware-owning loop.
bool decodeBusCommand(const uint8_t *p, size_t n, Command &c) {
  if (n != 15 || p[1] != 16 || mbWord(p+2) != 256 || mbWord(p+4) != 3 || p[6] != 6)
    return false;
  const uint16_t op = mbWord(p+7);
  const uint32_t value = (uint32_t(mbWord(p+9)) << 16) | mbWord(p+11);
  if ((op < uint16_t(CommandKind::PAUSE) || op > uint16_t(CommandKind::TARGET)) &&
      op != uint16_t(CommandKind::CSAFE_START) && op != uint16_t(CommandKind::CLEAR_FAULT) && op != uint16_t(CommandKind::SPEED)) return false;
  if (op == uint16_t(CommandKind::TARGET) && (value == 0 || value > MAX_TARGET)) return false;
  if (op == uint16_t(CommandKind::SPEED) && value > OPERATING_MAX_X100) return false;
  if (op != uint16_t(CommandKind::TARGET) && op != uint16_t(CommandKind::SPEED) && value != 0) return false;
  c = {CommandKind(op), value};
  return true;
}

void busSecondaryFrame(const uint8_t *p, size_t n) {
  if (!mbValid(p,n) || p[0] != NODE_ID) return; // No broadcast writes.
  uint8_t out[2*BUS_REG_COUNT+5] = {NODE_ID, p[1]};
  uint8_t error = 1;
  if (p[1] == 3 && n == 8) {
    const uint16_t start = mbWord(p+2), count = mbWord(p+4);
    error = 2;
    if (count && start < BUS_REG_COUNT && count <= BUS_REG_COUNT-start) {
      uint16_t r[BUS_REG_COUNT];
      xSemaphoreTake(snapshotMutex, portMAX_DELAY);
      memcpy(r, localRegs, sizeof(r));
      xSemaphoreGive(snapshotMutex);
      out[2] = count*2;
      for (uint16_t i=0; i<count; ++i) mbPut(out+3+2*i, r[start+i]);
      mbSend(out, 3+count*2);
      busSeen = true; busLastValid = millis();
      return;
    }
  } else if (p[1] == 16) {
    Command c;
    error = 3;
    if (decodeBusCommand(p,n,c)) {
      c.deadline = millis() + 1000;
      const bool urgentStop=c.kind==CommandKind::STOP || c.kind==CommandKind::PAUSE || (c.kind==CommandKind::SPEED && c.value==0);
  if ((urgentStop?xQueueSendToFront(commandQueue,&c,0):xQueueSend(commandQueue, &c, 0)) != pdTRUE) error = 6;
      else {
        memcpy(out,p,6); mbSend(out,6);
        busSeen = true; busLastValid = millis();
        return; // ACK means queued; local guards decide whether it executes.
      }
    }
  }
  out[1] |= 0x80; out[2] = error; mbSend(out,3);
}

// BEGIN ELECTION ENGINE -- pure state machine, exercised by host tests.
class BusElection {
 public:
  enum Action : uint8_t { NONE=0, CLAIM=1, HEARTBEAT=2 };
  static constexpr uint32_t LEASE_MS=6000, CLAIM_LEASE_MS=1800, SETTLE_MS=700;
  uint8_t id=0;
  bool primary=false, candidate=false;
  struct Seen { bool valid=false, primary=false; uint32_t at=0; } seen[9];
  uint32_t lastTrafficAt=0, scheduledAt=0, claimAt=0, announceAt=0, interval=700;
  bool scheduled=false;
  void begin(uint8_t address,uint32_t now) {
    id=address; primary=candidate=scheduled=false; lastTrafficAt=now;
    for(auto &s:seen)s=Seen{};
  }
  void observe(uint8_t address,bool isPrimary,uint32_t now) {
    if(address<1 || address>8 || address==id)return;
    seen[address]={true,isPrimary,now};
    if(isPrimary)lastTrafficAt=now;
  }
  // Valid polling traffic also holds off takeover, including during upgrades.
  void observePoll(uint32_t now) { lastTrafficAt=now; }
  uint8_t leader(uint32_t now) const {
    uint8_t best=primary?id:0;
    for(uint8_t n=1;n<=8;++n)if(seen[n].valid && seen[n].primary &&
        now-seen[n].at<LEASE_MS && (!best || n<best))best=n;
    return best;
  }
  Action tick(uint32_t now,uint32_t entropy) {
    bool higherPrimary=false;
    for(uint8_t n=1;n<=8;++n) {
      auto &s=seen[n];
      if(s.valid && now-s.at >= (s.primary?LEASE_MS:CLAIM_LEASE_MS))s.valid=false;
      if(!s.valid)continue;
      if(n<id) {
        primary=candidate=scheduled=false;
        return NONE;
      }
      if(s.primary)higherPrimary=true;
    }
    if(primary) {
      if(now-announceAt>=interval) {
        announceAt=now; interval=650+entropy%250; return HEARTBEAT;
      }
      return NONE;
    }
    if(candidate) {
      if(now-claimAt>=SETTLE_MS) {
        candidate=false;primary=true;announceAt=now;interval=650+entropy%250;
        return HEARTBEAT;
      }
      if(now-announceAt>=250) { announceAt=now;return CLAIM; }
      return NONE;
    }
    if(!higherPrimary && now-lastTrafficAt<LEASE_MS) { scheduled=false;return NONE; }
    if(!scheduled) { scheduled=true;scheduledAt=now+100*id+entropy%80; }
    if(int32_t(now-scheduledAt)>=0) {
      scheduled=false;candidate=true;claimAt=announceAt=now;return CLAIM;
    }
    return NONE;
  }
};
// END ELECTION ENGINE

void busTask(void *) {
  if (!RS485_ENABLED) { vTaskDelete(nullptr); return; }
  rs485.setRxBufferSize(256);
  rs485.begin(BUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  bool ready = bool(rs485) && rs485.setPins(RS485_RX, RS485_TX, -1, RS485_DE) &&
               rs485.setMode(UART_MODE_RS485_HALF_DUPLEX);
  rs485.setRxFIFOFull(1);
  xSemaphoreTake(snapshotMutex, portMAX_DELAY); sharedBusReady=ready; xSemaphoreGive(snapshotMutex);
  if (!ready) { Serial.println("RS485 UART init failed"); vTaskDelete(nullptr); return; }
  BusElection election; election.begin(NODE_ID,millis());
  uint8_t rx[128],request[16];size_t used=0;
  bool overflow=false,pending=false,writing=false,wasPrimary=false;
  uint8_t destination=0,scan=1,announcement=0;
  uint32_t lastByte=millis(),sentAt=0,nextPoll=millis();
  for (;;) {
    unsigned budget=256;
    while(budget-- && rs485.available()) {
      int b=rs485.read();if(used<sizeof(rx))rx[used++]=uint8_t(b);else overflow=true;
      lastByte=millis();
    }
    const uint32_t now=millis();
    if((used || overflow) && now-lastByte>=4) {
      if(!overflow && mbValid(rx,used)) {
        if(used==9 && rx[0]==0 && rx[1]==65 && rx[2]==0x46 && rx[3]==0x58 &&
           rx[4]==2 && rx[5]>=1 && rx[5]<=BUS_LAST_NODE && (rx[6]==1 || rx[6]==2)) {
          election.observe(rx[5],rx[6]==BusElection::HEARTBEAT,now);
          if(rx[5]!=NODE_ID && rx[6]==BusElection::HEARTBEAT) { busSeen=true;busLastValid=now; }
        } else {
          if(used==8 && rx[0]>=1 && rx[0]<=BUS_LAST_NODE && rx[1]==3 &&
             mbWord(rx+2)==0 && mbWord(rx+4)==BUS_REG_COUNT)election.observePoll(now);
          if(!election.primary)busSecondaryFrame(rx,used);
          else if(pending && rx[0]==destination) {
            bool accepted=false;uint16_t r[BUS_REG_COUNT];
            if(!writing && used==5+2*BUS_REG_COUNT && rx[1]==3 && rx[2]==2*BUS_REG_COUNT) {
              for(int i=0;i<BUS_REG_COUNT;++i)r[i]=mbWord(rx+3+2*i);
              accepted=r[0]==0x4658 && r[1]==3 && r[5]<=uint16_t(RunState::FAULT) && r[6]<=uint16_t(StopReason::MOTOR_LINK);
              if(accepted) {
                xSemaphoreTake(snapshotMutex,portMAX_DELAY);
                peers[destination].seen=true;peers[destination].lastSeen=now;
                memcpy(peers[destination].regs,r,sizeof(r));xSemaphoreGive(snapshotMutex);
              }
            } else if(writing && used==8 && rx[1]==16 && mbWord(rx+2)==256 && mbWord(rx+4)==3) {
              accepted=true;
              xSemaphoreTake(snapshotMutex,portMAX_DELAY);peers[destination].commandStatus=2;xSemaphoreGive(snapshotMutex);
              scan=destination;nextPoll=now;
            } else if(used==5 && rx[1]==uint8_t((writing?16:3)|0x80)) {
              pending=false;
              if(writing) { xSemaphoreTake(snapshotMutex,portMAX_DELAY);peers[destination].commandStatus=3;xSemaphoreGive(snapshotMutex); }
            }
            if(accepted) { pending=false;busSeen=true;busLastValid=now; }
          }
        }
      }
      used=0;overflow=false;
    }
    auto action=election.tick(now,esp_random());
    if(election.primary!=wasPrimary) {
      // Old UI requests must never replay after a future promotion.
      xSemaphoreTake(snapshotMutex,portMAX_DELAY);
      if(pending && writing)peers[destination].commandStatus=4;
      for(auto &p:peers)if(p.commandStatus==1)p.commandStatus=3;
      ++sharedBusEpoch;
      xSemaphoreGive(snapshotMutex);
      BusCommand discarded;while(xQueueReceive(busCommandQueue,&discarded,0)==pdTRUE) {}
      pending=false;scan=1;nextPoll=now;wasPrimary=election.primary;
    }
    if(!election.primary && !election.candidate)announcement=0;
    if(action!=BusElection::NONE)announcement=uint8_t(action);
    if(pending && now-sentAt>=BUS_TIMEOUT_MS) {
      if(writing) { xSemaphoreTake(snapshotMutex,portMAX_DELAY);peers[destination].commandStatus=4;xSemaphoreGive(snapshotMutex); }
      pending=false;
    }
    if(!pending && !used && !overflow && now-lastByte>=4) {
      if(announcement) {
        uint8_t hello[9]={0,65,0x46,0x58,2,NODE_ID,announcement,0,0};
        mbSend(hello,7);announcement=0;lastByte=millis();
      } else if(election.primary) {
        BusCommand bc;bool send=false;size_t len=0;
        if(xQueueReceive(busCommandQueue,&bc,0)==pdTRUE) {
          destination=bc.node;writing=true;
          xSemaphoreTake(snapshotMutex,portMAX_DELAY);
          const bool online=peers[destination].seen && now-peers[destination].lastSeen<BUS_OFFLINE_MS;
          const bool current=bc.epoch==sharedBusEpoch;
          if(!online || !current || now-bc.queuedAt>1500)peers[destination].commandStatus=3;
          xSemaphoreGive(snapshotMutex);
          if(online && current && now-bc.queuedAt<=1500) {
            request[0]=destination;request[1]=16;mbPut(request+2,256);mbPut(request+4,3);request[6]=6;
            mbPut(request+7,uint16_t(bc.command.kind));mbPut(request+9,bc.command.value>>16);mbPut(request+11,bc.command.value);
            len=13;send=true;
          }
        } else if(int32_t(now-nextPoll)>=0) {
          if(scan==NODE_ID && ++scan>BUS_LAST_NODE)scan=1;
          destination=scan;if(++scan>BUS_LAST_NODE)scan=1;
          writing=false;request[0]=destination;request[1]=3;mbPut(request+2,0);mbPut(request+4,BUS_REG_COUNT);
          len=6;send=true;nextPoll=now+250;
        }
        if(send) { mbSend(request,len);pending=true;sentAt=millis();lastByte=sentAt; }
      }
    }
    xSemaphoreTake(snapshotMutex,portMAX_DELAY);
    sharedBusSeen=busSeen;sharedBusLast=busLastValid;
    sharedPrimary=election.primary;sharedCandidate=election.candidate;sharedLeader=election.leader(now);
    xSemaphoreGive(snapshotMutex);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

String jsonQuoted(const String &s) {
  String out = "\"";
  for (size_t i=0; i<s.length(); ++i) {
    const uint8_t c=s[i];
    if (c=='"' || c=='\\') { out+='\\'; out+=char(c); }
    else if (c>=32 && c<127) out+=char(c);
    else out+='?';
  }
  return out+'"';
}
String peerJson(uint8_t id, const BusPeer &p) {
  const uint16_t *r=p.regs;
  uint64_t count=0,mac=0;
  for(int i=0;i<4;++i) count=(count<<16)|r[7+i];
  for(int i=0;i<3;++i) mac=(mac<<16)|r[2+i];
  char name[18]; snprintf(name,sizeof(name),"flux-%012llX",(unsigned long long)mac);
  String message;
  for(int i=0;i<40;++i) { char c=r[24+i/2]>>((i%2)?0:8); if(!c)break; message+=c; }
  const uint32_t age=millis()-p.lastSeen;
  String j="{\"node\":"+String(id)+",\"online\":"+String(age<BUS_OFFLINE_MS?"true":"false")+
    ",\"age_ms\":"+String(age)+",\"device\":"+jsonQuoted(name)+
    ",\"state\":"+jsonQuoted(stateName(RunState(r[5])))+",\"reason\":"+jsonQuoted(reasonName(StopReason(r[6])))+
    ",\"count\":"+u64(count)+",\"target\":"+String((uint32_t(r[11])<<16)|r[12]);
  j+=",\"board_code\":"+String(r[47]);
  j+=",\"csafe_enabled\":"+String((r[13]&(1<<6))?"true":"false");
  j+=",\"csafe_online\":"+String((r[13]&(1<<7))?"true":"false");
  j+=",\"csafe_start_pending\":"+String((r[13]&(1<<8))?"true":"false");
  j+=",\"csafe_can_start\":"+String((r[13]&(1<<9))?"true":"false");
  j+=",\"csafe_result\":"+String((r[13]>>10)&3);
  j+=",\"csafe_status\":"+String(r[14]>>8);
  j+=",\"motor_enabled\":"+String((r[48]&1)?"true":"false");
  j+=",\"motor_commissioned\":"+String((r[48]&2)?"true":"false");
  j+=",\"motor_ready\":"+String((r[48]&4)?"true":"false");
  j+=",\"motor_can_start\":"+String((r[48]&8)?"true":"false");
  j+=",\"motor_stopping\":"+String((r[48]&16)?"true":"false");
  j+=",\"motor_started\":"+String((r[48]&32)?"true":"false");
  j+=",\"motor_feedback_valid\":"+String((r[48]&64)?"true":"false");
  j+=",\"speed_mph\":"+String(r[49]/100.0f,2)+",\"max_speed_mph\":"+String(r[50]/100.0f,2);
  j+=",\"actual_mph\":"+String(r[51]/96.0f,3)+",\"reported_target_mph\":"+String(r[52]/96.0f,3);
  j+=",\"motor_requested_raw\":"+String(r[53])+",\"motor_status_raw\":"+String(r[54])+",\"motor_phase\":"+String(r[55]);
  const char *flags[]={"sensor","stop_button","fram","inputs_healthy","wiring_verified","completed"};
  for(int i=0;i<6;++i) j+=",\""+String(flags[i])+"\":"+String((r[13]&(1<<i))?"true":"false");
  j+=",\"relays\":"+String(r[14])+",\"max_scan_gap_ms\":"+String((uint32_t(r[15])<<16)|r[16])+",\"presets\":[";
  for(int i=0;i<3;++i) { if(i)j+=','; j+=String((uint32_t(r[17+2*i])<<16)|r[18+2*i]); }
  j+="],\"reset_message\":"+jsonQuoted(message)+",\"command_status\":"+String(p.commandStatus)+
    ",\"no_pulse_seconds\":"+String(r[44])+",\"arm_timeout_seconds\":"+String(r[45])+"}";
  return j;
}
void sendNodes() {
  if (!authenticated()) return;
  BusPeer copy[BUS_LAST_NODE+1]; String local; bool ready,primary,candidate; uint8_t leader;
  xSemaphoreTake(snapshotMutex,portMAX_DELAY);
  memcpy(copy,peers,sizeof(copy)); local=sharedStatus; ready=sharedBusReady;
  primary=sharedPrimary;candidate=sharedCandidate;leader=sharedLeader;
  xSemaphoreGive(snapshotMutex);
  String j="{\"local_node\":"+String(NODE_ID)+",\"primary\":"+String(primary?"true":"false")+
    ",\"leader\":"+String(leader)+",\"candidate\":"+String(candidate?"true":"false")+
    ",\"bus_enabled\":"+String(RS485_ENABLED?"true":"false")+
    ",\"bus_ready\":"+String(ready?"true":"false")+",\"nodes\":["+local;
  if(primary) for(uint8_t i=1;i<=BUS_LAST_NODE;++i) if(i!=NODE_ID && copy[i].seen) j+=","+peerJson(i,copy[i]);
  j+="]}";
  server.sendHeader("Cache-Control","no-store"); server.send(200,"application/json",j);
}

// A single packed NVS value makes role/address updates atomic.
bool validBusConfig(uint32_t packed) {
  const uint32_t role=packed>>8, id=packed&255;
  return packed==0 || (role==1 && id==1) || (role==2 && id>=2 && id<=8);
}
void loadBusSettings() {
  Preferences p; uint32_t packed=0;
  if(p.begin("flux-bus",true)) { packed=p.getUInt("config",0); p.end(); }
  if(!validBusConfig(packed)) packed=0;
  RS485_ENABLED=packed!=0; PREFERRED_PRIMARY=(packed>>8)==1; NODE_ID=packed&255;
}
// Only the hardware-owning loop calls this. A successful save holds permit
// OFF and blocks local arm until reboot, closing the save/re-arm race.
uint8_t applyBusSettings(uint32_t packed) {
  if(!validBusConfig(packed)) return 2;
  if(busConfigRestartPending) return 4;
  if(runState==RunState::RUNNING || runState==RunState::ARMED || millis()-lastObservedEdge<3000) return 1;
  setTreadmillStopped(true);
  Preferences p;
  if(!p.begin("flux-bus",false)) return 3;
  bool ok=p.putUInt("config",packed)==sizeof(uint32_t);
  p.end();
  if(!ok) return 3;
  busConfigRestartPending=true;
  busConfigRestartAt=millis()+1500;
  reportReset("Bus saved; restarting",false);
  return 0;
}

const char BUS_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RS-485 settings</title>
<style>body{font-family:system-ui;background:#15181d;color:#eee;max-width:560px;margin:2rem auto;padding:0 1rem}label{display:block;margin-top:1rem}input,select,button{font:inherit;padding:.7rem;margin:.4rem 0}a{color:#7fd3ff}</style>
</head><body><h1>RS-485 settings</h1><p id="device">Loading…</p><p>These settings apply to this physical unit. Use the firmware build matching this board. Role and address are independent of board type.</p>
<form id="form"><label>Role <select id="role" onchange="roleChanged()"><option value="0">Standalone — RS-485 off</option><option value="1">Preferred primary — address 1</option><option value="2">Secondary — automatic failover</option></select></label>
<label>Node address <input id="address" type="number" min="2" max="8" step="1" value="2" required></label>
<p>Use one preferred primary at address 1. Give each secondary a different address from 2 through 8. The lowest available address is elected primary; secondary Wi-Fi is off.</p>
<p>Stop this treadmill and wait for sensor activity to settle before saving. Saving restarts this unit; it does not reset the count or arm the treadmill. A secondary will disconnect this Wi-Fi session. Disconnect its RS-485 cable to let it become primary for later configuration.</p>
<button id="save" disabled>Save and restart</button></form><p id="result" role="status"></p><p><a href="/">Back to dashboard</a></p>
<script>
const el=id=>document.getElementById(id);
function roleChanged(){const role=Number(el('role').value);el('address').disabled=role!==2;if(role!==2)el('address').value=role===1?1:0;else if(Number(el('address').value)<2)el('address').value=2;}
async function load(){try{const r=await fetch('/api/bus');if(!r.ok)throw Error(r.status);const s=await r.json();el('device').textContent=s.device;el('role').value=s.role;el('address').value=s.node;roleChanged();el('save').disabled=false;}catch(e){el('result').textContent='Could not load settings. Reload this page.'}}
el('form').onsubmit=async e=>{e.preventDefault();if(!confirm('Save RS-485 settings and restart this stopped unit?'))return;el('save').disabled=true;
try{const r=await fetch('/api/bus?role='+el('role').value+'&node='+el('address').value,{method:'POST',headers:{'X-Flux-Control':'1'}});el('result').textContent=await r.text();if(r.ok)setTimeout(()=>location.href='/',7000);else el('save').disabled=false;}catch(e){el('result').textContent='Connection lost; reload and check saved settings before retrying.';el('save').disabled=false;}};
load();
</script></body></html>
)HTML";

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flux Capacitor</title><style>
body{font-family:system-ui;background:#15181d;color:#eee;max-width:800px;margin:2rem auto;padding:0 1rem}
.card{background:#222831;border-radius:12px;padding:1.2rem;margin:1rem 0}.count{font-size:2.7rem;font-weight:700}
button,input{font:inherit;padding:.7rem;margin:.25rem;border-radius:7px;border:0}button{cursor:pointer}
button:disabled{opacity:.4;cursor:default}.stop{background:#e34b4b}.muted{color:#adb5bd}
.row{display:flex;flex-wrap:wrap;gap:.4rem}[aria-selected=true]{outline:3px solid #7fd3ff}a{color:#7fd3ff}
dt{color:#adb5bd}dd{margin:0 0 .7rem}#notice{min-height:1.5rem;color:#ffd47f}
</style></head><body><h1>Flux Capacitor</h1><p id="bus" class="muted">Connecting…</p>
<div id="tabs" class="row" role="tablist" aria-label="Treadmill nodes"></div>
<main id="panel" role="tabpanel"><div class="card"><div id="identity"></div><h2 id="state">Loading…</h2>
<div class="count"><span id="count">—</span> / <span id="target">—</span></div><p id="reason"></p><p id="freshness" class="muted"></p></div>
<div class="card"><p id="armHelp">Arm locally. Reset does not arm the treadmill.</p>
<div class="row"><button id="csafeStart" data-control onclick="startCsafeNode()">Start belt</button><button data-control onclick="cmd('pause')">Pause and stop</button><button data-control class="stop" onclick="cmd('stop')">Stop</button>
<button id="clearFault" data-control onclick="cmd('clear-fault')">Clear fault (keep count)</button><button id="silence" data-control onclick="cmd('ack')">Silence buzzer</button><button data-control onclick="resetNode()">Reset count</button></div>
<p id="notice" role="status"></p></div>
<div id="motorPanel" class="card"><h2>Speed control</h2>
<p id="motorStatus"></p><p id="speedReadback"></p>
<form onsubmit="setSpeed(event)"><label for="speedValue">Speed preset</label>
<select id="speedValue" data-control onchange="updateSpeedEditor()"><option value="2.8">Walk — 2.8 mph</option><option value="3.8">Run — 3.8 mph</option><option value="custom">Custom</option></select>
<div id="customSpeedRow" hidden><label for="customSpeed">Custom speed (mph, up to 5)</label>
<input id="customSpeed" data-control type="number" min="0.01" max="5" step="0.01" value="2.8" disabled></div>
<button id="saveSpeed" data-control>Set speed</button></form>
<p>Choose Walk, Run, or Custom (up to 5 mph), then press Set speed. While stopped this only stages the speed; arm locally, then Start belt. While running it changes speed. Use Stop to stop.</p></div>
<div class="card"><h2>Node settings</h2><form onsubmit="setTarget(event)"><label for="newtarget">Target steps</label>
<input id="newtarget" type="number" min="1" max="1000000000" step="1" required><button id="saveTarget" data-control>Save target</button></form>
<p id="settings" class="muted"></p><p>Local edit increments, legacy presets and timeouts are configured in the sketch.</p></div>
<div class="card"><h2>Inputs and outputs</h2><dl id="details"></dl><p class="muted">Relay indicators show commanded coils, not contact feedback. Network Stop is not an emergency-stop circuit.</p></div></main>
<p><a href="/bus">RS-485 role and address</a> · <a href="/wifi">Wi-Fi settings</a> · <a href="/update">Firmware update</a> — for this web-hosting unit</p>
<script>
const el=id=>document.getElementById(id);let nodes=[],selected=null,host=null,fetching=false,stale=true;
const selectedNode=()=>nodes.find(n=>n.node===selected);
function render(){const s=selectedNode();if(!s)return;
el('tabs').replaceChildren();for(const n of nodes){let b=document.createElement('button');b.textContent=n.node===0?'Local (standalone)':`Node ${n.node}${n.node===host?' (local)':''}${n.online?'':' — OFFLINE'}`;
b.setAttribute('role','tab');b.setAttribute('aria-selected',String(n.node===selected));b.onclick=()=>{selected=n.node;el('newtarget').value='';el('speedValue').dataset.node='';el('notice').textContent='';render()};el('tabs').append(b)}
const online=s.online&&!stale;
el('silence').hidden=s.board_code===2;

el('armHelp').textContent=s.board_code===2?'Hold the local Arm/Pause button for 1 second, then release. Reset does not arm it.':'Hold B for 1 second, then release to arm/resume. Reset does not arm it. Hold A for 1 second to clear a fault without resetting the count.';
if(s.motor_enabled)el('armHelp').textContent+=' Short A/C adjust the highlighted setting; short B switches speed/steps while stopped or speed/count view while armed/running. Wait for zero-speed motor feedback, then hold B again or press Start belt. A subsequent B hold stops. Release/reset never restarts the belt.';
if(s.csafe_motor_control)el('armHelp').textContent+=' Hold B again to start, or use Start via CSAFE. After a Start request, hold B to stop. Reset and E-stop release never start the belt.';
el('identity').textContent=`Node ${s.node} · ${s.device}`;
el('state').textContent=online?s.state:'OFFLINE — last known state: '+s.state;
el('count').textContent=s.count.toLocaleString();el('target').textContent=s.target.toLocaleString();
el('reason').textContent=s.reason+(s.reset_message?' | '+s.reset_message:'');
const transport=['','Command queued','Command accepted by node; check state/result','Command rejected / expired','No acknowledgement; outcome unknown. Check node before retrying.'];
el('freshness').textContent=(online?'Updated':'STALE')+` · sample age ${Math.round(s.age_ms/1000)}s`+(transport[s.command_status]?' · '+transport[s.command_status]:'');
el('settings').textContent=(s.motor_enabled?'Standalone A/B/C settings editor':`Legacy presets A/B/C: ${s.presets.join(' / ')}`)+` · No-pulse timeout: ${s.no_pulse_seconds}s · Arm timeout: ${s.arm_timeout_seconds}s`;
el('details').replaceChildren();const details=[['Optical input',s.sensor?'ACTIVE':'inactive'],['E-stop',s.stop_button?'PRESSED':'released'],['Input hardware',s.board_code===2?'GPIO: wiring not supervised':s.inputs_healthy?'OK':'FAULT'],['FRAM',s.fram?'OK':'FAULT'],['Wiring check',s.wiring_verified?'Enabled':'BENCH LOCK'],['Maximum scan gap',s.max_scan_gap_ms+' ms']];
(s.board_code===2?['Interrupt permit']:['Interrupt permit','Green','Red','Buzzer (brown)']).forEach((name,i)=>details.push([`Relay ${i+1}: ${name}`,s.relays&(1<<i)?'ON':'OFF']));
if(s.csafe_enabled)details.push(['CSAFE link',s.csafe_online?'Responding':'No recent response'],['CSAFE protocol state',String(s.csafe_status&15)],['CSAFE command',s.csafe_start_pending?'Start pending':(['None','Pending','Response received (not belt confirmation)','Failed / timed out'][s.csafe_result]||'Unknown')]);
for(const [k,v] of details){let dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;el('details').append(dt,dd)}
document.querySelectorAll('[data-control]').forEach(b=>b.disabled=!online);
el('clearFault').disabled=!online||s.state!=='FAULT';
el('csafeStart').hidden=!s.motor_enabled&&!s.csafe_motor_control;
el('csafeStart').disabled=!online||!(s.motor_enabled?s.motor_can_start:s.csafe_can_start);
el('motorPanel').hidden=!s.motor_enabled;
el('motorStatus').textContent=!s.motor_commissioned?'Motor control locked: commission BoardConfig.h':s.motor_stopping?'Stopping: waiting for zero feedback':s.motor_ready?'Protocol responding (unknown status flags remain unverified)':'Motor link inactive or initializing; arm locally';
el('speedReadback').textContent=`Selected ${Number(s.speed_mph||0).toFixed(2)} mph · feedback ${s.motor_feedback_valid?Number(s.actual_mph).toFixed(2)+' mph':'unavailable'} · reported target ${s.motor_feedback_valid?Number(s.reported_target_mph).toFixed(2)+' mph':'unavailable'}`;
if(el('speedValue').dataset.node!==String(selected)) {
  const speed=Number(s.speed_mph||2.8);
  el('speedValue').value=speed===2.8?'2.8':speed===3.8?'3.8':'custom';
  el('customSpeed').value=String(speed);
  el('speedValue').dataset.node=String(selected);
}
const speedCap=Math.min(5,Number(s.max_speed_mph||0));
el('customSpeed').max=String(speedCap);
for(const option of el('speedValue').options)option.disabled=option.value==='custom'?speedCap<=0:Number(option.value)>speedCap;
updateSpeedEditor();
el('saveSpeed').disabled=!online||!s.motor_enabled||!s.motor_commissioned||s.motor_stopping||s.state==='FAULT';
el('saveTarget').disabled=!online||s.completed||['RUNNING','ARMED'].includes(s.state);
}
async function cmd(action,value){const id=selected,s=selectedNode();if(!s||!s.online||stale)return;
try{const r=await fetch(`/api/${action}?node=${id}`+(value===undefined?'':'&value='+encodeURIComponent(value)),{method:'POST',headers:{'X-Flux-Control':'1'}});
const message=await r.text();if(id===selected)el('notice').textContent=`Node ${id}: ${message}`;await refresh();}catch(e){el('notice').textContent=`Node ${id}: connection lost; command outcome unknown.`}}
function startCsafeNode(){if(confirm(`Start the belt on Node ${selected}? Ensure the treadmill is clear.`))cmd('start')}
function resetNode(){if(confirm(`Reset Node ${selected}'s step count? The treadmill must be stopped.`))cmd('reset')}
function updateSpeedEditor(){
const custom=el('speedValue').value==='custom';
el('customSpeedRow').hidden=!custom;
el('customSpeed').disabled=!custom||el('speedValue').disabled;
el('customSpeed').required=custom;
}
function setSpeed(e){e.preventDefault();
const value=el('speedValue').value==='custom'?el('customSpeed').value:el('speedValue').value;
const speed=Number(value),cap=Math.min(5,Number(selectedNode()?.max_speed_mph||0));
if(!Number.isFinite(speed)||speed<=0||speed>cap){el('notice').textContent=`Enter a speed above 0 and no more than ${cap} mph.`;return;}
cmd('speed',value);
}
function setTarget(e){e.preventDefault();cmd('target',el('newtarget').value)}
async function refresh(){if(fetching)return;fetching=true;try{const r=await fetch('/api/nodes',{signal:AbortSignal.timeout(4000)});if(!r.ok)throw Error(r.status);
const data=await r.json();nodes=data.nodes;host=data.local_node;if(!nodes.some(n=>n.node===selected))selected=host;stale=false;
el('bus').textContent=data.bus_enabled?`${data.primary?'Primary':data.candidate?'Electing primary':'Secondary'} · address ${host} · RS-485 ${data.bus_ready?'enabled, 19200 baud':'unavailable'}${data.primary?' · discovering addresses 1–8':' · primary '+(data.leader||'pending')}`:'Standalone · RS-485 disabled — use RS-485 role and address below to configure';
render();}catch(e){stale=true;el('bus').textContent='Web connection lost — controls disabled';render()}finally{fetching=false}}
setInterval(refresh,1000);refresh();
</script></body></html>
)HTML";

const char WIFI_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wi-Fi Settings</title><style>
body{font-family:system-ui;background:#15181d;color:#eee;max-width:560px;margin:2rem auto;padding:0 1rem}
.card{background:#222831;border-radius:12px;padding:1.2rem;margin-bottom:1rem}
label{display:block;margin-top:1rem}.hint{color:#adb5bd;font-size:.9rem}
input,button{box-sizing:border-box;font:inherit;padding:.7rem;margin-top:.35rem;border-radius:7px;border:0}
input,select{width:100%}button{cursor:pointer;background:#35b66a}.danger{background:#e34b4b;color:white}
a{color:#7fd3ff}
</style></head><body><h1>Wi-Fi Settings</h1>
<div class="card"><form method="post" action="/api/wifi">
<label>Network name SSID<input name="ssid" maxlength="32" required autocomplete="off"></label>
<label>Password<input name="password" type="password" maxlength="63" autocomplete="new-password"></label>
<p class="hint">Use a blank password only for an open network. After saving, Wi-Fi reconnects to this network without resetting the count. Its new address appears on the display.</p>
<button type="submit">Save and connect</button></form></div>
<div class="card"><form method="post" action="/api/wifi/clear" onsubmit="return confirm('Forget the saved Wi-Fi network?')">
<button class="danger" type="submit">Forget network and use access point</button></form></div>
<p><a href="/">Back to controller</a></p></body></html>
)HTML";

bool authenticated() {
  if (server.authenticate("admin", WEB_PASSWORD)) return true;
  server.requestAuthentication();
  return false;
}

void loadWifiSettings() {
  Preferences preferences;
  if (preferences.begin("flux-wifi", true)) {
    configuredWifiSsid = preferences.getString("ssid", WIFI_SSID);
    configuredWifiPassword = preferences.getString("password", WIFI_PASSWORD);
    preferences.end();
  } else {
    configuredWifiSsid = WIFI_SSID;
    configuredWifiPassword = WIFI_PASSWORD;
  }
}

bool saveWifiSettings(const String &ssid, const String &password) {
  Preferences preferences;
  if (!preferences.begin("flux-wifi", false)) return false;
  const bool ok = preferences.putString("ssid", ssid) == ssid.length() &&
                  preferences.putString("password", password) == password.length();
  preferences.end();
  if (ok) {
    configuredWifiSsid = ssid;
    configuredWifiPassword = password;
  }
  return ok;
}

void scheduleNetworkRestart() {
  networkRestartRequested = true;
  networkRestartAt = millis() + 1500;
}
void sendStatus() {
  if (!authenticated()) return;
  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  String body = sharedStatus;
  xSemaphoreGive(snapshotMutex);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", body);
}
void queueRequest(CommandKind kind, uint32_t value = 0);
void queueRequest(CommandKind kind, uint32_t value) {
  if (!authenticated()) return;
  if(otaGate.load()==2) { server.send(409,"text/plain","OTA lock: reboot before controls can be used"); return; }
  if (server.header("X-Flux-Control") != "1") {
    server.send(403, "text/plain", "Control header required"); return;
  }
  uint8_t node = NODE_ID;
  if (server.hasArg("node")) {
    String id=server.arg("node");
    if(id.length()!=1 || id[0]<'0' || id[0]>'8') {
      server.send(400,"text/plain","Invalid node address"); return;
    }
    node=id[0]-'0';
  }
  Command c{kind, value, millis()+1000};
  if(node != NODE_ID) {
    xSemaphoreTake(snapshotMutex,portMAX_DELAY);
    bool primary=sharedPrimary; uint32_t epoch=sharedBusEpoch;
    bool online=primary && node>=1 && sharedBusReady && peers[node].seen && millis()-peers[node].lastSeen<BUS_OFFLINE_MS;
    bool busy=peers[node].commandStatus==1;
    if(online && !busy) peers[node].commandStatus=1;
    xSemaphoreGive(snapshotMutex);
    if(!online) { server.send(503,"text/plain","Node offline; command not sent"); return; }
    if(busy) { server.send(409,"text/plain","A command is already pending for this node"); return; }
    BusCommand bc{node,c,millis(),epoch};
    const bool remoteStop=kind==CommandKind::STOP || kind==CommandKind::PAUSE || (kind==CommandKind::SPEED && value==0);
    if((remoteStop?xQueueSendToFront(busCommandQueue,&bc,0):xQueueSend(busCommandQueue,&bc,0))!=pdTRUE) {
      xSemaphoreTake(snapshotMutex,portMAX_DELAY); peers[node].commandStatus=3; xSemaphoreGive(snapshotMutex);
      server.send(503,"text/plain","Bus queue full; command not sent"); return;
    }
    server.send(202,"text/plain","Queued for node; check status and result"); return;
  }
  const bool urgentStop=kind==CommandKind::STOP || kind==CommandKind::PAUSE || (kind==CommandKind::SPEED && value==0);
  if ((urgentStop?xQueueSendToFront(commandQueue,&c,0):xQueueSend(commandQueue, &c, 0)) != pdTRUE) {
    server.send(503, "text/plain", "Busy; retry"); return;
  }
  server.send(202, "text/plain", "Queued; check state");
}

void serviceCommands() {
  Command c;
  // Bound work per scan; remote ARM is deliberately not enabled.
  if (xQueueReceive(commandQueue, &c, 0) != pdTRUE) return;
  if (otaGate.load()==2) return;
  const bool stopCommand=c.kind==CommandKind::STOP || c.kind==CommandKind::PAUSE || (c.kind==CommandKind::SPEED && c.value==0);
  if (!stopCommand && c.deadline && int32_t(millis()-c.deadline)>=0) {
    if(c.kind==CommandKind::BUS_CONFIG) { uint8_t result=5; xQueueSend(busConfigReplyQueue,&result,0); }
    reportReset("Command expired; retry",true); return;
  }
  if(c.kind==CommandKind::BUS_CONFIG) {
    uint8_t result=applyBusSettings(c.value);
    xQueueSend(busConfigReplyQueue,&result,0); return;
  }
  if(busConfigRestartPending) return;
  switch (c.kind) {
    case CommandKind::CSAFE_START: requestCsafeStart(); break;
    case CommandKind::SPEED: requestMotorSpeed(c.value); break;
    case CommandKind::STOP:
      requestTreadmillStop(StopReason::WEB, RunState::STOPPED); break;
    case CommandKind::PAUSE:
      requestTreadmillStop(StopReason::WEB, RunState::PAUSED); break;
    case CommandKind::ACK:
      // Silence only: never changes the RX permit or clears a fault.
      silenceExternalBeep();
      break;
    case CommandKind::CLEAR_FAULT: clearFault(); break;
    case CommandKind::RESET: resetCount(); break;
    case CommandKind::TARGET:
      if (c.value > 0 && c.value <= MAX_TARGET && framAvailable && !completionLatched &&
          runState != RunState::RUNNING && runState != RunState::ARMED) {
        stepTarget = c.value; writeFramRecord();
        reportReset(framAvailable ? "Target saved" : "Target save failed", !framAvailable);
      } else reportReset("Target change blocked",true);
      break;
    case CommandKind::ARM: case CommandKind::BUS_CONFIG: break;
  }
}

// OTA never touches NVS, FRAM, the bootloader or the partition table.
void serviceOtaGate() {
  if(otaGate.load()!=1) return;
  if(busConfigRestartPending || runState==RunState::ARMED || runState==RunState::RUNNING ||
     !inputsHealthy || millis()-lastObservedEdge<3000) {
    otaGate.store(3); return;
  }
  setTreadmillStopped(true);
  // Preserve a completed result; do not reset count or change target.
  captureCoast=false;
  if(framAvailable) {
    writeFramRecord();
    if(!framAvailable) { otaGate.store(3); return; }
  }
  silenceExternalBeep();
  reportReset("OTA LOCK - reboot to exit",true);
  otaGate.store(2);
}

bool otaRoleAllowed() {
  xSemaphoreTake(snapshotMutex,portMAX_DELAY);
  bool allowed=!RS485_ENABLED || sharedPrimary;
  xSemaphoreGive(snapshotMutex);
  return allowed;
}

void abortOta(const String &message) {
  if(otaWriting) esp_ota_abort(otaHandle);
  otaWriting=false; otaComplete=false; otaError=message;
}

bool otaAuthorized() {
  return server.authenticate("admin",WEB_PASSWORD) && server.header("X-Flux-Control")=="1";
}

// All upload state and flash APIs are owned by the web task.
void receiveOtaUpload() {
  HTTPUpload &upload=server.upload();
  if(upload.status==UPLOAD_FILE_START) {
    if(!otaAuthorized()) { otaError="Authentication/control header required"; return; }
    if(otaComplete || otaReboot.load()) { otaComplete=false; otaError="Only one firmware file is allowed"; return; }
    if(otaWriting) { abortOta("Only one firmware file is allowed"); return; }
    otaError=""; otaReceived=0; otaExpected=0; otaMarkerMatch=0; otaMarkerFound=false;
    if(!otaRoleAllowed()) { otaError="This unit is no longer primary"; return; }
    const String size=server.header("X-Flux-Size");
    if(size.length()==0 || size.length()>8) { otaError="Missing or invalid image size"; return; }
    for(size_t i=0;i<size.length();++i) {
      if(size[i]<'0'||size[i]>'9') { otaError="Invalid image size"; return; }
      otaExpected=otaExpected*10+(size[i]-'0');
    }
    const esp_partition_t *slot=esp_ota_get_next_update_partition(nullptr);
    const esp_partition_t *running=esp_ota_get_running_partition();
    if(!slot || !running || slot->address==running->address ||
       !esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_DATA_OTA,nullptr)) {
      otaError="No alternate OTA slot: install an OTA partition layout using USB"; return;
    }
    if(otaExpected<sizeof(esp_image_header_t) || otaExpected>slot->size) {
      otaError="Image is empty or larger than the installed OTA slot"; return;
    }
    if(otaGate.load()!=2) {
      otaGate.store(1);
      uint32_t start=millis();
      while(otaGate.load()==1 && millis()-start<1500) delay(1);
      if(otaGate.load()!=2) {
        // Cancel an unhandled request; never revoke a granted lock from this task.
        uint8_t requested=1; otaGate.compare_exchange_strong(requested,0);
        otaError="Update refused: stop this treadmill, wait 3 seconds, check inputs/storage"; return;
      }
    }
    if(esp_ota_begin(slot,otaExpected,&otaHandle)!=ESP_OK) {
      otaError="Cannot begin flash update; reboot before use"; return;
    }
    otaWriting=true; otaStarted=millis();
  } else if(upload.status==UPLOAD_FILE_WRITE) {
    if(!otaWriting) return;
    if(!otaAuthorized() || !otaRoleAllowed() || millis()-otaStarted>120000) {
      abortOta("Authorization/primary role lost or upload timed out"); return;
    }
    if(upload.currentSize>otaExpected-otaReceived) { abortOta("Too many bytes"); return; }
    for(size_t i=0;i<upload.currentSize && !otaMarkerFound;++i) {
      if(upload.buf[i]==OTA_PROJECT_MARKER[otaMarkerMatch]) ++otaMarkerMatch;
      else otaMarkerMatch=upload.buf[i]==OTA_PROJECT_MARKER[0]?1:0;
      if(otaMarkerMatch==sizeof(OTA_PROJECT_MARKER)-1) otaMarkerFound=true;
    }
    // esp_ota_write checks the image magic; esp_ota_end validates the full image,
    // including chip compatibility and checksum/hash before boot selection.
    if(esp_ota_write(otaHandle,upload.buf,upload.currentSize)!=ESP_OK) {
      abortOta("Invalid image or flash write failed"); return;
    }
    otaReceived+=upload.currentSize;
  } else if(upload.status==UPLOAD_FILE_END) {
    if(!otaWriting) return;
    if(otaReceived!=otaExpected || upload.totalSize!=otaExpected || !otaAuthorized() ||
       !otaRoleAllowed() || millis()-otaStarted>120000) {
      abortOta("Incomplete image or primary role changed"); return;
    }
    esp_err_t result=esp_ota_end(otaHandle);
    otaWriting=false; // esp_ota_end releases the handle, including on failure.
    if(result!=ESP_OK) { otaError="Image validation failed; previous firmware retained"; return; }
    if(!otaMarkerFound) { otaError="Wrong hardware/application: matching board OTA marker missing"; return; }
    otaComplete=true;
  } else if(upload.status==UPLOAD_FILE_ABORTED) {
    abortOta("Upload disconnected; retry or reboot before use");
  }
}

static const char OTA_PAGE[] PROGMEM=R"OTA(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Firmware update</title>
<style>body{font:17px system-ui;max-width:700px;margin:32px auto;padding:16px}button,input{margin:12px 0;padding:10px}progress{width:100%}</style></head><body>
<h1>Firmware update</h1><p id="unit">Reading device information...</p>
<p>This updates ONLY this controller. Stop its treadmill and wait for all motion to stop. Keep power connected. Use the application .bin for this controller hardware profile, NOT a merged image, bootloader, partition file, ZIP or sketch.</p>
<p>Once an upload is accepted, arming stays locked until reboot even if it fails. No automatic recovery from a firmware that fails to boot is promised; USB recovery may be required.</p>
<form id="f"><input id="file" type="file" accept=".bin" required><br><button id="send">Upload firmware</button></form>
<progress id="progress" max="100" value="0"></progress><p id="result" role="status"></p>
<button id="reboot" type="button">Reboot to leave update lock</button><p><a href="/">Dashboard</a></p>
<script>
const el=id=>document.getElementById(id);let info;
fetch('/api/firmware',{cache:'no-store'}).then(r=>r.json()).then(j=>{info=j;el('unit').textContent=`Device ${j.device}, node ${j.node}, firmware ${j.version}. OTA slot: ${j.slot_bytes} bytes.`;});
el('f').onsubmit=e=>{e.preventDefault();const file=el('file').files[0];if(!file)return;
if(info&&file.size>info.slot_bytes){el('result').textContent='Image exceeds the installed OTA slot.';return;}
if(!confirm('Update THIS stopped controller and reboot?'))return;
el('send').disabled=true;el('result').textContent='Uploading; do not disconnect power.';
const x=new XMLHttpRequest();x.open('POST','/api/firmware');x.setRequestHeader('X-Flux-Control','1');x.setRequestHeader('X-Flux-Size',String(file.size));x.timeout=150000;
x.upload.onprogress=e=>{if(e.lengthComputable)el('progress').value=e.loaded/e.total*100;};
x.onload=()=>{el('result').textContent=x.responseText;el('send').disabled=false;};
x.onerror=x.ontimeout=()=>{el('result').textContent='Connection lost. Check this controller before retrying; an accepted update keeps arming locked until reboot.';el('send').disabled=false;};
const body=new FormData();body.append('firmware',file);x.send(body);};
el('reboot').onclick=async()=>{if(!confirm('Reboot this controller to exit update lock?'))return;
const r=await fetch('/api/firmware/reboot',{method:'POST',headers:{'X-Flux-Control':'1'}});el('result').textContent=await r.text();};
</script></body></html>)OTA";

void setupOtaRoutes() {
  server.on("/update",HTTP_GET,[](){if(authenticated())server.send_P(200,"text/html",OTA_PAGE);});
  server.on("/api/firmware",HTTP_GET,[](){
    if(!authenticated())return;
    const esp_partition_t *slot=esp_ota_get_next_update_partition(nullptr);
    const esp_partition_t *running=esp_ota_get_running_partition();
    const size_t capacity=slot && running && slot->address!=running->address ? slot->size:0;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json","{\"version\":\""+String(FIRMWARE_VERSION)+"\",\"board\":"+jsonQuoted(BOARD_NAME)+",\"device\":"+jsonQuoted(deviceName)+
      ",\"project\":"+jsonQuoted(OTA_PROJECT_MARKER)+",\"node\":"+String(NODE_ID)+",\"slot_bytes\":"+String(capacity)+",\"locked\":"+(otaGate.load()==2?"true":"false")+"}");
  });
  server.on("/api/firmware",HTTP_POST,[](){
    if(!authenticated())return;
    if(!otaAuthorized()){server.send(403,"text/plain","Control header required");return;}
    if(otaComplete) {
      const esp_partition_t *slot=esp_ota_get_next_update_partition(nullptr);
      if(!otaRoleAllowed() || !slot || esp_ota_set_boot_partition(slot)!=ESP_OK) {
        otaComplete=false; server.send(409,"text/plain","Role changed or boot selection failed; reboot before use");return;
      }
      server.send(200,"text/plain","Image verified. Rebooting this controller; reconnect and verify firmware version. Treadmill remains stopped.");
      otaReboot.store(true);
    } else server.send(409,"text/plain",otaError.length()?otaError:"No complete firmware upload received");
  },receiveOtaUpload);
  server.on("/api/firmware/reboot",HTTP_POST,[](){
    if(!authenticated())return;
    if(!otaAuthorized()){server.send(403,"text/plain","Control header required");return;}
    if(otaGate.load()!=2){server.send(409,"text/plain","Not in update lock");return;}
    if(otaWriting)abortOta("Reboot requested");
    server.send(200,"text/plain","Rebooting; treadmill remains stopped");otaReboot.store(true);
  });
}

bool parseSpeedHundredths(const String &text,uint32_t &speed) {
  // Strict decimal, no signs, exponent, whitespace, NaN, truncation or overflow.
  if(!text.length() || text.length()>6)return false;
  uint32_t whole=0,fraction=0;unsigned decimals=0;bool dot=false,any=false;
  for(unsigned i=0;i<text.length();++i) {
    char c=text[i];
    if(c=='.' && !dot && any) {dot=true;continue;}
    if(c<'0'||c>'9')return false;
    any=true;
    if(dot) {if(++decimals>2)return false;fraction=fraction*10+unsigned(c-'0');}
    else whole=whole*10+unsigned(c-'0');
  }
  if(dot&&!decimals)return false;
  if(decimals==1)fraction*=10;
  speed=whole*100+fraction;
  return speed<=OPERATING_MAX_X100;
}

void setupWebServer() {
  const char *headers[] = {"X-Flux-Control", "X-Flux-Size"};
  server.collectHeaders(headers, 2);
  setupOtaRoutes();
  server.on("/", HTTP_GET, []() { if (authenticated()) server.send_P(200, "text/html", PAGE); });
  server.on("/bus", HTTP_GET, []() { if(authenticated())server.send_P(200,"text/html",BUS_PAGE); });
  server.on("/api/bus", HTTP_GET, []() {
    if(!authenticated())return;
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json","{\"device\":"+jsonQuoted(deviceName)+",\"role\":"+
      String(!RS485_ENABLED?0:PREFERRED_PRIMARY?1:2)+",\"node\":"+String(NODE_ID)+"}");
  });
  server.on("/api/bus", HTTP_POST, []() {
    if(!authenticated())return;
    if(otaGate.load()==2) { server.send(409,"text/plain","OTA lock: reboot first"); return; }
    if(server.header("X-Flux-Control")!="1") { server.send(403,"text/plain","Control header required"); return; }
    String role=server.arg("role"),node=server.arg("node");
    if(role.length()!=1 || role[0]<'0' || role[0]>'2' || node.length()!=1 || node[0]<'0' || node[0]>'8') {
      server.send(400,"text/plain","Invalid role or address"); return;
    }
    uint32_t packed=(uint32_t(role[0]-'0')<<8)|(node[0]-'0');
    if(!validBusConfig(packed)) { server.send(400,"text/plain","Primary: 1; secondary: 2-8; standalone: 0"); return; }
    uint8_t result;
    while(xQueueReceive(busConfigReplyQueue,&result,0)==pdTRUE) {}
    Command c{CommandKind::BUS_CONFIG,packed,millis()+750};
    if(xQueueSend(commandQueue,&c,0)!=pdTRUE) { server.send(503,"text/plain","Busy; retry"); return; }
    if(xQueueReceive(busConfigReplyQueue,&result,pdMS_TO_TICKS(1000))!=pdTRUE) {
      server.send(504,"text/plain","No confirmation. Check this unit's settings after any restart before retrying."); return;
    }
    const char *messages[]={"Saved. Restarting this unit; count retained. Returning to dashboard...",
      "Stop this treadmill and wait at least 3 seconds after the last sensor transition.",
      "Invalid role/address.","Could not save settings.","Restart already pending.","Request expired; retry."};
    server.send(result==0?200:409,"text/plain",messages[result<=5?result:3]);
  });
  server.on("/wifi", HTTP_GET, []() {
    if (authenticated()) server.send_P(200, "text/html", WIFI_PAGE);
  });
  server.on("/api/wifi", HTTP_POST, []() {
    if (!authenticated()) return;
    if(otaGate.load()==2) { server.send(409,"text/plain","OTA lock: reboot first"); return; }
    if (!server.hasArg("ssid") || !server.hasArg("password")) {
      server.send(400, "text/plain", "SSID and password fields are required"); return;
    }
    const String ssid = server.arg("ssid");
    const String password = server.arg("password");
    if (ssid.length() == 0 || ssid.length() > 32) {
      server.send(400, "text/plain", "SSID must contain 1 to 32 characters"); return;
    }
    if (password.length() != 0 && (password.length() < 8 || password.length() > 63)) {
      server.send(400, "text/plain", "Password must be blank or contain 8 to 63 characters"); return;
    }
    if (!saveWifiSettings(ssid, password)) {
      server.send(500, "text/plain", "Could not save Wi-Fi settings"); return;
    }
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width'><h2>Wi-Fi saved</h2><p>Wi-Fi is reconnecting. Its new address will appear on the display.</p>");
    scheduleNetworkRestart();
  });
  server.on("/api/wifi/clear", HTTP_POST, []() {
    if (!authenticated()) return;
    if(otaGate.load()==2) { server.send(409,"text/plain","OTA lock: reboot first"); return; }
    if (!saveWifiSettings("", "")) {
      server.send(500, "text/plain", "Could not clear Wi-Fi settings"); return;
    }
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width'><h2>Wi-Fi cleared</h2><p>Wi-Fi is switching to access-point mode.</p>");
    scheduleNetworkRestart();
  });
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/nodes", HTTP_GET, sendNodes);
  server.on("/api/speed", HTTP_POST, []() {
    if(!server.hasArg("value")) {server.send(400,"text/plain","value required in mph");return;}
    uint32_t speed=0;
    if(!parseSpeedHundredths(server.arg("value"),speed)) {server.send(400,"text/plain","Use decimal mph from 0 to 5.00 (0 stops), at most two decimal places");return;}
    queueRequest(CommandKind::SPEED,speed);
  });
  server.on("/api/start", HTTP_POST, []() { queueRequest(CommandKind::CSAFE_START); });
  server.on("/api/pause", HTTP_POST, []() { queueRequest(CommandKind::PAUSE); });
  server.on("/api/stop", HTTP_POST, []() {
    queueRequest(CommandKind::STOP);
  });
  server.on("/api/ack", HTTP_POST, []() {
    queueRequest(CommandKind::ACK);
  });
  server.on("/api/clear-fault", HTTP_POST, []() { queueRequest(CommandKind::CLEAR_FAULT); });
  server.on("/api/reset", HTTP_POST, []() { queueRequest(CommandKind::RESET); });
  server.on("/api/target", HTTP_POST, []() {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "value required"); return; }
    String s = server.arg("value");
    if (s.length() == 0 || s.length() > 10) { server.send(400, "text/plain", "Invalid target"); return; }
    for (unsigned i=0; i<s.length(); ++i)
      if (s[i]<'0' || s[i]>'9') { server.send(400, "text/plain", "Digits only"); return; }
    const uint64_t value = strtoull(s.c_str(), nullptr, 10);
    if (value == 0 || value > MAX_TARGET) { server.send(400, "text/plain", "Target out of range"); return; }
    queueRequest(CommandKind::TARGET, uint32_t(value));
  });
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

void startFallbackAp() {
  if (fallbackApStarted) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(deviceName.c_str(), AP_PASSWORD);
  fallbackApStarted = true;
  Serial.printf("Fallback AP: %s  http://%s\n", deviceName.c_str(), WiFi.softAPIP().toString().c_str());
}

// This task alone owns the radio and HTTP server. Never block waiting for Wi-Fi:
// a primary demotion must turn the radio off promptly even during association.
uint32_t wifiConnectStarted=0;
void setupWifi() {
  loadWifiSettings();
  WiFi.setHostname(deviceName.c_str());
  fallbackApStarted=false;
  wifiConnectStarted=lastWifiAttemptAt=millis();
  if(configuredWifiSsid.length()==0) { startFallbackAp();return; }
  WiFi.mode(WIFI_STA);
  WiFi.begin(configuredWifiSsid.c_str(),configuredWifiPassword.c_str());
}
void networkTask(void *) {
  bool radioOn=false,routesInstalled=false;
  WiFi.mode(WIFI_OFF);
  for(;;) {
    xSemaphoreTake(snapshotMutex,portMAX_DELAY);
    const bool primary=sharedPrimary;const uint8_t leader=sharedLeader;
    xSemaphoreGive(snapshotMutex);
    const bool wanted=!RS485_ENABLED || primary;
    if(!wanted && radioOn) {
      if(otaWriting) abortOta("Primary role lost; upload aborted. Reboot before use.");
      server.stop();WiFi.softAPdisconnect(true);WiFi.disconnect(true);WiFi.mode(WIFI_OFF);
      fallbackApStarted=false;radioOn=false;networkRestartRequested=false;
    }
    if(wanted && !radioOn) {
      setupWifi();
      if(!routesInstalled) { setupWebServer();routesInstalled=true; } else server.begin();
      radioOn=true;
    }
    if(radioOn) {
      serviceWifi();server.handleClient();
      if(otaWriting && millis()-otaStarted>120000) abortOta("Upload timed out; reboot before use.");
      if(networkRestartRequested && int32_t(millis()-networkRestartAt)>=0) {
        networkRestartRequested=false;
        WiFi.softAPdisconnect(true);WiFi.disconnect(true);WiFi.mode(WIFI_OFF);
        setupWifi();
      }
    }
    String address;
    if(!radioOn)address="Secondary N"+String(NODE_ID)+" / P"+String(leader);
    else if(WiFi.status()==WL_CONNECTED)address=WiFi.localIP().toString();
    else if(fallbackApStarted)address=WiFi.softAPIP().toString();
    else address="Wi-Fi connecting";
    xSemaphoreTake(snapshotMutex,portMAX_DELAY);sharedAddress=address;xSemaphoreGive(snapshotMutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void publishStatus() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();
  String s = jsonStatus();
  uint16_t regs[BUS_REG_COUNT]; buildBusSnapshot(regs);
  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  sharedStatus = s;
  memcpy(localRegs,regs,sizeof(regs));
  displayPrimary=sharedPrimary;displayCandidate=sharedCandidate;
  displayBusReady=sharedBusReady;
  displayBusOnline=sharedBusSeen && millis()-sharedBusLast<BUS_OFFLINE_MS;
  networkAddress = sharedAddress;
  xSemaphoreGive(snapshotMutex);
}

void serviceWifi() {
  if(WiFi.status()==WL_CONNECTED || configuredWifiSsid.length()==0)return;
  if(!fallbackApStarted && millis()-wifiConnectStarted>=WIFI_CONNECT_MS)startFallbackAp();
  if(millis()-lastWifiAttemptAt>=30000) {
    lastWifiAttemptAt=millis();WiFi.reconnect();
  }
}

void drawDisplay() {
#if FLUX_BOARD == FLUX_STAMPLC
  if (millis() - lastDisplayAt < DISPLAY_REFRESH_MS) return;
  lastDisplayAt = millis();
  auto &d = M5.Display;
  const int w = d.width(), h = d.height();
  const uint16_t dim = 0x4208;
  d.fillScreen(TFT_BLACK);
  d.setTextWrap(false);
  d.setTextSize(1);
  d.drawRect(0, 0, w, h, dim);
  // Top perimeter: live input state; pulse outline persists for 350 ms so a
  // short optical pulse is visible at the 250 ms display refresh interval.
  auto badge = [&](int x, int width, const char *label, uint16_t color) {
    d.drawRoundRect(x, 2, width, 17, 3, color);
    d.setTextColor(color, TFT_BLACK);
    d.setCursor(x + 4, 7); d.print(label);
  };
  const int third = w / 3;
  const bool pulseFlash = opticalPulseSeen && millis() - opticalPulseAt < 350;
  badge(2, third - 4, !inputsHealthy ? "OPT ERR" : sensorStable ? "OPT ON" : "OPT OFF",
        !inputsHealthy ? TFT_RED : (sensorStable || pulseFlash) ? TFT_CYAN : dim);
  badge(third + 2, third - 4, !inputsHealthy ? "EST ERR" : stopAuxStable ? "EST STOP" : "EST OK",
        !inputsHealthy || stopAuxStable ? TFT_RED : TFT_GREEN);
  badge(2 * third + 2, w - 2 * third - 4,
        !RS485_ENABLED ? "485 OFF" : !displayBusReady ? "485 ERR" : displayCandidate ? "ELECT" :
        displayBusOnline ? (displayPrimary ? "P485 OK" : "S485 OK") : (displayPrimary ? "P485 WAIT" : "S485 WAIT"),
        !RS485_ENABLED ? dim : !displayBusReady ? TFT_RED : displayBusOnline ? TFT_GREEN : dim);

  auto centered = [&](const String &text, int y, uint16_t color, int size) {
    d.setTextSize(size);
    d.setTextColor(color, TFT_BLACK);
    d.setCursor((w - d.textWidth(text.c_str())) / 2, y);
    d.print(text);
  };
  centered(stateName(runState), 23,
           runState == RunState::FAULT ? TFT_RED : TFT_WHITE, 1);
  const String count = u64(stepCount);
  int countSize = 3;
  while (countSize > 1 && int(count.length()) * 6 * countSize > w - 12) --countSize;
  const bool active=runState==RunState::ARMED || runState==RunState::RUNNING;
  if(MOTOR_ENABLED && active && localSpeedPage) {
    centered(motor.feedbackValid ? String(motor.actualRaw/96.0f,2)+" mph" : String("-- mph"),36,TFT_WHITE,3);
  } else centered(count, 36, TFT_WHITE, countSize);
  if(MOTOR_ENABLED && !active && runState!=RunState::FAULT && !completionLatched) {
    const int y=localEditField==LocalEditField::TARGET?62:75;
    d.fillRoundRect(3,y,w-6,13,2,dim);
  }
  centered(MOTOR_ENABLED && active && localSpeedPage ? String("COUNT ")+count+" / "+u64(stepTarget) : String("TARGET ")+u64(stepTarget),64,TFT_WHITE,1);
  const String presets = "A:" + u64(PRESETS[0]) + " B:" + u64(PRESETS[1]) + " C:" + u64(PRESETS[2]);
  centered(MOTOR_ENABLED ? String("SET ")+String(selectedSpeedX100/100.0f,2)+" ACT "+(motor.feedbackValid?String(motor.actualRaw/96.0f,2):String("--"))+" mph" : presets, 77, TFT_CYAN, 1);
  const bool showReset = resetMessage.length() && millis() - resetMessageAt < 8000;
  centered(otaGate.load()==2 ? String("OTA LOCK - reboot") : showReset ? resetMessage : !framAvailable ? "FRAM ERROR" : !inputsHealthy ? "INPUT ERROR" :
           !WIRING_VERIFIED ? "BENCH LOCK" : MOTOR_ENABLED && !MOTOR_COMMISSIONED ? "MOTOR SETUP LOCK" :
           (millis() / 4000) % 2 ? networkAddress : (MOTOR_ENABLED ? (runState==RunState::FAULT ? String("Hold A:Clear C:Reset") : completionLatched ? String("Hold C:Reset result") : active ? String("A:- B:View C:+ Hold B:")+(motorStarted || !motor.ready(millis())?"Stop":"Start") : String("A:- B:Field C:+ Hold B:Arm")) : String("Hold B:Arm C:Reset")),
           90, (showReset ? resetMessageError : !framAvailable || !inputsHealthy) ? TFT_RED : TFT_WHITE, 1);

  // Bottom perimeter: commanded relay coil states, not measured contacts.
  d.setTextSize(1);
  const uint16_t colors[4] = {TFT_GREEN, TFT_GREEN, TFT_RED, TFT_YELLOW};
  for (int i = 0; i < 4; ++i) {
    const int x = i * w / 4 + 2, next = (i + 1) * w / 4;
    const uint16_t color = relayCommanded[i] ? colors[i] : dim;
    d.drawRoundRect(x, h - 20, next - x - 2, 18, 3, color);
    d.setTextColor(color, TFT_BLACK);
    d.setCursor(x + 3, h - 14);
    d.printf("R%d %s", i + 1, relayCommanded[i] ? "ON" : "OFF");
  }
#endif
}

// Short presses never arm, start, reset a count or clear a fault.
void handleLocalShort(uint8_t button) {
  const bool active=runState==RunState::ARMED || runState==RunState::RUNNING;
  if(runState==RunState::FAULT || otaGate.load()==1 || otaGate.load()==2 || busConfigRestartPending) {
    boardTone(220,100);return;
  }
  if(button==1) {
    if(active)localSpeedPage=!localSpeedPage;
    else localEditField=localEditField==LocalEditField::SPEED?LocalEditField::TARGET:LocalEditField::SPEED;
    boardTone(660,50);return;
  }
  if(button>2 || motorStopping || completionLatched) {boardTone(220,100);return;}
  const bool increase=button==2;
  if(active || localEditField==LocalEditField::SPEED) {
    const uint32_t next=increase?RUN_SPEED_X100:WALK_SPEED_X100;
    if(next==selectedSpeedX100) {boardTone(220,50);return;}
    requestMotorSpeed(next);return;
  }
  // Target edits are stopped-only, retain accumulated count, and use existing FRAM verification.
  if(!framAvailable || captureCoast || millis()-lastObservedEdge<3000) {boardTone(220,100);return;}
  uint64_t next=stepTarget;
  if(increase)next=next>=MAX_TARGET-LOCAL_TARGET_STEP?MAX_TARGET:next+LOCAL_TARGET_STEP;
  else next=next<=LOCAL_TARGET_STEP?1:next-LOCAL_TARGET_STEP;
  if(next==stepTarget) {boardTone(220,50);return;}
  stepTarget=next;writeFramRecord();
  reportReset(framAvailable?"Step target saved":"Step target save failed",!framAvailable);
}

void handleLocalButtons() {
#if FLUX_BOARD == FLUX_STAMPLC
  { PlcIoGuard guard; M5StamPLC.update(); }
  // Resolve on release using elapsed time: a hold must never also select a preset.
  static uint32_t pressedAt[3] = {};
  static bool tracking[3] = {};
  if(otaGate.load()==1 || otaGate.load()==2 || busConfigRestartPending) {
    for(auto &pressed:tracking)pressed=false;
    return;
  }
  for (uint8_t i = 0; i < 3; ++i) {
    auto &button = i == 0 ? M5.BtnA :
                   i == 1 ? M5.BtnB : M5.BtnC;
    if (button.wasPressed()) { pressedAt[i] = millis(); tracking[i] = true; }
    if (!button.wasReleased() || !tracking[i]) continue;
    tracking[i] = false;
    const uint32_t held = millis() - pressedAt[i];
    if (i == 0 && held >= 1000 && runState == RunState::FAULT) { clearFault(); continue; }
    if (i == 1 && held >= 1000) { togglePause(); continue; }
    if (i == 2 && held >= 3000) { resetCount(); continue; }
    if (held >= 1000) continue;
    if(MOTOR_ENABLED) {handleLocalShort(i);continue;}
    if (completionLatched || runState == RunState::RUNNING || runState == RunState::ARMED ||
        !framAvailable || captureCoast || millis() - lastObservedEdge < 3000) {
      boardTone(220, 100); continue;
    }
    presetIndex = i;
    stepTarget = PRESETS[i];
    writeFramRecord();
    boardTone(523, 60);
  }
#else
  // Release-to-arm: require an observed release after boot/OTA lock.
  static bool candidate=true, stable=true, released=false, tracking=false;
  static uint32_t changedAt=0, pressedAt=0;
  if (otaGate.load()==2 || busConfigRestartPending) {
    released=false; tracking=false; return;
  }
  const uint32_t now=millis();
  const bool raw=digitalRead(ARM_GPIO);
  if(raw!=candidate) { candidate=raw; changedAt=now; }
  if(now-changedAt<30) return;
  if(!released) { if(raw) { released=true; stable=true; } return; }
  if(stable==candidate) return;
  stable=candidate;
  if(!stable) { pressedAt=now; tracking=true; return; }
  if(tracking) { tracking=false; if(now-pressedAt>=1000) togglePause(); }
#endif
}

void setup() {
  Serial.begin(115200);
  // Set the Waveshare output inactive before NVS, I2C or network startup.
#if FLUX_BOARD == FLUX_WAVESHARE_RELAY1
  digitalWrite(RELAY_GPIO, !RELAY_GPIO_ON);
  pinMode(RELAY_GPIO, OUTPUT);
  pinMode(SENSOR_GPIO, INPUT_PULLUP);
  pinMode(ESTOP_GPIO, INPUT_PULLUP);
  pinMode(ARM_GPIO, INPUT_PULLUP);
#endif
  loadBusSettings();
#if FLUX_BOARD == FLUX_STAMPLC
  auto plcConfig = M5StamPLC.config();
  plcConfig.enableModbusSlave = false; // this sketch owns UART1
  M5StamPLC.config(plcConfig);
  M5StamPLC.begin();
  M5StamPLC.setBacklight(true);
  // Perimeter layout is designed for the 240 x 135 landscape screen.
  auto &screen = M5.Display;
  if (screen.width() < screen.height())
    screen.setRotation((screen.getRotation() + 1) % 4);

  // Known, inactive output state before restoring any saved state.
  for (uint8_t i = 0; i < 4; ++i) M5StamPLC.writePlcRelay(i, false);
  if (acModule.begin()) acModule.writeRelay(false);
#endif
  setTreadmillStopped(true);
  setRelay(GREEN_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  setRelay(RED_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);

  if(MOTOR_ENABLED)csafeSerial.begin(MOTOR_BAUD,SERIAL_8N1,MOTOR_RX,MOTOR_TX);
  if(CSAFE_ENABLED) {
    csafeSerial.begin(9600,SERIAL_8N1,CSAFE_RX,CSAFE_TX);
    csafe.begin(millis()); csafe.stop(millis());
  }
  loadFramState();
  char suffix[13];
  snprintf(suffix, sizeof(suffix), "%012llX", (unsigned long long)ESP.getEfuseMac());
  deviceName = "flux-" + String(suffix);
  plcIoMutex=xSemaphoreCreateRecursiveMutex();
  inputSamples=xQueueCreate(INPUT_QUEUE_LENGTH,sizeof(InputSample));
  if(!plcIoMutex || !inputSamples) {Serial.println("Input sampler allocation failed");while(true)delay(1000);}
  if(xTaskCreatePinnedToCore(inputSamplerTask,"flux-input",4096,nullptr,3,nullptr,1)!=pdPASS) {
    Serial.println("Input sampler task failed");while(true)delay(1000);
  }
  // Allow the first acquisition before enabling controls.
  const uint32_t sampleBootAt=millis();
  while(!sampleStarted.load() && millis()-sampleBootAt<100)delay(1);
  lastObservedEdge = millis(); // require initial settling before first local Arm
  commandQueue = xQueueCreate(8, sizeof(Command));
  snapshotMutex = xSemaphoreCreateMutex();
  busCommandQueue = xQueueCreate(8,sizeof(BusCommand));
  busConfigReplyQueue = xQueueCreate(1,sizeof(uint8_t));
  if (!commandQueue || !snapshotMutex || !busCommandQueue || !busConfigReplyQueue) { while (true) delay(1000); }
  sharedStatus = jsonStatus();
  buildBusSnapshot(localRegs);
  if (RS485_ENABLED && xTaskCreatePinnedToCore(busTask, "flux-rs485", 6144, nullptr, 2, nullptr, 0) != pdPASS)
    Serial.println("RS485 task failed; local controls remain available");
  if (xTaskCreatePinnedToCore(networkTask, "flux-web", 8192, nullptr, 1, nullptr, 0) != pdPASS)
    Serial.println("Network task failed; local controls remain available");
  updateIndicators();

  if (!framAvailable) {
    runState = RunState::FAULT;
    stopReason = StopReason::STORAGE;
    updateIndicators();
    startExternalBeep(300);
  }
}

void loop() {
  scanInputs();
  serviceOtaGate();
  handleLocalButtons();
  serviceCommands();
  serviceStopPulse();
  serviceExternalBeep();
  servicePulseTimeout();
  serviceMotor();
  serviceCsafe();
  // Refresh indicator state if an I2C storage failure changed state internally.
  static RunState indicated = RunState::IDLE;
  if (runState != indicated) { indicated = runState; updateIndicators(); }
  publishStatus();
  drawDisplay();
  if(otaReboot.load()) {
    setTreadmillStopped(true);
    uint32_t until=millis()+1500;
    while(int32_t(millis()-until)<0) { serviceMotor();serviceCsafe(); delay(1); }
    ESP.restart();
  }
  if(busConfigRestartPending && int32_t(millis()-busConfigRestartAt)>=0) {
    setTreadmillStopped(true);
    csafe.stop(millis());
    uint32_t until=millis()+1500;
    while(int32_t(millis()-until)<0) { serviceMotor();serviceCsafe(); delay(1); }
    ESP.restart();
  }
  // Verbose motor frame dumps removed; faults remain in status and event messages.
  delay(1);
}
