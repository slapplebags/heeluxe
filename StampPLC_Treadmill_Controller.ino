/*
  StampPLC_Treadmill_Controller.ino

  Treadmill controller with optional Modbus RTU networking (firmware 7).
  Flash the SAME sketch on every unit. In each unit's web UI, open RS485
  settings and select Standalone, Primary (ID 1), or Secondary (unique ID 2..8).
  New units default to Standalone. Saved role/address survive power cycles.
  Applying bus settings requires a stopped/settled unit and restarts it.
  Configure one preferred primary (ID 1); every secondary has a unique ID 2..8.
  Connect PWR485 A-A, B-B, GND-GND; leave VIN disconnected between
  independently powered units. Enable the onboard RS485 120 ohm termination
  at the two bus ends only (the CAN termination switch is separate).
  Primary polls secondarys; each treadmill remains locally controlled on bus loss.

  Hardware plan
  -------------
  - M5Stack StamPLC + StamPLC AC power module
  - 12 V NPN optical interrupter -> isolated PLC input 1 (software channel 0)
  - Mushroom STOP auxiliary contact -> isolated PLC input 2 (channel 1)
  - Relay 1 -> treadmill's existing low-voltage normally-closed stop loop
  - Relay 2 -> green 12 V indicator
  - Relay 3 -> red 12 V indicator
  - Relay 4 -> 12 V external buzzer
  - MB85RC256V I2C FRAM on Grove PORT.A: SDA G2, SCL G1, address 0x50.
    Use 3.3 V power/pull-ups for a bare MB85RC256V breakout. Grove red is
    FIVE VOLTS, not 3.3 V. Do not connect it to an unshifted 3.3 V FRAM board.

  IMPORTANT
  ---------
  This controller does not start the treadmill and is not a safety controller.
  The mushroom's primary NC contact must interrupt the treadmill stop loop
  directly; firmware only reads a separate auxiliary contact. Keep the
  treadmill manufacturer's original safety devices intact.

  Required Arduino libraries
  --------------------------
  - M5StamPLC: https://github.com/m5stack/M5StamPLC
  - M5Unified (dependency of M5StamPLC)
  - No separate FRAM library required (checked block I2C transactions).

  Board: ESP32S3 Dev Module / esp32-s3-devkitc-1, 8 MB flash, PSRAM disabled.
  BENCH PROTOTYPE, NOT HARDWARE VALIDATED. First set WIRING_VERIFIED only
  after testing relay 1 with the treadmill disconnected and checking polarity.
  Relay 1 must close the permit loop when energized, OPEN it when unpowered.
  No mains switching. STOP stays asserted until an explicit local Arm.
  The motor must require a separate manual start after permit is restored.
  Yellow indicator unused. Separate red/green/buzzer leads must be confirmed.
  FRAM saves each observed pulse using alternating CRC-checked records.
  Loss during a save may lose the latest pulse; coast-down during an outage
  cannot be counted. Inspect/reset or reconcile the count before re-arming.
  Test at 10 Hz with >=20 ms active AND inactive widths, under web load.
  Network tasks never access treadmill I/O; requests go through bounded queues.
  Modbus reads status and queues guarded commands; no raw relay writes or ARM.
  No OTA, email, cloud service or automatic motor-start in this version.
  Web controls use HTTP basic authentication: trusted LAN only, no port forward.
  SETUP / COMMISSIONING
  ---------------------
  1. Put this file in a folder named StampPLC_Treadmill_Controller and open it
     in Arduino IDE. Install M5StamPLC and its dependencies. Select ESP32-S3.
  2. Change AP_PASSWORD and WEB_PASSWORD. Web username is admin. On first
     boot, join the flux-* access point, open the displayed address, and use
     Wi-Fi settings to save the existing network. The fallback AP returns if
     that network cannot be reached.
  3. NPN sensor: power per its datasheet, typically brown +12 V, blue 0 V,
     black output -> IN1; PLC input COM -> +12 V. Verify actual wire colors.
     Separate NO mushroom contact between IN2 and 0 V:
     pressing STOP closes it. This unit's observed input polarity is configured
     by STOP_AUX_ACTIVE_LEVEL below.
     A broken auxiliary wire cannot be detected with this NO arrangement.
     Never electrically join the treadmill's primary loop to this auxiliary.
  4. FRAM: SDA G2, SCL G1, GND and 3.3 V (or a properly level-shifted
     5 V-compatible breakout). Bare MB85RC256V must not see 5 V.
  5. Relay 1 is a dry RUN-PERMIT contact: closed only while ARMED/RUNNING.
     With a meter verify coil OFF = OPEN, ON = CLOSED. Verify that opening
     the treadmill loop stops it and closing it NEVER restarts it by itself.
     Keep original safety key and mushroom NC contact in series.
  6. Verify sensor polarity using the display. Default false assumes the
     optocoupler pulls the expander input LOW when detecting a pulse.
  7. Verify the indicator has independent color/sound inputs and compatible
     voltage/current. Relay 2 green, 3 red, 4 buzzer; yellow disconnected.
     Validate the AC module's 12 V current budget including all accessories.
  8. Set WIRING_VERIFIED=true after these checks, then test disconnected first:
     pulses at 2/3/10 Hz, target stop, pause, mushroom, FRAM unplugged, reset,
     Wi-Fi loss/web traffic, and power interruption at different pulse phases.
     Measure missed pulses and mechanical coast-down; target stop is a command
     threshold, NOT a guarantee of stopping at that exact physical step.
  9. Tap A/B/C while stopped to select 10,000/50,000/100,000 target steps.
     Edit PRESETS below to change these three values. Selection preserves count.
     Hold B >=1 s and release to arm/resume or pause. Hold C >=3 s and
     release to reset count while stopped. Use mushroom for STOP.
     Web acknowledge only silences; reset clears count/recovery fault once
     the E-stop is released and I/O/storage checks pass. Reset retries FRAM.
  10. Green = permit, red = not permitted; buzzer sounds briefly on stop/fault.
      No safety certification, redundant stop, mains isolation or motor feedback.
*/

#include <Arduino.h>
#include <M5StamPLC.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module for the StampPLC."
#endif
// Dedicated external bus; internal M5 bus stays on controller 0.
TwoWire framBus(1);
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Optional factory defaults. Normally leave these blank and configure Wi-Fi
// from the fallback access point's web page. Saved settings override defaults.
constexpr char WIFI_SSID[]     = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char AP_PASSWORD[]   = "fluxcapacitor"; // minimum 8 characters
constexpr char WEB_PASSWORD[]  = "change-this-password";
constexpr bool WIRING_VERIFIED = true; // this unit has completed bench wiring checks
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

// Four onboard dry contacts only. Yellow lead disconnected and insulated.
// AC module supplies power; its switched-mains output remains unused and OFF.
constexpr uint8_t TREADMILL_STOP_RELAY = 0;
constexpr uint8_t GREEN_RELAY          = 1;
constexpr uint8_t RED_RELAY            = 2;
constexpr uint8_t EXTERNAL_BUZZER_RELAY = 3;
M5StamPLC_AC acModule;

// Change these after observing the Input test example or serial diagnostics.
constexpr bool SENSOR_ACTIVE_LEVEL = false;
// Polarity verified on this unit after bench testing the mushroom input.
constexpr bool STOP_AUX_ACTIVE_LEVEL = true;

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
constexpr uint32_t EXTERNAL_BEEP_MS    = 1000;

constexpr uint64_t DEFAULT_TARGET = 100000;
// A, B, C select these TARGETS; selection never changes the accumulated count.
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
  INPUT_IO
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
void startExternalBeep(uint32_t durationMs = EXTERNAL_BEEP_MS);
void storageFault();
void setTreadmillStopped(bool stopped);
enum class CommandKind : uint8_t { ARM, PAUSE, STOP, ACK, RESET, TARGET, BUS_CONFIG };
struct Command { CommandKind kind; uint32_t value; uint32_t deadline = 0; };
QueueHandle_t commandQueue;
QueueHandle_t busConfigReplyQueue;
SemaphoreHandle_t snapshotMutex;
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
uint32_t buzzerOffAt = 0;
uint32_t lastPulseAt = 0;
uint32_t lastInputScanAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastWifiAttemptAt = 0;
uint32_t lastObservedEdge = 0;
bool seenInactive = false;
bool captureCoast = false;
bool completionLatched = false;
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
    && record.stopReason <= uint8_t(StopReason::INPUT_IO);
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
  if (!framBus.begin(2, 1, 400000)) {
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

void setRelay(uint8_t channel, bool active, bool activeLevel) {
  const bool energized = active ? activeLevel : !activeLevel;
  M5StamPLC.writePlcRelay(channel, energized);
  if (channel < 4) relayCommanded[channel] = energized;
}

void setTreadmillStopped(bool stopped) {
  setRelay(TREADMILL_STOP_RELAY, stopped, STOP_RELAY_ACTIVE_LEVEL);
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

  if (red) M5StamPLC.setStatusLight(255, 0, 0);
  else if (green) M5StamPLC.setStatusLight(0, 255, 0);
  else M5StamPLC.setStatusLight(0, 0, 32);
}

void startExternalBeep(uint32_t durationMs) {
  setRelay(EXTERNAL_BUZZER_RELAY, true, INDICATOR_RELAY_ACTIVE_LEVEL);
  buzzerOffAt = millis() + durationMs;
}

void serviceExternalBeep() {
  if (buzzerOffAt && static_cast<int32_t>(millis() - buzzerOffAt) >= 0) {
    setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
    buzzerOffAt = 0;
  }
}

void requestTreadmillStop(StopReason reason, RunState finalState) {
  if (finalState == RunState::TARGET_REACHED) completionLatched = true;
  captureCoast = !completionLatched && (captureCoast || runState == RunState::RUNNING);
  // Hold the permit loop open. Never automatically reclose after three seconds.
  setTreadmillStopped(true);
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
  if (busConfigRestartPending || completionLatched || !inputsHealthy || !WIRING_VERIFIED || !framAvailable || stopAuxStable || stopAuxCandidate ||
      stepTarget == 0 ||
      stepCount >= stepTarget || millis() - lastObservedEdge < 3000 ||
      runState == RunState::FAULT) {
    M5StamPLC.tone(220, 250);
    return;
  }
  stopReason = StopReason::NONE;
  runState = RunState::ARMED;
  lastPulseAt = millis();
  writeFramRecord();
  if (!framAvailable) return;
  captureCoast = false;
  stopPulseActive = false;
  setTreadmillStopped(false);
  updateIndicators();
  M5StamPLC.tone(880, 80);
}

void togglePause() {
  if (runState == RunState::RUNNING || runState == RunState::ARMED) {
    requestTreadmillStop(StopReason::LOCAL_BUTTON, RunState::PAUSED);
    return;
  } else if (runState == RunState::PAUSED) {
    armTest();
    return;
  } else {
    armTest();
    return;
  }
  writeFramRecord();
  updateIndicators();
}

void reportReset(const char *message, bool error) {
  resetMessage = message;
  resetMessageAt = millis();
  resetMessageError = error;
  Serial.println(message);
  M5StamPLC.tone(error ? 220 : 660, error ? 250 : 100);
}

void resetCount() {
  if (runState == RunState::RUNNING || runState == RunState::ARMED) {
    reportReset("Reset: stop run first", true);
    return;
  }
  // Reset never grants treadmill permission, even when recovering a fault.
  setTreadmillStopped(true);
  uint8_t ports[2];
  inputsHealthy = M5.In_I2C.readRegister(0x59, 0x00, ports, 2, 400000);
  if (!inputsHealthy) {
    requestTreadmillStop(StopReason::INPUT_IO, RunState::FAULT);
    reportReset("Reset: input IO fault", true);
    return;
  }
  const uint8_t pins[8] = {4,5,6,7,12,13,14,15};
  const uint16_t bits = ports[0] | (uint16_t(ports[1]) << 8);
  const bool stopRaw = bool(bits & (1U << pins[STOP_AUX_INPUT_CHANNEL])) == STOP_AUX_ACTIVE_LEVEL;
  if (stopRaw || stopAuxStable || stopAuxCandidate ||
      millis() - stopAuxCandidateSince < STOP_DEBOUNCE_MS) {
    reportReset("Reset: release E-stop", true);
    return;
  }
  if (!framAvailable) {
    // Reinitialize only the dedicated external FRAM bus, not the PLC IO bus.
    framBus.end();
    if (!framBus.begin(2, 1, 400000)) {
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
  const uint64_t previousCount = stepCount;
  const bool previousCompletion = completionLatched;
  const bool previousCaptureCoast = captureCoast;
  captureCoast = false;
  stepCount = 0;
  completionLatched = false;
  runState = RunState::IDLE;
  stopReason = StopReason::NONE;
  writeFramRecord();
  if (!framAvailable) {
    // Keep the displayed count if the reset could not be verified in storage.
    stepCount = previousCount;
    completionLatched = previousCompletion;
    captureCoast = previousCaptureCoast;
    updateIndicators();
    reportReset("Reset: FRAM save failed", true);
    return;
  }
  stopPulseActive = false;
  seenInactive = false; // require a fresh inactive sensor level before counting
  setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  buzzerOffAt = 0;
  updateIndicators();
  reportReset("Reset OK - stopped", false);
}

void processStepPulse() {
  if (completionLatched) return; // completed count remains frozen until reset
  if (runState != RunState::ARMED && runState != RunState::RUNNING && !captureCoast) return;
  if (runState == RunState::ARMED) runState = RunState::RUNNING;
  ++stepCount;
  lastPulseAt = millis();
  if (stepCount >= stepTarget && runState == RunState::RUNNING) {
    requestTreadmillStop(StopReason::TARGET, RunState::TARGET_REACHED);
  } else {
    writeFramRecord();
  }
}

void scanInputs() {
  const uint32_t now = millis();
  if (now - lastInputScanAt < INPUT_SCAN_MS) return;
  const uint32_t gap = now - lastInputScanAt;
  if (lastInputScanAt && gap > maxScanGap) maxScanGap = gap;
  lastInputScanAt = now;

  uint8_t ports[2];
  if (!M5.In_I2C.readRegister(0x59, 0x00, ports, 2, 400000)) {
    inputsHealthy = false;
    if (stopReason != StopReason::INPUT_IO)
      requestTreadmillStop(StopReason::INPUT_IO, RunState::FAULT);
    return;
  }
  inputsHealthy = true;
  const uint16_t portBits = ports[0] | (uint16_t(ports[1]) << 8);
  const uint8_t pins[8] = {4,5,6,7,12,13,14,15};
  const bool sensorRaw = bool(portBits & (1U << pins[SENSOR_INPUT_CHANNEL])) == SENSOR_ACTIVE_LEVEL;
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
      processStepPulse();
    }
  }
  if (!sensorRaw && now - sensorCandidateSince >= SENSOR_DEBOUNCE_MS) seenInactive = true;

  const bool stopRaw = bool(portBits & (1U << pins[STOP_AUX_INPUT_CHANNEL])) == STOP_AUX_ACTIVE_LEVEL;
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
  json.reserve(320);
  json += "{\"device\":\"" + deviceName + "\",";
  json += "\"state\":\"" + String(stateName(runState)) + "\",";
  json += "\"reason\":\"" + String(reasonName(stopReason)) + "\",";
  json += "\"count\":" + u64(stepCount) + ",";
  json += "\"target\":" + u64(stepTarget) + ",";
  json += "\"sensor\":" + String(sensorStable ? "true" : "false") + ",";
  json += "\"stop_button\":" + String(stopAuxStable ? "true" : "false") + ",";
  json += "\"fram\":" + String(framAvailable ? "true" : "false") + ",";
  json += "\"reset_message\":\"" + resetMessage + "\",";
  json += "\"max_scan_gap_ms\":" + String(maxScanGap) + ",";
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
// Holding registers: 0..47 snapshot, 256..258 atomic command mailbox (FC16).
constexpr uint8_t BUS_LAST_NODE = 8;
constexpr uint16_t BUS_REG_COUNT = 48;
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
  r[0] = 0x4658; r[1] = 2;
  uint64_t mac = ESP.getEfuseMac();
  for (int i=0; i<3; ++i) r[2+i] = mac >> (16*(2-i));
  r[5] = uint16_t(runState); r[6] = uint16_t(stopReason);
  for (int i=0; i<4; ++i) r[7+i] = stepCount >> (16*(3-i));
  r[11] = stepTarget >> 16; r[12] = stepTarget;
  r[13] = sensorStable | (stopAuxStable << 1) | (framAvailable << 2) |
          (inputsHealthy << 3) | (WIRING_VERIFIED << 4) | (completionLatched << 5);
  for (int i=0; i<4; ++i) r[14] |= uint16_t(relayCommanded[i]) << i;
  r[15] = maxScanGap >> 16; r[16] = maxScanGap;
  for (int i=0; i<3; ++i) { r[17+i*2] = PRESETS[i] >> 16; r[18+i*2] = PRESETS[i]; }
  r[23] = presetIndex;
  // Last operator result, up to 40 printable ASCII characters.
  for (size_t i=0; i<40 && i<resetMessage.length(); ++i)
    r[24+i/2] |= uint16_t(uint8_t(resetMessage[i])) << ((i%2) ? 0 : 8);
  r[44] = NO_PULSE_FAULT_MS / 1000;
  r[45] = ARM_WAIT_TIMEOUT_MS / 1000;
  r[46] = resetMessageError;
}

// Pure validation before a remote command reaches the hardware-owning loop.
bool decodeBusCommand(const uint8_t *p, size_t n, Command &c) {
  if (n != 15 || p[1] != 16 || mbWord(p+2) != 256 || mbWord(p+4) != 3 || p[6] != 6)
    return false;
  const uint16_t op = mbWord(p+7);
  const uint32_t value = (uint32_t(mbWord(p+9)) << 16) | mbWord(p+11);
  if (op < uint16_t(CommandKind::PAUSE) || op > uint16_t(CommandKind::TARGET)) return false;
  if (op == uint16_t(CommandKind::TARGET) && (value == 0 || value > MAX_TARGET)) return false;
  if (op != uint16_t(CommandKind::TARGET) && value != 0) return false;
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
      if (xQueueSend(commandQueue, &c, 0) != pdTRUE) error = 6;
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
  rs485.begin(BUS_BAUD, SERIAL_8N1, 39, 0);
  bool ready = bool(rs485) && rs485.setPins(39, 0, -1, 46) &&
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
              accepted=r[0]==0x4658 && r[1]==2 && r[5]<=uint16_t(RunState::FAULT) && r[6]<=uint16_t(StopReason::INPUT_IO);
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
</head><body><h1>RS-485 settings</h1><p id="device">Loading…</p><p>These settings apply to this physical unit. The same firmware runs on every unit.</p>
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
<div class="card"><p>Arm/resume at this treadmill: hold B for 1 second and release. Reset does not arm it.</p>
<div class="row"><button data-control onclick="cmd('pause')">Pause and stop</button><button data-control class="stop" onclick="cmd('stop')">Stop</button>
<button data-control onclick="cmd('ack')">Silence buzzer</button><button data-control onclick="resetNode()">Reset count</button></div>
<p id="notice" role="status"></p></div>
<div class="card"><h2>Node settings</h2><form onsubmit="setTarget(event)"><label for="newtarget">Target steps</label>
<input id="newtarget" type="number" min="1" max="1000000000" step="1" required><button id="saveTarget" data-control>Save target</button></form>
<p id="settings" class="muted"></p><p>Button presets and timeouts are configured in the sketch.</p></div>
<div class="card"><h2>Inputs and outputs</h2><dl id="details"></dl><p class="muted">Relay indicators show commanded coils, not contact feedback. Network Stop is not an emergency-stop circuit.</p></div></main>
<p><a href="/bus">RS-485 role and address</a> · <a href="/wifi">Wi-Fi settings</a> — for this web-hosting unit</p>
<script>
const el=id=>document.getElementById(id);let nodes=[],selected=null,host=null,fetching=false,stale=true;
const selectedNode=()=>nodes.find(n=>n.node===selected);
function render(){const s=selectedNode();if(!s)return;
el('tabs').replaceChildren();for(const n of nodes){let b=document.createElement('button');b.textContent=n.node===0?'Local (standalone)':`Node ${n.node}${n.node===host?' (local)':''}${n.online?'':' — OFFLINE'}`;
b.setAttribute('role','tab');b.setAttribute('aria-selected',String(n.node===selected));b.onclick=()=>{selected=n.node;el('newtarget').value='';el('notice').textContent='';render()};el('tabs').append(b)}
const online=s.online&&!stale;el('identity').textContent=`Node ${s.node} · ${s.device}`;
el('state').textContent=online?s.state:'OFFLINE — last known state: '+s.state;
el('count').textContent=s.count.toLocaleString();el('target').textContent=s.target.toLocaleString();
el('reason').textContent=s.reason+(s.reset_message?' | '+s.reset_message:'');
const transport=['','Command queued','Command accepted by node; check state/result','Command rejected / expired','No acknowledgement; outcome unknown. Check node before retrying.'];
el('freshness').textContent=(online?'Updated':'STALE')+` · sample age ${Math.round(s.age_ms/1000)}s`+(transport[s.command_status]?' · '+transport[s.command_status]:'');
el('settings').textContent=`Presets A/B/C: ${s.presets.join(' / ')} · No-pulse timeout: ${s.no_pulse_seconds}s · Arm timeout: ${s.arm_timeout_seconds}s`;
el('details').replaceChildren();const details=[['Optical input',s.sensor?'ACTIVE':'inactive'],['E-stop',s.stop_button?'PRESSED':'released'],['Input hardware',s.inputs_healthy?'OK':'FAULT'],['FRAM',s.fram?'OK':'FAULT'],['Wiring check',s.wiring_verified?'Enabled':'BENCH LOCK'],['Maximum scan gap',s.max_scan_gap_ms+' ms']];
['Treadmill permit','Green','Red','Buzzer'].forEach((name,i)=>details.push([`Relay ${i+1}: ${name}`,s.relays&(1<<i)?'ON':'OFF']));
for(const [k,v] of details){let dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;el('details').append(dt,dd)}
document.querySelectorAll('[data-control]').forEach(b=>b.disabled=!online);
el('saveTarget').disabled=!online||s.completed||['RUNNING','ARMED'].includes(s.state);
}
async function cmd(action,value){const id=selected,s=selectedNode();if(!s||!s.online||stale)return;
try{const r=await fetch(`/api/${action}?node=${id}`+(value===undefined?'':'&value='+encodeURIComponent(value)),{method:'POST',headers:{'X-Flux-Control':'1'}});
const message=await r.text();if(id===selected)el('notice').textContent=`Node ${id}: ${message}`;await refresh();}catch(e){el('notice').textContent=`Node ${id}: connection lost; command outcome unknown.`}}
function resetNode(){if(confirm(`Reset Node ${selected}'s step count? The treadmill must be stopped.`))cmd('reset')}
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
input{width:100%}button{cursor:pointer;background:#35b66a}.danger{background:#e34b4b;color:white}
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
    if(xQueueSend(busCommandQueue,&bc,0)!=pdTRUE) {
      xSemaphoreTake(snapshotMutex,portMAX_DELAY); peers[node].commandStatus=3; xSemaphoreGive(snapshotMutex);
      server.send(503,"text/plain","Bus queue full; command not sent"); return;
    }
    server.send(202,"text/plain","Queued for node; check status and result"); return;
  }
  if (xQueueSend(commandQueue, &c, 0) != pdTRUE) {
    server.send(503, "text/plain", "Busy; retry"); return;
  }
  server.send(202, "text/plain", "Queued; check state");
}

void serviceCommands() {
  Command c;
  // Bound work per scan; remote ARM is deliberately not enabled.
  if (xQueueReceive(commandQueue, &c, 0) != pdTRUE) return;
  if (c.deadline && int32_t(millis()-c.deadline)>=0) {
    if(c.kind==CommandKind::BUS_CONFIG) { uint8_t result=5; xQueueSend(busConfigReplyQueue,&result,0); }
    reportReset("Command expired; retry",true); return;
  }
  if(c.kind==CommandKind::BUS_CONFIG) {
    uint8_t result=applyBusSettings(c.value);
    xQueueSend(busConfigReplyQueue,&result,0); return;
  }
  if(busConfigRestartPending) return;
  switch (c.kind) {
    case CommandKind::STOP:
      requestTreadmillStop(StopReason::WEB, RunState::STOPPED); break;
    case CommandKind::PAUSE:
      requestTreadmillStop(StopReason::WEB, RunState::PAUSED); break;
    case CommandKind::ACK:
      buzzerOffAt = 0;
      setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
      // Acknowledge silences only; never clears a fault or enables movement.
      break;
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

void setupWebServer() {
  const char *headers[] = {"X-Flux-Control"};
  server.collectHeaders(headers, 1);
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
    if (!saveWifiSettings("", "")) {
      server.send(500, "text/plain", "Could not clear Wi-Fi settings"); return;
    }
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width'><h2>Wi-Fi cleared</h2><p>Wi-Fi is switching to access-point mode.</p>");
    scheduleNetworkRestart();
  });
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/nodes", HTTP_GET, sendNodes);
  server.on("/api/pause", HTTP_POST, []() { queueRequest(CommandKind::PAUSE); });
  server.on("/api/stop", HTTP_POST, []() {
    queueRequest(CommandKind::STOP);
  });
  server.on("/api/ack", HTTP_POST, []() {
    queueRequest(CommandKind::ACK);
  });
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
  centered(count, 36, TFT_WHITE, countSize);
  centered("TARGET " + u64(stepTarget), 64, TFT_WHITE, 1);
  const String presets = "A:" + u64(PRESETS[0]) + " B:" + u64(PRESETS[1]) + " C:" + u64(PRESETS[2]);
  centered(presets, 77, TFT_CYAN, 1);
  const bool showReset = resetMessage.length() && millis() - resetMessageAt < 8000;
  centered(showReset ? resetMessage : !framAvailable ? "FRAM ERROR" : !inputsHealthy ? "INPUT ERROR" :
           !WIRING_VERIFIED ? "BENCH LOCK" :
           (millis() / 4000) % 2 ? networkAddress : String("Hold B:Arm C:Reset"),
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
}

void handleLocalButtons() {
  M5StamPLC.update();
  // Resolve on release using elapsed time: a hold must never also select a preset.
  static uint32_t pressedAt[3] = {};
  static bool tracking[3] = {};
  for (uint8_t i = 0; i < 3; ++i) {
    auto &button = i == 0 ? M5.BtnA :
                   i == 1 ? M5.BtnB : M5.BtnC;
    if (button.wasPressed()) { pressedAt[i] = millis(); tracking[i] = true; }
    if (!button.wasReleased() || !tracking[i]) continue;
    tracking[i] = false;
    const uint32_t held = millis() - pressedAt[i];
    if (i == 1 && held >= 1000) { togglePause(); continue; }
    if (i == 2 && held >= 3000) { resetCount(); continue; }
    if (held >= 1000) continue;
    if (completionLatched || runState == RunState::RUNNING || runState == RunState::ARMED ||
        !framAvailable || captureCoast || millis() - lastObservedEdge < 3000) {
      M5StamPLC.tone(220, 100); continue;
    }
    presetIndex = i;
    stepTarget = PRESETS[i];
    writeFramRecord();
    M5StamPLC.tone(523, 60);
  }
}

void setup() {
  Serial.begin(115200);
  loadBusSettings();
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
  setTreadmillStopped(true);
  setRelay(GREEN_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  setRelay(RED_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);
  setRelay(EXTERNAL_BUZZER_RELAY, false, INDICATOR_RELAY_ACTIVE_LEVEL);

  loadFramState();
  char suffix[13];
  snprintf(suffix, sizeof(suffix), "%012llX", (unsigned long long)ESP.getEfuseMac());
  deviceName = "flux-" + String(suffix);
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
  handleLocalButtons();
  serviceCommands();
  serviceStopPulse();
  serviceExternalBeep();
  servicePulseTimeout();
  // Refresh indicator state if an I2C storage failure changed state internally.
  static RunState indicated = RunState::IDLE;
  if (runState != indicated) { indicated = runState; updateIndicators(); }
  publishStatus();
  drawDisplay();
  if(busConfigRestartPending && int32_t(millis()-busConfigRestartAt)>=0) {
    setTreadmillStopped(true);
    ESP.restart();
  }
  delay(1);
}
