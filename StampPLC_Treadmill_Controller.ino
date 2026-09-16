/*
  StampPLC_Treadmill_Controller.ino

  Standalone controller for ONE treadmill / Time Machine.

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
  Network task never accesses hardware; requests go through a bounded queue.
  No RS485, OTA, email, cloud service or automatic motor-start in this version.
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
     release to reset count while stopped and settled. Use mushroom for STOP.
     Web acknowledge only silences; reset clears count/recovery fault once
     inputs are quiet. Reboot required after repairing a runtime FRAM fault.
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

// ---------------------------------------------------------------------------
// Installation configuration -- verify these during bench commissioning.
// ---------------------------------------------------------------------------

// Optional factory defaults. Normally leave these blank and configure Wi-Fi
// from the fallback access point's web page. Saved settings override defaults.
constexpr char WIFI_SSID[]     = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char AP_PASSWORD[]   = "fluxcapacitor"; // minimum 8 characters
constexpr char WEB_PASSWORD[]  = "change-this-password";
constexpr bool WIRING_VERIFIED = true; // this unit has completed bench wiring checks
constexpr uint32_t MAX_TARGET = 1000000000;

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
enum class CommandKind : uint8_t { ARM, PAUSE, STOP, ACK, RESET, TARGET };
struct Command { CommandKind kind; uint32_t value; };
QueueHandle_t commandQueue;
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
  if (completionLatched || !inputsHealthy || !WIRING_VERIFIED || !framAvailable || stopAuxStable || stopAuxCandidate ||
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
  json += "\"wiring_verified\":" + String(WIRING_VERIFIED ? "true" : "false") + "}";
  return json;
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flux Capacitor</title><style>
body{font-family:system-ui;background:#15181d;color:#eee;max-width:720px;margin:2rem auto;padding:0 1rem}
.card{background:#222831;border-radius:12px;padding:1.2rem;margin-bottom:1rem}.count{font-size:3rem;font-weight:700}
button,input{font:inherit;padding:.7rem;margin:.25rem;border-radius:7px;border:0}button{cursor:pointer}
.go{background:#35b66a}.stop{background:#e34b4b}.muted{color:#adb5bd}.row{display:flex;flex-wrap:wrap;gap:.4rem}
</style></head><body><h1>Flux Capacitor</h1><div class="card"><div id="state">Loading...</div>
<div class="count"><span id="count">-</span> / <span id="target">-</span></div><div class="muted" id="reason"></div></div>
<div class="card row"><span>A/B/C select targets. Hold B for 1 second and release to arm/pause. Hold C for 3 seconds to reset while stopped.</span><button onclick="cmd('/api/pause')">Pause and stop</button>
<button class="stop" onclick="cmd('/api/stop')">Stop</button><button onclick="cmd('/api/ack')">Acknowledge</button>
<button onclick="if(confirm('Reset the step count?'))cmd('/api/reset')">Reset count</button></div>
<div class="card"><form onsubmit="setTarget(event)"><label>Target steps </label><input id="newtarget" type="number" min="1" max="1000000000" step="1" required>
<button type="submit">Set target</button></form></div>
<div class="card"><a href="/wifi" style="color:#7fd3ff">Wi-Fi settings</a></div>
<div class="muted" id="details"></div>
<script>
const el=id=>document.getElementById(id);
async function cmd(u){let r=await fetch(u,{method:'POST',headers:{'X-Flux-Control':'1'}});if(!r.ok)alert(await r.text());await refresh()}
async function setTarget(e){e.preventDefault();await cmd('/api/target?value='+encodeURIComponent(el('newtarget').value))}
async function refresh(){try{let s=await(await fetch('/api/status')).json();el('state').textContent=s.state;el('count').textContent=s.count.toLocaleString();
el('target').textContent=s.target.toLocaleString();el('reason').textContent=s.reason+(s.reset_message?' | '+s.reset_message:'');
el('details').textContent=`${s.device} | sensor ${s.sensor?'ON':'off'} | stop ${s.stop_button?'PRESSED':'released'} | FRAM ${s.fram?'OK':'MISSING'} | max scan gap ${s.max_scan_gap_ms}ms | wiring ${s.wiring_verified?'enabled':'BENCH LOCK'}`;}catch(e){el('state').textContent='Connection lost'}}
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
<p class="hint">Use a blank password only for an open network. After saving, the controller restarts and joins this network. Its new address appears on the display.</p>
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
  Command c{kind, value};
  if (xQueueSend(commandQueue, &c, 0) != pdTRUE) {
    server.send(503, "text/plain", "Busy; retry"); return;
  }
  server.send(202, "text/plain", "Queued; check state");
}

void serviceCommands() {
  Command c;
  // Bound work per scan; remote ARM is deliberately not enabled.
  if (xQueueReceive(commandQueue, &c, 0) != pdTRUE) return;
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
      if (!completionLatched && runState != RunState::RUNNING && runState != RunState::ARMED) {
        stepTarget = c.value; writeFramRecord();
      }
      break;
    case CommandKind::ARM: break;
  }
}

void setupWebServer() {
  const char *headers[] = {"X-Flux-Control"};
  server.collectHeaders(headers, 1);
  server.on("/", HTTP_GET, []() { if (authenticated()) server.send_P(200, "text/html", PAGE); });
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
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width'><h2>Wi-Fi saved</h2><p>The controller is restarting. Its new address will appear on the display.</p>");
    scheduleNetworkRestart();
  });
  server.on("/api/wifi/clear", HTTP_POST, []() {
    if (!authenticated()) return;
    if (!saveWifiSettings("", "")) {
      server.send(500, "text/plain", "Could not clear Wi-Fi settings"); return;
    }
    server.send(200, "text/html", "<!doctype html><meta name=viewport content='width=device-width'><h2>Wi-Fi cleared</h2><p>The controller is restarting in access-point mode.</p>");
    scheduleNetworkRestart();
  });
  server.on("/api/status", HTTP_GET, sendStatus);
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

void setupWifi() {
  loadWifiSettings();
  WiFi.setHostname(deviceName.c_str());

  if (configuredWifiSsid.length() == 0) {
    startFallbackAp();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(configuredWifiSsid.c_str(), configuredWifiPassword.c_str());
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_MS) delay(100);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi: http://%s  (%s)\n", deviceName.c_str(), WiFi.localIP().toString().c_str());
  } else {
    startFallbackAp();
  }
}

void networkTask(void *) {
  setupWifi();
  setupWebServer();
  for (;;) {
    serviceWifi();
    server.handleClient();
    if (networkRestartRequested && static_cast<int32_t>(millis() - networkRestartAt) >= 0) {
      delay(50);
      ESP.restart();
    }
    String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    xSemaphoreTake(snapshotMutex, portMAX_DELAY);
    sharedAddress = ip;
    xSemaphoreGive(snapshotMutex);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void publishStatus() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();
  String s = jsonStatus();
  xSemaphoreTake(snapshotMutex, portMAX_DELAY);
  sharedStatus = s;
  networkAddress = sharedAddress;
  xSemaphoreGive(snapshotMutex);
}

void serviceWifi() {
  if (WiFi.status() == WL_CONNECTED || configuredWifiSsid.length() == 0) return;
  if (millis() - lastWifiAttemptAt >= 30000) {
    lastWifiAttemptAt = millis();
    WiFi.reconnect();
    if (!fallbackApStarted) startFallbackAp();
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
  // No Modbus service in this standalone firmware. UART activity alone is
  // not proof of a healthy link; future OK must require valid peer replies.
  badge(2 * third + 2, w - 2 * third - 4, "485 OFF", dim);

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
  if (!commandQueue || !snapshotMutex) { while (true) delay(1000); }
  sharedStatus = jsonStatus();
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
  delay(1);
}
