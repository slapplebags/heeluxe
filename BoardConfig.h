#pragma once
// One source tree, two hardware builds. Select BEFORE compiling/uploading.
#define FLUX_STAMPLC 1
#define FLUX_WAVESHARE_RELAY1 2
#ifndef FLUX_BOARD
#define FLUX_BOARD FLUX_STAMPLC
#endif

// TREADMILL CONTROL SWITCH: change this one line, then compile/upload.
// SERIAL: StampPLC replaces console; commands start/stop/speed through MAX3232.
// RELAY: original console controls start/speed; R1 opens a serial conductor to stop.
#define FLUX_CONTROL_RELAY 1
#define FLUX_CONTROL_SERIAL 2
#ifndef FLUX_CONTROL_MODE
#define FLUX_CONTROL_MODE FLUX_CONTROL_SERIAL
#endif
#if FLUX_CONTROL_MODE != FLUX_CONTROL_RELAY && FLUX_CONTROL_MODE != FLUX_CONTROL_SERIAL
#error "Select FLUX_CONTROL_RELAY or FLUX_CONTROL_SERIAL"
#endif

#if FLUX_BOARD == FLUX_WAVESHARE_RELAY1
constexpr bool BOARD_HAS_DISPLAY = false;
constexpr uint16_t BOARD_CODE = 2;
constexpr char BOARD_NAME[] = "Waveshare Relay-1CH";
constexpr char BOARD_OTA_MARKER[] = "FLUX_WAVESHARE_RELAY1_TREADMILL_OTA_V1";
// Keep false until the disconnected-treadmill bench checks in README pass.
constexpr bool BOARD_WIRING_VERIFIED = false;
constexpr int FRAM_SDA = 2, FRAM_SCL = 1;
constexpr int SENSOR_GPIO = 4, ESTOP_GPIO = 5, ARM_GPIO = 6;
constexpr int RELAY_GPIO = 47;
constexpr bool RELAY_GPIO_ON = true;
constexpr bool BOARD_SENSOR_ACTIVE = false;
constexpr bool BOARD_ESTOP_ACTIVE = false; // NO dry contact to logic GND
constexpr int RS485_RX = 18, RS485_TX = 17, RS485_DE = 21;
#elif FLUX_BOARD == FLUX_STAMPLC
constexpr bool BOARD_HAS_DISPLAY = true;
constexpr uint16_t BOARD_CODE = 1;
constexpr char BOARD_NAME[] = "StampPLC";
constexpr char BOARD_OTA_MARKER[] = "FLUX_STAMPLC_TREADMILL_OTA_V1";
constexpr bool BOARD_WIRING_VERIFIED = true; // preserve commissioned v8 setting
constexpr int FRAM_SDA = 2, FRAM_SCL = 1;
constexpr bool BOARD_SENSOR_ACTIVE = false;
constexpr bool BOARD_ESTOP_ACTIVE = true; // preserve commissioned opto polarity
constexpr int RS485_RX = 39, RS485_TX = 0, RS485_DE = 46;
#else
#error "Unknown FLUX_BOARD"
#endif

// MAX3232 CSAFE support: enabled for StampPLC PORT.C only.
// Waveshare GPIO4/5 already serve sensor/E-stop: DO NOT reuse those pins.
#if FLUX_BOARD == FLUX_STAMPLC
constexpr bool CSAFE_ENABLED = false; // UART2 now speaks captured base protocol
constexpr int CSAFE_RX = 4, CSAFE_TX = 5;
#else
constexpr bool CSAFE_ENABLED = false;
constexpr int CSAFE_RX = -1, CSAFE_TX = -1;
#endif

// Tested 750T CSAFE state commands did not actuate the belt.
// Keep motor commands disabled; CSAFE read-only monitoring stays available.
constexpr bool CSAFE_MOTOR_CONTROL_ENABLED = false;

// Console replacement prototype. Verify TX/RX direction on the ACTUAL base cable.
// PORT.C pin assignment is a build choice, not a claim about treadmill pinout.
constexpr bool MOTOR_ENABLED = FLUX_BOARD == FLUX_STAMPLC && FLUX_CONTROL_MODE == FLUX_CONTROL_SERIAL;
constexpr bool MOTOR_COMMISSIONED = true; // verify wiring/init with belt clear, then set true
constexpr int MOTOR_RX = 4, MOTOR_TX = 5;
constexpr uint32_t MOTOR_BAUD = 19200;
constexpr uint32_t MOTOR_MAX_MPH_X100 = 100; // prototype cap: tested 1.00 mph
constexpr uint32_t MOTOR_DEFAULT_MPH_X100 = 50;
constexpr uint32_t MOTOR_STOP_LIMIT_MS = 5000; // fallback disconnect deadline; not measured watchdog
static_assert(MOTOR_DEFAULT_MPH_X100 > 0 && MOTOR_DEFAULT_MPH_X100 <= MOTOR_MAX_MPH_X100, "Invalid motor speed defaults");
static_assert(MOTOR_MAX_MPH_X100 <= 100, "Only 0.5 and 1.0 mph calibrated; validate before raising prototype cap");
static_assert(!(MOTOR_ENABLED && CSAFE_ENABLED), "UART2 cannot run both motor protocol and CSAFE");

// Standalone StampPLC front-panel edit increments.
constexpr uint32_t LOCAL_SPEED_STEP_X100 = 5; // 0.05 mph; minimum selectable nonzero speed
constexpr uint32_t LOCAL_TARGET_STEP = 1000;
static_assert(LOCAL_SPEED_STEP_X100 > 0 && LOCAL_SPEED_STEP_X100 <= MOTOR_MAX_MPH_X100, "Invalid local speed increment");
static_assert(LOCAL_TARGET_STEP > 0 && LOCAL_TARGET_STEP <= 1000000000, "Invalid local step increment");

// Serial target-completion parking. Verify OPT with the disk stationary:
// -1 disabled until verified, 1 slot reads OPT ON, 0 slot reads OPT OFF.
constexpr int PARK_SLOT_LEVEL = -1;
constexpr uint32_t PARK_JOG_LIMIT_MS = 1200;
constexpr uint32_t PARK_SPEED_X100 = 50; // 0.50 mph, calibrated capture point
static_assert(PARK_SLOT_LEVEL>=-1 && PARK_SLOT_LEVEL<=1 && PARK_SPEED_X100<=MOTOR_MAX_MPH_X100, "Invalid parking setup");
