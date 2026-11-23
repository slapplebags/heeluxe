#pragma once
#include <stdint.h>

// Keep command types in a header so they are visible before Arduino-generated
// function prototypes in the sketch translation unit.
enum class CommandKind : uint8_t { ARM, PAUSE, STOP, ACK, RESET, TARGET, BUS_CONFIG, CSAFE_START, CLEAR_FAULT, SPEED };
struct Command { CommandKind kind; uint32_t value; uint32_t deadline = 0; };
