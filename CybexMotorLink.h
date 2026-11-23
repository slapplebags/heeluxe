#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Captured base-controller protocol, NOT CSAFE.
// No Arduino dependencies.
class CybexMotorLink {
 public:
  enum class Phase : uint8_t { OFF, INITIALIZING, READY, FAULT };

  static constexpr uint32_t POLL_MS = 131;
  static constexpr uint32_t RESPONSE_MS = 100;
  static constexpr uint32_t FEEDBACK_MS = 2000;
  static constexpr size_t MAX_BODY = 64;
  static constexpr size_t MAX_WIRE = 2 * MAX_BODY + 2;
  static constexpr size_t RAW_HISTORY = 128;

  Phase phase = Phase::OFF;
  uint32_t txCount = 0, rxCount = 0, badFrames = 0;
  uint32_t timeouts = 0, unmatched = 0;
  uint32_t rawRxBytes = 0, rawTxBytes = 0;
  uint32_t diagnosticRevision = 0;

  uint16_t actualRaw = 0, reportedTargetRaw = 0;
  uint8_t speedStatus = 0;
  uint32_t feedbackSequence = 0;
  bool feedbackValid = false;

  static uint16_t crc(const uint8_t *p, size_t n) {
    uint16_t c = 0xffff;
    while (n--) {
      c ^= *p++;
      for (unsigned i = 0; i < 8; ++i)
        c = (c >> 1) ^ ((c & 1) ? 0xa001 : 0);
    }
    return c;
  }

  static size_t encode(uint8_t seq, uint16_t cmd,
                       const uint8_t *data, size_t n,
                       uint8_t *out, size_t cap,
                       uint8_t address = 0) {
    if (n > MAX_BODY - 7) return 0;
    uint8_t b[MAX_BODY];
    b[0] = uint8_t(5 + n);
    b[1] = address;
    b[2] = seq;
    b[3] = uint8_t(cmd >> 8);
    b[4] = uint8_t(cmd);
    if (n) memcpy(b + 5, data, n);

    uint16_t c = crc(b, 5 + n);
    b[5 + n] = uint8_t(c);
    b[6 + n] = uint8_t(c >> 8);

    if (cap < 2) return 0;
    size_t k = 0;
    out[k++] = 0xf0;
    for (size_t i = 0; i < n + 7; ++i) {
      if (b[i] >= 0xf0 && b[i] <= 0xf3) {
        if (k + 2 >= cap) return 0;
        out[k++] = 0xf3;
        out[k++] = b[i] & 3;
      } else {
        if (k + 1 >= cap) return 0;
        out[k++] = b[i];
      }
    }
    out[k++] = 0xf1;
    return k;
  }

  void begin(uint32_t now) {
    phase = Phase::INITIALIZING;
    desired = 0;
    sequence = initIndex = pollIndex = 0;
    pending = urgent = false;
    used = 0;
    inside = escaped = false;
    feedbackValid = false;
    actualRaw = reportedTargetRaw = 0;
    speedStatus = 0;
    nextAt = lastFeedback = bootAt = lastByte = now;

    // Clear per-attempt evidence; cumulative counters remain intact.
    rawHead = rawUsed = 0;
    lastTxSize = lastGoodSize = lastBadSize = 0;
    lastTxWritten = 0;
    rejectReason = "none";
    faultReason = "none";
    rejectedPending = false;
    rejectedExpectedCmd = 0;
    rejectedExpectedSeq = 0;
    rejectedExpectedTarget = 0;
    ++diagnosticRevision;
  }

  void disable() {
    phase = Phase::OFF;
    desired = 0;
    pending = urgent = false;
    inside = escaped = false;
    used = 0;
    feedbackValid = false;
    // Deliberately retain diagnostic evidence after a fault.
  }

  void fail() {
    phase = Phase::FAULT;
    desired = 0;
    pending = urgent = false;
    feedbackValid = false;
    ++diagnosticRevision;
  }

  bool ready(uint32_t now) const {
    return phase == Phase::READY && feedbackValid &&
           now - lastFeedback < FEEDBACK_MS;
  }

  bool stopped(uint32_t now) const {
    return ready(now) && actualRaw == 0 &&
           reportedTargetRaw == 0 && desired == 0;
  }

  bool setTarget(uint16_t raw, uint16_t maximum, uint32_t now) {
    if (raw > maximum || (raw && !ready(now))) return false;
    if (phase != Phase::READY) return raw == 0;
    desired = raw;
    urgent = true;
    return true;
  }

  void stop(uint32_t now) {
    desired = 0;
    if (phase == Phase::READY) {
      pending = false;
      urgent = true;
      nextAt = now;
    }
  }

  uint16_t requestedRaw() const { return desired; }

  uint32_t feedbackAge(uint32_t now) const {
    return feedbackValid ? now - lastFeedback : UINT32_MAX;
  }

  void startInitialization(uint32_t now) {
    begin(now);
  }

  template<class Port>
  void tick(Port &port, uint32_t now) {
    unsigned budget = 128;
    while (budget-- && port.available()) {
      int v = port.read();
      if (v >= 0) {
        uint8_t b = uint8_t(v);
        ++rawRxBytes;
        rawHistory[rawHead] = b;
        rawHead = (rawHead + 1) % RAW_HISTORY;
        if (rawUsed < RAW_HISTORY) ++rawUsed;
        ++diagnosticRevision;
        consume(b, now);
      }
    }

    if (phase == Phase::OFF || phase == Phase::FAULT) return;

    if (pending && now - sentAt >= RESPONSE_MS) {
      if (inside)
        rememberReject("Incomplete frame at response timeout", false);
      faultReason = "Matching response timeout";
      ++timeouts;
      fail();
      return;
    }

    if (phase == Phase::READY &&
        now - lastFeedback >= FEEDBACK_MS) {
      faultReason = "Speed feedback timeout";
      ++timeouts;
      fail();
      return;
    }

    if (pending || int32_t(now - nextAt) < 0) return;

    if (phase == Phase::INITIALIZING) {
      static const uint16_t cmds[] = {
        0x0002, 0x0113, 0x0114, 0x0113, 0x0114
      };
      static const uint32_t at[] = {0, 30, 45, 182, 199};

      if (now - bootAt < at[initIndex]) return;

      uint8_t data[18] = {};
      size_t n = 18;
      if (initIndex) {
        n = 3;
        data[0] = 6;
        data[1] = (initIndex & 1) ? 0xae : 0x5b;
        data[2] = 0;
      }
      send(port, now, cmds[initIndex], data, n);
      return;
    }

    static const uint16_t cycle[] = {
      0x0104, 0x0106, 0x0001, 0x0105, 0x0107, 0x0108,
      0x0109, 0x010a, 0x010b, 0x010c, 0x0115
    };

    uint16_t cmd = urgent ? 0x0104 : cycle[pollIndex];
    uint8_t data[3] = {};
    size_t n = 1;

    if (cmd == 0x0104) {
      n = 3;
      data[0] = uint8_t(desired >> 8);
      data[1] = uint8_t(desired);
    } else if (cmd == 0x0106 || cmd == 0x0001) {
      n = 2;
    }

    bool wasUrgent = urgent;
    if (send(port, now, cmd, data, n)) {
      urgent = false;
      if (!wasUrgent) pollIndex = (pollIndex + 1) % 11;
    }
  }

  // Call only while OFF/FAULT so USB logging cannot delay motor polling.
  // 'out' can be Arduino Serial or another Print-compatible object.
  template<class Output>
  void dumpDiagnostics(Output &out) const {
    out.println();
    out.println("=== MOTOR SERIAL DIAGNOSTICS ===");
    out.print("Phase: ");
    out.println(phaseName());
    out.print("Initialization replies accepted this attempt: ");
    out.println(unsigned(initIndex));

    out.print("TX frames: "); out.println(txCount);
    out.print("TX bytes accepted by UART: "); out.println(rawTxBytes);
    out.print("Raw RX bytes: "); out.println(rawRxBytes);
    out.print("Accepted RX frames: "); out.println(rxCount);
    out.print("Bad frames: "); out.println(badFrames);
    out.print("Unmatched frames: "); out.println(unmatched);
    out.print("Timeouts: "); out.println(timeouts);

    out.print("Fault: "); out.println(faultReason);
    out.print("Last rejection: "); out.println(rejectReason);

    out.print("Last request command: 0x");
    hex16(out, expectedCmd);
    out.print(" sequence: 0x");
    hex8(out, expectedSeq);
    out.println();

    out.print("Last TX write accepted ");
    out.print(lastTxWritten);
    out.print(" of ");
    out.print(lastTxSize);
    out.println(" bytes");
    hexDump(out, "Last TX wire bytes", lastTx, lastTxSize);

    out.println("Recent RX wire bytes (oldest first):");
    if (!rawUsed) {
      out.println("  (none)");
    } else {
      size_t start = (rawHead + RAW_HISTORY - rawUsed) % RAW_HISTORY;
      for (size_t i = 0; i < rawUsed; ++i) {
        hex8(out, rawHistory[(start + i) % RAW_HISTORY]);
        out.print(' ');
        if ((i + 1) % 16 == 0) out.println();
      }
      if (rawUsed % 16) out.println();
    }

    hexDump(out, "Last accepted decoded body",
            lastGood, lastGoodSize);
    hexDump(out, "Last rejected/partial decoded body",
            lastBad, lastBadSize);

    if (lastBadSize) {
      out.print("Expected when rejected: pending=");
      out.print(rejectedPending ? 1 : 0);
      out.print(" address=0x7F sequence=0x");
      hex8(out, rejectedExpectedSeq);
      out.print(" command=0x");
      hex16(out, rejectedExpectedCmd);
      out.print(" target=");
      out.println(rejectedExpectedTarget);

      out.print("Body bytes: ");
      out.print(lastBadSize);
      out.print("; declared length: ");
      out.println(unsigned(lastBad[0]));

      if (lastBadSize >= 5) {
        out.print("Received address=0x");
        hex8(out, lastBad[1]);
        out.print(" sequence=0x");
        hex8(out, lastBad[2]);
        out.print(" command=0x");
        hex16(out, uint16_t(lastBad[3] << 8 | lastBad[4]));
        out.println();
      }

      if (lastBadSize >= 7) {
        out.print("Payload bytes: ");
        out.println(lastBadSize - 7);
        out.print("CRC calculated=0x");
        hex16(out, crc(lastBad, lastBadSize - 2));
        out.print(" received=0x");
        hex16(out, uint16_t(lastBad[lastBadSize - 2] |
                           lastBad[lastBadSize - 1] << 8));
        out.println();
      }
    }

    out.println("=== END MOTOR DIAGNOSTICS ===");
  }

 private:
  uint8_t body[MAX_BODY] = {};
  uint8_t sequence = 0, expectedSeq = 0;
  uint8_t initIndex = 0, pollIndex = 0;
  uint16_t desired = 0, expectedCmd = 0, expectedTarget = 0;
  size_t used = 0;
  bool inside = false, escaped = false;
  bool pending = false, urgent = false;
  uint32_t nextAt = 0, sentAt = 0, lastFeedback = 0;
  uint32_t bootAt = 0, lastByte = 0;

  uint8_t rawHistory[RAW_HISTORY] = {};
  size_t rawHead = 0, rawUsed = 0;
  uint8_t lastTx[MAX_WIRE] = {};
  uint8_t lastGood[MAX_BODY] = {}, lastBad[MAX_BODY] = {};
  size_t lastTxSize = 0, lastTxWritten = 0;
  size_t lastGoodSize = 0, lastBadSize = 0;
  const char *rejectReason = "none";
  const char *faultReason = "none";
  bool rejectedPending = false;
  uint8_t rejectedExpectedSeq = 0;
  uint16_t rejectedExpectedCmd = 0, rejectedExpectedTarget = 0;

  const char *phaseName() const {
    switch (phase) {
      case Phase::OFF: return "OFF";
      case Phase::INITIALIZING: return "INITIALIZING";
      case Phase::READY: return "READY";
      case Phase::FAULT: return "FAULT";
    }
    return "UNKNOWN";
  }

  template<class Output>
  static void hex8(Output &out, uint8_t value) {
    static const char digits[] = "0123456789ABCDEF";
    out.print(digits[value >> 4]);
    out.print(digits[value & 15]);
  }

  template<class Output>
  static void hex16(Output &out, uint16_t value) {
    hex8(out, uint8_t(value >> 8));
    hex8(out, uint8_t(value));
  }

  template<class Output>
  static void hexDump(Output &out, const char *label,
                      const uint8_t *bytes, size_t n) {
    out.print(label);
    out.println(":");
    if (!n) {
      out.println("  (none)");
      return;
    }
    for (size_t i = 0; i < n; ++i) {
      hex8(out, bytes[i]);
      out.print(' ');
      if ((i + 1) % 16 == 0) out.println();
    }
    if (n % 16) out.println();
  }

  void rememberReject(const char *reason, bool countBad = true) {
    rejectReason = reason;
    lastBadSize = used;
    if (used) memcpy(lastBad, body, used);
    rejectedPending = pending;
    rejectedExpectedSeq = expectedSeq;
    rejectedExpectedCmd = expectedCmd;
    rejectedExpectedTarget = expectedTarget;
    if (countBad) ++badFrames;
    ++diagnosticRevision;
  }

  void rejectUnmatched(const char *reason) {
    ++unmatched;
    rememberReject(reason, false);
  }

  template<class Port>
  bool send(Port &p, uint32_t now, uint16_t cmd,
            const uint8_t *data, size_t n) {
    uint8_t wire[MAX_WIRE];
    size_t size = encode(sequence, cmd, data, n,
                         wire, sizeof(wire));
    if (!size || p.availableForWrite() < int(size)) return false;

    expectedSeq = sequence++;
    expectedCmd = cmd;
    expectedTarget = cmd == 0x0104
      ? uint16_t(data[0] << 8 | data[1]) : 0;
    pending = true;
    sentAt = now;
    nextAt = now + POLL_MS;

    memcpy(lastTx, wire, size);
    lastTxSize = size;
    lastTxWritten = p.write(wire, size);
    rawTxBytes += uint32_t(lastTxWritten);
    ++diagnosticRevision;

    if (lastTxWritten != size) {
      faultReason = "UART accepted only part of TX frame";
      fail();
      return false;
    }

    ++txCount;
    return true;
  }

  void consume(uint8_t b, uint32_t now) {
    if (inside && now - lastByte >= RESPONSE_MS) {
      rememberReject("Gap inside frame exceeded response timeout");
      inside = escaped = false;
      used = 0;
    }
    lastByte = now;

    if (b == 0xf0) {
      // Record a restart without changing the original bad-frame counter.
      if (inside)
        rememberReject("New frame start before previous frame ended", false);
      inside = true;
      escaped = false;
      used = 0;
      return;
    }
    if (!inside) return;

    if (escaped) {
      if (b > 3) {
        rememberReject("Invalid escape byte; expected 00..03");
        inside = escaped = false;
        used = 0;
        return;
      }
      b = uint8_t(0xf0 | b);
      escaped = false;
    } else if (b == 0xf3) {
      escaped = true;
      return;
    } else if (b == 0xf1) {
      inside = false;
      frame(now);
      used = 0;
      return;
    } else if (b == 0xf2) {
      rememberReject("Unexpected unescaped F2 inside frame");
      inside = false;
      used = 0;
      return;
    }

    if (used >= MAX_BODY) {
      rememberReject("Decoded frame exceeds buffer");
      inside = false;
      used = 0;
      return;
    }
    body[used++] = b;
  }

  void frame(uint32_t now) {
    if (used < 7) {
      rememberReject("Frame shorter than minimum 7-byte body");
      return;
    }
    if (body[0] < 5 || size_t(body[0]) + 2 != used) {
      rememberReject("Declared length does not match decoded body");
      return;
    }
    if (crc(body, used - 2) !=
        uint16_t(body[used - 2] | body[used - 1] << 8)) {
      rememberReject("CRC mismatch");
      return;
    }

    uint16_t cmd = uint16_t(body[3] << 8 | body[4]);
    size_t n = used - 7;
    const uint8_t *data = body + 5;

    if (!pending) {
      rejectUnmatched("Valid frame but no request pending");
      return;
    }
    if (body[1] != 0x7f) {
      rejectUnmatched("Reply address is not 7F; possible echo");
      return;
    }
    if (body[2] != expectedSeq) {
      rejectUnmatched("Reply sequence does not match pending request");
      return;
    }
    if (cmd != expectedCmd) {
      rejectUnmatched("Reply command does not match pending request");
      return;
    }

    if (cmd == 0x0002 && n != 18) {
      rememberReject("0002 reply payload length is not 18");
      return;
    }
    // 0113: accept the original 3-byte format and the verified
    // 5-byte response from this treadmill.
    if (cmd == 0x0113 && n != 3 && n != 5) {
      rememberReject("0113 reply payload length is not 3 or 5");
      return;
    }
    // Keep the existing 0114 check until its response is captured.
    if (cmd == 0x0114 && n != 3 && n != 5) {
      rememberReject("0114 reply payload length is not 3 or 5");
      return;
    }
    if ((cmd == 0x0104 || cmd == 0x0105) && n != 5) {
      rememberReject("0104/0105 reply payload length is not 5");
      return;
    }
    if (!n) {
      rememberReject("Reply has empty payload");
      return;
    }

    if (cmd == 0x0113 || cmd == 0x0114) {
      if (data[0] != 6 ||
          data[1] != (cmd == 0x0113 ? 0xae : 0x5b)) {
        rememberReject(cmd == 0x0113
          ? "0113 reply payload does not begin 06 AE"
          : "0114 reply payload does not begin 06 5B");
        return;
      }
    }

    if (cmd == 0x0104 &&
        uint16_t(data[2] << 8 | data[3]) != expectedTarget) {
      rememberReject("0104 returned target differs from requested target");
      return;
    }

    memcpy(lastGood, body, used);
    lastGoodSize = used;
    pending = false;
    ++rxCount;
    ++diagnosticRevision;

    if (phase == Phase::INITIALIZING) {
      if (++initIndex == 5) {
        phase = Phase::READY;
        lastFeedback = now;
        nextAt = now + 83;
        pollIndex = 0;
      } else {
        nextAt = now;
      }
    }

    if (cmd == 0x0104 || cmd == 0x0105) {
      actualRaw = uint16_t(data[0] << 8 | data[1]);
      reportedTargetRaw = uint16_t(data[2] << 8 | data[3]);
      ++feedbackSequence;
      speedStatus = data[4];
      lastFeedback = now;
      feedbackValid = true;
    }
  }
};