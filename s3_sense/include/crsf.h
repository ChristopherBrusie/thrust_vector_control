#pragma once
#include <Arduino.h>

/* ============================================================
   CRSF (Crossfire Serial Protocol) Parser for ExpressLRS
   ============================================================
   ExpressLRS receivers output CRSF at 420,000 baud over UART.
   Wire: receiver TX  →  ESP32 RX pin (one wire is enough for RC input).
         receiver RX  →  ESP32 TX pin (optional; needed only for telemetry).

   Packet structure (RC_CHANNELS_PACKED, type 0x16):
     [0]        0xC8  sync / device address
     [1]        len   payload length + 2 (type byte + CRC byte)
     [2]        0x16  frame type
     [3..24]    22 bytes — 16 channels × 11 bits packed LSB-first
     [25]       CRC8(DVB-S2) over bytes [2..24]

   Channel value range : 172 (min) … 992 (center) … 1811 (max)
   Mapped to µs range  : 1000 … 1500 … 2000
   ============================================================ */

// ---- Protocol constants ----
static constexpr uint8_t  CRSF_SYNC_BYTE         = 0xC8;
static constexpr uint8_t  CRSF_FRAMETYPE_CHANNELS = 0x16;
static constexpr uint8_t  CRSF_MAX_FRAME_LEN      = 64;
static constexpr uint16_t CRSF_CH_MIN             = 172;
static constexpr uint16_t CRSF_CH_MID             = 992;
static constexpr uint16_t CRSF_CH_MAX             = 1811;
static constexpr int      CRSF_NUM_CHANNELS        = 16;

// ---- Failsafe timeout ----
static constexpr uint32_t CRSF_FAILSAFE_MS = 1000;   // ms without a valid packet → failsafe

/* ----------------------------------------------------------
   Inline helpers
   ---------------------------------------------------------- */

// Raw CRSF value (172-1811) → PWM microseconds (1000-2000)
inline int crsfToUs(uint16_t v) {
  return (int)map((long)v, CRSF_CH_MIN, CRSF_CH_MAX, 1000, 2000);
}

// Raw CRSF value → float in [-1, +1]  (0 at center)
inline float crsfToNorm(uint16_t v) {
  return constrain(
    (float)((int)v - (int)CRSF_CH_MID) / (float)(CRSF_CH_MAX - CRSF_CH_MID),
    -1.0f, 1.0f
  );
}

// Raw CRSF value → float in [minOut, maxOut]
inline float crsfToRange(uint16_t v, float minOut, float maxOut) {
  return crsfToNorm(v) * (maxOut - minOut) / 2.0f + (maxOut + minOut) / 2.0f;
}

// Is a switch channel "high" (>= 1700 µs equivalent)?
inline bool crsfSwitchHigh(uint16_t v) { return v >= 1750; }

/* ----------------------------------------------------------
   CRC-8 DVB-S2  (polynomial 0xD5)
   ---------------------------------------------------------- */
static inline uint8_t crsf_crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

/* ----------------------------------------------------------
   CRSFReceiver class
   ---------------------------------------------------------- */
class CRSFReceiver {
public:
  // Public channel array — updated on every valid packet.
  // Index: 0-based (CH1 = channels[0]).
  uint16_t channels[CRSF_NUM_CHANNELS];

  // Housekeeping
  uint32_t lastPacketMs  = 0;
  uint32_t packetCount   = 0;   // total good packets received
  uint32_t crcErrors     = 0;   // CRC mismatches (sanity check)
  bool     packetReady   = false;  // true for one loop() after a new packet arrives

  // ---- Initialise ----
  void begin(HardwareSerial& serial, int rxPin, int txPin = -1,
             uint32_t baud = 420000) {
    _serial = &serial;
    // SERIAL_8N1 is the correct CRSF framing.
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    _resetState();
    // Safe defaults: all sticks centred, throttle at bottom.
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) channels[i] = CRSF_CH_MID;
    channels[2] = CRSF_CH_MIN;   // throttle channel (CH3, index 2) → min
  }

  // ---- Call once per main loop iteration — non-blocking ----
  void update() {
    packetReady = false;
    if (!_serial) return;
    while (_serial->available()) {
      if (_processByte((uint8_t)_serial->read()))
        packetReady = true;   // a complete, valid packet was just decoded
    }
  }

  // ---- Failsafe detection ----
  bool isFailsafe() const {
    return (lastPacketMs == 0) || ((millis() - lastPacketMs) > CRSF_FAILSAFE_MS);
  }

  // ---- Convenience: print current channel values over Serial ----
  void printChannels() const {
    Serial.print("[CRSF] ");
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
      Serial.print("CH"); Serial.print(i + 1); Serial.print(':');
      Serial.print(crsfToUs(channels[i]));
      if (i < CRSF_NUM_CHANNELS - 1) Serial.print(' ');
    }
    Serial.println();
  }

private:
  HardwareSerial* _serial = nullptr;

  // Parser state machine
  enum class State : uint8_t { SYNC, LENGTH, PAYLOAD };
  State   _state       = State::SYNC;
  uint8_t _buf[CRSF_MAX_FRAME_LEN];
  uint8_t _bufIdx      = 0;
  uint8_t _frameLen    = 0;   // total expected bytes (sync + len + payload + crc)

  void _resetState() {
    _state  = State::SYNC;
    _bufIdx = 0;
  }

  // Returns true when a complete valid RC packet is decoded.
  bool _processByte(uint8_t b) {
    switch (_state) {

      case State::SYNC:
        if (b == CRSF_SYNC_BYTE) {
          _buf[0] = b;
          _bufIdx = 1;
          _state  = State::LENGTH;
        }
        return false;

      case State::LENGTH:
        // "len" field = number of remaining bytes (type + payload + CRC).
        // Total frame = 2 (sync + len) + len.
        if (b < 2 || b > (CRSF_MAX_FRAME_LEN - 2)) {
          _resetState();   // implausible length — resync
          return false;
        }
        _buf[1]   = b;
        _frameLen = b + 2;   // include sync and len bytes
        _bufIdx   = 2;
        _state    = State::PAYLOAD;
        return false;

      case State::PAYLOAD:
        _buf[_bufIdx++] = b;
        if (_bufIdx < _frameLen) return false;   // not done yet

        // Full frame received — validate and decode.
        _resetState();
        return _parseFrame();
    }
    return false;  // unreachable
  }

  bool _parseFrame() {
    // Frame layout:
    //   buf[0]            sync
    //   buf[1]            len  (payload+2)
    //   buf[2]            type
    //   buf[3 .. end-1]   payload
    //   buf[end]          CRC over buf[2..end-1]
    uint8_t type        = _buf[2];
    uint8_t totalBytes  = _buf[1] + 2;
    uint8_t crcReceived = _buf[totalBytes - 1];

    // CRC covers type byte + payload (not sync, len, or crc itself).
    uint8_t crcCalc = crsf_crc8(&_buf[2], (uint8_t)(_buf[1] - 1));
    if (crcCalc != crcReceived) {
      crcErrors++;
      return false;
    }

    if (type != CRSF_FRAMETYPE_CHANNELS) return false;  // not an RC packet

    _decodeChannels(&_buf[3]);
    lastPacketMs = millis();
    packetCount++;
    return true;
  }

  // Unpack 16 × 11-bit channels from 22 bytes, LSB-first.
  void _decodeChannels(const uint8_t* p) {
    channels[ 0] = ((uint16_t)p[ 0]       | (uint16_t)p[ 1] << 8)         & 0x7FF;
    channels[ 1] = ((uint16_t)p[ 1] >> 3  | (uint16_t)p[ 2] << 5)         & 0x7FF;
    channels[ 2] = ((uint16_t)p[ 2] >> 6  | (uint16_t)p[ 3] << 2
                  | (uint16_t)p[ 4] << 10)                                  & 0x7FF;
    channels[ 3] = ((uint16_t)p[ 4] >> 1  | (uint16_t)p[ 5] << 7)         & 0x7FF;
    channels[ 4] = ((uint16_t)p[ 5] >> 4  | (uint16_t)p[ 6] << 4)         & 0x7FF;
    channels[ 5] = ((uint16_t)p[ 6] >> 7  | (uint16_t)p[ 7] << 1
                  | (uint16_t)p[ 8] << 9)                                   & 0x7FF;
    channels[ 6] = ((uint16_t)p[ 8] >> 2  | (uint16_t)p[ 9] << 6)         & 0x7FF;
    channels[ 7] = ((uint16_t)p[ 9] >> 5  | (uint16_t)p[10] << 3)         & 0x7FF;
    channels[ 8] = ((uint16_t)p[11]       | (uint16_t)p[12] << 8)         & 0x7FF;
    channels[ 9] = ((uint16_t)p[12] >> 3  | (uint16_t)p[13] << 5)         & 0x7FF;
    channels[10] = ((uint16_t)p[13] >> 6  | (uint16_t)p[14] << 2
                  | (uint16_t)p[15] << 10)                                  & 0x7FF;
    channels[11] = ((uint16_t)p[15] >> 1  | (uint16_t)p[16] << 7)         & 0x7FF;
    channels[12] = ((uint16_t)p[16] >> 4  | (uint16_t)p[17] << 4)         & 0x7FF;
    channels[13] = ((uint16_t)p[17] >> 7  | (uint16_t)p[18] << 1
                  | (uint16_t)p[19] << 9)                                   & 0x7FF;
    channels[14] = ((uint16_t)p[19] >> 2  | (uint16_t)p[20] << 6)         & 0x7FF;
    channels[15] = ((uint16_t)p[20] >> 5  | (uint16_t)p[21] << 3)         & 0x7FF;
  }
};