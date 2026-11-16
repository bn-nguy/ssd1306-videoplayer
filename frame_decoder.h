// frame_decoder.h - streaming decoder for frames_data.h (made by compress_frames.py)
//
// Decodes one frame per call straight into an SSD1306 page-format buffer
// (byte = 8 vertical pixels, bit 0 = top), using the previous frame that is
// still in that buffer as context. No second frame buffer is needed.
// RAM used: ~6.7 KB (probability tables 6 KB + row windows 0.5 KB).
#pragma once
#include <stdint.h>
#include <string.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#define pgm_read_byte(a) (*(const uint8_t *)(a))
#endif
#include "frames_data.h"

class FrameDecoder {
 public:
  // buf    : display buffer (Adafruit_SSD1306::getBuffer())
  // stride : display width in pixels (bytes per page row), e.g. 128
  // xoff   : display column where the frame's left edge goes, e.g. (128 - FRAME_W) / 2
  void begin(uint8_t *buf, uint8_t stride, uint8_t xoff) {
    buf_ = buf;
    stride_ = stride;
    xoff_ = xoff;
    restart();
  }

  // Start the stream over (clears the display buffer, resets the model).
  void restart() {
    memset(buf_, 0, (size_t)stride_ * ((FRAME_H + 7) / 8));
    src_ = frame_stream;
    range_ = 0xFFFFFFFFu;
    code_ = 0;
    for (uint8_t i = 0; i < 4; i++) code_ = (code_ << 8) | pgm_read_byte(src_++);
    for (uint16_t i = 0; i < NCTX; i++) {
      prob_[i] = 32768;
      cnt_[i] = 0;
    }
    frame_ = 0;
  }

  // Decode the next frame into the buffer; loops back to the first frame at the end.
  void nextFrame() {
    if (frame_ >= FRAME_COUNT) restart();

    uint8_t *cNN = rows_[0], *cN = rows_[1], *cC = rows_[2];  // current frame rows y-2, y-1, y
    uint8_t *pN = rows_[3], *pC = rows_[4], *pS = rows_[5];   // previous frame rows y-1, y, y+1
    memset(rows_, 0, sizeof(rows_));
    extractRow(pC, 0);
    if (FRAME_H > 1) extractRow(pS, 1);

    uint32_t range = range_, code = code_;
    const uint8_t *src = src_;
    uint16_t *prob = prob_;
    uint8_t *cnt = cnt_;

    for (uint16_t y = 0; y < FRAME_H; y++) {
      for (uint16_t i = 2; i < FRAME_W + 2u; i++) {
        uint16_t ctx = cN[i] | (cC[i - 1] << 1) | (cN[i - 1] << 2) | (cN[i + 1] << 3) |
                       (cC[i - 2] << 4) | (cNN[i] << 5) | (pC[i] << 6) | (pN[i] << 7) |
                       (pS[i] << 8) | (pC[i - 1] << 9) | (pC[i + 1] << 10);
        int32_t p = prob[ctx];
        uint32_t bound = (uint32_t)(((uint64_t)range * (uint32_t)p) >> 16);
        uint8_t bit;
        int32_t d;
        if (code < bound) {
          range = bound;
          bit = 0;
          d = 65535 - p;
        } else {
          code -= bound;
          range -= bound;
          bit = 1;
          d = -p;
        }
        uint8_t c = cnt[ctx];
        p += d / (int32_t)(c + 2);
        prob[ctx] = (uint16_t)(p < 32 ? 32 : (p > 65503 ? 65503 : p));
        if (c < 30) cnt[ctx] = c + 1;
        while (range < 0x1000000u) {
          range <<= 8;
          code = (code << 8) | pgm_read_byte(src++);
        }
        cC[i] = bit;
      }

      writeRow(cC, y);

      uint8_t *t = cNN;  // rotate current-frame rows
      cNN = cN;
      cN = cC;
      cC = t;
      t = pN;  // rotate previous-frame rows
      pN = pC;
      pC = pS;
      pS = t;
      if (y + 2 < FRAME_H) extractRow(pS, y + 2);
      else memset(pS, 0, ROWLEN);
    }

    range_ = range;
    code_ = code;
    src_ = src;
    frame_++;
  }

  uint16_t frameIndex() const { return frame_; }

 private:
  static constexpr uint16_t NCTX = 2048;
  static constexpr uint16_t ROWLEN = FRAME_W + 4;  // 2 zero pixels of padding each side

  // Previous-frame row y (from the display buffer) -> one byte per pixel at dst[2..]
  void extractRow(uint8_t *dst, uint16_t y) const {
    const uint8_t *b = buf_ + (y >> 3) * stride_ + xoff_;
    const uint8_t m = 1u << (y & 7);
    for (uint16_t x = 0; x < FRAME_W; x++) dst[x + 2] = (b[x] & m) ? 1 : 0;
  }

  // Decoded row y -> display buffer
  void writeRow(const uint8_t *src, uint16_t y) {
    uint8_t *b = buf_ + (y >> 3) * stride_ + xoff_;
    const uint8_t m = 1u << (y & 7);
    for (uint16_t x = 0; x < FRAME_W; x++) {
      if (src[x + 2]) b[x] |= m;
      else b[x] &= (uint8_t)~m;
    }
  }

  uint8_t *buf_ = nullptr;
  uint8_t stride_ = 128, xoff_ = 0;
  const uint8_t *src_ = nullptr;
  uint32_t range_ = 0, code_ = 0;
  uint16_t frame_ = 0;
  uint16_t prob_[NCTX];
  uint8_t cnt_[NCTX];
  uint8_t rows_[6][ROWLEN];
};
