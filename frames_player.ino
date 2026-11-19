// frames_player.ino - play the compressed frame sequence on an SSD1306 OLED
// Board : Arduino UNO R4 Minima          Display: 128x64 SSD1306, I2C (SDA/SCL pins)
// Libs  : Adafruit SSD1306, Adafruit GFX (install from the Library Manager)
// Files : frames_data.h (stream from compress_frames.py) and frame_decoder.h
//         must sit in this sketch folder next to this file.

#include <Wire.h>
#include <Adafruit_GFX.h>
#define SSD1306_NO_SPLASH  // drop the Adafruit logo bitmap
#include <Adafruit_SSD1306.h>

#include "frames_data.h"
#include "frame_decoder.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C          // some modules use 0x3D
#define I2C_HZ 400000UL         // 1000000UL often works too, for a faster refresh
#define DEBUG 1                 // 1 = test pattern at start + Serial diagnostics (115200 baud)
#define INVERT_COLORS 0         // 1 = swap black and white (hardware invert, no cost)

static_assert(FRAME_W <= SCREEN_WIDTH && FRAME_H <= SCREEN_HEIGHT, "frames larger than the display");

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, I2C_HZ, I2C_HZ);
FrameDecoder decoder;
uint32_t nextUs;

#if DEBUG
static uint16_t litPixels() {  // number of lit pixels in the display buffer
  const uint8_t *b = display.getBuffer();
  uint16_t n = 0;
  for (uint16_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT / 8; i++) n += __builtin_popcount(b[i]);
  return n;
}
#endif

void setup() {
#if DEBUG
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) {}
  Serial.println("--- frames_player debug ---");
  Serial.print("frames: ");
  Serial.print(FRAME_COUNT);
  Serial.print("  stream bytes: ");
  Serial.print(FRAME_STREAM_SIZE);
  Serial.print("  first stream bytes: ");
  Serial.print(frame_stream[0], HEX);
  Serial.print(' ');
  Serial.println(frame_stream[1], HEX);  // expect 0 0
#endif

  bool ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
#if DEBUG
  Serial.println(ok ? "display.begin: OK" : "display.begin: FAILED");
#endif
  if (!ok) {
    for (;;) {}
  }
  display.invertDisplay(INVERT_COLORS);

#if DEBUG
  display.clearDisplay();  // test pattern: border + diagonal, 1.5 s
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.drawLine(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.display();
  Serial.println("test pattern shown (border + diagonal). If you can't see it, the problem is wiring/display.");
  delay(1500);
#endif

  display.clearDisplay();
  decoder.begin(display.getBuffer(), SCREEN_WIDTH, (SCREEN_WIDTH - FRAME_W) / 2);
  nextUs = micros();
}

void loop() {
#if DEBUG
  uint32_t t0 = micros();
#endif
  decoder.nextFrame();
#if DEBUG
  uint32_t decodeUs = micros() - t0;
#endif

  while ((int32_t)(micros() - nextUs) < 0) {}  // wait for this frame's slot
#if DEBUG
  uint32_t t1 = micros();
#endif
  display.display();
#if DEBUG
  uint32_t displayUs = micros() - t1;
  uint16_t idx = decoder.frameIndex();
  if (idx <= 150 && idx % 15 == 0) {  // once per second for the first 10 s
    Serial.print("frame ");
    Serial.print(idx);
    Serial.print("  lit pixels: ");
    Serial.print(litPixels());
    Serial.print("  decode us: ");
    Serial.print(decodeUs);
    Serial.print("  display() us: ");
    Serial.println(displayUs);
  }
#endif
  nextUs += FRAME_INTERVAL_US;
  if ((int32_t)(micros() - nextUs) > (int32_t)FRAME_INTERVAL_US) nextUs = micros();  // fell behind: resync
}
