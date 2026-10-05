// Adapted from WT-MM/PyFlexiTac firmware/template.ino (MIT),
// derived there from Binghao Huang's FlexiTac Arduino firmware.
// ATmega328P Nano, 16 MHz, 5 V. SRCLK and RCLK must share D3.
#include <Arduino.h>
#include <stdio.h>

#if !defined(__AVR_ATmega328P__)
#error "This direct-register firmware requires an ATmega328P Nano."
#endif
#ifndef BAUD_RATE
#define BAUD_RATE 2000000
#endif
#ifndef ROW_COUNT
#define ROW_COUNT 12
#endif
#ifndef MUX_CHANNEL_OFFSET
#define MUX_CHANNEL_OFFSET 4
#endif
// 0: existing binary scan; 1: independent fixed-output text probe.
#ifndef DIAGNOSTIC_TEXT
#define DIAGNOSTIC_TEXT 0
#endif
#ifndef ADC_PRESCALER
#define ADC_PRESCALER 16
#endif
#ifndef SAMPLE_SETTLE_US
#define SAMPLE_SETTLE_US 0
#endif
#ifndef ADC_DISCARD_FIRST
#define ADC_DISCARD_FIRST 0
#endif
#ifndef FIXED_MUX_CHANNEL
#define FIXED_MUX_CHANNEL 4
#endif
#ifndef FIXED_COLUMN
#define FIXED_COLUMN 0
#endif

constexpr uint8_t COLUMN_COUNT = 32;
constexpr uint8_t PIN_ADC_INPUT = A0;
constexpr uint8_t PIN_SHIFT_REGISTER_DATA = 2;
constexpr uint8_t PIN_SHIFT_REGISTER_CLOCK = 3;
constexpr uint8_t PIN_MUX_CHANNEL_0 = 4;
constexpr uint8_t PIN_MUX_INHIBIT = 8;

static_assert(ROW_COUNT > 0 && MUX_CHANNEL_OFFSET >= 0 &&
              ROW_COUNT + MUX_CHANNEL_OFFSET <= 16,
              "Rows plus offset must fit the single 16-channel mux.");
static_assert(FIXED_MUX_CHANNEL >= 0 && FIXED_MUX_CHANNEL < 16 &&
              FIXED_COLUMN >= 0 && FIXED_COLUMN < 32, "Invalid fixed selection.");
static_assert(SAMPLE_SETTLE_US >= 0 && SAMPLE_SETTLE_US <= 16000,
              "Invalid settling time.");

// Tied clocks latch the PREVIOUS shift state. Two clocks put a new '1'
// on QA. The upstream extra clock is intentional.
static void shiftColumn(bool data = false)
{
  if (data) PORTD |= _BV(PD2);
  else PORTD &= ~_BV(PD2);
  PORTD |= _BV(PD3);
  PORTD &= ~_BV(PD3);
  PORTD &= ~_BV(PD2);
}

static void clearColumns()
{
  // 32 clocks clear the shift chain; one more clears the storage registers.
  for (uint8_t i = 0; i < COLUMN_COUNT + 1; ++i) shiftColumn();
}

static void selectMux(uint8_t channel, bool enabled = true)
{
  digitalWrite(PIN_MUX_INHIBIT, HIGH); // HC4067 E is active LOW.
  for (uint8_t i = 0; i < 4; ++i)
    digitalWrite(PIN_MUX_CHANNEL_0 + i, (channel >> i) & 1);
  digitalWrite(PIN_MUX_INHIBIT, enabled ? LOW : HIGH);
}

static uint16_t sampleADC()
{
#if SAMPLE_SETTLE_US > 0
  delayMicroseconds(SAMPLE_SETTLE_US);
#endif
#if ADC_DISCARD_FIRST
  (void)analogRead(PIN_ADC_INPUT);
#endif
  return analogRead(PIN_ADC_INPUT);
}

#if DIAGNOSTIC_TEXT
static uint8_t fixedMux = FIXED_MUX_CHANNEL;
static uint8_t fixedColumn = FIXED_COLUMN;
static bool muxEnabled = true;
static bool columnEnabled = true;

static void applyFixedSelection()
{
  digitalWrite(PIN_MUX_INHIBIT, HIGH);
  clearColumns();
  if (columnEnabled) {
    shiftColumn(true);
    for (uint8_t i = 0; i <= fixedColumn; ++i) shiftColumn();
  }
  selectMux(fixedMux, muxEnabled);
}

static void printHelp()
{
  Serial.println(F("FIXED TEXT ONLY; 115200 baud; indices are zero-based."));
  Serial.println(F("m N: MUX 0..15; c N: scan column 0..31; e 0/1: MUX enable; d 0/1: column drive; h: help"));
  Serial.println(F("Columns stay static between commands. raw10=0..1023; raw8=raw10>>2. No binary frames."));
}

static void readCommands()
{
  static char command[24];
  static uint8_t length = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (!length && !overflow) continue;
      command[length] = '\0';
      char op = 0, extra = 0;
      int value = -1;
      int fields = sscanf(command, " %c %d %c", &op, &value, &extra);
      bool valid = !overflow && fields == 2;
      if (!overflow && fields == 1 && op == 'h') printHelp();
      else if (valid && op == 'm' && value >= 0 && value < 16) fixedMux = value;
      else if (valid && op == 'c' && value >= 0 && value < 32) fixedColumn = value;
      else if (valid && op == 'e' && (value == 0 || value == 1)) muxEnabled = value;
      else if (valid && op == 'd' && (value == 0 || value == 1)) columnEnabled = value;
      else { valid = false; Serial.println(F("ERR command; type h")); }
      if (valid) { applyFixedSelection(); Serial.println(F("OK")); }
      length = 0;
      overflow = false;
    } else if (length < sizeof(command) - 1) command[length++] = ch;
    else overflow = true;
  }
}
#endif

void setup()
{
  // Set output latches before enabling outputs; inhibit the analog mux first.
  digitalWrite(PIN_MUX_INHIBIT, HIGH);
  pinMode(PIN_MUX_INHIBIT, OUTPUT);
  digitalWrite(PIN_SHIFT_REGISTER_DATA, LOW);
  digitalWrite(PIN_SHIFT_REGISTER_CLOCK, LOW);
  pinMode(PIN_SHIFT_REGISTER_DATA, OUTPUT);
  pinMode(PIN_SHIFT_REGISTER_CLOCK, OUTPUT);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(PIN_MUX_CHANNEL_0 + i, LOW);
    pinMode(PIN_MUX_CHANNEL_0 + i, OUTPUT);
  }
  pinMode(PIN_ADC_INPUT, INPUT);
  digitalWrite(PIN_ADC_INPUT, LOW); // no internal pull-up
  analogReference(DEFAULT);       // AVCC reference
  DIDR0 |= _BV(ADC0D);

#if ADC_PRESCALER == 16
  constexpr uint8_t adcBits = _BV(ADPS2); // 1 MHz at 16 MHz
#elif ADC_PRESCALER == 128
  constexpr uint8_t adcBits = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 125 kHz
#else
#error "Supported ADC_PRESCALER values: 16, 128."
#endif
  ADCSRA = (ADCSRA & ~(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0))) | adcBits;
  clearColumns();
  (void)analogRead(PIN_ADC_INPUT); // warm-up conversion, never sent

#if DIAGNOSTIC_TEXT
  Serial.begin(115200);
  applyFixedSelection();
  printHelp();
#else
  Serial.begin(BAUD_RATE);
#endif
}

void loop()
{
#if DIAGNOSTIC_TEXT
  readCommands();
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    uint16_t low = 1023, high = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16; ++i) {
      uint16_t raw = sampleADC();
      if (raw < low) low = raw;
      if (raw > high) high = raw;
      sum += raw;
    }
    Serial.print(F("ms=")); Serial.print(millis());
    Serial.print(F(" mux=")); Serial.print(fixedMux);
    Serial.print(F(" col=")); Serial.print(fixedColumn);
    Serial.print(F(" enable=")); Serial.print(muxEnabled);
    Serial.print(F(" drive=")); Serial.print(columnEnabled);
    Serial.print(F(" raw10_mean=")); Serial.print(sum / 16.0, 2);
    Serial.print(F(" min=")); Serial.print(low);
    Serial.print(F(" max=")); Serial.print(high);
    Serial.print(F(" raw8=")); Serial.println((sum / 16) >> 2);
  }
#else
  Serial.write(0xAA);
  Serial.write(0x55);
  for (uint8_t row = 0; row < ROW_COUNT; ++row) {
    selectMux(row + MUX_CHANNEL_OFFSET);
    shiftColumn(true);
    shiftColumn(); // latch QA with shared SRCLK/RCLK
    for (uint8_t column = 0; column < COLUMN_COUNT; ++column) {
      Serial.write(static_cast<uint8_t>(sampleADC() >> 2));
      shiftColumn(); // after column 31, both chains contain only zeroes
    }
  }
#endif
}
