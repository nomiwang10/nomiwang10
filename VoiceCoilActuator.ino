// Voice Coil Actuator Controller (ESP32)
//
// Hardware:
//  - DAC output: GPIO25 (ESP32 built-in DAC1) -> actuator driver
//  - Toggle button: GPIO5 (active LOW, uses INPUT_PULLUP)
//  - Rotary encoder: CLK -> GPIO32, DT -> GPIO33 (no SW pin used here)
//  - I2C LCD 16x2 at 0x27 on SDA=21, SCL=22
//
// Behavior:
//  - Button toggles actuator ON/OFF (debounced).
//  - Encoder changes frequency in 1 Hz steps within [1, 20] Hz.
//  - When ON, outputs a 0–255 DAC sine wave (offset/amplitude set below).
//  - LCD shows current state: "Frequency: <f> Hz" when ON, or "Actuator OFF".

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// === LCD Setup ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Pin Assignments ===
const int DAC_OUTPUT   = 25;
const int BTN_TOGGLE   = 5;
const int ENCODER_CLK  = 32;  // CLK wired to GPIO32
const int ENCODER_DT   = 33;  // DT wired to GPIO33

// === Frequency Settings ===
const int freqMin   = 1;
const int freqMax   = 20;
const int freqStep  = 1;
int frequency       = freqMin;

// === Sine Wave Parameters ===
// Use 8‑bit DAC range [0..255]. Keep offset+amplitude within [0..255].
const int amplitude        = 127;
const int offset           = 128;
const int samplesPerCycle  = 300;

// === State Variables ===
bool actuatorOn               = false;
int  lastCLKState             = HIGH;
unsigned long lastTogglePress = 0;
const int debounceDelay       = 200;  // ms

// === Quadrature Accumulator ===
static int8_t  lastEncState = 0;  // holds previous (CLK,DT) state
static int16_t encAccumulator = 0; // sums +/-1 per transition

// lookup table: (prev<<2 | curr) -> +1, -1 or 0
// order index bits: prevCLK prevDT currCLK currDT
const int8_t enc_states[16] = {
  // 0000, 0001, 0010, 0011,
     0,   -1,    +1,    0,
  // 0100, 0101, 0110, 0111,
    +1,    0,     0,   -1,
  // 1000, 1001, 1010, 1011,
    -1,    0,     0,   +1,
  // 1100, 1101, 1110, 1111
     0,   +1,    -1,    0
};

// Forward declarations
void handleToggleButton();
void handleEncoder();
void generateSineWave(int freq);
void updateDisplay();

void setup() {
  pinMode(DAC_OUTPUT, OUTPUT);
  pinMode(BTN_TOGGLE, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);

  // initialize quadrature state
  int msb = digitalRead(ENCODER_CLK);
  int lsb = digitalRead(ENCODER_DT);
  int startEncState = (msb << 1) | lsb;
  lastEncState = startEncState;
  lastCLKState = msb;

  Serial.begin(115200);

  Wire.begin(21, 22);       // I2C: SDA, SCL
  lcd.init();
  lcd.backlight();
  updateDisplay();
}

void loop() {
  handleToggleButton();
  handleEncoder();

  if (actuatorOn) {
    generateSineWave(frequency);
  } else {
    dacWrite(DAC_OUTPUT, offset);
    delay(10);
  }
}

void handleToggleButton() {
  static bool lastButtonState = HIGH;
  bool current = digitalRead(BTN_TOGGLE);

  if (current == LOW && lastButtonState == HIGH) { // pressed
    if (millis() - lastTogglePress > (unsigned long)debounceDelay) {
      actuatorOn = !actuatorOn;

      // Optional reset when turning ON:
      // frequency = freqMin;     // reset to 1 Hz
      // encAccumulator = 0;      // clear any encoder residue

      Serial.println(actuatorOn ? "Actuator ON" : "Actuator OFF");
      updateDisplay();
      lastTogglePress = millis();
    }
  }
  lastButtonState = current;
}

void handleEncoder() {
  int msb = digitalRead(ENCODER_CLK);
  int lsb = digitalRead(ENCODER_DT);
  int currEncState = (msb << 1) | lsb;
  int index = (lastEncState << 2) | currEncState;
  int8_t delta = enc_states[index & 0x0F];
  lastEncState = currEncState;

  if (delta != 0) {
    encAccumulator += delta;
    // A typical mechanical encoder produces 4 transitions per detent
    if (encAccumulator >= 4) {
      frequency = constrain(frequency + freqStep, freqMin, freqMax);
      Serial.print("Full CW -> freq = ");
      Serial.println(frequency);
      updateDisplay();
      encAccumulator = 0;
    } else if (encAccumulator <= -4) {
      frequency = constrain(frequency - freqStep, freqMin, freqMax);
      Serial.print("Full CCW -> freq = ");
      Serial.println(frequency);
      updateDisplay();
      encAccumulator = 0;
    }
  }
}

void generateSineWave(int freq) {
  if (freq <= 0) return;

  unsigned long startTime = micros();
  unsigned long period = 1000000UL / (unsigned long)freq;

  for (int i = 0; i < samplesPerCycle; i++) {
    float angle = (2.0 * PI * i) / samplesPerCycle;
    int value = offset + (int)(amplitude * sinf(angle));
    dacWrite(DAC_OUTPUT, value);

    // schedule next sample time for uniform spacing
    unsigned long target = startTime + (period * (unsigned long)i) / (unsigned long)samplesPerCycle;
    while (micros() < target) {
      // keep button & encoder responsive even during waveform
      handleToggleButton();
      handleEncoder();
      if (!actuatorOn) return; // bail out quickly if turned OFF
      // small pause to avoid tight spin (optional)
      // delayMicroseconds(5);
    }
  }
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (actuatorOn) {
    lcd.print("Frequency:");
    lcd.setCursor(0, 1);
    lcd.print(frequency);
    lcd.print(" Hz");
  } else {
    lcd.print("Actuator OFF");
  }
}
