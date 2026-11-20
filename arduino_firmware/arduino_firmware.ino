/*
 * Arduino UNO + Zumo shield v1.2  --  motor / sensor controller
 *
 * Talks to the Wemos D1 mini over SoftwareSerial (UNO LINK_RX / LINK_TX),
 * 38400 baud, newline-terminated text commands.
 *
 * Commands (D1 mini -> UNO)             Reply
 *   M <left> <right>   drive, -400..400  (none, to keep traffic low)
 *   S                  stop              OK
 *   L <0|1>            LED on D13        OK
 *   B <freq> <ms>      buzzer (freq 0 = off) OK
 *   T                  telemetry         T <mV> <btn> <led> <s0..s5>
 *                      s0..s5 = RAW reflectance readings, left -> right
 *
 * Events (UNO -> D1 mini)
 *   E B <0|1>          button state changed (debounced)
 *
 * Failsafe: motors stop if no M/S command arrives for TIMEOUT_MS.
 *
 * Libraries: "ZumoShield" (Pololu). Use the QTRSensors.h that ships with it;
 * if you also have the standalone QTRSensors library installed, remove it,
 * the two APIs differ.
 */
#include <SoftwareSerial.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>

// ---------------- pins ----------------
const uint8_t LINK_RX = 2;
const uint8_t LINK_TX = 6;
const uint8_t LED_PIN = 13;
const uint8_t BUZZER_PIN = 3;
const uint8_t BUTTON_PIN = 12;          // assumed wired to GND (active low)
const uint8_t BATT_PIN = A1;

// Reflectance array, left -> right (s0 = D4 ... s5 = D5).
const uint8_t NUM_SENSORS = 6;
unsigned char sensorPins[NUM_SENSORS] = {4, A3, 11, A0, A2, 5};

// ---------------- config ----------------
const unsigned long LINK_BAUD = 38400;  // use 9600 on both sides if you see garbage
const unsigned long TIMEOUT_MS = 1000;

// A1 sees 2/3 of the battery voltage.
const float VBAT_DIVIDER = 6.8 / 16.8;  // normally this should be 2/3, but we had to replace the resistor
const float ADC_REF_MV = 5000.0;

// ---------------- state ----------------
SoftwareSerial link(LINK_RX, LINK_TX);
ZumoMotors motors;
// No emitter pin: the library does not control the array's LED ON line.
QTRSensorsRC qtr(sensorPins, NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);

unsigned long lastCmd = 0;
uint8_t ledState = 0;

char buf[40];
uint8_t len = 0;

bool btnRaw = false, btnStable = false;
unsigned long btnChanged = 0;

inline void ok()  { link.println(F("OK")); }
inline void err() { link.println(F("ERR")); }

// ---------------- actions ----------------
void sendTelemetry() {
  unsigned int v[NUM_SENSORS];
  qtr.read(v, QTR_EMITTERS_ON);          // raw RC discharge time: ~0 = bright surface, 2500 = dark / nothing

  float mv = analogRead(BATT_PIN) * (ADC_REF_MV / 1023.0) / VBAT_DIVIDER;

  link.print(F("T ")); link.print((long)mv);
  link.print(' ');     link.print(btnStable ? 1 : 0);
  link.print(' ');     link.print(ledState);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { link.print(' '); link.print(v[i]); }
  link.println();
}

void handleLine(char *line) {
  int a, b;
  switch (line[0]) {
    case 'M':
      if (sscanf(line + 1, "%d %d", &a, &b) == 2) {
        motors.setSpeeds(constrain(a, -400, 400), constrain(b, -400, 400));
        lastCmd = millis();
      } else err();
      break;
    case 'S':
      motors.setSpeeds(0, 0);
      lastCmd = millis();
      ok();
      break;
    case 'L':
      if (sscanf(line + 1, "%d", &a) == 1) {
        ledState = a ? 1 : 0;
        digitalWrite(LED_PIN, ledState);
        ok();
      } else err();
      break;
    case 'B':
      if (sscanf(line + 1, "%d %d", &a, &b) == 2) {
        if (a <= 0) noTone(BUZZER_PIN);
        else tone(BUZZER_PIN, constrain(a, 31, 12000), constrain(b, 1, 5000));
        ok();
      } else err();
      break;
    case 'T':
      sendTelemetry();
      break;
    default:
      err();
  }
}

void pollButton() {
  bool raw = (digitalRead(BUTTON_PIN) == LOW);
  if (raw != btnRaw) { btnRaw = raw; btnChanged = millis(); }
  if (millis() - btnChanged > 20 && raw != btnStable) {
    btnStable = raw;
    link.print(F("E B "));
    link.println(btnStable ? 1 : 0);
  }
}

// ---------------- Arduino ----------------
void setup() {
  link.begin(LINK_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  motors.flipLeftMotor(true);    // uncomment if a motor spins the wrong way
  motors.flipRightMotor(true);
}

void loop() {
  while (link.available()) {
    char c = link.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  pollButton();

  if (millis() - lastCmd > TIMEOUT_MS) motors.setSpeeds(0, 0);   // failsafe
}
