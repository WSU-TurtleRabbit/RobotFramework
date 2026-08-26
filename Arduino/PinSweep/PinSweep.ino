// Kicker-pin discovery diagnostic (Nano Every).
//
// The kicker on this chassis fires on Arduino RESET (all pins floating) but
// not on commanded pulses from D5 — so either the signal is on a different
// pin or it needs a different drive. This sketch walks every candidate pin
// with a 400 ms pulse of each polarity, ONE STEP PER SERIAL BYTE, printing
// what it is about to do. Watch the robot: the step that kicks names the pin.
//
// Serial 115200: send any byte -> next step. 'r' -> restart sequence.
// Pins D0/D1 are serial and never touched.

const int PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
const int N_PINS = sizeof(PINS) / sizeof(PINS[0]);
const unsigned long PULSE_MS = 400;

int step_idx = 0;  // 2 steps per pin: even = LOW pulse, odd = HIGH pulse

void setup() {
  Serial.begin(115200);
  // Leave every pin as INPUT (floating, same as the reset state that the
  // kicker demonstrably tolerates until a real drive happens).
  Serial.println("PINSWEEP ready: send any byte per step, 'r' to restart");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      step_idx = 0;
      Serial.println("restarted");
      return;
    }
    if (step_idx >= N_PINS * 2) {
      Serial.println("sweep complete - 'r' to restart");
      return;
    }
    int pin = PINS[step_idx / 2];
    bool level = (step_idx % 2 == 1);  // even step: LOW, odd step: HIGH
    Serial.print("STEP ");
    Serial.print(step_idx);
    Serial.print(": D");
    Serial.print(pin);
    Serial.print(level ? " HIGH" : " LOW");
    Serial.println(" 400ms");
    digitalWrite(pin, level ? HIGH : LOW);
    pinMode(pin, OUTPUT);
    delay(PULSE_MS);
    pinMode(pin, INPUT);   // release back to float
    Serial.println("done");
    step_idx++;
  }
}
