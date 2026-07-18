// Arduino Code you will put into the Arduino Nano Every 
///
//==============================================================
//  WARNING: The way the kicker gets activated is DIFFERENT between 
//  different robots. The old perf board is driven by a LOW pulse,
//  the new kicker is driven by a HIGH pulse.
//
//  >>> TO SWITCH VERSIONS: change the single value "kickerActiveLevel" <<<
//        HIGH  ->  new kicker     (HIGH pulse fires the solenoid)
//        LOW   ->  old perf board (LOW pulse fires the solenoid)
//  The idle level, the boot state, and the pulse are all derived
//  from it, so this is the ONLY line you ever need to touch.
//
//===============================================================
//  SERIAL PROTOCOL (115200 8N1), backward compatible:
//    'K'           kick with the default pulse (legacy behavior)
//    'k' + <byte>  kick with a pulse of <byte> milliseconds (MatchCtrl)
//    'D'           dribbler full on (legacy behavior)
//    'd' + <byte>  dribbler at 1500 + <byte> microseconds (MatchCtrl)
//    'S'           dribbler stop
//  A parameter byte must follow within PARAM_TIMEOUT_MS or the pending
//  command is dropped (never fire on a misaligned stream).
//===============================================================
///

const int kickerOutputPin = 5; // Digital pin connected to kicker
const int dribblerPin = 3; 

// === KICKER VERSION SELECT (the one value to change) ===
const bool kickerActiveLevel = HIGH;               // HIGH = new kicker, LOW = old perf board
const bool kickerIdleLevel   = !kickerActiveLevel; // resting level is just the opposite of active

int dribblerPower = 1600;
int dribblerStopPin = 1500; // (note: this is a microseconds value, not a pin)
int kickerPulseTime = 10;   // Default pulse duration for kicker (ms)

unsigned long pervious_time = 0;
int kicker_timeout = 5000;  // Minimum gap between kicks (ms) — capacitor recharge guard

// --- non-blocking kicker pulse state ---
bool kickActive = false;
unsigned long kickPulseEnd = 0;

// --- parameterized-command state machine ('k'/'d' awaiting a parameter byte) ---
byte pendingParam = 0;              // 0 = none pending, else 'k' or 'd'
unsigned long pendingSince = 0;
const unsigned long PARAM_TIMEOUT_MS = 50;

char incomingByte; // Variable to store incoming serial data

#include <Servo.h>
Servo esc; 

void fireKick(int pulseMs) {
  unsigned long current_time = millis();
  if (current_time - pervious_time < (unsigned long)kicker_timeout){
    // Reject kicks closer than the recharge guard (hardware protection).
    Serial.print(current_time - pervious_time);
    Serial.println(" Kicking frequnecy is too fast");
    return;
  }
  Serial.println("Kicking");
  digitalWrite(kickerOutputPin, kickerActiveLevel); // Energize solenoid (HIGH new / LOW old)
  kickActive = true;
  kickPulseEnd = current_time + pulseMs;            // released in loop(), non-blocking
  pervious_time = current_time;
}

void setup() {
  // Pre-load the output latch to the IDLE level BEFORE making the pin an output.
  // This matters for the old (active-LOW) board: if the pin became an OUTPUT first,
  // it would sit at LOW (= active) for an instant and twitch the solenoid at boot.
  digitalWrite(kickerOutputPin, kickerIdleLevel); // set the resting level first...
  pinMode(kickerOutputPin, OUTPUT);               // ...then drive it, already parked at idle

  Serial.begin(115200); // Initialize serial communication

  // Attach the dribbler to the pin
  esc.attach(dribblerPin);
  esc.writeMicroseconds(dribblerStopPin); // Initialize dribbler to stop position
}

void loop() {

  // Finish an active kick pulse (non-blocking: dribbler commands are never
  // stalled behind a long parameterized pulse).
  if (kickActive && (long)(millis() - kickPulseEnd) >= 0) {
    digitalWrite(kickerOutputPin, kickerIdleLevel); // Release solenoid (back to idle)
    kickActive = false;
  }

  // A pending parameter byte that never arrived: drop the command (a
  // misaligned stream must never fire the kicker).
  if (pendingParam != 0 && millis() - pendingSince > PARAM_TIMEOUT_MS) {
    pendingParam = 0;
  }

  // Check for incoming serial data
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    if (pendingParam != 0) {
      // This byte is the parameter of the pending command.
      byte param = (byte)incomingByte;
      if (pendingParam == 'k') {
        if (param > 0) fireKick((int)param); // pulse in milliseconds
      } else { // 'd'
        esc.writeMicroseconds(dribblerStopPin + (int)param);
      }
      pendingParam = 0;
    }
    else if (incomingByte == 'K') { // Legacy kick: default pulse
      fireKick(kickerPulseTime);
    }
    else if (incomingByte == 'k') { // Parameterized kick: pulse byte follows
      pendingParam = 'k';
      pendingSince = millis();
    }
    else if (incomingByte == 'D') { // Legacy dribble: full on
      esc.writeMicroseconds(dribblerPower);
    }
    else if (incomingByte == 'd') { // Parameterized dribble: us offset follows
      pendingParam = 'd';
      pendingSince = millis();
    }
    else if (incomingByte == 'S') { // Stop dribble
      esc.writeMicroseconds(dribblerStopPin);
    }
  }
}
