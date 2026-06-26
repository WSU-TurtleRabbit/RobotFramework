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
//===============================================================
///

const int kickerOutputPin = 5; // Digital pin connected to kicker
const int dribblerPin = 3; 

// === KICKER VERSION SELECT (the one value to change) ===
const bool kickerActiveLevel = HIGH;               // HIGH = new kicker, LOW = old perf board
const bool kickerIdleLevel   = !kickerActiveLevel; // resting level is just the opposite of active

int dribblerPower = 1600;
int dribblerStopPin = 1500; // (note: this is a microseconds value, not a pin)
int kickerPulseTime = 10;   // Pulse duration for kicker (ms)

unsigned long pervious_time = 0;
int kicker_timeout = 5000;  // Minimum gap between kicks (ms)

char incomingByte; // Variable to store incoming serial data

#include <Servo.h>
Servo esc; 

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

  // Check for incoming serial data
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'K') { // If 'K' is typed
      unsigned long current_time = millis();
      if (current_time - pervious_time < kicker_timeout){  // reject kicks closer than the timeout
      Serial.print(current_time - pervious_time);
      Serial.println(" Kicking frequnecy is too fast"); // Deactivate kicker
      
      // pervious_time = current_time;   // intentionally NOT updated: timer runs from last SUCCESSFUL kick
      }
      else{
      Serial.println("Kicking");
      digitalWrite(kickerOutputPin, kickerActiveLevel); // Energize solenoid (HIGH new / LOW old)
      delay(kickerPulseTime);                           // Pulse duration
      digitalWrite(kickerOutputPin, kickerIdleLevel);   // Release solenoid (back to idle)
      pervious_time = current_time;
      }
    } else if (incomingByte == 'D') { // If 'D' is typed
      esc.writeMicroseconds(dribblerPower); // Turn on dribbler
    } else if (incomingByte == 'S') { // If 'S' is typed
      esc.writeMicroseconds(dribblerStopPin); // Turn off dribbler
    }
  }


}
