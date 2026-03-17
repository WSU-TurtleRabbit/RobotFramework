// Arduino Code you will put into the Arduino Nano Every 


const int kickerOutputPin = 5; // Digital pin connected to kicker
const int dribblerPin = 3; 

int dribblerPower = 1600;
int dribblerStopPin = 1500; 
int kickerPulseTime = 10; // Pulse duration for kicker
bool pinStatus = HIGH; // Variable to store the pin status of kicker

unsigned long pervious_time = 0;
int kicker_timeout = 5000;

char incomingByte; // Variable to store incoming serial data

#include <Servo.h>
Servo esc; 

void setup() {
  pinMode(kickerOutputPin, OUTPUT); // Set kicker pin as output
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
      if (current_time - pervious_time < kicker_timeout){  // it will kick evey 5 seconds
      Serial.print(current_time - pervious_time);
      Serial.println("Kicking frequnecy is too fast"); // Deactivate kicker
      
      // pervious_time = current_time;
      }
      else{
      Serial.println("Kicking");
      digitalWrite(kickerOutputPin, LOW); // Activate kicker
      delay(kickerPulseTime); // Pulse duration
      digitalWrite(kickerOutputPin, HIGH);
      pervious_time = current_time;
      }
    } else if (incomingByte == 'D') { // If 'D' is typed
      esc.writeMicroseconds(dribblerPower); // Turn on dribbler
    } else if (incomingByte == 'S') { // If 'S' is typed
      esc.writeMicroseconds(dribblerStopPin); // Turn off dribbler
    }
  }


}
