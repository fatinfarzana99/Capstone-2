#include <Servo.h>

Servo gripperArm;
int angle = 0;
bool shouldStop = false;

const int trigPin = 10;
const int echoPin = 11;
long duration;
int distance;

void setup() {
  gripperArm.attach(9); // Attach the servo to pin 7
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);   // Initialize serial communication
}

void loop() {
 /* Serial.println("Test");
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read command from serial
    
    if (command == 'O') {
      OpenGripperArm(); // Open gripper arm if command is 'O'
    } else if (command == 'C') {
      CloseGripperArm(); // Close gripper arm if command is 'C'
    }
  }*/

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 10)
  {
    shouldStop = true;
  }

  if (distance > 10)
  {
    shouldStop = false;
    gripperArm.attach(9);
  }

  if (shouldStop == false) {
    OpenGripperArm();
    delay(40);
    //CloseGripperArm();
    //delay(40);
  }


   else {
    
    StopGripperArm();
  }
}

void OpenGripperArm() {
  for (angle = 0; angle <= 90; angle++) {
    gripperArm.write(angle); // Set servo angle to open gripper
    delay(40);
  }
}

void CloseGripperArm() {
  for (angle = 90; angle >= 0; angle--) {
    gripperArm.write(angle); // Set servo angle to close gripper
    delay(40);
  }
}

void StopGripperArm() {
  gripperArm.detach(); // Detach the servo to stop its movement
}
