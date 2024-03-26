/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-stepper-motor-and-limit-switch
 */

#include <ezButton.h>
#include <AccelStepper.h>
#include <NewPing.h>

#define TRIGGER_PIN_3  51  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_3     53  // Change to the pin number you're using for the second ultrasonic sensor
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
#define PING_INTERVAL 20 // Set the desired ping interval in milliseconds (e.g., 20ms for approximately 50 pings per second)
unsigned long lastPingTime = 0; // Variable to store the time of the last ping
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2

#define RELAY_PIN_IN1_AC_MOTOR 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2_AC_MOTOR 3 // Change this to the appropriate pin

#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)

ezButton limitSwitch(50); // create ezButton object that attach to pin A0;

AccelStepper stepper(AccelStepper::DRIVER, 6, 7);

bool isStopped = false;

void setup() {
  Serial.begin(9600);

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

  stepper.setMaxSpeed(50.0);   // set the maximum speed
  stepper.setAcceleration(30.0); // set acceleration
  stepper.setSpeed(30);         // set initial speed
  stepper.setCurrentPosition(0); // set position
  stepper.moveTo(MAX_POSITION);

    pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);
}

void RotatePlatformUpEvent()
{   
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, HIGH);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);

    //delay(10000);

    //StopPlatformRotation();
}

void RotatePlatformDownEvent()
{   
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, LOW);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, HIGH);

    //delay(10000);

    //StopPlatformRotation();
}

void StopPlatformRotation()
{
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, LOW);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);

}

void loop() {
  limitSwitch.loop(); // MUST call the loop() function first
    unsigned long currentTime = millis();
    int distance3 = sonar3.ping_cm(); // Send ping from sensor 2, get distance in cm

    // Check if it's time to send another ping
    if (currentTime - lastPingTime >= PING_INTERVAL) {
      // Send pings from both sensors

      // Print distances
      Serial.print("Distance Sensor 3: ");
      Serial.print(distance3);
      Serial.println("cm");

      // Update last ping time
      lastPingTime = currentTime;
    }

  if (limitSwitch.isPressed()) {
    Serial.println(F("The limit switch: TOUCHED"));
    isStopped = true;
  }

  if ((isStopped == false) && (distance3 <= 10)) {
    // without this part, the move will stop after reaching maximum position
    if (stepper.distanceToGo() == 0) { // if motor moved to the maximum position
      stepper.setCurrentPosition(0);   // reset position to 0
      stepper.moveTo(MAX_POSITION);       // move the motor to maximum position again
    }
    RotatePlatformUpEvent();
    stepper.run(); // MUST be called in loop() function
  } 
  else {
    // without calling stepper.run() function, motor stops immediately
    // NOTE: stepper.stop() function does NOT stops motor immediately
    Serial.println(F("The stepper motor is STOPPED"));
    StopPlatformRotation();
  }
}
