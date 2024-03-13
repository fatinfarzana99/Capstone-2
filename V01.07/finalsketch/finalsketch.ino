// Define the pin connected to the relay module
#include <Servo.h>
#define TRIG_PIN_US_SENSOR 10
#define ECHO_PIN_US_SENSOR 11
int sound = 250;
long duration, distance;

//bool servoPositionSet = false; // Flag to track if servo position is set

Servo GRIPPER;
int angle = 0;
int pos = 0; 

#define RELAY_PIN_IN1_AC_MOTOR 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2_AC_MOTOR 3 // Change this to the appropriate pin

#define RELAY_PIN_IN3_ACTUATOR 4 // Change this to the appropriate pin
#define RELAY_PIN_IN4_ACTUATOR 5 // Change this to the appropriate pin

#define STEP_PIN_STEPPER_MOTOR 6 // Change this to the appropriate pin
#define DIR_PIN_STEPPER_MOTOR 7 // Change this to the appropriate pin
#define EN_PIN_STEPPER_MOTOR 8 // Change this to the appropriate pin

#define LIMIT_SWITCH 12 // Change this to the appropriate pin

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(TRIG_PIN_US_SENSOR, OUTPUT);
  pinMode(ECHO_PIN_US_SENSOR, INPUT);

  pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  pinMode(RELAY_PIN_IN3_ACTUATOR, OUTPUT);
  pinMode(RELAY_PIN_IN4_ACTUATOR, OUTPUT);

  pinMode(STEP_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(DIR_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(EN_PIN_STEPPER_MOTOR, OUTPUT);
  digitalWrite(EN_PIN_STEPPER_MOTOR, LOW);

  GRIPPER.attach(9);

  pinMode(LIMIT_SWITCH, INPUT);
}
void LinearActuatorForward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, HIGH);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}
void LinearActuatorBackward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, HIGH);
}
void LinearActuatorOff()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}

void ForwardRetrieval()
{   
    //Continuously get distance readings
        digitalWrite(TRIG_PIN_US_SENSOR, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN_US_SENSOR, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN_US_SENSOR, LOW);
        duration = pulseIn(ECHO_PIN_US_SENSOR, HIGH);
        distance = (duration/2) / 29.1;
        delay(1500);
        

    if (distance < 10) 
    {
      //Stop Linear Actuator
      digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
      digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
      Serial.print(distance);
      Serial.println(" cm");
      delay(3000);
      //GRIPPER.write(180); // Set servo angle to open gripper
            for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
      GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      } 

      for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees

        GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);      // waits 15ms for the servo to reach the position
    }
      Serial.println("Grab Tray");
      delay(2000);
      GRIPPER.detach();
    }

    else 
    {
      //Extend Linear Actuator
      LinearActuatorForward();
      //LinearActuatorBackward();
      Serial.print(distance);
      Serial.println(" cm");
      GRIPPER.attach(9);
      GRIPPER.write(0);
      Serial.println("Continue Extension");
      delay(2000);
    }



}

void BackwardStoring()
{
    // Tray on the platform
  //LinearActuatorForward();
  LinearActuatorBackward();
}


void RotatePlatform()
{
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, HIGH);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);
    delay(1500);
}


void loop() {
  // put your main code here, to run repeatedly:
  ForwardRetrieval();
  //Serial.println(" cm");
  //delay(1000);
  //BackwardStoring();
  //delay(1000);
  //RotatePlatform();
  //delay(1000);

}
