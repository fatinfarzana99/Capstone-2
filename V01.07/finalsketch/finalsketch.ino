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
bool currentstate = true;
bool retractactuator = false; 

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
        GRIPPER.write(0);

    if ((distance < 10) && retractactuator == false) 
    {
      //Stop Linear Actuator
      digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
      digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
      Serial.print(distance);
      Serial.println(" cm");
      //delay(3000);
      if(currentstate == true)
      {
          for (pos = 0; pos <= 180; pos += 1) 
          { // goes from 0 degrees to 180 degrees
              // in steps of 1 degree
            GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          } 

          for (pos = 180; pos >= 0; pos -= 1) 
          { // goes from 180 degrees to 0 degrees

            GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);      // waits 15ms for the servo to reach the position
            
          }

      currentstate = false;
      retractactuator = true;
      }

      Serial.println("Grab Tray");
      delay(2000);
    }

          if(currentstate == false && retractactuator == true)
      {
           for (pos = 0; pos <= 180; pos += 1) 
          { // goes from 0 degrees to 180 degrees
              // in steps of 1 degree
            GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15ms for the servo to reach the position
          } 

          for (pos = 180; pos >= 0; pos -= 1) 
          { // goes from 180 degrees to 0 degrees

            GRIPPER.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);      // waits 15ms for the servo to reach the position
            
          }
        retractactuator = false;
        delay(1000);
        BackwardStoring();
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


/*
#include <Servo.h>

const int trigPin = 10;
const int echoPin = 11;


const int trigPin1 = 12;
const int echoPin1 = 13;

// defines variables
long duration;
int distance;

long duration2;
int distance2;

Servo myservo;  
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

    // Attach servos to pins
  myservo.attach(9);

  // Initialize servos position
  myservo.write(0);

}
void loop() {
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


  // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration2 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance2 = duration2 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance2: ");
  Serial.println(distance2);


  if (distance <= 10 && distance2 >=10) {
    myservo.write(180);
    delay(15);
  } else if (distance2 <= 10 && distance >=10) {
    myservo.write(0);
    delay(15);
  }

}*/
