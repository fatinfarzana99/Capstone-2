#include <Servo.h>
#include <AccelStepper.h>

Servo GRIPPER;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

#define RELAY_PIN_IN1_AC_MOTOR 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2_AC_MOTOR 3 // Change this to the appropriate pin

#define RELAY_PIN_IN3_ACTUATOR 4 // Change this to the appropriate pin
#define RELAY_PIN_IN4_ACTUATOR 5 // Change this to the appropriate pin

#define STEPS_PER_REVOLUTION 200  // Change according to your motor's specifications
#define STEP_PIN_STEPPER_MOTOR 6 // Change this to the appropriate pin
#define DIR_PIN_STEPPER_MOTOR 7 // Change this to the appropriate pin
#define EN_PIN_STEPPER_MOTOR 8 // Change this to the appropriate pin
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN_STEPPER_MOTOR, DIR_PIN_STEPPER_MOTOR);

bool linearActuatorForwardFlag = false;
bool linearActuatorBackwardFlag = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(RELAY_PIN_IN3_ACTUATOR, OUTPUT);
  pinMode(RELAY_PIN_IN4_ACTUATOR, OUTPUT);

  pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  /*pinMode(STEP_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(DIR_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(EN_PIN_STEPPER_MOTOR, OUTPUT);
  digitalWrite(EN_PIN_STEPPER_MOTOR, LOW);*/
  stepper.setMaxSpeed(1000);  // Adjust as needed
  stepper.setAcceleration(500);  // Adjust as needed

    GRIPPER.attach(9);
}

/****************************************************************************************/
void RotatePlatformEvent()
{
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, HIGH);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);

    delay(10000);

    StopPlatformRotation();
}

void StopPlatformRotation()
{
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, LOW);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);

}
/****************************************************************************************/


/****************************************************************************************/
void LinearActuatorBackward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, HIGH);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);

    delay(60000); // Delay for 1 minute
    delay(43000); // Delay for 43 seconds // wait for a second
    
    linearActuatorBackwardFlag = true;
}
void LinearActuatorForward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, HIGH);

    delay(60000); // Delay for 1 minute
    delay(43000); // Delay for 43 seconds // wait for a second

    linearActuatorForwardFlag = true;
}
void LinearActuatorOff()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}
/****************************************************************************************/

/****************************************************************************************/
void stepperMotor()
{
  // Set the speed at which the stepper motor will rotate
  stepper.setSpeed(-100); // Adjust as needed
  // Rotate the motor continuously
  stepper.runSpeed();
}

/****************************************************************************************/

void RetrieveTrayEvent()
{
  // Sweep from 0 to 180 degrees
  for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }
  
  // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }
    LinearActuatorForward();
  // Optional delay before returning to original position
    for (pos = 0; pos <= 90; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }
    delay(1000);

    LinearActuatorBackward();
    delay(2000);

  // Sweep from current position to 180 degrees
    for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }

    // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }
}

void gripperTest()
{
    for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }
  
  // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }

    for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }

    // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
  }

while(1)
{

}
}

  


void loop() {
  // put your main code here, to run repeatedly:
  
  //RetrieveTrayEvent();
  //RotatePlatformEvent();

  //LinearActuatorBackward();
  //LinearActuatorForward();

  stepperMotor();

  //gripperTest();





}

/*#include <Servo.h>

// Pin definitions for Ultrasonic Sensor 1
#define TRIG_PIN_US_SENSOR 12
#define ECHO_PIN_US_SENSOR 13

// Pin definitions for Ultrasonic Sensor 2
#define TRIG_PIN_US_SENSOR_2 10
#define ECHO_PIN_US_SENSOR_2 11

// Define servo objects
Servo myservo;  


// Function to read distance from Ultrasonic Sensor 1
long readDistanceFromSensor1() {
  digitalWrite(TRIG_PIN_US_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_US_SENSOR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_US_SENSOR, LOW);
  return pulseIn(ECHO_PIN_US_SENSOR, HIGH) / 2 / 29.1;
}

// Function to read distance from Ultrasonic Sensor 2
long readDistanceFromSensor2() {
  digitalWrite(TRIG_PIN_US_SENSOR_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_US_SENSOR_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_US_SENSOR_2, LOW);
  return pulseIn(ECHO_PIN_US_SENSOR_2, HIGH) / 2 / 29.1;
}

void setup() {
  pinMode(TRIG_PIN_US_SENSOR, OUTPUT);
  pinMode(ECHO_PIN_US_SENSOR, INPUT);
  pinMode(TRIG_PIN_US_SENSOR_2, OUTPUT);
  pinMode(ECHO_PIN_US_SENSOR_2, INPUT);
  
  // Attach servos to pins
  myservo.attach(9);

  // Initialize servos position
  myservo.write(0);

}

void loop() {
  Serial.begin(9600);
  long distance1 = readDistanceFromSensor1();
  long distance2 = readDistanceFromSensor2();
        Serial.print(distance1);
      Serial.println(" cm");
            //Serial.print(distance2);
      //Serial.println(" cm");
  if (distance1 <= 10) {
    myservo.write(180);
  } else if (distance2 <= 10) {
    myservo.write(0);
  }

}*/

