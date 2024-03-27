#include <Servo.h>
#include <AccelStepper.h>
#include <NewPing.h>
#include <ezButton.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

ezButton limitSwitch(50);  // create ezButton object that attach to pin 7;
ezButton limitSwitch2(52);  // create ezButton object that attach to pin 7;
Servo GRIPPER;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

#define RELAY_PIN_IN1_AC_MOTOR 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2_AC_MOTOR 3 // Change this to the appropriate pin
#define RELAY_PIN_IN3_ACTUATOR 4 // Change this to the appropriate pin
#define RELAY_PIN_IN4_ACTUATOR 5 // Change this to the appropriate pin
#define STEPS_PER_REVOLUTION 200  // Change according to your motor's specifications
#define STEP_PIN_STEPPER_MOTOR 6 // Change this to the appropriate pin
#define DIR_PIN_STEPPER_MOTOR 7 // Change this to the appropriate pin
#define EN_PIN_STEPPER_MOTOR 8 // Change this to the appropriate pin
#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN_STEPPER_MOTOR, DIR_PIN_STEPPER_MOTOR);

#define TRIGGER_PIN_1  12
#define ECHO_PIN_1     13
#define TRIGGER_PIN_2  10  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_2     11  // Change to the pin number you're using for the second ultrasonic sensor
#define TRIGGER_PIN_3  51  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_3     53  // Change to the pin number you're using for the second ultrasonic sensor
#define TRIGGER_PIN_4  46  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_4     48  // Change to the pin number you're using for the second ultrasonic sensor
#define TRIGGER_PIN_5  42  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_5     44  // Change to the pin number you're using for the second ultrasonic sensor
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
#define PING_INTERVAL 0 // Set the desired ping interval in milliseconds (e.g., 20ms for approximately 50 pings per second)
unsigned long lastPingTime = 0; // Variable to store the time of the last ping

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 1
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2

bool linearActuatorForwardFlag = false;
bool linearActuatorBackwardFlag = false;
bool PlatformDown = false;
bool linearActuatorDown = false;
bool isStopped = false;
bool isStopped2 = false;



/************************************INIT*************************************/
void setup() {
  Serial.begin(115200);
  while (! Serial) 
  {
      delay(1);
  }
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) 
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  pinMode(RELAY_PIN_IN3_ACTUATOR, OUTPUT);
  pinMode(RELAY_PIN_IN4_ACTUATOR, OUTPUT);

  pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  GRIPPER.attach(9);

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch2.setDebounceTime(50); // set debounce time to 50 milliseconds

  stepper.setMaxSpeed(50.0);   // set the maximum speed
  stepper.setAcceleration(30.0); // set acceleration
  stepper.setSpeed(30);         // set initial speed
  stepper.setCurrentPosition(0); // set position
  stepper.moveTo(MAX_POSITION);
}

/**********************************AC MOTOR***********************************/
void RotatePlatformUpEvent()
{   
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, HIGH);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);
}

void RotatePlatformDownEvent()
{   
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, LOW);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, HIGH);
}

void StopPlatformRotation()
{
    digitalWrite(RELAY_PIN_IN1_AC_MOTOR, LOW);
    digitalWrite(RELAY_PIN_IN2_AC_MOTOR, LOW);
}

/**********************************LINEAR ACTUATOR***********************************/
void LinearActuatorBackward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, HIGH);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}

void LinearActuatorForward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, HIGH);
}

void LinearActuatorOff()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}

/**********************************STEPPER MOTOR***********************************/
void LinearActuatorGoDown()
{
    stepper.setSpeed(-50);
    stepper.runSpeed();
}

void LinearActuatorGoUp()
{
    stepper.setSpeed(50);
    stepper.runSpeed();
}

void LinearActuatorGoStop()
{
  stepper.setSpeed(0); 
  stepper.runSpeed();
}

/**********************************GRIPPER ARM***********************************/
void gripperOpen()
{
        // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
      GRIPPER.write(pos); // move the servo to the current position
      delay(15); // wait for the servo to reach the position
  }
}

void gripperClose()
{
      for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
    }
}

  /*********************************MAIN ENTRY*************************************/
void loop() {
  // put your main code here, to run repeatedly:
  /*********************************UNIT TESTING*************************************/
  //RetrieveTrayEvent();
  //StopPlatformRotation();
  //RotatePlatformUpEvent();
  //RotatePlatformDownEvent();
  //LinearActuatorBackward();
  //LinearActuatorForward();
  //LinearActuatorGoDown();
  //LinearActuatorGoUp();
  /*********************************UNIT TESTING*************************************/


  /*********************************START OPERATION*************************************/   
  limitSwitch.loop(); // MUST call the loop() function first
  limitSwitch2.loop(); // MUST call the loop() function first

  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

    unsigned long currentTime = micros();
    int distance1 = sonar1.ping_cm(); // Send ping from sensor 1, get distance in cm
    int distance2 = sonar2.ping_cm(); // Send ping from sensor 2, get distance in cm
    int distance3 = sonar3.ping_cm(); // Send ping from sensor 2, get distance in cm
    int distance4 = sonar4.ping_cm(); // Send ping from sensor 2, get distance in cm
    int distance5 = sonar5.ping_cm(); // Send ping from sensor 2, get distance in cm

    // Check if it's time to send another ping
    if (currentTime - lastPingTime >= PING_INTERVAL) {
      // Send pings from both sensors

      // Print distances
      Serial.print("Distance Sensor 1: ");
      Serial.print(distance1);
      Serial.println("cm");

      Serial.print("Distance Sensor 2: ");
      Serial.print(distance2);
      Serial.println("cm");

      Serial.print("Distance Sensor 3: ");
      Serial.print(distance3);
      Serial.println("cm");

      Serial.print("Distance Sensor 4: ");
      Serial.print(distance4);
      Serial.println("cm");

      Serial.print("Distance Sensor 5: ");
      Serial.print(distance5);
      Serial.println("cm"); 
      // Update last ping time
      lastPingTime = currentTime;
    }

    /* if((distance1 <= 3) && (distance2 <= 3)) //if tray is detected
    {
        gripperClose(); // close the gripper 
        LinearActuatorBackward();
        linearActuatorBackwardFlag = true; //set flag for actuator to retract
    }

    else 
    {
      gripperOpen(); // open gripper
       //LinearActuatorForward();
       LinearActuatorBackward();
      linearActuatorForwardFlag = true;
    } */

/*********************************************************************************************/
    if (limitSwitch.isPressed()) 
    {
        Serial.println(F("The limit switch: TOUCHED"));
        isStopped = true;
    }

    if ( (isStopped == false ) && (measure.RangeMilliMeter <= 100) ) //distance3 <= 100
    {
        if (stepper.distanceToGo() == 0) 
        { 
          stepper.setCurrentPosition(0);   // reset position to 0
          stepper.moveTo(MAX_POSITION);       // move the motor to maximum position again
        }
        LinearActuatorGoDown();
        RotatePlatformDownEvent();
    } 
    else 
    {
        Serial.println(F("The stepper motor is STOPPED"));
        //StopPlatformRotation();
    } 

/**************************************************************************************************/
    if (limitSwitch2.isPressed()) 
    {
        Serial.println(F("The limit switch: TOUCHED"));
        isStopped2 = true;
    }

    if ( (isStopped2 == false ) && (distance3 <= 10) ) //distance3 <= 100
    {
        if (stepper.distanceToGo() == 0) 
        { 
          stepper.setCurrentPosition(0);   // reset position to 0
          stepper.moveTo(MAX_POSITION);       // move the motor to maximum position again
        }
        LinearActuatorGoUp();
        RotatePlatformUpEvent();
    } 
    else 
    {
        Serial.println(F("The stepper motor is STOPPED"));
        //StopPlatformRotation();
    } 

}
