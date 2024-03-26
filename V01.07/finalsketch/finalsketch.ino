#include <Servo.h>
#include <AccelStepper.h>
#include <NewPing.h>
#include <ezButton.h>

ezButton limitSwitch(50);  // create ezButton object that attach to pin 7;
ezButton limitSwitch2(52);  // create ezButton object that attach to pin 7;

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
#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)

bool linearActuatorForwardFlag = false;
bool linearActuatorBackwardFlag = false;
bool PlatformDown = false;
bool linearActuatorDown = false;
bool isStopped = false;

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
#define PING_INTERVAL 20 // Set the desired ping interval in milliseconds (e.g., 20ms for approximately 50 pings per second)
unsigned long lastPingTime = 0; // Variable to store the time of the last ping



#define LIMITSWITCH_1 50 // Change this to the appropriate pin
#define LIMITSWITCH_2 52 // Change this to the appropriate pin

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 1
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2


void setup() {
  Serial.begin(9600);

  pinMode(RELAY_PIN_IN3_ACTUATOR, OUTPUT);
  pinMode(RELAY_PIN_IN4_ACTUATOR, OUTPUT);

  pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  stepper.setMaxSpeed(50.0);   // set the maximum speed
  stepper.setAcceleration(30.0); // set acceleration
  stepper.setSpeed(30);         // set initial speed
  stepper.setCurrentPosition(0); // set position
  stepper.moveTo(MAX_POSITION);

  GRIPPER.attach(9);

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch2.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void StepperGoDown()
{
  limitSwitch.loop(); // MUST call the loop() function first

  if (limitSwitch.isPressed()) {
    Serial.println(F("The limit switch: TOUCHED"));
    isStopped = true;
  }

  if (isStopped == false) {
    // without this part, the move will stop after reaching maximum position
    if (stepper.distanceToGo() == 0) { // if motor moved to the maximum position
      stepper.setCurrentPosition(0);   // reset position to 0
      stepper.moveTo(MAX_POSITION);       // move the motor to maximum position again
    }

    stepper.run(); // MUST be called in loop() function
  } else {
    // without calling stepper.run() function, motor stops immediately
    // NOTE: stepper.stop() function does NOT stops motor immediately
    Serial.println(F("The stepper motor is STOPPED"));
  }
}

/**********************************AC MOTOR***********************************/
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
/**********************************AC MOTOR***********************************/


/**********************************LINEAR ACTUATOR***********************************/
void LinearActuatorBackward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, HIGH);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);

    //delay(60000); // Delay for 1 minute
    //delay(43000); // Delay for 43 seconds // wait for a second
    
    linearActuatorBackwardFlag = true;
}

void LinearActuatorForward()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, HIGH);

    //delay(60000); // Delay for 1 minute
    //delay(43000); // Delay for 43 seconds // wait for a second
}

void LinearActuatorOff()
{
    digitalWrite(RELAY_PIN_IN3_ACTUATOR, LOW);
    digitalWrite(RELAY_PIN_IN4_ACTUATOR, LOW);
}
/**********************************LINEAR ACTUATOR***********************************/


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
/**********************************STEPPER MOTOR***********************************/


/**********************************GRIPPER ARM***********************************/
void gripperOpen()
{

        // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
      GRIPPER.write(pos); // move the servo to the current position
      delay(15); // wait for the servo to reach the position
  }
  /*while(1)
  {

  }*/

}

void gripperClose()
{
      for (pos = 0; pos <= 180; pos += 1) { 
    GRIPPER.write(pos); // move the servo to the current position
    delay(15); // wait for the servo to reach the position
    }

    /*while(1)
    {

    }*/
}
/**********************************GRIPPER ARM***********************************/


void StopLinearActuatorAndBelt()
{

  if (limitSwitch.isPressed()) {
    Serial.println(F("The limit switch: TOUCHED"));
    isStopped = true;
  }

  if (isStopped == false) {
    stepper.run(); // MUST be called in loop() function
  } else {
    // without calling stepper.run() function, motor stops immediately
    // NOTE: stepper.stop() function does NOT stops motor immediately
    Serial.println(F("The stepper motor is STOPPED"));
  }

    
}

void RetrieveTray()
{
      /*if((distance1 <= 10) && (distance2 <= 10)) //if tray is detected
    {
        gripperClose(); // close the gripper 
        delay(1000); //1 second dealay
        linearActuatorBackwardFlag = true; //set flag for actuator to retract
    }

    else if((distance1 > 10) && (distance2 > 10))
    {
      gripperOpen(); // open gripper
      linearActuatorForwardFlag = true;
    }
    
    if(linearActuatorBackwardFlag == true) // set when tray detected
    {
      gripperClose(); // close gripper
      LinearActuatorBackward(); // set linear actuator to retract
      linearActuatorBackwardFlag = false;   // reset flag
    }

    else if(linearActuatorBackwardFlag == false) // default is false
    {
      LinearActuatorForward(); // linear actuator forward to start retrieval
    }*/

}

 /***********************************************************************************/

void loop() {
  // put your main code here, to run repeatedly:
  /************UNIT TESTING**************/
  //RetrieveTrayEvent();
  //StopPlatformRotation();
  //RotatePlatformUpEvent();
  //RotatePlatformDownEvent();
  //LinearActuatorBackward();
  //LinearActuatorForward();
  //LinearActuatorGoDown();
  //LinearActuatorGoUp();
  /************UNIT TESTING**************/

//StopPlatformRotation();

//LinearActuatorGoDown();
//delay(50);
//LinearActuatorGoStop();


  /*********************************MAIN ENTRY*************************************/
    limitSwitch.loop(); // MUST call the loop() function first
    unsigned long currentTime = millis();
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

    if((distance1 <= 3) && (distance2 <= 3)) //if tray is detected
    {
        gripperClose(); // close the gripper 
        LinearActuatorBackward();
        linearActuatorBackwardFlag = true; //set flag for actuator to retract
    }

    else if((distance1 > 10) && (distance2 > 10))
    {
      gripperOpen(); // open gripper
      LinearActuatorForward();
      linearActuatorForwardFlag = true;
    }

    if((distance1 <= 10) && (distance2 <= 10) && (distance3 <= 10) )
    {
      PlatformDown = true;
      //RotatePlatformDownEvent();
      LinearActuatorGoDown();
    }

    else if((distance1 <= 10) && (distance2 <= 10) && (distance3 > 10) )
    {
      LinearActuatorGoStop();
      //StopPlatformRotation();
    }


    if (limitSwitch.isPressed()) 
    {
        Serial.println(F("The limit switch: TOUCHED"));
        isStopped = true;
    }

    if ((isStopped == false) && (distance3 <= 10)) 
    {
        // without this part, the move will stop after reaching maximum position
        if (stepper.distanceToGo() == 0) 
        { // if motor moved to the maximum position
          stepper.setCurrentPosition(0);   // reset position to 0
          stepper.moveTo(MAX_POSITION);       // move the motor to maximum position again
        }
        RotatePlatformDownEvent();
        stepper.run(); // MUST be called in loop() function
    } 
    else 
    {
        // without calling stepper.run() function, motor stops immediately
        // NOTE: stepper.stop() function does NOT stops motor immediately
        Serial.println(F("The stepper motor is STOPPED"));
        StopPlatformRotation();
    }


/*************************************************************************************/
    /*if((distance1 <= 10) && (distance2 <= 10) && (distance3 <= 10) )
    {
      PlatformDown = true;
      //RotatePlatformDownEvent();
      //LinearActuatorGoDown();
    }

    else if((distance1 <= 10) && (distance2 <= 10) && (distance3 > 10) )
    {
      //RotatePlatformDownEvent();
      StopPlatformRotation();    
    }

    if(PlatformDown == true)
    {
      LinearActuatorGoDown();
      RotatePlatformDownEvent();
      PlatformDown = false;
    }*/
    //LinearActuatorGoDown();


}



