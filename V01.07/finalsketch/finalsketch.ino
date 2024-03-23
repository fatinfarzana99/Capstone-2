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

bool linearActuatorForwardFlag = false;
bool linearActuatorBackwardFlag = false;
bool PlatformDown = false;

#define TRIGGER_PIN_1  12
#define ECHO_PIN_1     13
#define TRIGGER_PIN_2  10  // Change to the pin number you're using for the second ultrasonic sensor
#define ECHO_PIN_2     11  // Change to the pin number you're using for the second ultrasonic sensor
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
#define PING_INTERVAL 20 // Set the desired ping interval in milliseconds (e.g., 20ms for approximately 50 pings per second)
unsigned long lastPingTime = 0; // Variable to store the time of the last ping



#define LIMITSWITCH_1 50 // Change this to the appropriate pin
#define LIMITSWITCH_2 52 // Change this to the appropriate pin

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 1
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance for sensor 2



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(RELAY_PIN_IN3_ACTUATOR, OUTPUT);
  pinMode(RELAY_PIN_IN4_ACTUATOR, OUTPUT);

  pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  stepper.setMaxSpeed(1000);  // Adjust as needed
  stepper.setAcceleration(500);  // Adjust as needed

  GRIPPER.attach(9);

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  limitSwitch2.setDebounceTime(50); // set debounce time to 50 milliseconds

}
void LimitSwitches()
{
  limitSwitch.loop(); // MUST call the loop() function first
  limitSwitch2.loop(); // MUST call the loop() function first

  /***********************LIMIT SWITCH 1*****************************/
  if(limitSwitch.isPressed())
    Serial.println("The limit switch 1: UNTOUCHED -> TOUCHED");

  if(limitSwitch.isReleased())
    Serial.println("The limit switch 1: TOUCHED -> UNTOUCHED");

  int state = limitSwitch.getState();
  if(state == HIGH)
    Serial.println("The limit switch 1: UNTOUCHED");
  else
    Serial.println("The limit switch 1: TOUCHED");

  /***********************LIMIT SWITCH 2*****************************/
  if(limitSwitch2.isPressed())
    Serial.println("The limit switch 2: UNTOUCHED -> TOUCHED");

  if(limitSwitch2.isReleased())
    Serial.println("The limit switch 2: TOUCHED -> UNTOUCHED");

  int state2 = limitSwitch2.getState();
  if(state2 == HIGH)
    Serial.println("The limit switch 2: UNTOUCHED");
  else
    Serial.println("The limit switch 2: TOUCHED");
}

 /***************************************************************************************/
 /**********************************AC MOTOR*********************************************/
/****************************************************************************************/
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
/****************************************************************************************/

 /***************************************************************************************/
 /**********************************LINEAR ACTUATOR****************************************/
/****************************************************************************************/
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
/****************************************************************************************/

 /***************************************************************************************/
 /**********************************STEPPER MOTOR****************************************/
/****************************************************************************************/
void LinearActuatorGoDown()
{
  // Set the speed at which the stepper motor will rotate
  stepper.setSpeed(-300); // Adjust as needed
  // Rotate the motor continuously
  stepper.runSpeed();

}

void LinearActuatorGoUp()
{
  // Set the speed at which the stepper motor will rotate
  stepper.setSpeed(300); // Adjust as needed
  // Rotate the motor continuously
  stepper.runSpeed();
}

void LinearActuatorGoStop()
{
  // Set the speed at which the stepper motor will rotate
  stepper.setSpeed(0); // Adjust as needed
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
    delay(2000);

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

 /***********************************************************************************/
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


void StopLinearActuatorAndBelt()
{



  if(limitSwitch2.isPressed())
    Serial.println("The limit switch 2: UNTOUCHED -> TOUCHED");

  if(limitSwitch2.isReleased())
    Serial.println("The limit switch 2: TOUCHED -> UNTOUCHED");

  int state2 = limitSwitch2.getState();
  if(state2 == HIGH)
  {    Serial.println("The limit switch 2: UNTOUCHED");
       
       //LinearActuatorGoUp();
       //LinearActuatorGoStop();
      // RotatePlatformDownEvent();
      LinearActuatorGoStop();
  }

  else
  {
    Serial.println("The limit switch 2: TOUCHED");
    LinearActuatorGoDown();
    //StopPlatformRotation();

    
  }

 // 

}
 /***********************************************************************************/
  


void loop() {
  // put your main code here, to run repeatedly:
  
  //RetrieveTrayEvent();
  //RotatePlatformEvent();

  //LinearActuatorBackward();
  //LinearActuatorForward();

  //LinearActuatorGoDown();
  //LinearActuatorGoUp();



  //delay(1000);

  //gripperClose();
  /*************US SENSOR******************************************************/
    /*unsigned long currentTime = millis();
    int distance1 = sonar1.ping_cm(); // Send ping from sensor 1, get distance in cm
    int distance2 = sonar2.ping_cm(); // Send ping from sensor 2, get distance in cm
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

      // Update last ping time
      lastPingTime = currentTime;
    }

    if((distance1 <= 10) && (distance2 <= 10))
    {
        gripperClose();
        delay(1000);
        //LinearActuatorBackward();
        linearActuatorBackwardFlag = true;
    }

    else
    {
      gripperOpen();
      //LinearActuatorForward();
      //LinearActuatorBackward();
      linearActuatorForwardFlag = true;
    }

    if(linearActuatorBackwardFlag == true)
    {
      LinearActuatorBackward();
      PlatformDown = true;
      linearActuatorBackwardFlag = false;
    }

    else if(linearActuatorBackwardFlag == false)
    {
      LinearActuatorForward();
    }
    
    if(PlatformDown == true)
    {
      StopLinearActuatorAndBelt();
      PlatformDown = false;
    }*/
    limitSwitch.loop(); // MUST call the loop() function first
    limitSwitch2.loop(); // MUST call the loop() function first

  if(limitSwitch.isPressed())
    Serial.println("The limit switch 1: UNTOUCHED -> TOUCHED");

  if(limitSwitch.isReleased())
    Serial.println("The limit switch 1: TOUCHED -> UNTOUCHED");

  int state = limitSwitch.getState();
  if(state == HIGH)
  {
    Serial.println("The limit switch 1: UNTOUCHED");
    //RotatePlatformUpEvent();
    //RotatePlatformDownEvent();
    //StopPlatformRotation();
    LinearActuatorGoStop();
  }

  else
  {
    Serial.println("The limit switch 1: TOUCHED");  
    //StopPlatformRotation();
    
    LinearActuatorGoUp();
  }
  
 // StopLinearActuatorAndBelt();



}



