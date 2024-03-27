#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
#define RELAY_PIN_IN1_AC_MOTOR 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2_AC_MOTOR 3 // Change this to the appropriate pin

#define STEP_PIN_STEPPER_MOTOR 6 // Change this to the appropriate pin
#define DIR_PIN_STEPPER_MOTOR 7 // Change this to the appropriate pin
#define EN_PIN_STEPPER_MOTOR 8 // Change this to the appropriate pin

bool stepperMotor = false;
void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
      pinMode(RELAY_PIN_IN1_AC_MOTOR, OUTPUT);
  pinMode(RELAY_PIN_IN2_AC_MOTOR, OUTPUT);

  pinMode(STEP_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(DIR_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(EN_PIN_STEPPER_MOTOR, OUTPUT);
  pinMode(EN_PIN_STEPPER_MOTOR, LOW);
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
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  if(measure.RangeMilliMeter <= 100)
  { 
    //RotatePlatformDownEvent();
    //stepperMotor = true;
    digitalWrite(DIR_PIN_STEPPER_MOTOR, HIGH);
    for(int x = 0; x < 800; x++)
    {
      digitalWrite(STEP_PIN_STEPPER_MOTOR, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN_STEPPER_MOTOR, LOW);
      delayMicroseconds(500);

    }
  }

  else
  {

    //StopPlatformRotation();
  }

  /*if (stepperMotor == true)
  {
    digitalWrite(DIR_PIN_STEPPER_MOTOR, HIGH);
    for(int x = 0; x < 800; x++)
    {
      digitalWrite(STEP_PIN_STEPPER_MOTOR, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN_STEPPER_MOTOR, LOW);
      delayMicroseconds(500);

    }
    stepperMotor = false;
  }*/
    
 // delay(100);
}
