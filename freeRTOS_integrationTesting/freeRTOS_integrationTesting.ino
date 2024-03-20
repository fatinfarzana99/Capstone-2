#include <Servo.h>
#include <Arduino_FreeRTOS.h> // Include the FreeRTOS library

Servo object; // Servo object for the gripper arm
int angle = 0;

#define RELAY_PIN_IN1 2 // Pin connected to the relay module
#define RELAY_PIN_IN2 3 // Pin connected to the relay module

const int stepPin = 4; // Pin connected to the stepper motor driver
const int dirPin = 5;  // Pin connected to the stepper motor driver
const int enPin = 6;   // Pin connected to the stepper motor driver
int in1 = 9;           // Pin connected to the linear actuator
int in2 = 8;           // Pin connected to the linear actuator

TaskHandle_t TaskLinearActuator_Handle; // Task handle for the linear actuator task
TaskHandle_t TaskGripperArm_Handle;     // Task handle for the gripper arm task

void setup() {
  // Initialize pin modes
  pinMode(RELAY_PIN_IN1, OUTPUT);
  pinMode(RELAY_PIN_IN2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  object.attach(7); // Attach servo to pin 7

  // Create tasks
  xTaskCreate(TaskLinearActuator, "LinearActuator", 10000, NULL, 1, &TaskLinearActuator_Handle);
  xTaskCreate(TaskGripperArm, "GripperArm", 10000, NULL, 1, &TaskGripperArm_Handle);
}

// Task for controlling the linear actuator
void TaskLinearActuator(void* pvParameters) {
  for (;;) {
    TurnMotorA();          // Turn linear actuator in one direction
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
    TurnMotorAAntiCW();    // Turn linear actuator in the other direction
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
  }
}

// Task for controlling the gripper arm
void TaskGripperArm(void* pvParameters) {
  for (;;) {
    OpenGripperArm();      // Open gripper arm
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
    CloseGripperArm();     // Close gripper arm
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
  }
}

/*********************************MAIN LOOP*********************************/
void loop() {
  // Empty, since tasks are running in FreeRTOS scheduler
}
/*********************************MAIN LOOP*********************************/


// Function to turn the linear actuator in one direction
void TurnMotorA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

// Function to turn off the linear actuator
void TurnOFFA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// Function to turn the linear actuator in the other direction
void TurnMotorAAntiCW(){              
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// Function to open gripper arm
void OpenGripperArm()
{
    for(angle = 0; angle<=180; angle++)
    {
      object.write(angle);
       delay(40);
    }  
}

// Function to close gripper arm
void CloseGripperArm()
{
        for(angle = 180; angle >= 0; angle--)
      {
        object.write(angle);
        delay(40);
      }
}
