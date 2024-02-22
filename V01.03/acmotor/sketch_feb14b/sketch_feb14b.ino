
//INTEGRATION TESTING WITHOUT SENSORS
// Define the pin connected to the relay module
#include <Servo.h>
  
Servo object;
int angle = 0;
#define RELAY_PIN_IN1 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2 3 // Change this to the appropriate pin

const int stepPin = 4; 
const int dirPin = 5; 
const int enPin = 6;

int in1 = 9; //Declaring the pins where in1 in2 from the driver are wired 
int in2 = 8; //here they are wired with D9 and D8 from Arduino

void setup() {
  // Set the relay pin as an output
  pinMode(RELAY_PIN_IN1, OUTPUT);
  pinMode(RELAY_PIN_IN2, OUTPUT);

  pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(in2, OUTPUT);

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  object.attach(7);
}

void TurnMotorA(){              
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void TurnOFFA(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void TurnMotorAAntiCW(){              
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void loop() {
  // Turn on the motor for 5 seconds
  TurnMotorA(); //in the loop we use the function to turn the motor for 3s and stop it for 2s
  delay(5000);

  TurnOFFA();
  delay(1000);
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  for(int x = 0; x < 800; x++) 
  {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(500); 
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(500); 
  }
  delay(5000); // One second delay*/

  for(angle = 0; angle<=180; angle++)
    {
      object.write(angle);
       delay(40);
    }  

  digitalWrite(RELAY_PIN_IN1, HIGH);
  digitalWrite(RELAY_PIN_IN2, LOW);
  delay(5000); // 5000 milliseconds = 5 seconds
/*****************************************************************************************/
  // Turn off the motor for 5 seconds
  TurnMotorAAntiCW(); 
  delay(5000);

  TurnOFFA();
  delay(1000);

  digitalWrite(dirPin,LOW); // Enables the motor to move in a particular direction
  for(int x = 0; x < 800; x++) 
  {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(500); 
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(500); 
  }
  delay(5000);

      for(angle = 180; angle >= 0; angle--)
      {
        object.write(angle);
        delay(40);
      }

  digitalWrite(RELAY_PIN_IN2, HIGH);
  digitalWrite(RELAY_PIN_IN1, LOW);
  delay(5000); // 5000 milliseconds = 5 seconds
}
