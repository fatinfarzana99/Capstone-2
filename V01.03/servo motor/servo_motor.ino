#include <Servo.h>

//gripper arm

Servo object;
int angle = 0;

//const int trigPin = 9;
//const int echoPin = 10;

//int in1 = 11; //Declaring the pins where in1 in2 from the driver are wired 
//int in2 = 12; //here they are wired with D9 and D8 from Arduino

// defines variables
//long duration;
//int distance;


void setup() {
  // put your setup code here, to run once:
  //pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  //pinMode(in2, OUTPUT);

  //pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  //Serial.begin(9600); // Starts the serial communication
  object.attach(8);
}

void TurnMotorACW(){              
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void TurnMotorAAntiCW(){              
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void TurnOFFA(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
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

  


  if(distance >= 5 && distance <=13)
  {
      TurnOFFA();
      for(angle = 0; angle<=180; angle++)
      {
        object.write(angle);
        delay(40);
      }

  }
  else
  {
    TurnMotorACW();
      for(angle = 180; angle >= 0; angle--)
      {
        object.write(angle);
        delay(40);
      }
  }


}
