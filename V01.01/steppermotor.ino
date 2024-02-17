//integration

const int stepPin = 5; 
const int dirPin = 2; 
const int enPin = 8;
int in1 = 7; //Declaring the pins where in1 in2 from the driver are wired 
int in2 = 6; //here they are wired with D9 and D8 from Arduino

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

const int trigPin1 = 11;
const int echoPin1 = 12;

// defines variables
long duration;
int distance;
long duration1;
int distance1;

void setup() {
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);

  pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(in2, OUTPUT);
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
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

void LinearActuator()
{
    if(distance1 >= 15 && distance1 <= 30 ) //limit switch logic 
  {
    /*digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
    for(int x = 0; x < 800; x++) 
    {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(500); 
      digitalWrite(stepPin,LOW); 
      delayMicroseconds(500); 
    }
    delay(1000); // One second delay*/

  TurnMotorACW();
  delay(1000);

  }

  if(distance1 > 30 && distance <= 60)
  {
    TurnMotorAAntiCW();
    delay(1000);
  }

  else
  {
      TurnOFFA();
      delay(1000);
  }
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
  duration1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance1 = duration1 * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance1: ");
  Serial.println(distance1);
  
  LinearActuator();


  /*digitalWrite(dirPin,LOW); //Changes the direction of rotation
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  delay(1000); */
}