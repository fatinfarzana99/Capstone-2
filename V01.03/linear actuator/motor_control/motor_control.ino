//DC MOTOR CONTROL

int in1 = 5; //Declaring the pins where in1 in2 from the driver are wired 
int in2 = 4; //here they are wired with D9 and D8 from Arduino
void setup() {
  pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(in2, OUTPUT);
}
void TurnMotorA(){              
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void TurnOFFA(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
void TurnMotorAcw(){              
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void loop() {
  //TurnMotorA(); //in the loop we use the function to turn the motor for 3s and stop it for 2s
  TurnMotorAcw();
  //delay(3000);
  //TurnOFFA();
  //delay(2000);
}
