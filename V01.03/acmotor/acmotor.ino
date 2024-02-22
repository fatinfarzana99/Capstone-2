#define RELAY_PIN_IN1 2 // Change this to the appropriate pin
#define RELAY_PIN_IN2 3 // Change this to the appropriate pin

void setup() {
  // put your setup code here, to run once:
  pinMode(RELAY_PIN_IN1, OUTPUT);
  pinMode(RELAY_PIN_IN2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(RELAY_PIN_IN1, HIGH);
  digitalWrite(RELAY_PIN_IN2, LOW);
  delay(5000); // 5000 milliseconds = 5 seconds
}
