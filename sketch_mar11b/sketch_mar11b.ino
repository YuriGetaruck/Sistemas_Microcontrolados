int pinLed=13;


void setup() {
  Serial.begin(9600);
  pinMode(pinLed, OUTPUT);

}

void loop() {
  digitalWrite(pinLed, LOW);
  delay(200);
  digitalWrite(pinLed, HIGH);
  delay(200);

}
