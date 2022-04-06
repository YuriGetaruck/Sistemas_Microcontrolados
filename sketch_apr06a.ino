const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);  
}

void loop() {
  digitalWrite(LED_BUILTIN, state);
}


void blink(){
  state = !state;
}
