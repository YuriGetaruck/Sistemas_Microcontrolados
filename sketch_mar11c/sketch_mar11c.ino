int high = 27;
int low = 0;
int ciclo = 0;
int pinLed = 13;

void setup() {
  Serial.begin(9600);
  pinMode(pinLed, OUTPUT);
}

void loop() {
  digitalWrite(pinLed, HIGH);
  delay(high);
  digitalWrite(pinLed, LOW);
  delay(low);

  if(ciclo == 0){
    high--;
    low++;
  }else{
    high++;
    low--;
  }
  if(high == 0) ciclo=1;
  if(low == 0) ciclo =0;

  Serial.print(high);
  Serial.print('\t');
  Serial.println(low);
}
