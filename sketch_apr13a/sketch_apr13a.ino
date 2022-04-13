unsigned long time;
unsigned long flag = 0;
unsigned long flag1 = 0;
int pinLed = 13;
volatile byte state = LOW;
int contSeg = 57;
int contMin = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("time:");
  time = millis();

  //Serial.println(time);
  
  if(time - flag > 300){
    digitalWrite(LED_BUILTIN, state);
    state = !state;
    flag = time;
    //Serial.println(state);
  }
    
    if(time - flag1 > 1000){


      

      contSeg = contSeg + 1;
        
      
      if(contSeg > 59){
          contMin = contMin + 1; 
          contSeg = 0;
      }
      

      Serial.print(contMin);
      Serial.print(":");
      Serial.println(contSeg);
      flag1 = time;
      
    }
}
