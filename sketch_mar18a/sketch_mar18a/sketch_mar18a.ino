int tempo_de_queda = 300;
int ledPin = 13;
int inPinSobe = 52;
int inPinDesce = 53;
int espera = 1.17;
int degrau = 1;
int controle = 0;
int i = 0;



void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(inPinSobe, INPUT_PULLUP);
  pinMode(inPinDesce, INPUT_PULLUP);
}

void loop() {
  



  Serial.print(digitalRead(inPinSobe));
  Serial.print(digitalRead(inPinDesce));
  Serial.println();
  
  if(digitalRead(inPinSobe)==LOW){
    if(controle == 0){
      while ( i <= 255 ){
          analogWrite( ledPin, i );
          delay( espera );
          i = i + degrau;
      }
      controle = 1;
    }
  }
  if(digitalRead(inPinDesce)==LOW){
    //i = 255;
    if(controle == 1){
      while ( i >= 0 ){
          analogWrite( ledPin, i );
          delay( espera );
          i = i - degrau;
      }
    controle=0;
    }
  }
}
