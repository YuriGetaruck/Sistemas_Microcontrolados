int tempo_de_queda = 300;
int ledPin = 13;
int analoPin = 2;
int espera = 1.17;
int degrau = 1;



void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {


  int i = 0;

    while ( i <= 255 ){
        analogWrite( ledPin, i );
        delay( espera );
        i = i + degrau;
    }

    i = 255;
    while ( i >= 0 ){
        analogWrite( ledPin, i );
        delay( espera );
        i = i - degrau;
    }

}
