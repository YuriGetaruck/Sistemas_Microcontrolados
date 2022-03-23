int byteEntrada = 0;


void setup() {
  Serial.begin(9600);
 
}

void loop() {


  if (Serial.available()){
    byteEntrada = Serial.read();
    Serial.print("recebido: ");
    Serial.println(byteEntrada, HEX);  
  }
  
}
