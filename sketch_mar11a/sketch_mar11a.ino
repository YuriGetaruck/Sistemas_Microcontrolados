void setup() {
  Serial.begin(9600);

}

int a, b, c = 0;

void loop() {

  a++;
  b=b+2;
  c--;

  delay(250);
  Serial.print(a);
  Serial.print("\t");
  Serial.print(b);
  Serial.print("\t");
  Serial.println(c);
  
  //Serial.println(";");
 
}
