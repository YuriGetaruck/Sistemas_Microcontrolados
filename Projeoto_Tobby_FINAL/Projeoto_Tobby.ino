/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 15;

int m0,m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11; 

double L1 = 69;
double L2 = 47;
int delaypata = 15;


//double x = 30;
//double y = 90;




void setup() {
  Serial.begin(9600);

  pwm.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  pwm.setOscillatorFrequency(25200000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  Serial.print("0 - posição padrão \n1 - Fica em pé \n2 - Senta \n3 - Deita \n4 - Anda Individual \n5 - Anda 1e3 2e4\n6 - Anda 1e2 2e4\n7 - Finge de morto por 3 sec\n8 - Tomba esquerda\n9 - Tomba direita\n10 Desvira Lateral\n11 - Desvira Total  \n20 - Mostra posição salva dos motores\n\n500 - teste de trasição\n1500 - teste de motor");
  delay(10);

  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

// ***** valores de referência *****
// centro 1500
// -90 = 500
// +90 = 2500



void stand() {
  
  //for(int i = 0; i < 1000; i=i+10){
    pwm.writeMicroseconds(0, 1700);
    m0 = 1700;
    pwm.writeMicroseconds(5, 1300);
    m5 = 1300;
    pwm.writeMicroseconds(6, 1300);
    m6 = 1300;
    pwm.writeMicroseconds(11, 1500);
    m11 = 1500;
    pwm.writeMicroseconds(7, 1000);
    m7 = 1000;
    pwm.writeMicroseconds(10, 2000);
    m10 = 2000;
    pwm.writeMicroseconds(1, 2000);
    m1 = 2000;
    pwm.writeMicroseconds(4, 1000);
    m4 = 1000;
  //}
}

void stand2(){
    int x = 10;
    int y = 80;
    
    double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
  
    double angmotor1 = 90-(theta1rad*180/3.14);
    double angmotor2 = 90-(theta2rad*180/3.14);

    double angmotor3 = 90+(theta1rad*180/3.14);
    double angmotor4 = 90+(theta2rad*180/3.14);

    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
    if(angmotor3 > 90){
      angmotor3 = angmotor3 - 180;
    }
    if(angmotor4 > 90){
      angmotor4 = angmotor4 - 180;
    }

    double posmotor3 = (angmotor3 * 100/9) + 1500;
    double posmotor4 = (angmotor4 * 100/9) + 1500;
    
    long double posmotor1 = (angmotor1 * 100/9) + 1500;
    long double posmotor2 = (angmotor2 * 100/9) + 1500;

    pwm.writeMicroseconds(0, posmotor2);
    pwm.writeMicroseconds(1, posmotor1);

    pwm.writeMicroseconds(5, posmotor4);
    pwm.writeMicroseconds(4, posmotor3);

    delay(200);

    pwm.writeMicroseconds(11, posmotor2);
    pwm.writeMicroseconds(10, posmotor1);

    pwm.writeMicroseconds(6, posmotor4);
    pwm.writeMicroseconds(7, posmotor3);

    delay(1000);
}


void zero() {
  pwm.writeMicroseconds(0, 1500);
  m0 = 1500;
  pwm.writeMicroseconds(1, 1500);
  m1 = 1500;
  pwm.writeMicroseconds(2, 1500);
  m2 = 1500;
  pwm.writeMicroseconds(3, 1500);
  m3 = 1500;
  pwm.writeMicroseconds(4, 1500);
  m4 = 1500;
  pwm.writeMicroseconds(5, 1500);
  m5 = 1500;
  pwm.writeMicroseconds(6, 1500);
  m6 = 1500;
  pwm.writeMicroseconds(7, 1500);
  m7 = 1500;
  pwm.writeMicroseconds(8, 1500);
  m8 = 1500;
  pwm.writeMicroseconds(9, 1500);
  m9 = 1500;
  pwm.writeMicroseconds(10, 1500);
  m10 = 1500;
  pwm.writeMicroseconds(11, 1500);
  m11 = 1500;
}

void deita(){

  //pwm.writeMicroseconds(2, 500);
  //pwm.writeMicroseconds(3, 2500);
  //pwm.writeMicroseconds(8, 2500);
  //pwm.writeMicroseconds(9, 500);

  pwm.writeMicroseconds(10, 2500);
  m10 = 2500;
  pwm.writeMicroseconds(7, 500);
  m7 = 500;
  pwm.writeMicroseconds(4, 500);
  m4 = 500;
  pwm.writeMicroseconds(1, 2500);
  m1 = 2500;
  pwm.writeMicroseconds(0, 2500);
  m0 = 2500;
  pwm.writeMicroseconds(11, 2500);
  m11 = 2500;
  pwm.writeMicroseconds(6, 500);
  m6 = 500;
  pwm.writeMicroseconds(5, 500);
  m5 = 500;
}
//fator de conversao angulo * 100/9 + 1500
void senta(){

  pwm.writeMicroseconds(0, 2500);
  m0 = 2500;
  pwm.writeMicroseconds(5, 500);
  m5 = 500;
  pwm.writeMicroseconds(10, 1000);
  m10 = 1000;
  pwm.writeMicroseconds(7, 2000);
  m7 = 2000;
}

void posicao(){
  Serial.println(m0);
  Serial.println(m1);
  Serial.println(m2);
  Serial.println(m3);
  Serial.println(m4);
  Serial.println(m5);
  Serial.println(m6);
  Serial.println(m7);
  Serial.println(m8);
  Serial.println(m9);
  Serial.println(m10);
  Serial.println(m11);
}

void passoesq(){

          pwm.writeMicroseconds(1, 2000);
          pwm.writeMicroseconds(0, 2100);

          pwm.writeMicroseconds(2, 1500);
          pwm.writeMicroseconds(3, 1500);
          
          pwm.writeMicroseconds(4, 1900);
          pwm.writeMicroseconds(5, 500);
          
          pwm.writeMicroseconds(8, 1500);
          pwm.writeMicroseconds(9, 1500);
          
          pwm.writeMicroseconds(6, 1500);
          pwm.writeMicroseconds(7, 1000);
          
          pwm.writeMicroseconds(11, 2300);
          pwm.writeMicroseconds(10, 1000);
}

void passodir(){
          pwm.writeMicroseconds(1, 1000);
          pwm.writeMicroseconds(0, 2300);
          
          pwm.writeMicroseconds(2, 1500);
          pwm.writeMicroseconds(3, 1500);
          
          pwm.writeMicroseconds(4, 1100);
          pwm.writeMicroseconds(5, 500);
          
          pwm.writeMicroseconds(8, 1500);
          pwm.writeMicroseconds(9, 1500);
          
          pwm.writeMicroseconds(6, 500);
          pwm.writeMicroseconds(7, 2000);
          
          pwm.writeMicroseconds(11, 1500);
          pwm.writeMicroseconds(10, 2000);

  
}


void perna1(){
  double x = 10;
  double y = 80;


  for(int i = 0; i < 280; i++){

    delay(delaypata);
    
    double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
  
    double angmotor1 = 90-(theta1rad*180/3.14);
    double angmotor2 = 90-(theta2rad*180/3.14);

    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
   
    double posmotor1 = (angmotor1 * 100/9) + 1500;
    double posmotor2 = (angmotor2 * 100/9) + 1500;

    //Serial.print(x);
    //Serial.print("\t");
    //Serial.println(posmotor1);
    //Serial.print(y);
    //Serial.print("\t");
    //Serial.println(posmotor2);

  
    pwm.writeMicroseconds(0, posmotor2);
    pwm.writeMicroseconds(1, posmotor1);

    if( i < 20){
      y--;
      //y--;
    }else if(i > 20 && i < 70){
      x++;
    }else if(i > 70 && i < 114){
      y++;
    }else if(i > 114 && i < 164){
      x--;
      //y--;
    }else if(i > 164 && i < 208){
      y--;
    }else if(i > 208 && i < 228){
      x++;
    }else if(i > 228 && i < 248){
      y++;
    }
  }  
}

void perna2(){
  
 
  double x = 10;
  double y = 80;


  for(int i = 0; i < 280; i++){
    
    delay(delaypata);

    
    double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
  
    double angmotor1 = 90+(theta1rad*180/3.14);
    double angmotor2 = 90+(theta2rad*180/3.14);

    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
  
    long double posmotor1 = (angmotor1 * 100/9) + 1500;
    long double posmotor2 = (angmotor2 * 100/9) + 1500;
 
  
    pwm.writeMicroseconds(5, posmotor2);
    pwm.writeMicroseconds(4, posmotor1);

    if( i < 20){
      y--;
      //y--;
    }else if(i > 20 && i < 70){
      x++;
    }else if(i > 70 && i < 114){
      y++;
    }else if(i > 114 && i < 164){
      x--;
      //y--;
    }else if(i > 164 && i < 208){
      y--;
    }else if(i > 208 && i < 228){
      x++;
    }else if(i > 228 && i < 248){
      y++;
    }
  }

}

void perna3(){
  double x = 10;
  double y = 80;


  for(int i = 0; i < 280; i++){

    
    delay(delaypata);

    
    double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
  
    double angmotor1 = 90-(theta1rad*180/3.14);
    double angmotor2 = 90-(theta2rad*180/3.14);

    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
  
    long double posmotor1 = (angmotor1 * 100/9) + 1500;
    long double posmotor2 = (angmotor2 * 100/9) + 1500;
  
    pwm.writeMicroseconds(11, posmotor2);
    pwm.writeMicroseconds(10, posmotor1);

    if( i < 20){
      y--;
      //y--;
    }else if(i > 20 && i < 70){
      x--;
    }else if(i > 70 && i < 114){
      y++;
    }else if(i > 114 && i < 164){
      x++;
      //y--;
    }else if(i > 164 && i < 208){
      y--;
    }else if(i > 208 && i < 228){
      x--;
    }else if(i > 228 && i < 248){
      y++;
    }
  }
}

void perna4(){

  double x = 10;
  double y = 80;


  for(int i = 0; i < 280; i++){
    
    delay(delaypata);

    
    double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
  
    double angmotor1 = 90+(theta1rad*180/3.14);
    double angmotor2 = 90+(theta2rad*180/3.14);

    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
  
    long double posmotor1 = (angmotor1 * 100/9) + 1500;
    long double posmotor2 = (angmotor2 * 100/9) + 1500;
  
    pwm.writeMicroseconds(6, posmotor2);
    pwm.writeMicroseconds(7, posmotor1);

    if( i < 20){
      y--;
      //y--;
    }else if(i > 20 && i < 70){
      x--;
    }else if(i > 70 && i < 114){
      y++;
    }else if(i > 114 && i < 164){
      x++;
      //y--;
    }else if(i > 164 && i < 208){
      y--;
    }else if(i > 208 && i < 228){
      x--;
    }else if(i > 228 && i < 248){
      y++;
    }
  }
}

void anda(){

  double xfrente = 10;
  double yfrente = 90;
  double xtras = 10;
  double ytras = 90;

  for(int i = 0; i < 180; i++){
  
    //criação dos angulos para movimentação
    double theta2rad_frente = acos((xfrente*xfrente+yfrente*yfrente-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad_frente = atan(yfrente/xfrente)-atan(L2*sin(theta2rad_frente)/(L1+L2*cos(theta2rad_frente)));// 1 e 2

    double theta2rad_tras = acos((xtras*xtras+ytras*ytras-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad_tras = atan(ytras/xtras)-atan(L2*sin(theta2rad_tras)/(L1+L2*cos(theta2rad_tras)));// 3 e 4

    //converte os angulos de rad para graus
    double angmotor1 = 90-(theta1rad_frente*180/3.14); //1 e 3
    double angmotor2 = 90-(theta2rad_frente*180/3.14); //1 e 3
    double angmotor3 = 90+(theta1rad_tras*180/3.14); //2 e 4
    double angmotor4 = 90+(theta2rad_tras*180/3.14); //2 e 4

    //tratamento de erro para manter os angulos dentro do intervalo possivel
    if(angmotor1 > 90){
      angmotor1 = angmotor1 - 180;
    }
    if(angmotor2 > 90){
      angmotor2 = angmotor2 - 180;
    }
    if(angmotor3 > 90){
      angmotor3 = angmotor3 - 180;
    }
    if(angmotor4 > 90){
      angmotor4 = angmotor4 - 180;
    }

    Serial.print("angulo motor 1:\t");
    Serial.println(angmotor1);
    Serial.print("angulo motor 2:\t");
    Serial.println(angmotor2);
    Serial.print("angulo motor 3:\t");
    Serial.println(angmotor3);
    Serial.print("angulo motor 4:\t");
    Serial.println(angmotor4);

    //convertendo para a posição do motor em milisegundos
    double posmotor1 = (angmotor1 * 100/9) + 1500;
    double posmotor2 = (angmotor2 * 100/9) + 1500;
    double posmotor3 = (angmotor3 * 100/9) + 1500;
    double posmotor4 = (angmotor4 * 100/9) + 1500;

    Serial.print("Posmotor1: ");
    Serial.println(posmotor1);
    Serial.print("Posmotor2: ");
    Serial.println(posmotor2);
    Serial.print("Posmotor3: ");
    Serial.println(posmotor3);
    Serial.print("Posmotor4: ");
    Serial.println(posmotor4);


    pwm.writeMicroseconds(0, posmotor2);//perna 1
    pwm.writeMicroseconds(1, posmotor1);


    pwm.writeMicroseconds(5, posmotor4);//perna 2
    pwm.writeMicroseconds(4, posmotor3);

    
    pwm.writeMicroseconds(11, posmotor2);//perna 3
    pwm.writeMicroseconds(10, posmotor1);


    pwm.writeMicroseconds(6, posmotor4);// perna 4
    pwm.writeMicroseconds(7, posmotor3);
    
    if( i < 30){//atras 3 e 4
      xfrente--;
      yfrente--;
      xtras--;
    }else if(i > 30 && i < 60){
      yfrente++;
      ytras--;
    }else if(i > 60 && i < 90){
      xfrente++;
      xtras++;
      ytras++;
    }else if(i > 90 && i < 120){//==============
      xfrente--;
      yfrente--;
      xtras--;
    }else if(i > 120 && i < 150){
      yfrente++;
      ytras--;
    }else if(i > 150 && i < 180){
      xfrente++;
      xtras++;
      ytras++;
    }  
  }
}

void anda2(){
  
  double x1 = 10;
  double y1 = 90;
  
  double x2 = 10;
  double y2 = 90;

  int controle = 0;
  
  for(int i = 0; i < 90; i++){

    delay(15);
    
    double theta2rad1 = acos((x1*x1+y1*y1-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad1 = atan(y1/x1)-atan(L2*sin(theta2rad1)/(L1+L2*cos(theta2rad1)));

    double theta2rad2 = acos((x2*x2+y2*y2-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad2 = atan(y2/x2)-atan(L2*sin(theta2rad2)/(L1+L2*cos(theta2rad2)));

    double theta2rad3 = acos((x1*x1+y1*y1-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad3 = atan(y1/x1)-atan(L2*sin(theta2rad3)/(L1+L2*cos(theta2rad3)));

    double theta2rad4 = acos((x2*x2+y2*y2-L1*L1-L2*L2)/(2*L1*L2));
    double theta1rad4 = atan(y2/x2)-atan(L2*sin(theta2rad4)/(L1+L2*cos(theta2rad4)));
    
    double angmotor11 = 90-(theta1rad1*180/3.14);
    double angmotor21 = 90-(theta2rad1*180/3.14);

    double angmotor12 = 90-(theta1rad2*180/3.14);
    double angmotor22 = 90-(theta2rad2*180/3.14);

    double angmotor13 = 90+(theta1rad3*180/3.14);
    double angmotor23 = 90+(theta2rad3*180/3.14);

    double angmotor14 = 90+(theta1rad4*180/3.14);
    double angmotor24 = 90+(theta2rad4*180/3.14);

    

    if(angmotor11 > 90){
      angmotor11 = angmotor11 - 180;
    }
    if(angmotor21 > 90){
      angmotor21 = angmotor21 - 180;
    }
    if(angmotor12 > 90){
      angmotor12 = angmotor12 - 180;
    }
    if(angmotor22 > 90){
      angmotor22 = angmotor22 - 180;
    }
    if(angmotor13 > 90){
      angmotor13 = angmotor13 - 180;
    }
    if(angmotor23 > 90){
      angmotor23 = angmotor23 - 180;
    }
    if(angmotor14 > 90){
      angmotor14 = angmotor14 - 180;
    }
    if(angmotor24 > 90){
      angmotor24 = angmotor24 - 180;
    }

    long double posmotor11 = (angmotor11 * 100/9) + 1500;
    long double posmotor21 = (angmotor21 * 100/9) + 1500;
    
    long double posmotor12 = (angmotor12 * 100/9) + 1500;
    long double posmotor22 = (angmotor22 * 100/9) + 1500;
    
    long double posmotor13 = (angmotor13 * 100/9) + 1500;
    long double posmotor23 = (angmotor23 * 100/9) + 1500;
    
    long double posmotor14 = (angmotor14 * 100/9) + 1500;
    long double posmotor24 = (angmotor24 * 100/9) + 1500;

    

      pwm.writeMicroseconds(0, posmotor21);
      pwm.writeMicroseconds(1, posmotor11);

      pwm.writeMicroseconds(11, posmotor22);
      pwm.writeMicroseconds(10, posmotor12);

      pwm.writeMicroseconds(5, posmotor23);
      pwm.writeMicroseconds(4, posmotor13);

      pwm.writeMicroseconds(6, posmotor24);
      pwm.writeMicroseconds(7, posmotor14);

    if( i < 30){
      x1--;
      x2--;
      y2--;
    }else if(i > 30 && i < 60){
      y1--;
      y2++;
    }else if(i > 60 && i < 90){
      x1++;
      y1++;
      x2++;
    }else if(i > 90 && i < 120){//==============
      x1--;
      x2--;
      y2--;
    }else if(i > 120 && i < 150){
      y1--;
      y2++;
    }else if(i > 150 && i < 180){
      x1++;
      y1++;
      x2++;
    }
  }
}

void anda3(){
  for(int i = 0; i < 10; i++){
    perna1();
    perna3();
    perna2();
    perna4();
  }
}

void passo13()
{
  double x1 = 10;
  double y1 = 90;

  double x2 = 10;
  double y2 = 90;

  for (int i = 0; i < 90; i++)
  {
    
    delay(15);
    double theta2rad1 = acos((x1 * x1 + y1 * y1 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    double theta1rad1 = atan(y1 / x1) - atan(L2 * sin(theta2rad1) / (L1 + L2 * cos(theta2rad1)));

    double theta2rad2 = acos((x2 * x2 + y2 * y2 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    double theta1rad2 = atan(y2 / x2) - atan(L2 * sin(theta2rad2) / (L1 + L2 * cos(theta2rad2)));

    double angmotor11 = 90 - (theta1rad1 * 180 / 3.14);
    double angmotor21 = 90 - (theta2rad1 * 180 / 3.14);

    double angmotor12 = 90 - (theta1rad2 * 180 / 3.14);
    double angmotor22 = 90 - (theta2rad2 * 180 / 3.14);

    if (angmotor11 > 90)
    {
      angmotor11 = angmotor11 - 180;
    }
    if (angmotor21 > 90)
    {
      angmotor21 = angmotor21 - 180;
    }
    if (angmotor12 > 90)
    {
      angmotor12 = angmotor12 - 180;
    }
    if (angmotor22 > 90)
    {
      angmotor22 = angmotor22 - 180;
    }

    long double posmotor11 = (angmotor11 * 100 / 9) + 1500;
    long double posmotor21 = (angmotor21 * 100 / 9) + 1500;

    long double posmotor12 = (angmotor12 * 100 / 9) + 1500;
    long double posmotor22 = (angmotor22 * 100 / 9) + 1500;

    pwm.writeMicroseconds(0, posmotor21);
    pwm.writeMicroseconds(1, posmotor11);

    pwm.writeMicroseconds(11, posmotor22);
    pwm.writeMicroseconds(10, posmotor12);

    if (i < 30)
    {
      x1--;
      x2--;
      y2--;
    }
    else if (i > 30 && i < 60)
    {
      y1--;
      y2++;
    }
    else if (i > 60 && i < 90)
    {
      x1++;
      y1++;
      x2++;
    }
    else if (i > 90 && i < 120)
    { //==============
      x1--;
      x2--;
      y2--;
    }
    else if (i > 120 && i < 150)
    {
      y1--;
      y2++;
    }
    else if (i > 150 && i < 180)
    {
      x1++;
      y1++;
      x2++;
    }
  }
}

void passo24()
{
  double x1 = 10;
  double y1 = 90;

  double x2 = 10;
  double y2 = 90;

  for (int i = 0; i < 90; i++)
  {
    delay(15);
    double theta2rad3 = acos((x1 * x1 + y1 * y1 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    double theta1rad3 = atan(y1 / x1) - atan(L2 * sin(theta2rad3) / (L1 + L2 * cos(theta2rad3)));

    double theta2rad4 = acos((x2 * x2 + y2 * y2 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    double theta1rad4 = atan(y2 / x2) - atan(L2 * sin(theta2rad4) / (L1 + L2 * cos(theta2rad4)));

    double angmotor13 = 90 + (theta1rad3 * 180 / 3.14);
    double angmotor23 = 90 + (theta2rad3 * 180 / 3.14);

    double angmotor14 = 90 + (theta1rad4 * 180 / 3.14);
    double angmotor24 = 90 + (theta2rad4 * 180 / 3.14);

    if (angmotor13 > 90)
    {
      angmotor13 = angmotor13 - 180;
    }
    if (angmotor23 > 90)
    {
      angmotor23 = angmotor23 - 180;
    }
    if (angmotor14 > 90)
    {
      angmotor14 = angmotor14 - 180;
    }
    if (angmotor24 > 90)
    {
      angmotor24 = angmotor24 - 180;
    }

    long double posmotor13 = (angmotor13 * 100 / 9) + 1500;
    long double posmotor23 = (angmotor23 * 100 / 9) + 1500;

    long double posmotor14 = (angmotor14 * 100 / 9) + 1500;
    long double posmotor24 = (angmotor24 * 100 / 9) + 1500;

    pwm.writeMicroseconds(5, posmotor23);
    pwm.writeMicroseconds(4, posmotor13);

    pwm.writeMicroseconds(6, posmotor24);
    pwm.writeMicroseconds(7, posmotor14);

    if (i < 30)
    {
      x1--;
      x2--;
      y2--;
    }
    else if (i > 30 && i < 60)
    {
      y1--;
      y2++;
    }
    else if (i > 60 && i < 90)
    {
      x1++;
      y1++;
      x2++;
    }
    else if (i > 90 && i < 120)
    { //==============
      x1--;
      x2--;
      y2--;
    }
    else if (i > 120 && i < 150)
    {
      y1--;
      y2++;
    }
    else if (i > 150 && i < 180)
    {
      x1++;
      y1++;
      x2++;
    }
  }
}

void fingemorto(){
  stand2();
  delay(300);
  tombaesq();
  delay(3000);
  
  pwm.writeMicroseconds(9, 2000);
  pwm.writeMicroseconds(3, 1000);

  delay(300);

  pwm.writeMicroseconds(10, 2500);
  pwm.writeMicroseconds(11, 2500);
  pwm.writeMicroseconds(4, 500);
  pwm.writeMicroseconds(5, 500);

  delay(300);
  
  pwm.writeMicroseconds(9, 1500);
  pwm.writeMicroseconds(3, 1500);
  
  stand2();
  
}
void tombaesq(){
  
  delay(500);
  zero();
  pwm.writeMicroseconds(9, 2000);
  pwm.writeMicroseconds(3, 1000);
  delay(500);
  zero();
}

void tombadir(){
  delay(500);
  zero();
  pwm.writeMicroseconds(8, 1000);
  pwm.writeMicroseconds(2, 2000);
  delay(500);
  zero();
}


void loop() {
  
  int controle;

  if (Serial.available() > 0) {

    controle = Serial.parseInt();
    Serial.println(controle);

    if(controle == 0){//posicao zero
      zero();
    }
    if(controle == 1) {//fica em pé
      zero();
      delay(500);
      stand2();
    }
    if(controle == 2) {// semta
      zero();
      delay(500);
      senta();
    }
    if(controle == 3) {// deita
      zero();
      delay(500);
      deita();
    }

    if(controle == 4){ // anda individual
      stand2();
      anda3();
    }
    if(controle == 5){ // anda parzinho
      stand2();
      for(int i = 0; i < 10; i++){
        passo13();
        delay(100);
        passo24();
        delay(100);
      }
    }
    if(controle == 6){ // anda frente  e tras
      for(int i = 0; i < 10; i++){
        anda2();
      }
    }
    if(controle == 7){// finge de morto
      fingemorto();
    }
    if(controle == 8){//tomba para a esquerda
      tombaesq();
    }
    if(controle == 9){//tomba para a direita
      tombadir();
    }
    
    if (controle == 20){ // retorna a posicao
      posicao();
    }
    if (controle == 99){ // guarda
        pwm.writeMicroseconds(0, 900);
        pwm.writeMicroseconds(1, 900);
        pwm.writeMicroseconds(2, 2300);
        pwm.writeMicroseconds(3, 1500);
        pwm.writeMicroseconds(4, 1500);
        pwm.writeMicroseconds(5, 2100);
        pwm.writeMicroseconds(6, 2100);
        pwm.writeMicroseconds(7, 800);
        pwm.writeMicroseconds(8, 800);
        pwm.writeMicroseconds(9, 2300);
        pwm.writeMicroseconds(10, 500);
        pwm.writeMicroseconds(11, 900);
    }
    
    if(controle == 500){
      
      delay(10);
      
      if(controle > m4){
        for( int j = m4; j <= controle; j+=2){
          pwm.writeMicroseconds(4, j);
        }
        
        m4 = controle;
        Serial.print("M4 esta em:");
        Serial.print(m4);
      }
      if(controle < m4){
        for( int k = m4; k <= controle; k-=2){
          pwm.writeMicroseconds(4, k);
        }
        
        m4 = controle;
        Serial.print("M4 esta em:");
        Serial.print(m4);
        
      }
    }
    if( controle == 1500){
      perna1();
    }
  }

}
