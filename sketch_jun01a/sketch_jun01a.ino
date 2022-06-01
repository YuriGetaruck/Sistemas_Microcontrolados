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


//double x = 30;
//double y = 90;




void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

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

  double y = 90;

  double x = 30;


  double theta2rad = acos((x*x+y*y-L1*L1-L2*L2)/(2*L1*L2));
  double theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));

  double angmotor1 = 90-(theta1rad*180/3.1415);
  double angmotor2 = 90-(theta2rad*180/3.1415);

  double posmotor1 = (angmotor1 * 100/9) + 1500;
  double posmotor2 = (angmotor2 + 100/9) + 1500;

  pwm.writeMicroseconds(0, posmotor1);
  pwm.writeMicroseconds(1, posmotor2);
    
}


void loop() {



  
  int controle;

  if (Serial.available() > 0) {

    controle = Serial.parseInt();
    Serial.println(controle);

    if(controle == 0){
      zero();
    }
    if(controle == 1) {
      zero();
      delay(500);
      stand();
    }
    if(controle == 3) {
      zero();
      delay(500);
      deita();
    }
    if(controle == 2) {
      zero();
      delay(500);
      senta();
    }
    if(controle == 4){
      perna1();
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
    if(controle == 5){
        passoesq();
    }
    if(controle == 6){
        passodir();
      
    }
    if (controle == 10){
      posicao();
    }
    
    if(controle == 499){
      
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
      pwm.writeMicroseconds(4, 1500);
    }
  }

}
