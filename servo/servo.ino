#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  90 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO 0

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  pwm.setPWM(SERVO, 0, SERVOMIN);
}

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

void loop() {
//  // Drive each servo one at a time using setPWM()
//  Serial.println(servonum);
//  pwm.setPWM(servonum, 0, 2048);

  for(int i = SERVOMIN; i <= SERVOMAX; i++) {
    pwm.setPWM(SERVO, 0, i);
    if (i == SERVOMIN)
      delay(0);
    if (i == SERVOMAX)
      delay(0);
    delay(5);
  }
  for(int j = SERVOMAX; j >= SERVOMIN; j--){
    
    pwm.setPWM(SERVO, 0, j);
    
    if (j == SERVOMAX)
      delay(0);
    if (j == SERVOMIN)
      delay(0);
    delay(5);    
    
   }
//
//
//  servonum++;
//  if (servonum > 5) servonum = 0; // Testing the first 8 servo channels
}
