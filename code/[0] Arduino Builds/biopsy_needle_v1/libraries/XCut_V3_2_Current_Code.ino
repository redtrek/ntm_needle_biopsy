
#include <MsTimer2.h>
#include <Wire.h>
#include <stdio.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
enum states {
  STANDBY,
  CUTTING,
  REMOVAL,
  EXITING
};

enum states deviceState;

const int button = 13;
const int DIR = 7; // analog
const int PW = 6; //digit  al
const int pwm = 0;
const int encoder_outputA = 2; //must be 2 or 3 for arduino uno
const int encoder_outputB = 3; //must be 2 or 3 for arduino uno
const int high = 255;

volatile long ppsA = 0;
volatile long count = 0;
int m = 0;
int buttonVal = 0;
int outputA = 0;
int outputB = 0;

char a ='+';
char b ='-';
char c;
float velocity = 0;
float revolution = 0;
        float v(float n)
        {
          float vel = n/0.1/34.014/12*60;
          // ppr = 48/4 = 12
          return vel;
        }
        float revo(float m)
        {
          float re = m/12/34.014;
            return re;          
        }
void flash() 
{
float velocity = v(ppsA);
//Serial.print(velocity); 
//Serial.println("RPM  "); 
ppsA=0;
}

void setup() {
  Serial.begin(115200);

  if (!ina219.begin()) {
    Serial.println("can not find INA219 chip");
    while (1) { delay(10); }
  }
  
  uint32_t current;
  attachInterrupt(0,CountA, FALLING);
  MsTimer2::set(100, flash);
  MsTimer2::start();

  pinMode(button, INPUT);

  pinMode(DIR, OUTPUT);
  pinMode(PW, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  
  
  deviceState = STANDBY;

  digitalWrite(DIR, LOW); // this will brake, regardless of what ph is 
  digitalWrite(PW, LOW); 


}

void loop() {
  // put your main code here, to run repeatedly:
  float current = 0;
  float current_mA = 0;

 // int volt = analogRead(A2);
  // 10 for no syringe, 11 for syringe on, 12 for needle on
 // int stall = 10;
// float current= (volt+0.5) * (5.0 / 1023.0)*1000/40;
  buttonVal = digitalRead(button);
  float revolution = revo(count);
  Serial.print(revolution);
    Serial.print(","); 
//  Serial.println(" rounds"); 
//  Serial.print(" ");
 // Serial.print(volt);
 // Serial.println(" ");
  current = ina219.getCurrent_mA();
      Serial.print(current);
    Serial.println();
// Serial.print("Current:       "); 
// Serial.print(current); 
// Serial.println(" mA");

  if (revolution > 0 || revolution <-60)
  {     digitalWrite(DIR, LOW);
      analogWrite(PW, LOW);}

  if(deviceState == STANDBY){    
    if(buttonVal == HIGH){
      deviceState = CUTTING;
      digitalWrite(DIR, HIGH);
       analogWrite(PW, 255);
      delay(100);
    }
  }
            
  else if(deviceState == CUTTING){
    if(buttonVal == HIGH){
      digitalWrite(DIR, LOW);
      digitalWrite(PW, LOW);
      deviceState = REMOVAL;
      delay(100);
    }
  }

  else if(deviceState == REMOVAL){
    if(buttonVal == HIGH){
      deviceState = EXITING;
      digitalWrite(DIR, LOW);
      analogWrite(PW, 255);
      delay(100);
    }
  }
            
  else if(deviceState == EXITING){
    if(buttonVal == HIGH){
      digitalWrite(DIR, LOW);
      digitalWrite(PW, LOW);
      deviceState = STANDBY;
      delay(100);
    }
  }

  //Serial.print(deviceState);
  //Serial.print(" ");

  delay(100);
}

void CountA() 
{
  if(digitalRead(encoder_outputB) == HIGH)
  {
    c=a;
    count--;
  }
  
  if(digitalRead(encoder_outputB) == LOW)
  {
    c=b;
    count++;
  }
  ppsA++;
}
