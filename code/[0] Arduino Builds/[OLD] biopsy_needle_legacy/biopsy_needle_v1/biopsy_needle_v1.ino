// Aspiration-Assisted Biopsy Needle Device - Motor Code
// Desc:
// Author(s): 

#include <MsTimer2.h>
#include <Wire.h> // For I2C Communication Protocl
#include <stdio.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;


// ==== Handling Motor Operation ==== //
enum states {
  STANDBY,
  CUTTING,
  REMOVAL,
  EXITING
};
enum states deviceState;
// ==== Handling Motor Operation ==== //


// ==== Function Prototypes ==== //
float pulseToVelocity(float n);
float numRevolutions(float m);
void flash();
void CountA();
// ==== Function Prototypes ==== //


// ==== Pin Assignments ==== //
const int button = 13;
const int DIR = 7; // Direction control of motor? analog
const int PW = 6; // Controls PWM i.e. speed control of motor - digit  al
const int encoder_outputA = 2; // must be 2 or 3 for arduino uno
const int encoder_outputB = 3; // must be 2 or 3 for arduino uno
// ==== Pin Assignments ==== //


// ==== Constants and Global Values ==== //
const int high = 255;
const int pwm = 0;
volatile long ppsA = 0; // Pulses per second
volatile long count = 0;
int m = 0;
int buttonVal = 0;
int outputA = 0;
int outputB = 0;

float velocity = 0;
float revolution = 0;
char a ='+';
char b ='-';
char c;
// ==== Constants and Global Values ==== //


void setup() {
  Serial.begin(115200);

  // Check if current sensing chip is functioning correctly. Output message and infinitely loop in case of error.
  if (!ina219.begin()) {
    Serial.println("Could not INA219 chip.");
    while (1) { delay(10); }
  }
  
  attachInterrupt(0, CountA, FALLING); // Interrupt for encoder_outputA
  MsTimer2::set(100, flash); // Every 100 milliseconds, calculate motor velocity.
  MsTimer2::start();

  // Set pin modes
  pinMode(button, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PW, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  
  // Handle motor state. Begin in standby mode i.e. MOTOR OFF
  deviceState = STANDBY;
  digitalWrite(DIR, LOW); // this will brake, regardless of what ph is 
  digitalWrite(PW, LOW); 
}

void loop() {
  float current = 0;
  float current_mA = 0;
  
/*
  int volt = analogRead(A2);
  // 10 for no syringe, 11 for syringe on, 12 for needle on
  int stall = 10;
  float current= (volt+0.5) * (5.0 / 1023.0)*1000/40;
*/
  
  buttonVal = digitalRead(button);
  float revolution = numRevolutions(count);
  Serial.print(revolution);
  Serial.print(","); 
/*
  Serial.println(" rounds"); 
  Serial.print(" ");
  Serial.print(volt);
  Serial.println(" ");
*/
  current = ina219.getCurrent_mA();
  Serial.print(current);
  Serial.println();
/*
  Serial.print("Current: "); 
  Serial.print(current); 
  Serial.println(" mA");
*/
  if (revolution > 0 || revolution < -60)
  {
    digitalWrite(DIR, LOW);
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

  // Debug device state
  // Serial.print(deviceState);
  // Serial.print(" ");

  delay(100);
}

// Converts pulse counts into velocity measures (RPM)
float pulseToVelocity(float n)
{
  float vel = n/0.1/34.014/12*60;
  // ppr = 48/4 = 12
  return vel;
}

// Converts encoder counts to revolutions
float numRevolutions(float m)
{
  float re = m/12/34.014;
  return re;          
}

void flash() 
{
  float velocity = pulseToVelocity(ppsA);
  //Serial.print(velocity); 
  //Serial.println("RPM  "); 
  ppsA=0;
}

// Interrupt Service Routine
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
