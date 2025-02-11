// Aspiration-Assisted Biopsy Needle Device - Motor Code
// Desc: This code handles the operation of a 12 V 10 A motor as well as a current reading device. It features and LCD interface and potentiometer inputs.
// Author(s): Thomas Chang, Dane

// TODO: 1mm increment for distance. RPM function. Merge states. See about keeping track of position as the device is powered off.
#include <Wire.h> // For I2C Communication Protocl
#include <stdio.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

Adafruit_INA219 ina219;

// ==== Experimental Variables ====//
const int fwRev = -1000;
const int bwRev = 1000;


// ==== Handling Motor Operation ==== //
enum states {
  STANDBY,
  CUTTING,
  REMOVAL,
  EXITING
};
enum states deviceState;


// ==== Function Prototypes ==== //
float numRevolutions(float numCounts);
void countA();
long inputSpeed(long iterations);
float getRPM();
void buttonHandler();


// ==== Flags ==== //
bool countA_flag = false;


// ==== Pin Assignments ==== //
const int encoder_outputA = 2;
const int encoder_outputB = 3;
const int PWM = 6; // Controls PWM i.e. speed control of motor (analogWrite 0 to 255)
const int DIR = 7; // Direction control for motor
const int button = 13;

const int rs = 4; // LCD reset
const int en = 5; // LCD enable
int d4 = 8; // LCD data
int d5 = 9; // LCD data
int d6 = 10; // LCD data
int d7 = 11; // LCD data

const int speed_pin = 14; // (Pin A0) Attached to potentiometer to adjust speed.


// ==== Constants and Global Values ==== //
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
volatile long count = 0;
int m = 0;
int outputA = 0;
int outputB = 0;

float velocity = 0;
float revolution = 0;
float maxOpCurrent = 0;

long speed_cutting = 0;
long speed_exiting = 0;
const long inputScale = 5.0;
const long potIterations = 1000;

float prevTime = 0;
int numPulses;

void setup() {
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.print("Powering On...");
  lcd.clear();
  lcd.print("Testing...");
  
  // Check if current sensing chip is functioning correctly.
  if (!ina219.begin()) {
    Serial.println("ERROR: Could not find the INA219 chip.");
    while (1) { delay(10); }
  }

  // Interrupt for encoder_outputA: attachInterrupt(pin, ISR, trigger mode). Counts whenever encoder_outputA is falling.
  attachInterrupt(0, countA, FALLING); 
  
  // - Pin Modes - //
  pinMode(button, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  
  // Handle motor state. Begin in standby mode i.e. MOTOR OFF
  deviceState = STANDBY;
  digitalWrite(DIR, LOW);
  digitalWrite(PWM, LOW); 
}

void loop() {
  // ==== CHECK FOR INTERRUPTS AND TIMERS ====//
  
  // ---- countA signal ISR ---- //
  if (countA_flag == true) {
    noInterrupts();
    
    // outputB is high/low : output A is leading/following
    if(digitalRead(encoder_outputB) == HIGH) {
      count--;
    } else if(digitalRead(encoder_outputB) == LOW) {
      count++;
    }
     
    countA_flag = false;
    interrupts();
  }
  // ---- countA signal ISR ---- //
  
  // ==== CHECK FOR INTERRUPTS AND TIMERS ====//
  
  float current = 0;
  float current_mA = 0;
  
  float revolution = numRevolutions(count);
  //Serial.print(revolution);
  //Serial.print(","); 

  current = ina219.getCurrent_mA();
  //Serial.print(current);
  //Serial.println();
 
  Serial.println(String(getRPM()) + "rpm");

  // Read button inputs to change states
  buttonHandler();
  
  // State Machine
  switch(deviceState)
  {
    case STANDBY:
      // Set motor off: direction = forwards, speed = 0
      digitalWrite(DIR, HIGH);
      digitalWrite(PWM, LOW);

      // Handle speed input information
      speed_cutting = inputSpeed(potIterations) / 100.0 * 255;
      lcd.clear();
      lcd.print("Cut Speed: " + String(speed_cutting));

      // Reset maximum current reading for this run;
      maxOpCurrent = 0;
      
      break;
    
    case CUTTING:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CUTTING...");
      lcd.setCursor(0,1);
      lcd.print("Max(mA): " + String(maxOpCurrent));

      if (abs(current) > abs(maxOpCurrent)) {
        maxOpCurrent = current;
      }
      
      // Safety Check: If the motor has moved too far forward while cutting, place it in the removal state.
      if (revolution < fwRev)
      {
        digitalWrite(DIR, LOW); // Redundancy: Set motor direction backwards.
        digitalWrite(PWM, LOW); // Redundancy: Set motor off.
        deviceState = REMOVAL;
      } else {
        digitalWrite(DIR, HIGH); // Set motor forwards.
        analogWrite(PWM, speed_cutting);
      }
      break;
      
    case REMOVAL:
      // Set motor off: direction = backwards, speed = 0
      digitalWrite(DIR, LOW);
      digitalWrite(PWM, LOW);

      // Speed input information
      speed_exiting = inputSpeed(potIterations) / 100.0 * 255;
      lcd.clear();
      lcd.print("Exit Speed: " + String(speed_exiting));
      
      // Reset maximum current for future operation.
      maxOpCurrent = 0;
      
      break;
    
    case EXITING:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("EXITING...");  
      lcd.setCursor(0,1);
      lcd.print("Max(mA): " + String(maxOpCurrent));

      if (abs(current) > abs(maxOpCurrent)) {
        maxOpCurrent = current;
      }
      
      // Safety Check: If the motor has moved to far backward while exiting, place it in the standby state.
      if (revolution > bwRev)
      {
        digitalWrite(DIR, LOW); // Redundancy: Set motor direction forward.
        digitalWrite(PWM, LOW); // Redundancy: Set motor off.
        deviceState = STANDBY;
      } else {
        digitalWrite(DIR, LOW); // Set motor direction backward.
        analogWrite(PWM, speed_exiting);  // Set to user defined speed.
      }
      break;
  }

  // Debug device state
  // Serial.print(deviceState);
  // Serial.print(" ");

  delay(100);
}

// Converts encoder counts to revolutions
float numRevolutions(float numCounts)
{
  // Revolutions = numCounts * (1 revolution / 12 counts) * (1 large gear revolution / 34.014 small gear revolutions)
  // 1 revolution / 12 counts comes from quadrature encoder documentation for motor i.e. 48 counts for 4 channels for 1 revolution. This code uses one channel to count so this is 12 counts for 1 revolution.
  float re = numCounts/12/34.014;
  return re;          
}

// Interrupt Service Routine: Jumps here every time encoder_outputA falls.
void countA() 
{
  countA_flag = true;
  numPulses++;
}

// Reads analog pin attached to 5V supplied 10k potentiometer. Users input a percentage of the full speed of the motor here.
long inputSpeed(long iterations)
{
  long pot_read = 0;
  
  // Here we accumulate several readings of the potentiometer and average them for a consistent value.
  for (int i = 0; i < iterations; i++) {
    pot_read += analogRead(speed_pin);
  }
  pot_read /= iterations;

  // Next we map the potential potentiometer values to a scale of (0 to 100) from the Arduino's built in ADC scale (0 to 1024).
  pot_read = map(pot_read, 0, 1024, 0, 100);

  return pot_read;
}

// Returns RPM using global values gathered from the interrupt. Uses number of pulses divided against a 1 second time frmae.
float getRPM() {
  unsigned long currTime = micros();
  if (currTime - prevTime >= 1000000) {
    float rpm = (numPulses / 12.0 / 34.014) * 60;
    numPulses = 0;
    prevTime = currTime;
    return rpm;
  }
  return 0;
}

void buttonHandler() {
  int buttonVal = digitalRead(button);
  if (buttonVal == HIGH) {
    delay(100);
    if (deviceState == STANDBY) {
      deviceState = CUTTING;
    } else if (deviceState == CUTTING) {
      deviceState = REMOVAL;
    } else if (deviceState == REMOVAL) {
      deviceState = EXITING;
    } else {
      deviceState = STANDBY;
    }
  }
}
