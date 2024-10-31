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

// ==== EXPERIMENTAL VARIABLES ====//
const int fwRev = -60;
const int bwRev = 0;

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
float numRevolutions(float numCounts);
void countA();
long inputSpeed(long iterations);
void writeCurrentEEPROM(float detectedCurrent, bool isMax); // Writes if there was excess current to the EEPROM. This will be read at the start of operation to indicate irregularities.
void readCurrentEEPROM(bool isMax);
// ==== Function Prototypes ==== //


// ==== Pin Assignments ==== //
const int encoder_outputA = 2; // must be 2 or 3 for arduino uno
const int encoder_outputB = 3; // must be 2 or 3 for arduino uno
const int PWM = 6; // Controls PWM i.e. speed control of motor (analogWrite 0 to 255)
const int DIR = 7; // Direction control of motor? analog
const int button = 13;

const int rs = 4; // LCD reset
const int en = 5; // LCD enable
int d4 = 8; // LCD data
int d5 = 9; // LCD data
int d6 = 10; // LCD data
int d7 = 11; // LCD data

const int speed_pin = 14; // (Pin A0) Attached to potentiometer to adjust speed.
// ==== Pin Assignments ==== //


// ==== Constants and Global Values ==== //
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
volatile long count = 0;
int m = 0;
int buttonVal = 0;
int outputA = 0;
int outputB = 0;

float velocity = 0;
float revolution = 0;

const float excessCurrent = 1000; // Anything over 1A is considered an excess current.
float maxCurrent = 1000; // 1A is the defaulted value. This will be replaced on startup by whatever is written in EEPROM.
float maxOpCurrent = 0;

long speed_cutting = 0;
long speed_exiting = 0;
const long inputScale = 5.0;
const long potIterations = 1000;
// ==== Constants and Global Values ==== //


// ==== ACTUAL CODE ==== //
void setup() {
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.print("Powering On...");
  lcd.clear();
  lcd.print("Testing...");
  
  // -- Startup: Reading current data i.e. last excess and max current --//
  /*
  maxCurrent = readCurrentEEPROM(true);
  Serial.println("The maximum current recorded on this device was: " + String(maxCurrent));
  float lastExcess = readCurrentEEPROM(false);
  Serial.println("The last recorded excess current recorded on this device was: " + String(lastExcess));
  */
  // -- Startup: Reading current data i.e. last excess and max current --//
  
  // Check if current sensing chip is functioning correctly. Output message and infinitely loop in case of error.
  if (!ina219.begin()) {
    Serial.println("Could not INA219 chip.");
    while (1) { delay(10); }
  }
  
  attachInterrupt(0, countA, FALLING); // Interrupt for encoder_outputA: attachInterrupt(pin, ISR, trigger mode). Counts whenever encoder_outputA is falling.

  // - Pin Modes - //
  pinMode(button, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  // - Pin Modes - //
  
  // Handle motor state. Begin in standby mode i.e. MOTOR OFF
  deviceState = STANDBY;
  digitalWrite(DIR, LOW);
  digitalWrite(PWM, LOW); 
}

void loop() {
  float current = 0;
  float current_mA = 0;
  
  buttonVal = digitalRead(button);
  
  float revolution = numRevolutions(count);
  Serial.print(revolution);
  Serial.print(","); 

  current = ina219.getCurrent_mA();
  Serial.print(current);
  Serial.println();
/*
  Serial.print("Current: "); 
  Serial.print(current); 
  Serial.println(" mA");
*/
 
  // EEPROM CODE: EXPERIMENTAL
  /*
  if (current > excessCurrent) {
    if (current > maxCurrent) {
      writeCurrentEEPROM(current, true); // Save new max current
    }
    writeCurrentEEPROM(current, false); // Save the excess current
  }
  */

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
      
      if (buttonVal == HIGH){
        deviceState = CUTTING;
        delay(100);
      }

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
        analogWrite(PWM, speed_cutting); // Set full power.
        if (buttonVal == HIGH){
          deviceState = REMOVAL;
          delay(100);
        }
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
      
      if (buttonVal == HIGH){
        deviceState = EXITING;
        delay(100);
      }

      // Reset maximum current for future operation.
      maxCurrent = 0;
      
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
        if (buttonVal == HIGH){
          deviceState = STANDBY;
          delay(100);
        }
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
  if(digitalRead(encoder_outputB) == HIGH) // If outputA falls and outputB is high, then output A is leading.
  {
    // Changes position by 1 count on the -axis.
    count--;
  }
  
  if(digitalRead(encoder_outputB) == LOW) // If outputA falls and outputB is low, then output B is leading.
  {
    // Changes position by 1 count on the +axis.
    count++;
  }
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

  // Return the value as a decimal. Experimentally determined with inputScale = 5. This makes pot_read change by quantities of 5. The ADC isn't too stable so any more precise and there could be issues.
  //pot_read = round((pot_read + 5.0) / 5.0) * 5.0 / 100.0; // = 0.05-1.05 (5% speed to 105%)
  
  return pot_read;//min(pot_read, 1.0); // EDGE CASE: Previous calculation shouldn't practically ever reach 105% but just in case, limit the speed to 100%
}





/*
// This function writes any detected excess max currets
void writeCurrentEEPROM(float detectedCurrent, bool isMax)
{
  int address = 0;
  if (isMax) {address = 8;}
  
  byte* pointer = (byte*)(void*)&detectedCurrent; // This code is a pointer conversion technique. We cast &detectedCurrent from float* to generic void*. Then we cast this to a byte*.
  for (int i = 0; i < sizeof(detectedCurrent); i++) {
    EEPROM.write(address + i, *(pointer + i));
  }
}

// This function reads any saved max currents.
void readCurrentEEPROM(bool isMax)
{
  // Excess stored at byte 0. Max stored at byte 8.
  int address = 0;
  if (isMax) {address = 8;}
  
  float currentEEPROM;
  byte* pointer = (byte*)(void*)&currentEEPROM;
  for (int i = 0; i < sizeof(currentEEPROM); i++) {
    *(pointer + i) = EEPROM.read(address + i);
  }
  return currentEEPROM;
}
*/
