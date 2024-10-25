// Aspiration-Assisted Biopsy Needle Device - Motor Code
// Desc: This code handles the operation of a 12 V 10 A motor as well as a current reading device. It features and LCD interface and potentiometer inputs.
// Author(s): Thomas Chang, Dane
#include <Wire.h> // For I2C Communication Protocl
#include <stdio.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;


// ==== Handling Motor Operation ==== //
enum states {
  STANDBY,
  SPEED_INPUT,
  CUTTING,
  REMOVAL,
  SPEED_INPUT_REVERSE,
  EXITING
};
enum states deviceState;
// ==== Handling Motor Operation ==== //


// ==== Function Prototypes ==== //
float numRevolutions(float numCounts);
void countA();
long inputSpeed();
void writeCurrentEEPROM(float detectedCurrent, bool isMax); // Writes if there was excess current to the EEPROM. This will be read at the start of operation to indicate irregularities.
void readCurrentEEPROM(bool isMax);
// ==== Function Prototypes ==== //


// ==== Pin Assignments ==== //
const int button = 13;
const int DIR = 7; // Direction control of motor? analog
const int PWM = 6; // Controls PWM i.e. speed control of motor - digital 8-bit value relationship: 0 (min speed) to 255 (max speed)
const int encoder_outputA = 2; // must be 2 or 3 for arduino uno
const int encoder_outputB = 3; // must be 2 or 3 for arduino uno
const int speed_pin = 14; // (Pin A0) Attached to potentiometer to adjust speed.
// ==== Pin Assignments ==== //


// ==== Constants and Global Values ==== //
volatile long count = 0;
int m = 0;
int buttonVal = 0;
int outputA = 0;
int outputB = 0;

float velocity = 0;
float revolution = 0;

const float excessCurrent = 1000; // Anything over 1A is considered an excess current.
float maxCurrent = 1000; // 1A is the defaulted value. This will be replaced on startup by whatever is written in EEPROM.

int speed_cutting = 0;
int speed_exiting = 0;
const long inputScale = 5.0;
const potIterations = 1000;
// ==== Constants and Global Values ==== //


// ==== ACTUAL CODE ==== //
void setup() {
  Serial.begin(115200);
  
  // -- Startup: Reading current data i.e. last excess and max current --//
  maxCurrent = readCurrentEEPROM(true);
  Serial.println("The maximum current recorded on this device was: " + String(maxCurrent));
  float lastExcess = readCurrentEEPROM(false);
  Serial.println("The last recorded excess current recorded on this device was: " + String(lastExcess));
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
  
  if (current > excessCurrent) {
    if (current > maxCurrent) {
      writeCurrentEEPROM(current, true); // Save new max current
    }
    writeCurrentEEPROM(current, false); // Save the excess current
  }
/*
  Serial.print("Current: "); 
  Serial.print(current); 
  Serial.println(" mA");
*/

  switch(deviceState)
  {
    case STANDBY:
      digitalWrite(DIR, HIGH); // Set motor direction forwards.
      digitalWrite(PWM, LOW); // Set motor off.
      if (buttonVal == HIGH){
        deviceState = SPEED_INPUT;
        delay(100);
      }
      break;
    
    case SPEED_INPUT:
      speed_cutting = inputSpeed(potIterations) * 255;
      Serial.print("Cutting speed: " + String(speed_cutting));
      if (buttonVal == HIGH){
        deviceState = CUTTING;
        delay(100);
      }
      break;
    
    case CUTTING:
      // Safety Check: If the motor has moved too far forward while cutting, place it in the removal state.
      if (revolution < -20)
      {
        digitalWrite(DIR, LOW); // Redundancy: Set motor direction backwards.
        digitalWrite(PWM, LOW); // Redundancy: Set motor off.
        deviceState = REMOVAL;
      } else {
        digitalWrite(DIR, HIGH); // Set motor forwards.
        analogWrite(PWM, speed_cutting);   // Set full power.
        if (buttonVal == HIGH){
          deviceState = REMOVAL;
          delay(100);
        }
      }
      break;
      
    case REMOVAL:
      digitalWrite(DIR, LOW); // Set motor direction backward.
      digitalWrite(PWM, LOW); // Set motor off.
      if (buttonVal == HIGH){
        deviceState = SPEED_INPUT_REVERSE;
        delay(100);
      }
      break;

    case SPEED_INPUT_REVERSE:
      speed_exiting = inputSpeed(potIterations) * 255;
      Serial.print("Exiting speed: " + String(speed_exiting));
      if (buttonVal == HIGH){
        deviceState = CUTTING;
        delay(100);
      }
      break;
    
    case EXITING:
      // Safety Check: If the motor has moved to far backward while exiting, place it in the standby state.
      if (revolution > 0)
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
long inputSpeed(int iterations)
{
  long pot_read = 0;
  
  // Here we accumulate several readings of the potentiometer and average them for a consistent value.
  for (int i = 0; i < iterations; i++) {
    pot_read += analogRead(speed_pin);
  }
  pot_read /= iterations;

  // Next we map the potential potentiometer values to a scale of (0 to 100) from the Arduino's built in ADC scale (0 to 1024).
  pot_read = map(pot_read, 0, 1024, 0, 100);

  // Return the value as a decimal. Experimentally determined with inputScale = 5. What this does is make the input change by quantities of 5%. The ADC isn't too stable so any more precise and there could be issues.
  pot_read = round((pot_read + inputScale) / inputScale) * inputScale / 100; // = 0.05-1.05 (5% speed to 105%)
  return min(pot_read, 1.0); // EDGE CASE: Previous calculation shouldn't practically ever reach 105% but just in case, limit the speed to 100%
}

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
