// Aspiration-Assisted Biopsy Needle Device - Motor Code
// Desc: This code handles the operation of a 12 V 10 A motor as well as a current reading device.
// Author(s): Thomas Chang, Dane
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
float numRevolutions(float numCounts);
void flash();
void CountA();
// ==== Function Prototypes ==== //


// ==== Pin Assignments ==== //
const int button = 13;
const int DIR = 7; // Direction control of motor? analog
const int PWM = 6; // Controls PWM i.e. speed control of motor - digital 8-bit value relationship: 0 (min speed) to 255 (max speed)
const int encoder_outputA = 2; // must be 2 or 3 for arduino uno
const int encoder_outputB = 3; // must be 2 or 3 for arduino uno
// ==== Pin Assignments ==== //


// ==== Constants and Global Values ==== //
const int high = 255;
volatile long count = 0;
int m = 0;
int buttonVal = 0;
int outputA = 0;
int outputB = 0;

float velocity = 0;
float revolution = 0;
// ==== Constants and Global Values ==== //


// ==== ACTUAL CODE ==== //
void setup() {
  Serial.begin(115200);

  // Check if current sensing chip is functioning correctly. Output message and infinitely loop in case of error.
  if (!ina219.begin()) {
    Serial.println("Could not INA219 chip.");
    while (1) { delay(10); }
  }
  
  attachInterrupt(0, CountA, FALLING); // Interrupt for encoder_outputA: attachInterrupt(pin, ISR, trigger mode). Counts whenever encoder_outputA is falling.

  // Set pin modes
  pinMode(button, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  
  // Handle motor state. Begin in standby mode i.e. MOTOR OFF
  deviceState = STANDBY;
  digitalWrite(DIR, LOW); // this will brake, regardless of what ph is <== What???h
  digitalWrite(PWM, LOW); 
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

/*
  // Conditionally handles motor stop functionality. When cutting, turn off the motor once reaching a specified distance. When removing, turn off once back in the original position.
  if (revolution > 0 || revolution < -60)
  {
    digitalWrite(DIR, LOW);
    analogWrite(PWM, LOW);
  }
*/







  
  if (deviceState == STANDBY){
    digitalWrite(DIR, HIGH); // Set motor direction forwards.
    digitalWrite(PWM, LOW); // Set motor off.
       
    if (buttonVal == HIGH){
      deviceState = CUTTING;
      delay(100);
    }
  }
  else if (deviceState == CUTTING){
    // Safety Check: If the motor has moved too far forward while cutting, place it in the removal state.
    if (revolution < -20)
    {
      digitalWrite(DIR, LOW); // Redundancy: Set motor direction backwards.
      digitalWrite(PWM, LOW); // Redundancy: Set motor off.
      deviceState = REMOVAL;
    } else {
      digitalWrite(DIR, HIGH); // Set motor forwards.
      analogWrite(PWM, 255);   // Set full power.
      
      if (buttonVal == HIGH){
        deviceState = REMOVAL;
        delay(100);
      }
    }
  }
  else if (deviceState == REMOVAL){
    digitalWrite(DIR, LOW); // Set motor direction backward.
    digitalWrite(PWM, LOW); // Set motor off.
    
    if (buttonVal == HIGH){
      deviceState = EXITING;
      delay(100);
    }
  }
  else if (deviceState == EXITING){
    // Safety Check: If the motor has moved to far backward while exiting, place it in the standby state.
    if (revolution > 0)
    {
      digitalWrite(DIR, LOW); // Redundancy: Set motor direction forward.
      digitalWrite(PWM, LOW); // Redundancy: Set motor off.
      deviceState = STANDBY;
    } else {
      digitalWrite(DIR, LOW); // Set motor direction backward.
      analogWrite(PWM, 255);  // Set full power.
      
      if (buttonVal == HIGH){
        deviceState = STANDBY;
        delay(100);
      }
    }
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
void CountA() 
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
