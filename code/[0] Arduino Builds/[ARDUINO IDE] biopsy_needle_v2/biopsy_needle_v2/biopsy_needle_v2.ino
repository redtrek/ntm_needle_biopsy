// Non-traditional Manufacturing Laboratory (CEN3907C)
// Aspiration-Assisted Biopsy Needle Device
// Author(s): Thomas Chang, Dane Ungurait
// Desc: This code handles the operation of the motor, sensors, inputs, and USB capabilities of the UF device.


#include <Wire.h>
#include <stdio.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_TinyUSB.h>


// ==== Experimental Variables ====//
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


// ==== Function Prototypes ==== //
float numRevolutions(float numCounts);
void countA();
long inputSpeed(long iterations);
float getRPM();
void buttonHandler();

long msc_read_req(long unsigned address, unsigned char* buffer, long unsigned length);
long msc_write_req(long unsigned address, unsigned char* buffer, long unsigned length);
void msc_flush();
void msc_enable();


// ==== Flags ==== //
bool countA_flag = false;


// ==== Pin Assignments and Addressing ==== //
#define SPI0_SCK 18         // SCK
#define SPI0_MOSI 19        // MOSI
#define SPI0_MISO 20        // MISO
#define SPI0_CS 1           // RX

#define encoder_outputA 24  // 24
#define encoder_outputB 25  // 25
#define PWM 0               // TX
#define DIR 6               // D4

#define button 11           // 11
//#define lcd_rst
//#define lcd_en
//#define lcd_4
//#define lcd_5
//#define lcd_6
//#define lcd_7

#define I2C0_SDA 12          // 12
#define I2C0_SCL 13          // 13
#define OLED_ADDR   0x3C
#define INA219_ADDR 0x40

#define speed_pin 26        // A0

// ==== Constants and Global Values ==== //
Adafruit_INA219 ina219;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
SdFat SD;
FsFile testFile;
Adafruit_USBD_MSC msc;

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

  // I2C Initialization
  Wire.begin();
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  // OLED Initialization
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.println("Hello!");
  display.display();

  /*
  // Current Sensor Initialization
  if (!ina219.begin()) {
    Serial.println("ERROR: Could not find the INA219 chip.");
    while (1) { delay(10); }
  }
  */
  
  // SD Card Initialization
  SPI.setTX(SPI0_MOSI);
  SPI.setRX(SPI0_MISO);
  SPI.setSCK(SPI0_SCK);

  if (!SD.begin(SPI0_CS)) {
    Serial.println("SD card could not be properly initialized.");
    while(1) {delay(10);}
  }
  Serial.println("SD card recognized");

  /*
  // Interrupt for encoder_outputA: attachInterrupt(pin, ISR, trigger mode). Counts whenever encoder_outputA is falling.
  attachInterrupt(encoderA_pin, countA, FALLING); 
  
  // Pin Modes //
  pinMode(button, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(encoder_outputA, INPUT);
  pinMode(encoder_outputB, INPUT);
  
  // Handle motor state. Begin in standby mode i.e. MOTOR OFF
  deviceState = STANDBY;
  digitalWrite(DIR, LOW);
  digitalWrite(PWM, LOW); 
  */

  // Test file demonstrating memory.
  testFile = SD.open("test.txt", FILE_WRITE);

  if (testFile) {
    testFile.println("First SD card test. Hi Darren!");
    testFile.close();
  } else {
    Serial.println("Could not open test.txt");
  }

  testFile = SD.open("test.txt");
  if (testFile) {
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
  } else {
    Serial.println("Could not read from test.txt");
  }

  //msc_enable();
}

void loop() {
/*
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
*/
  delay(100);
}

// ==== Function Definitions ==== //

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

long msc_read_req(long unsigned address, void* buffer, long unsigned length) {
  if(SD.card()->readSectors(address, (unsigned char*)buffer, length/512)) {
    return length;
  } else {
    return -1;
  }
}

long msc_write_req(long unsigned address, unsigned char* buffer, long unsigned length) {
  if(SD.card()->writeSectors(address, buffer, length/512)) {
    return length;
  } else {
    return -1;
  }
}

void msc_flush() {
  SD.card()->syncDevice();
  //SD.cacheClear();
}

// Enables MSC capability of RP2040 and SD card.
void msc_enable() {
   //msc.setID();
  msc.setReadWriteCallback(msc_read_req, msc_write_req, msc_flush);

  msc.setUnitReady(false); // Unit ready is just a boolean that says whether reading and writing is possible at the moment. msc.UnitReady() will return this boolean but it has no bearing on the actual code.
  msc.begin(); // Initialize the MSC - Allows host PC to recognize RP2040 device as storage.

  // Re-enumeration: Essentially taking the USB out and putting it in again. Ensures the PC recognizes as storage rather than just an interface to flash programs and act as serial port.
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
  
  uint32_t block_count = SD.card()->sectorCount();
  msc.setCapacity(block_count, 512);
  msc.setUnitReady(true);
}
