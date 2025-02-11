#include <EEPROM.h>
#include <LiquidCrystal.h>

// ==== PIN ASSIGNMENTS ==== //
int rs = 2;
int en = 3;
int d4 = 8;
int d5 = 9;
int d6 = 10;
int d7 = 11;

int input_pin = 14;
// ==== PIN ASSIGNMENTS ==== //

// ==== GLOBALS AND OBJECTS ==== //
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
long iterations = 1;
// ==== GLOBALS AND OBJECTS ==== //

void setup() {
  pinMode(input_pin, INPUT);
  
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Powering On...");
  lcd.clear();
  lcd.print("Testing...");
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  long input = 0;
  for (int i = 0; i < iterations; i++) {
    input += analogRead(input_pin);
  }

  input /= iterations;
  input = map(input, 0, 1024, 0, 100);
  lcd.clear();
  lcd.print("Input: " + String( min( round((input+5)/5.0)*5, 100)) + "%");
  Serial.println(input);
  delay(100);
}
