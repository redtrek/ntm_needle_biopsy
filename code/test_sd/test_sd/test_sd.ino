#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_TinyUSB.h>

// ==== Pin Assignments ==== //
#define SPI0_SCK 18
#define SPI0_MOSI 19
#define SPI0_MISO 20
#define SPI0_CS 1

// ==== SD Card and MSC ==== //
SdFat SD;
FsFile testFile;

Adafruit_USBD_MSC msc;

// ==== Prototypes ==== //
long msc_read_req(long unsigned address, unsigned char* buffer, long unsigned length);
long msc_write_req(long unsigned address, unsigned char* buffer, long unsigned length);
void msc_flush();
void msc_enable();

void setup() {
  
  Serial.begin(115200);

  // Setup SPI
  SPI.setTX(SPI0_MOSI);
  SPI.setRX(SPI0_MISO);
  SPI.setSCK(SPI0_SCK);

  if (!SD.begin(SPI0_CS)) {
    Serial.println("SD card could not be properly initialized.");
    while(1) {delay(10);}
  }
  Serial.println("SD card recognized");

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

  msc_enable();
}

void loop() {
  delay(10);
}

// ==== Functiom Definitions ==== //
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
