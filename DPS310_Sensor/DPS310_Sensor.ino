/*
INFO
  Board used: Adafruit Feather M0 Basic Proto
  Datasheet: REF
  Sensor used: DPS310 Digital Barometric Pressure Sensor
  Website: REF

  Code influenced by:
  https://docs.arduino.cc/tutorials/communication/BarometricPressureSensor
  [DETAILS]

Circuit:


SPI Details:
  Configuration: (Mode 3)
  Speed: (10 MHz)
  Control byte: (bit 7 of address, w = 0, r = 1)

Coding Checklist
  Write readRegister function
  Code data preprocessing section (l. 89)
Debug Checklist
    Check SPI.h is included and has all necessary files
    Check whether SerialUSB or Serial should be used
    Check if correct SPI mode is set in CFG_REG after changing (SPI commands work)
    Check interrupt is successfully received when FIFO is full
    Check if variable data structure is suitable for storing and manipulating measurements
    Does pin_ss have to be declared high in setup?
*/



// Definitions

// Includes
#include <SPI.h>

//Registers
#define PRS_CFG 0x06 // Pressure Config Register (Write mode) 
#define TMP_CFG 0x07 // Temperature Config Register (Write mode)
#define MEAS_CFG 0x08 // Measurement Config Register (Write mode)
#define CFG_REG 0X09 // Interrupt and FIFO Config Register (Write mode)

const byte READ = 0b10000000; // bit 7 of address used as read/write command. RW = 1 = read.

// Pins
const int pin_ss = 5; // Pin D5 used as slave select
const int pin_interrupt = 6; // Pin D6 used to receive interrupt


void setup() {
  // put your setup code here, to run once:

  delay(50); // Allows time for sensor to initialise

  // Set up pins
  //   pinMode(22, INPUT); // Define MISO as input pin ** NEEDED? **
  //   pinMode(23, OUTPUT); // Define MOSI as output pin
  //   pinMode(24, OUTPUT); // Define SCK as output pin
  pinMode(pin_ss, OUTPUT); // Define D5 pin (SS) as output (active low)
  pinMode(pin_interrupt, INPUT); // Define D6 pin (SDO) as input. Used as interrupt pin

  Serial.begin(9600); // Open serial connection

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // ** CHECK MSB FIRST **
  Serial.print("Starting setup...\n");

  // Set up registers
  writeRegister(CFG_REG, 0xC7); // Sets SPI mode to 3-wire with interrupt, and enables FIFO --> NB. May throw error without WRITE specified.
  writeRegister(MEAS_CFG, 0x07); // Sets operating mode to background
  writeRegister(PRS_CFG, 0x26); // Sets desired pressure reading parameters ** SPECIFY IN HEADER **
  writeRegister(TMP_CFG, 0xA0); // Sets desired temperature reading parameters ** SPECIFY IN HEADER **

  // INSERT delay IF NEEDED

  // Debugging register checks
  // Read FIFO_STS (0X0B), [bit 1] = 1: FIFO is full, [bit 0] = 1: FIFO is empty
  // Check if empty --> flush or store data if not
  // Read MEAS_CFG (0x08), [6] = 1: Sensor is ready for use
  Serial.print("Setup complete.\n");

}

void loop() {

  if (digitalRead(pin_interrupt) == HIGH) {
    // Copy all data to memory and then process in bulk
    int i = 0;
    // INSERT loop here to iterate through pressure measurements (i < 32)
    // Filter based on ratio of pressure to temperature measurements
    byte pressure_b2 = readRegister(0x00, 1);
    byte pressure_b1 = readRegister(0x01, 1);
    byte pressure_b0 = readRegister(0x02, 1);
    // Remove metadata like whether pressure or temperature
    long int pressure_combined = ((pressure_b2 << 16) | (pressure_b1 << 8) | pressure_b0); // 
    
    
    // Copy one line at a time and process it (Optional)
  }

}

//Read from register in DPS310
unsigned int readRegister(byte thisRegister, int bytesToRead) {

  byte inByte = 0; // Incoming byte from the SPI

  unsigned int result = 0; // Result to return

//  Serial.print(thisRegister, BIN); // PURPOSE?
//  Serial.print("\t");
  byte dataToSend = thisRegister | READ; // Format so that DPS knows this is a read request
  digitalWrite(pin_ss, LOW); // Select DPS
  
  SPI.transfer(dataToSend); // Send register address

  // Send a value of zero to read the first byte returned
  result = SPI.transfer(0x00);

    bytesToRead--;

  // if you still have another byte to read:
  if (bytesToRead > 0) {
    
    result = result << 8; // shift the first byte left, then get the second byte:
    inByte = SPI.transfer(0x00);
    
    result = result | inByte; // combine the byte you just got with the previous one:

    bytesToRead--; // decrement the number of bytes left to read:
  
  digitalWrite(pin_ss, HIGH); // deselect DPS
}

// Write to register in DPS310
void writeRegister(byte thisRegister, byte thisValue) {
 byte dataToSend = thisRegister; // Write request needs no additional formatting
 digitalWrite(pin_ss, LOW); // Select DPS310
 
 SPI.transfer(dataToSend); // Send register address
 SPI.transfer(thisValue); // Send value to store in register
 
 digitalWrite(pin_ss, HIGH); // Deselect DPS
}
