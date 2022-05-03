/*
INFO
  Board used: Adafruit Feather M0 Basic Proto
  Datasheet: REF
  Sensor used: DPS310 Digital Barometric Pressure Sensor
  Website:

Circuit:


SPI Details:
  Configuration: (Mode 3)
  Speed: (10 MHz)
  Control byte: (bit 7 of address, w = 0, r = 1)

Coding Checklist
  Write readRegister and writeRegister functions
  Check whether READ and WRITE are needed, and if so what value they should be
  Code serial connection to computer for debugging
  Code data preprocessing section (l. 89)
Debug Checklist
    Check SPI.h is included and has all necessary files
    Check whether SerialUSB or Serial should be used
    Check if correct SPI mode is set in CFG_REG after changing (SPI commands work)
    Check interrupt is successfully received when FIFO is full
    Check if variable data structure is suitable for storing and manipulating measurements
*/



// Definitions

// Includes
#include <SPI.h>

//Registers
#define PRS_CFG 0x06 // Pressure Config Register (Write mode) 
#define TMP_CFG 0x07 // Temperature Config Register (Write mode)
#define MEAS_CFG 0x08 // Measurement Config Register (Write mode)
#define CFG_REG 0X09 // Interrupt and FIFO Config Register (Write mode)
#define VAL 0x00 // Placeholder whilst programming

//const byte READ = ;
//const byte WRITE = ;

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
  pinMode(5, OUTPUT); // Define D5 pin (SS) as output (active low)
  pinMode(6, INPUT); // Define D6 pin (SDO) as input. Used as interrupt pin

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
    byte pressure_b2 = readRegister(0x00, VAL);
    byte pressure_b1 = readRegister(0x01, VAL); // left shift 8 times
    byte pressure_b0 = readRegister(0x02, VAL); // Left shift 16 times
    // Remove metadata like whether pressure or temperature
    long pressure_combined = ((pressure_b2 << 16) | (pressure_b1 << 8) | pressure_b0); // DEBUG: test that measurements are being combined correctly before entering loop



    // Copy one line at a time and process it (Optional)
  }

}

//Read from register in DPS310
unsigned int readRegister(byte thisRegister, int bytesToRead) {

}

// Write to register in DPS310
void writeRegister(byte thisRegister, byte thisValue) {

}
