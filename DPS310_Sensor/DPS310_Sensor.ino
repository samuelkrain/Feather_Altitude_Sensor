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

*/



// Definitions

// Includes
#include <SPI.h>

//Registers
#define PRS_CFG 0x06 // Pressure Config Register (Write mode) 
#define TMP_CFG 0x07 // Temperature Config Register (Write mode)
#define MEAS_CFG 0x08 // Measurement Config Register (Write mode)
#define CFG_REG 0X09 // Interrupt and FIFO Config Register (Write mode)

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

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // ** CHECK MSB FIRST **

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

  
}

void loop() {
  
//  if (digitalRead(pin_interrupt) == HIGH) {
//
//    //
//  }
  

}
