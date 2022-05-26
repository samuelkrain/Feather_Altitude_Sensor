/*
INFO
  Board used: Adafruit Feather M0 Basic Proto
  Website: https://learn.adafruit.com/adafruit-feather-m0-basic-proto?view=all
  Sensor used: DPS310 Digital Barometric Pressure Sensor
  Website: https://www.mouser.co.uk/new/infineon/infineon-dps310-sensor/

  Code influenced by:
  https://docs.arduino.cc/tutorials/communication/BarometricPressureSensor
  [DETAILS]

Circuit:


SPI Configuration:
  Configuration: (Mode 3) CPOL = , CPHA = 
  Speed: (10 MHz, defined by sensor)
  Control byte: (bit 7 of address, w = 0, r = 1)

Sensor Configuration:
  Sensor operating mode: Background
  Pressure oversampling rate: 64x (high precision)
  Pressure measurement rate: 4 measurements per sec
  Pressure scaling factor, kP: 1040384
  Temperature oversampling rate: 1x (single measurement)
  Temperature measurement rate: 4 measurements per sec
  Temperature scaling factor, kT: 524288
  Estimated measurement time (single measurement): 108 ms
  Estimated measurement time (per second): 432 ms/s
  

Coding Checklist
  Code data preprocessing section (l. 89)
  Calculate compensated values
  Store values
  Loop measurement system
  Check number of measurements taken over flight period
  
Debug Checklist
    - Check that readRegister and writeRegister perform as expected
    - Check if correct SPI mode is set in CFG_REG after changing (SPI commands work)
    - Check interrupt is successfully received when FIFO is full
    - Check if variable data structure is suitable for storing and manipulating measurements
    - Does pin_ss have to be declared high in setup?
    - Check if sensor updates calibration coefficients or if they are fixed (if fixed then )
    - Check ratio of pressure to temperature measurements in readings (should be 1:1)
*/

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
//const int pin_interrupt = 6; // Pin D6 used to receive interrupt

// Establish SPI connection with sensor, and serial connection with computer if applicable
// Configure sensor to desired operating mode
void setup() {

  delay(50); // Allows time for sensor to initialise

  // Set up pins
  //   pinMode(22, INPUT); // Define MISO as input pin ** NEEDED? **
  //   pinMode(23, OUTPUT); // Define MOSI as output pin
  //   pinMode(24, OUTPUT); // Define SCK as output pin
  pinMode(pin_ss, OUTPUT); // Define D5 pin (SS) as output (active low)
//  pinMode(pin_interrupt, INPUT); // Define D6 pin (SDO) as input. Used as interrupt pin

  Serial.begin(9600); // Open serial connection

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // ** CHECK MSB FIRST **
  Serial.print("Starting setup...\n");

  // Set up registers
//  writeRegister(CFG_REG, 0xC7); // Sets SPI mode to 3-wire with interrupt, and enables FIFO
  writeRegister(CFG_REG, 0x06); // Sets SPI mode to 4-wire, and enables FIFO
  writeRegister(MEAS_CFG, 0x07); // Sets operating mode to background
  writeRegister(PRS_CFG, 0x26); // Sets desired pressure reading parameters (See *Sensor Configuration*)
  writeRegister(TMP_CFG, 0xA0); // Sets desired temperature reading parameters

  // INSERT delay IF NEEDED

  // Debugging register checks
  // Read FIFO_STS (0X0B), [bit 1] = 1: FIFO is full, [bit 0] = 1: FIFO is empty
  // Check if empty --> flush or store data if not
  // Read MEAS_CFG (0x08), [6] = 1: Sensor is ready for use
  Serial.print("Setup complete.\n");

}

void loop() {

    // Initialise pressure and temperature scaling factors (based on oversampling rate)
    int kP = 1040384; // Pressure scaling factor
    int kT = 524288; // Temperature scaling factor
    
     // Read calibration coefficients
     // Values larger than 8 bits are split between multiple registers and must be combined
     byte c0_upper = readRegister(0x10,1);
     byte c0_lower = readRegister(0x11,1) & 0b11110000;
     byte c0 = ((c0_upper << 4) | (c0_lower >> 4));
     
     byte c1_upper = readRegister(0x11,1) & 0b00001111;
     byte c1_lower = readRegister(0x12,1);
     byte c1 = ((c1_upper << 4) | c1_lower);
     
     byte c00_2 = readRegister(0x13,1);
     byte c00_1 = readRegister(0x14,1);
     byte c00_0 = readRegister(0x15,1) & 0b11110000;
     byte c00 = ((c00_2 << 12) | (c00_1 << 4) | (c00_0 >> 4));

     byte c10_2 = readRegister(0x15,1) & 0b00001111;
     byte c10_1 = readRegister(0x16,1);
     byte c10_0 = readRegister(0x17,1);
     byte c10 = ((c10_2 << 16) | (c10_1 << 8) | c10_0);

     byte c01_1 = readRegister(0x18,1);
     byte c01_0 = readRegister(0x19,1);
     byte c01 = ((c01_1 << 8) | c01_0);

     byte c11_1 = readRegister(0x1A,1);
     byte c11_0 = readRegister(0x1B,1);
     byte c11 = ((c11_1 << 8) | c11_0);

     byte c20_1 = readRegister(0x1C,1);
     byte c20_0 = readRegister(0x1D,1);
     byte c20 = ((c20_1 << 8) | c20_0);

     byte c21_1 = readRegister(0x1E,1);
     byte c21_0 = readRegister(0x1F,1);
     byte c21 = ((c21_1 << 8) | c21_0);

     byte c30_1 = readRegister(0x20,1);
     byte c30_0 = readRegister(0x21,1);
     byte c30 = ((c30_1 << 8) | c30_0);
     
    
    // Copy all data to memory and then process in bulk
    
    
    // Loop to iterate through pressure measurements (i < 32)
    // Filter based on ratio of pressure to temperature measurements
    for (int i = 0; i<32; i++) {
      bool isPressure[32];
      unsigned int readingCombined[32];
      
      byte reading_b2 = readRegister(0x00, 1);
      byte reading_b1 = readRegister(0x01, 1);
      byte reading_b0 = readRegister(0x02, 1);
      
      readingCombined[i] = ((reading_b2 << 16) | (reading_b1 << 8) | reading_b0); // Combine bytes into one value. ** CHECK INT
      
      // Remove metadata like whether pressure or temperature measurement
      int flagMetadata = 0b00000001; // Check whether int?
      
      if ((reading_b0 & flagMetadata) == 1) { // Check if value is a pressure or temperature measurement . 1 in binary?
        isPressure[i] = 1;
      } else {
        isPressure[i] = 0;
      }
    // ** NB ** measurement may have to be bit-shifted down one bit to remove pressure/temperature flag
    }
//
//    int n = 0;
//    for (n<32) {
//      long int readingScaled[32];
//      double readingComp[32]; // **CHECK THIS IS APPROPRIATE DATATYPE**
//      
//      // Scale measurement by either pressure or temperature scaling factor
//      // Multiply measurements by calibration coefficients
//      if (isPressure[n] == 1) {
//        readingScaled[n] = readingCombined[i]/kP;
//        readingComp[n] = c00 + readingScaled[n]*(c10 + readingScaled[n] *(c20+ readingScaled[n] *c30)) + readingScaled[n+1] *c01 +
//        readingComp[n+1] * readingScaled[n] *(c11+readingScaled[n]*c21);
//      } else {
//        readingScaled[n] = readingCombined[n]/kT;
//        readingComp[n] =  c0*0.5 + c1*readingScaled[n];
//      }
//      n ++;
//      }

 
  
    
  }

//Read from register in DPS310
unsigned int readRegister(byte thisRegister, int bytesToRead) {

  byte inByte = 0; // Incoming byte from the SPI

  unsigned int result = 0; // Result to return

//  Serial.print(thisRegister, BIN);
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

return (result);

}

// Write to register in DPS310
 void writeRegister(byte thisRegister, byte thisValue) {
 byte dataToSend = thisRegister; // Write request needs no additional formatting
 digitalWrite(pin_ss, LOW); // Select DPS310
 
 SPI.transfer(dataToSend); // Send register address
 SPI.transfer(thisValue); // Send value to store in register
 
 digitalWrite(pin_ss, HIGH); // Deselect DPS
}
