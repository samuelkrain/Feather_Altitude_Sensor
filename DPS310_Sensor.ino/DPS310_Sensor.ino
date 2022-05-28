/* Coding To Do
 *  - Configure sensor for fast accurate readings
 *  - Add loop with array/vector to store data
 *    - Work out how many readings can be stored before memory is full --> affects what solution is used to record data
 *    - Use a vector to store data as it can resize to accommodate additional readings
 *  - Continually calculate max altitude reached
 * 
 * The board used is an Adafruit Feather M0 Proto
 *  Website: https://learn.adafruit.com/adafruit-feather-m0-basic-proto?view=all
 * 
 * The sensor used is a DPS310 from Infineon Technologies, with the sensor breakout board designed by Adafruit
 *  Sensor page: https://www.infineon.com/cms/en/product/sensor/pressure-sensors/pressure-sensors-for-iot/dps310/
 *  Adafruit board page: https://www.adafruit.com/product/4494
 * 
 * Uses Adafruit_DPS310 Library
 *  GitHub: https://github.com/adafruit/Adafruit_DPS310
 * 
 * Sensor Configuration:
  Sensor operating mode: Background
  Pressure oversampling rate: 64x (high precision)
  Pressure measurement rate: 64 measurements per sec
  Temperature oversampling rate: 64x (single measurement)
  Temperature measurement rate: 64 measurements per sec
  Estimated measurement time (single measurement):
  Estimated measurement time (per second): 
 * 
 */

#include <Adafruit_DPS310.h>
#include <vector>

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Serial.println("DPS310");
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS OK!");

// Configures sensor to read pressure at a rate of 64Hz, with an oversampling rate of 64x
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  // Configures sensor to read temperature at a rate of 64Hz, with an oversampling rate of 64x
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

// Prints sensor details to serial for reference
  dps_temp->printSensorDetails();
  dps_pressure->printSensorDetails();
}

void loop() {
  sensors_event_t temp_event, pressure_event; // Unsure what this line does
  
  while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
    return; // wait until there's something to read
  }


  dps.getEvents(&temp_event, &pressure_event);
  Serial.print(F("Temperature = "));
  double t = temp_event.temperature;
  Serial.print(t);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  double p = pressure_event.pressure;
  Serial.print(p);
  Serial.println(" hPa"); 

  double alt = dps.readAltitude(1014); // Calculates altitude using temperature and pressure, with input of sea level pressure

  Serial.print(F("Altitude = "));
  Serial.print(alt);
  Serial.println(" m");
  

  Serial.println();
}
