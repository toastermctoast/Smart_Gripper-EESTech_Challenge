#ifndef CONFIG_H
#define CONFIG_H

// Select one to turn on
// Enable or disable magnetic sensor + button control
#define ENABLE_MAGNETIC_SENSOR true

// Enable or disable commander functionality, please check: https://docs.simplefoc.com/commander_interface
#define ENABLE_COMMANDER false

#define ENABLE_READ_ANGLE false

// define SPI pins for TLE5012 sensor
#define PIN_SPI1_SS0 94  // Chip Select (CS) pin
#define PIN_SPI1_MOSI 69 // MOSI pin
#define PIN_SPI1_MISO 95 // MISO pin
#define PIN_SPI1_SCK 68  // SCK pin

// define driver pins
#define U 11
#define V 10
#define W 9
#define EN_U 6
#define EN_V 5
#define EN_W 3

enum ObjectType {
  NO_OBJECT,
  SOFT_OBJECT,
  MEDIUM_OBJECT,
  HARD_OBJECT
};

#endif // CONFIG_H