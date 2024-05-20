/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (8)

double peakAltitude = 0.00;
double altitude = 0.00;
//Adafruit_BMP280 bmp; // I2C
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
double seaLevel = 0.00;
boolean released = false;

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  seaLevel = bmp.readAltitude(1013.25);

  pinMode(5, OUTPUT); // Configure pin 7 as an Output
  pinMode(6, OUTPUT); // Configure pin 8 as an Output
  digitalWrite(5, HIGH); // Initialize pin 7 as Low
  digitalWrite(6, LOW); // Initialize pin 7 as Low

  pinMode(17, INPUT_PULLUP);//set pin for button press
  

  delay(2000);
  
  digitalWrite(5, HIGH); // Initialize pin 7 as Low
  digitalWrite(6, HIGH); // Initialize pin 7 as Low
}

void loop() {
    altitude = bmp.readAltitude(1013.25) - seaLevel;
    Serial.print(F("Approx altitude = "));
    Serial.print(altitude); /* Adjusted to local forecast! */
    Serial.println(" m");


    if (altitude > peakAltitude){
      peakAltitude = altitude; 
    }

    if (digitalRead(17) == 0){
      //release the linear actuator
       digitalWrite(5, LOW);
       digitalWrite(6, HIGH);

       delay (2000);

       digitalWrite(6, HIGH);
    }

    if (altitude < 0.5 && peakAltitude > 0.8 && released == false){
      //then release the linear actuator to deploy payload
          released = true;
          digitalWrite(5, LOW);
          digitalWrite(6, HIGH);
        
          delay(2000); // 2 seconds
          
          // Stops Actuator
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
                
    }

    Serial.println();
    delay(100);
}
