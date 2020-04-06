#include <Arduino.h>
#include <WiFi.h>
#include "SSD1306Wire.h"
#include "Adafruit_BMP280.h"

const uint8_t I2C_ADDRESS_DISPLAY = 0x3c;
const uint8_t DISPLAY_SDA = 5;
const uint8_t DISPLAY_SCL = 4;

SSD1306Wire display(I2C_ADDRESS_DISPLAY, DISPLAY_SDA, DISPLAY_SCL); // uses Wire

Adafruit_BMP280 bmp(&Wire); // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void display_setup()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Waking up!");
  display.display();
}

void displayMeasurements(float temperature, float pressure)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "p [hPa]");
  display.drawString(72, 0, "Temp [deg]");
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 15, String(pressure));
  display.drawString(84, 15, String(temperature));
  display.display();
}

void bmp280_setup()
{
  if (!bmp.begin(BMP280_ADDRESS_ALT))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
      delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  bmp_temp->printSensorDetails();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting setup...");
  Serial.println("  display...");
  display_setup();
  Serial.println("  bmp280..."); 
  bmp280_setup();
  Serial.println("...done!");
}

void loop()
{
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  displayMeasurements(temp_event.temperature, pressure_event.pressure);
  
  // Serial.print(F("Temperature = "));
  // Serial.print(temp_event.temperature);
  // Serial.println(" *C");

  // Serial.print(F("Pressure = "));
  // Serial.print(pressure_event.pressure);
  // Serial.println(" hPa");

  // Serial.println();
}