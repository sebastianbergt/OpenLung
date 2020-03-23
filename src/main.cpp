#include <Arduino.h>
#include <WiFi.h>
#include "SSD1306Wire.h"
#include "Adafruit_BMP280.h"

const uint8_t I2C_ADDRESS_DISPLAY = 0x3c;
const uint8_t DISPLAY_SDA = 5;
const uint8_t DISPLAY_SCL = 4;

const uint8_t BMP280_0_SDA = 14;
const uint8_t BMP280_0_SCL = 12;
const uint8_t BMP280_1_SDA = 13;
const uint8_t BMP280_1_SCL = 15;

SSD1306Wire display(I2C_ADDRESS_DISPLAY, DISPLAY_SDA, DISPLAY_SCL);

const int rx_pin = 13; //Serial rx pin no D2, green
const int tx_pin = 12; //Serial tx pin no D1, blue


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

void co2_sensor_setup()
{
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(false);
  while (mhz19_uart->getStatus() < 0)
  {
    Serial.println("  MH-Z19B reply is not received or inconsistent.");
    delay(100);
  }
}


void displayMeasurements(const measurement_t &m)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "p [hPa]");
  display.drawString(72, 0, "Temp [deg]");
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 15, String(m.co2_ppm));
  display.drawString(84, 15, String(m.temperature));
  display.display();
}

void publishMeasurements(const measurement_t &m)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    std::string data;
    prepareSchema(m, data);

    WiFiClient client; // TCP
    if (!client.connect(host, port))
    {
      Serial.println("Connection failed.");
      delay(1000);
    } else {
      if (client.connected())
      {
        client.write(data.c_str(), data.length());
        client.flush(); // wait maximum 1s
        client.stop();
      }
    }
  }
  else
  {
    wifi_setup();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting setup...");

  Serial.println("  display...");
  display_setup();
  Serial.println("  wifi...");
  wifi_setup();
  Serial.println("  co2 sensor...");
  co2_sensor_setup();

  Serial.println("...done!");
}

void loop()
{
  measurement_t m = mhz19_uart->getMeasurement();
  if(m.state != -1) {
    displayMeasurements(m);
    publishMeasurements(m);
  }
  delay(1000);
}