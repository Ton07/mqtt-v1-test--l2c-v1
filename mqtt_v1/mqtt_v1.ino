#include <SimpleDHT.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define WLAN_SSID "Tonnam"
#define WLAN_PASS "Tonnam110349"
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define IO_USERNAME "Tonnam07"
#define IO_KEY "aio_oJiC69TnR7jyHGBCXdas7r2vcCPn"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Temperature");

SimpleDHT22 dht22(15);  // DHT22 connected to pin 15
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns and 2 rows

SemaphoreHandle_t dataMutex;

byte hum = 0;
byte temp = 0;

void readDHT22(void *pvParameters) {
  while (1) {
    dht22.read(&temp, &hum, NULL);
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    Temperature.publish(temp);
    xSemaphoreGive(dataMutex);

    Serial.printf("Temperature: %d °C, Humidity: %d %%\n", temp, hum);
    delay(5000);
  }
}

void receiveMQTT(void *pvParameters) {
  while (1) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    // Process MQTT data and display on I2C
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(hum);
    lcd.print("%");
    xSemaphoreGive(dataMutex);

    delay(5000);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C
  lcd.begin(16, 2);

  dataMutex = xSemaphoreCreateMutex();

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  connect();

  xTaskCreatePinnedToCore(readDHT22, "readDHT22", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(receiveMQTT, "receiveMQTT", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // The loop function is not used in this example
}

void connect() {
  Serial.print("Connecting to Adafruit IO... ");
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.printf("Connection failed, error %d. Retrying...\n", ret);
    delay(10000);
  }
  Serial.println("Adafruit IO Connected!");
}
