/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * This example connects to HiveMQ's showcase broker.
 *
 * You can quickly test sending and receiving messages from the HiveMQ webclient
 * available at http://www.hivemq.com/demos/websocket-client/.
 *
 * Subscribe to the topic GsmClientTest/ledStatus
 * Publish "toggle" to the topic GsmClientTest/led and the LED on your board
 * should toggle and you should see a new message published to
 * GsmClientTest/ledStatus with the newest LED status.
 *
 **************************************************************/

//Added libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp32-hal-adc.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//default
#include "utilities.h"
#include <PubSubClient.h>

#ifndef BOARD_BAT_ADC_PIN
#error "No support this board"
#endif

// WiFi network name and password:
const char *networkName = "Totalplay-2.4G-5180";
const char *networkPswd = "4kEaB5CmM8L2zMQu";

// const char *udpAddress = "192.168.100.219";
// const int udpPort = 4578;

// MQTT details
const char *broker = "192.168.100.120";

const char *topicTemp       = "ESP32Client/temp";
const char *topicPressure       = "ESP32Client/pressure";
const char *topicHumidity       = "ESP32Client/humidity";
const char *topicInit      = "ESP32Client/init";
const char *topicLedStatus = "ESP32Client/ledStatus";
const char *mqttUser = "administrator";
const char *mqttPassword = "10121983";

WiFiClient espClient;
PubSubClient  client(espClient);

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

//Are we currently connected?
boolean connected = false;

uint32_t timeStamp = 0;
char buf[256];

//Sensor
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
float temperature, humidity, pressure, altitude;
unsigned long delayTime;

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial


void connectToWiFi(const char *ssid, const char *pwd)
{
    Serial.println("Connecting to WiFi network: " + String(ssid));

    // delete old config
    WiFi.disconnect(true);

    //Initiate connection
    WiFi.begin(ssid, pwd);

    Serial.println("Waiting for WIFI connection...");
    
    while (WiFi.status() != WL_CONNECTED)
       {  delay(500);
          Serial.print(".") ;
       }
    Serial.println("Connected to the WiFi network");
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();



    // Only proceed if incoming message's topic matches
    if (String(topic) == topicHumidity) {
            delayTime = 1000;
        float humidity =bme.readHumidity();
        char msg_out[20];
        dtostrf(humidity,2,2,msg_out);
        Serial.println(humidity);
        client.publish(topicHumidity, msg_out);
    }
}

boolean mqttConnect()
{
    SerialMon.print("Connecting to ");
    SerialMon.print(broker);

    // Or, if you want to authenticate MQTT:
    boolean status = client.connect("ESP32Client", "administrator", "10121983");

    if (status == false) {
        SerialMon.println(" fail");
        return false;
    }
    SerialMon.println(" success");
    client.publish(topicInit, "ESP32Client started");
    client.subscribe(topicHumidity);

    float humidity =bme.readHumidity();
    char msg_out[20];
    dtostrf(humidity,2,2,msg_out);

    client.publish(topicHumidity, msg_out);
    return client.connected();
}

void sensorTest()
{
    //Sensor
    Serial.println(F("BME280 test"));
    bool status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin(0x76);  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
    }
    Serial.println("-- Default Test --");
    delayTime = 1000;
  
    Serial.println();

}

void setup()
{
    Serial.begin(115200);
    delay(100);
    // Turn on DC boost to power on the modem
#ifdef BOARD_POWERON_PIN
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

    sensorTest();

    // MQTT Broker setup
    client.setServer(broker, 1883);
    client.setCallback(mqttCallback);
        //Connect to the WiFi network
    connectToWiFi(networkName, networkPswd);
}

void loop()
{
    if (!client.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            lastReconnectAttempt = t;
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        delay(100);
        return;
    }

    client.loop();
}