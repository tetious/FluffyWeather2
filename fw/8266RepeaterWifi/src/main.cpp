#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "../../secure.h"
#include "../../shared.h"

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void blink(unsigned int time, unsigned int count)
{
  for (unsigned int j = 0; j < count; j++)
  {
    digitalWrite(LED_BUILTIN_AUX, LOW);
    delay(time / 2);
    digitalWrite(LED_BUILTIN_AUX, HIGH);
    delay(time / 2);
  }
}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    blink(100, 1);
    blink(200, 1);
    blink(100, 1);
    if (mqttClient.connect("ToonyWeather"))
    {
      // Subscribe or resubscribe to a topic
    }
    else
    {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pinMode(LED_BUILTIN_AUX, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    blink(100, 1);
  }

  mqttClient.setServer(mqtt_server, 1883);
  //client.setCallback(callback);
  reconnect_mqtt();
  blink(500, 4);
  mqttClient.publish("test/log", "Connected!");
  Serial.begin(115200);
}

void loop()
{
  if (!mqttClient.connected())
  {
    reconnect_mqtt();
  }
  if (Serial.available())
  {
    data_struct data;
    Serial.readBytes((uint8_t *)&data, sizeof(data_struct));
    mqttClient.publish("test/weather", (const uint8_t *)&data, sizeof(data_struct));
  }
}
