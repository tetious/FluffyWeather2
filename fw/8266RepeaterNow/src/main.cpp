#include <ESP8266WiFi.h>
#include <espnow.h>

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

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  Serial.write(incomingData, len);
}

void setup() {
  pinMode(LED_BUILTIN_AUX, OUTPUT);
  Serial.begin(115200);
  //  Serial.print("ESP8266 Board MAC Address:  ");
  //  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != 0) {
    blink(100, 3);
    blink(500, 3);
    blink(100, 3);
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  blink(500, 4);
}

void loop() {
  
}
