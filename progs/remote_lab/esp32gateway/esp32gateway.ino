#include <WiFi.h>

#define LED_PIN 2

const char* ssid = "Nepike";
const char* password = "123453119670";

WiFiServer server(2000);
WiFiClient client;

#define RXD2 16
#define TXD2 17

void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  bool ledState = false;

  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_PIN, HIGH);

  server.begin();

  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {

  // ожидание клиента
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }

  // TCP → Mega
  while (client.available()) {
    uint8_t c = client.read();
    Serial2.write(c);
  }

  // Mega → TCP
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    client.write(c);
  }
}