#include <WiFi.h>

const char* ssid = "Nepike";
const char* password = "123453119670";

WiFiServer server(2000);
WiFiClient client;

#define RXD2 16
#define TXD2 17
#define LED_PIN 2

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

  if (!client || !client.connected()) {
    client = server.available();

    if (client) {
      Serial.println("Client connected");
      // clear buffers
      while (Serial2.available()) Serial2.read();
      while (client.available()) client.read();
    }
    delay(1);
    return;
  }

  // TCP → UART
  while (client.available() && Serial2.availableForWrite() > 0) {
    Serial2.write(client.read());
  }

  // UART → TCP
  while (Serial2.available()) {
    if (!client.connected()) {
      break;
    }

    uint8_t c = Serial2.read();
    if (client.write(c) != 1) {
      break;
    }
  }

  delay(1); // very VERY important
}