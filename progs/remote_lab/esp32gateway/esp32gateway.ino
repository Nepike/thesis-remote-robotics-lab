#include <WiFi.h>

const char* ssid = "Nepike";
const char* password = "123453119670";

WiFiServer server(2000);
WiFiClient client;

#define RXD2 16
#define TXD2 17
#define LED_PIN 2

// Буфер для пакетной отправки в TCP
#define TCP_BUF_SIZE 64
uint8_t tcpBuffer[TCP_BUF_SIZE];
uint8_t tcpBufPos = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_PIN, HIGH);

  server.begin();
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2); // скорость под девайс!
}

void loop() {
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      client.setNoDelay(true);
      Serial.println("Client connected");
      
      while (Serial2.available()) Serial2.read();
      while (client.available()) client.read();
      tcpBufPos = 0;
    }
    yield();
    return;
  }

  // TCP → UART
  while (client.available() && Serial2.availableForWrite() > 0) {
    Serial2.write(client.read());
  }

  // UART → TCP
  while (Serial2.available()) {
    if (!client.connected()) break;
    
    uint8_t c = Serial2.read();
    tcpBuffer[tcpBufPos++] = c;
    
    if (tcpBufPos >= TCP_BUF_SIZE) {
      flushTcpBuffer();
    }
  }
  
  if (tcpBufPos > 0 && !client.available() && !Serial2.available()) {
    flushTcpBuffer();
  }

  yield();
}


void flushTcpBuffer() {
  if (tcpBufPos > 0 && client.connected()) {
    client.write(tcpBuffer, tcpBufPos);
    client.flush();  // гарантируем отправку
    tcpBufPos = 0;
  }
}