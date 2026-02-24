#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// ===== НАСТРОЙКИ LED =====
#define LED_PIN 2   // Обычно встроенный LED на GPIO2

// Настройки Wi-Fi
const char* ssid = "Nepike";
const char* password = "123453119670";

// Настройки сервера
WiFiServer server(2000);
WiFiClient client;

// Настройки UART
HardwareSerial arduinoSerial(2); // UART2 на пинах 16(RX), 17(TX)

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(57600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED выключен

  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  bool ledState = false;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // МИГАНИЕ LED
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  // Wi-Fi подключен
  Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(LED_PIN, HIGH); // LED горит постоянно

  // Запуск TCP-сервера
  server.begin();
  Serial.println("TCP server started on port 2000");
}

void loop() {
  // Проверка подключения клиента
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
    }
  }

  // Получение данных от TCP-клиента и отправка в UART
  if (client && client.connected()) {
    while (client.available()) {
      uint8_t byte = client.read();
      arduinoSerial.write(byte);
    }
  }

  // Получение данных от UART и отправка TCP-клиенту
  while (arduinoSerial.available()) {
    uint8_t byte = arduinoSerial.read();
    if (client && client.connected()) {
      client.write(byte);
    }
  }

  delay(1);
}