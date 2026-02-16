#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Настройки Wi-Fi
const char* ssid = "Nopike";
const char* password = "11111111";

// Настройки сервера
WiFiServer server(2000);
WiFiClient client;

// Настройки UART
HardwareSerial arduinoSerial(2); // UART2 на пинах 16(RX), 17(TX)

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(57600); // Скорость должна совпадать с rosserial

  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());

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

  // Небольшая задержка для стабильности
  delay(1);
}