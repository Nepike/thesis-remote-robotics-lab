/*
 * custom-master.ino
 *
 * Роль: MASTER-устройство (ESP8266)
 *
 * Задачи:
 * 1. Принимать команды по HTTP (JSON)
 * 2. Преобразовывать их в бинарные DataPack пакеты (NPK)
 * 3. Отправлять пакеты Slave-устройству (Arduino)
 * 4. При необходимости принимать RESPONSE пакеты
 *
 * MASTER:
 *  - всегда ИНИЦИИРУЕТ команды
 *  - может ожидать RESPONSE
 */

#include <NPK.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// SoftwareSerial используется как транспорт между платами
#define RX_PIN 3
#define TX_PIN 1

SoftwareSerial nanoSerial(RX_PIN, TX_PIN);

// Параметры Wi-Fi
const char* ssid = "Nopike";
const char* password = "11111111";

ESP8266WebServer server(80);

/*
 * Вспомогательная функция индикации активности ESP
 * Мигает встроенным светодиодом при отправке пакета
 */
void espBlink() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
}

/*
 * Универсальная функция отправки DataPack пакета Slave-устройству
 *
 * ВАЖНО:
 * DataPack::get_serialized_data() выделяет память через new[]
 * поэтому после отправки обязательно освобождаем её
 */
void sendPack(const DataPack& pack) {
  uint8_t* data = pack.get_serialized_data();
  size_t size = pack.get_serialized_size();

  nanoSerial.write(data, size);
  free(data);

  espBlink();
}

/*
 * Приём RESPONSE пакета от Slave
 *
 * Используется только для команд, которые ожидают ответ
 * (GETRANGE, GETSPEED и т.п.)
 */
DataPack receiveResponse() {
  unsigned long startTime = millis();

  // Ожидаем хотя бы стартовый маркер
  while (nanoSerial.available() < 2) {
    if (millis() - startTime > 1000) {
      // Таймаут — возвращаем искусственный RESPONSE
      DataPack timeoutPack(DataPack::Command::RESPONSE);
      timeoutPack.append_arg("RESPONSE TIMEOUT!");
      return timeoutPack;
    }
  }

  return DataPack(nanoSerial);
}

/*
 * HTTP endpoint /command
 *
 * Ожидает JSON вида:
 * {
 *   "cmd": "go",
 *   "args": [1, 150, 1000]
 * }
 *
 * Каждая HTTP-команда транслируется в DataPack
 */
void handleCommand() {

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  String jsonStr = server.arg("plain");
  StaticJsonDocument<256> doc;

  if (deserializeJson(doc, jsonStr)) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  const char* cmd = doc["cmd"];
  JsonArray args = doc["args"];

  /*
   * Пример команды GO
   * Аргументы:
   *  - direction (uint8)
   *  - speed     (uint8)
   *  - time      (uint16)
   */
  if (strcmp(cmd, "go") == 0) {

    DataPack pack(DataPack::Command::GO);
    pack.append_arg((uint8_t)args[0]);
    pack.append_arg((uint8_t)args[1]);
    pack.append_arg((uint16_t)args[2]);

    sendPack(pack);
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  }

  /*
   * Пример команды LCD
   * Передаём строку для отображения
   */
  else if (strcmp(cmd, "lcd") == 0) {

    DataPack pack(DataPack::Command::LCD);
    pack.append_arg(args[0].as<const char*>());

    sendPack(pack);
    server.send(200, "text/plain", "LCD command sent");
  }

  /*
   * Пример команды-запроса (REQUEST → RESPONSE)
   */
  else if (strcmp(cmd, "getrange") == 0) {

    DataPack request(DataPack::Command::GETRANGE);
    sendPack(request);

    DataPack response = receiveResponse();
    if (response.is_valid()) {
      const char* value = response.get_next_val<const char*>();
      server.send(200, "text/plain", value);
    } else {
      server.send(500, "text/plain", "Invalid response");
    }
  }

  else {
    server.send(404, "text/plain", "Unknown command");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  nanoSerial.begin(9600);

  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Отображаем IP адрес на Slave LCD
  DataPack pack(DataPack::Command::LCD);
  pack.append_arg(WiFi.localIP().toString().c_str());
  sendPack(pack);

  server.on("/command", HTTP_POST, handleCommand);
  server.begin();
}

void loop() {
  server.handleClient();
}
