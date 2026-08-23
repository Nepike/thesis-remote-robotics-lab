/*
 * custom-slave.ino
 *
 * Роль: SLAVE-устройство (Arduino)
 *
 * Задачи:
 * 1. Принимать бинарные DataPack пакеты от Master
 * 2. Проверять CRC и целостность
 * 3. Выполнять команду
 * 4. (Опционально) возвращать RESPONSE пакет
 *
 * SLAVE:
 *  - никогда не инициирует команды
 *  - только отвечает на входящие пакеты
 */

#include <SoftwareSerial.h>
#include <NPK.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

SoftwareSerial espSerial(3, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

/*
 * Основная точка обработки команд NPK
 *
 * Каждая команда:
 *  - читает аргументы через get_next_val<T>()
 *  - выполняет действие
 *  - при необходимости заполняет response
 */
DataPack DataPack::perform_command() {

  DataPack response(Command::RESPONSE);

  if (!is_valid()) {
    response.is_valid_ = false;
    return response;
  }

  switch (command_) {

    case Command::BLINK:
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      break;

    case Command::LCD: {
      lcd.clear();
      const char* text = get_next_val<const char*>();
      lcd.print(text);
      break;
    }

    case Command::GETSPEED: {
      uint8_t encoder = get_next_val<uint8_t>();
      long value = (encoder == 0) ? 123 : 456; // пример
      response.append_arg(String(value).c_str());
      break;
    }

    default:
      response.is_valid_ = false;
      break;
  }

  return response;
}

void setup() {
  espSerial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  // Проверяем наличие входящего пакета
  if (espSerial.available() >= 2) {

    DataPack pack(espSerial);

    if (!pack.is_valid()) return;

    DataPack response = pack.perform_command();

    // RESPONSE отправляется только если в нём есть аргументы
    if (response.is_valid() && response.get_args_count() > 0) {

      uint8_t* data = response.get_serialized_data();
      espSerial.write(data, response.get_serialized_size());
      free(data);
    }
  }
}
