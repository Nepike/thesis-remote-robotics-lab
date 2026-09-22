/*
 * esp32microbot — прошивка трёхколёсного микро-робота лаборатории.
 *
 * Роль в системе.
 *   В отличие от esp32gateway (тупой мост «TCP ↔ UART» перед Arduino робота),
 *   здесь ESP32 — это И приёмник, И контроллер одновременно. Отдельного
 *   низкоуровневого МК нет: та же самая микросхема держит WiFi, разбирает
 *   команды сервера, крутит моторы и читает датчики.
 *
 *   Следствие, о котором стоит помнить: управляющий цикл делит процессор со
 *   стеком WiFi. Поэтому в loop() нет ни одного delay() — всё расписано по
 *   millis(), а loop() обязан прокручиваться быстро.
 *
 * Что умеет.
 *   - принимает twist (v, ω) по TCP, как yarp-13 через cmd_vel;
 *   - отдаёт телеметрию: три дальномера, напряжение батареи, текущий setpoint;
 *   - принимает коэффициенты ПИД (регулятор пока разомкнут, см. раздел ПИД);
 *   - выбор WiFi-сети с экранчика четырьмя кнопками, выбор живёт в NVS;
 *   - дедман: пропали команды или отвалился клиент — моторы в ноль.
 *
 * Протокол (строки, \n в конце, регистр команды не важен).
 *   Сервер -> робот:
 *     TWIST <v> <w>        v м/с, w рад/с; каждая команда кормит дедман
 *     STOP                 немедленный ноль
 *     PID <kp> <ki> <kd>   коэффициенты регулятора (оба колеса)
 *     ARM <0|1>            программное разрешение силовой части
 *     PING                 проверка живости, робот отвечает PONG
 *   Робот -> сервер (телеметрия, _TELEMETRY_HZ раз в секунду):
 *     t=<мс>,rf_l=<см>,rf_c=<см>,rf_r=<см>,vbat=<В>,v=<м/с>,w=<рад/с>,
 *     pwm_l=<-255..255>,pwm_r=<...>,armed=<0|1>,kp=<..>,ki=<..>,kd=<..>
 *
 *   Формат «ключ=значение через запятую» взят не с потолка: ровно так уже
 *   разговаривает SimpleSerialDevice в manager/DeviceDrivers.py. Заодно строку
 *   видно глазами в `nc <ip> 2000`, без всяких библиотек разбора JSON на МК.
 *
 * Схема подключения — см. раздел «Распиновка». Питание платы дисплея — 3.3 В,
 * не 5 В: кнопки подтянуты к питанию модуля, а 5 В на GPIO ESP32 убивает вход.
 */

#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ============================================================================
//  Распиновка
// ============================================================================
//
// Аналоговые входы взяты ТОЛЬКО из ADC1 (GPIO32-39). Это не вкусовщина: у ESP32
// блок ADC2 физически занят радиомодулем, и пока WiFi активен, analogRead() с
// любого пина ADC2 возвращает мусор либо просто виснет. Все четыре выбранных
// пина вдобавок input-only, то есть случайно назначить их выходом невозможно.

#define PIN_RF_LEFT    36   // Sharp GP2Y0A21, левый   (ADC1_CH0, он же VP)
#define PIN_RF_CENTER  39   // Sharp GP2Y0A21, средний (ADC1_CH3, он же VN)
#define PIN_RF_RIGHT   34   // Sharp GP2Y0A21, правый  (ADC1_CH6)
#define PIN_VBAT       35   // делитель напряжения батареи (ADC1_CH7)

// Драйвер моторов TB6612FNG. На мотор: одна линия ШИМ + два бита направления.
// STBY общий на обе половины моста: низкий уровень — силовая часть обесточена.
#define PIN_MOTOR_L_PWM  13
#define PIN_MOTOR_L_IN1  27
#define PIN_MOTOR_L_IN2  14
#define PIN_MOTOR_R_PWM   4
#define PIN_MOTOR_R_IN1  16
#define PIN_MOTOR_R_IN2  17
#define PIN_MOTOR_STBY   18

// Экран: I2C на штатных пинах ESP32.
#define PIN_I2C_SDA  21
#define PIN_I2C_SCL  22

// Кнопки на плате экрана (K1..K4). Замыкают вход на землю, подтяжка к питанию
// модуля — то есть нажатие это НОЛЬ. Внутренний INPUT_PULLUP включаем сверх
// внешнего специально: если провод кнопки оборвётся, вход прочитается как
// «не нажато». Отказ проводки не должен сам по себе что-то нажимать.
#define PIN_BTN_UP    32   // K1, маркировка «∧»
#define PIN_BTN_DOWN  33   // K2, маркировка «∨»
#define PIN_BTN_OK    25   // K3, маркировка «#»
#define PIN_BTN_BACK  26   // K4, маркировка «*»

#define PIN_LED        2   // встроенный светодиод платы

// Свободны под энкодеры, когда они появятся: GPIO19, GPIO23, GPIO5, GPIO15.
// Прерывания на ESP32 вешаются на любой GPIO, так что выбор непринципиален.


// ============================================================================
//  Геометрия и пределы
// ============================================================================

const float WHEEL_BASE_M     = 0.105f;  // расстояние между ведущими колёсами, м
const float MAX_WHEEL_SPEED  = 0.35f;   // скорость колеса при ШИМ 255, м/с
const int   PWM_MAX          = 255;
const int   PWM_MIN_MOVE     = 45;      // ниже этого мотор только гудит

// Дедман. Если TWIST не приходил дольше — setpoint обнуляется.
// Сервер стримит setpoint на 10 Гц, так что 500 мс это пять потерянных кадров.
const uint32_t CMD_TIMEOUT_MS = 500;

const uint32_t CONTROL_PERIOD_MS   = 20;   // 50 Гц: моторы и регулятор
const uint32_t SENSOR_PERIOD_MS    = 50;   // 20 Гц: опрос дальномеров
const uint32_t TELEMETRY_PERIOD_MS = 100;  // 10 Гц: отправка телеметрии
const uint32_t DISPLAY_PERIOD_MS   = 200;  // 5 Гц: перерисовка экрана

// Делитель напряжения батареи: VBAT --[R1]--+--[R2]-- GND, вход АЦП в середине.
// При 2S Li-ion (до 8.4 В) и R1=100к, R2=47к на вход приходит максимум 2.69 В —
// в пределах шкалы АЦП с аттенюацией 11 дБ.
const float VBAT_R1 = 100000.0f;
const float VBAT_R2 =  47000.0f;
const float VBAT_DIVIDER = (VBAT_R1 + VBAT_R2) / VBAT_R2;


// ============================================================================
//  Известные сети
// ============================================================================
//
// Ввод пароля четырьмя кнопками — это пытка, поэтому сети прошиты списком, а с
// экрана выбирается только нужная. Выбор переживает перезагрузку (NVS), так что
// меню нужно ровно один раз — когда робот переехал в другую лабораторию.
//
// TODO: вынести в конфигурацию (та же проблема, что и в esp32gateway).

struct WifiCred {
  const char* ssid;
  const char* pass;
};

const WifiCred WIFI_NETWORKS[] = {
  { "Nepike",  "123453119670" },
  { "lab-ap",  "" },
};
const int WIFI_NETWORK_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;

WiFiServer tcpServer(2000);   // порт совпадает с полем "port" в devices.json
WiFiClient tcpClient;

Preferences prefs;
int currentNetwork = 0;       // индекс в WIFI_NETWORKS


// ============================================================================
//  Экран
// ============================================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64     // поставить 32, если модуль половинной высоты
#define OLED_I2C_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayPresent = false;

enum UiScreen { SCREEN_STATUS, SCREEN_MENU };
UiScreen uiScreen = SCREEN_STATUS;
int menuCursor = 0;


// ============================================================================
//  Состояние робота
// ============================================================================

// Уставка, пришедшая от сервера.
float targetV = 0.0f;       // м/с
float targetW = 0.0f;       // рад/с
uint32_t lastCmdMs = 0;     // когда последний раз кормили дедман

// Реально выданный на моторы ШИМ — уходит в телеметрию, по нему видно
// насыщение и работу компенсации мёртвой зоны.
int pwmLeft  = 0;
int pwmRight = 0;

// Программное разрешение силовой части. Снимается кнопкой «*» на экране или
// командой ARM 0 — удобно, когда робот на столе и крутить колёса не надо.
bool armed = true;

// Показания датчиков.
int   rfLeft = -1, rfCenter = -1, rfRight = -1;   // см, -1 = вне диапазона
float vbat = 0.0f;                                 // В


// ============================================================================
//  Дальномеры Sharp GP2Y0A21YK0F
// ============================================================================
//
// ВАЖНО, про этот датчик нужно знать две вещи.
//
// 1. Характеристика нелинейная и обратная: чем ближе препятствие, тем ВЫШЕ
//    напряжение. Пересчёт — эмпирическая степенная аппроксимация паспортного
//    графика, рабочий диапазон 10..80 см.
//
// 2. Ближе 10 см характеристика ЗАГИБАЕТСЯ обратно: на 4 см датчик отдаёт
//    примерно столько же, сколько на 30 см, и отличить эти два случая
//    принципиально невозможно. Поэтому строить на одном Sharp защиту от
//    столкновения нельзя — вплотную он рапортует «далеко». Здесь всё, что
//    выходит за 10..80 см, помечается как -1 («не знаю»), и решение оставлено
//    серверу. Для настоящего рефлекса нужен либо бампер, либо ToF-датчик.
//
// Выход датчика питается от 5 В и доходит до ~3.1 В на 6 см — это у самого
// верха шкалы АЦП, поэтому близкие препятствия упираются в насыщение. Для
// управления это не страшно (насыщение = «очень близко»), но точности там нет.

const float SHARP_CAL_A = 27.86f;   // d = A * U^B, паспортная аппроксимация
const float SHARP_CAL_B = -1.15f;
const int   SHARP_MIN_CM = 10;
const int   SHARP_MAX_CM = 80;
const int   SHARP_SAMPLES = 5;      // медиана: выбросы у Sharp это норма жизни

// Медиана нечётного числа отсчётов. Сортировка вставками на пяти элементах —
// это ~10 сравнений, дешевле любой библиотеки.
static uint32_t medianOf(uint32_t* buf, int n) {
  for (int i = 1; i < n; i++) {
    uint32_t key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }
  return buf[n / 2];
}

static int readSharpCm(int pin) {
  uint32_t samples[SHARP_SAMPLES];
  for (int i = 0; i < SHARP_SAMPLES; i++) {
    // analogReadMilliVolts применяет заводскую калибровку АЦП из eFuse —
    // заметно честнее, чем линейно пересчитывать сырые отсчёты 0..4095.
    samples[i] = analogReadMilliVolts(pin);
  }
  float volts = medianOf(samples, SHARP_SAMPLES) / 1000.0f;

  if (volts < 0.05f) return -1;   // датчик не подключён или видит пустоту

  float cm = SHARP_CAL_A * powf(volts, SHARP_CAL_B);
  if (cm < SHARP_MIN_CM || cm > SHARP_MAX_CM) return -1;
  return (int)(cm + 0.5f);
}

static void readSensors() {
  rfLeft   = readSharpCm(PIN_RF_LEFT);
  rfCenter = readSharpCm(PIN_RF_CENTER);
  rfRight  = readSharpCm(PIN_RF_RIGHT);

  uint32_t mv = analogReadMilliVolts(PIN_VBAT);
  float raw = (mv / 1000.0f) * VBAT_DIVIDER;
  // Экспоненциальное сглаживание: напряжение проседает на каждом рывке моторов,
  // и мигающая цифра на экране только мешает.
  vbat = (vbat == 0.0f) ? raw : (vbat * 0.9f + raw * 0.1f);
}


// ============================================================================
//  ПИД — ЗАГОТОВКА
// ============================================================================
//
// Сам регулятор написан целиком и рабочий. Не замкнут контур: на моторах пока
// нет энкодеров, а значит нет измеренной скорости, которую можно было бы
// подать в update(). Пока что ШИМ считается разомкнуто, из уставки напрямую.
//
// Что нужно сделать, когда энкодеры появятся:
//   1. повесить прерывания на фазы, считать тики за период;
//   2. в controlStep() посчитать реальную скорость колеса в м/с:
//        v = ticks / TICKS_PER_REV * WHEEL_CIRCUM / dt;
//   3. заменить feedforward на pidLeft.update(targetVl, measuredVl, dt),
//      оставив feedforward слагаемым — сходится заметно быстрее;
//   4. поднять MICROBOT_CLOSED_LOOP.
// Коэффициенты уже ходят по сети и лежат в телеметрии, так что подбирать их
// можно будет прямо с клиента, не перепрошивая робота.
//
// Флаг специально не компилируется сам по себе: под ним стоит обращение к
// measuredVL/measuredVR, которых пока нет. Это не забытая ошибка, а защёлка —
// включить замкнутый контур, не заведя обратную связь, нельзя.

#define MICROBOT_CLOSED_LOOP 0

struct Pid {
  float kp = 0.8f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f;
  float prevError = 0.0f;
  float outMin = -PWM_MAX, outMax = PWM_MAX;

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
  }

  float update(float target, float measured, float dt) {
    float error = target - measured;
    float p = kp * error;

    // Anti-windup: интеграл копим только если выход ещё не упёрся в предел.
    // Иначе после долгого насыщения регулятор «переедет» уставку и будет
    // отрабатывать назад — классические качели.
    float dTerm = (dt > 0.0f) ? kd * (error - prevError) / dt : 0.0f;
    float candidate = p + ki * (integral + error * dt) + dTerm;
    if (candidate > outMin && candidate < outMax) {
      integral += error * dt;
    }

    prevError = error;
    float out = p + ki * integral + dTerm;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    return out;
  }
};

Pid pidLeft, pidRight;


// ============================================================================
//  Моторы (TB6612FNG)
// ============================================================================

const int LEDC_CH_LEFT  = 0;
const int LEDC_CH_RIGHT = 1;
const int LEDC_FREQ_HZ  = 20000;   // выше слышимого, моторы не пищат
const int LEDC_BITS     = 8;       // разрешение 0..255, как PWM_MAX

static void motorsBegin() {
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY,  OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, LOW);   // силовая часть спит до конца инициализации

  // API светодиодного ШИМ переименовали в Arduino-core 3.x. Проект живёт на
  // 2.x (см. комментарий про flush() в esp32gateway), но пусть собирается и там.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_MOTOR_L_PWM, LEDC_FREQ_HZ, LEDC_BITS, LEDC_CH_LEFT);
  ledcAttachChannel(PIN_MOTOR_R_PWM, LEDC_FREQ_HZ, LEDC_BITS, LEDC_CH_RIGHT);
#else
  ledcSetup(LEDC_CH_LEFT,  LEDC_FREQ_HZ, LEDC_BITS);
  ledcSetup(LEDC_CH_RIGHT, LEDC_FREQ_HZ, LEDC_BITS);
  ledcAttachPin(PIN_MOTOR_L_PWM, LEDC_CH_LEFT);
  ledcAttachPin(PIN_MOTOR_R_PWM, LEDC_CH_RIGHT);
#endif
}

static void ledcWritePwm(int channel, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // В 3.x канал адресуется через пин, к которому он привязан.
  ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// pwm в диапазоне [-255, 255]: знак — направление, модуль — заполнение.
// Ноль означает мягкий выбег (IN1=IN2=0), а не электрическое торможение:
// для лёгкого робота резкий брейк на каждой остановке лишний.
static void motorWrite(int in1, int in2, int channel, int pwm) {
  if (pwm > PWM_MAX)  pwm = PWM_MAX;
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWritePwm(channel, abs(pwm));
}

static void motorsStop() {
  pwmLeft = 0;
  pwmRight = 0;
  motorWrite(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, LEDC_CH_LEFT,  0);
  motorWrite(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, LEDC_CH_RIGHT, 0);
  pidLeft.reset();
  pidRight.reset();
}


// ============================================================================
//  Кинематика и управляющий цикл
// ============================================================================

// Компенсация мёртвой зоны: мотор трогается только с некоторого ШИМ, поэтому
// всё ненулевое поднимаем до PWM_MIN_MOVE. Без этого маленькие уставки просто
// греют обмотку, а робот стоит.
static int applyDeadband(float pwm) {
  int v = (int)(pwm + (pwm >= 0 ? 0.5f : -0.5f));
  if (v == 0) return 0;
  if (abs(v) < PWM_MIN_MOVE) v = (v > 0) ? PWM_MIN_MOVE : -PWM_MIN_MOVE;
  return v;
}

static void controlStep(float dt) {
  // Дедман. Проверяется здесь, а не в разборе команд: молчание — это тоже
  // событие, и заметить его может только тот, кто тикает по таймеру.
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    targetV = 0.0f;
    targetW = 0.0f;
  }

  if (!armed) {
    digitalWrite(PIN_MOTOR_STBY, LOW);
    motorsStop();
    return;
  }
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  // Дифференциальная раскладка twist на колёса — та же, что у yarp-13.
  float vL = targetV - targetW * WHEEL_BASE_M / 2.0f;
  float vR = targetV + targetW * WHEEL_BASE_M / 2.0f;

#if MICROBOT_CLOSED_LOOP
  // Сюда подставить измеренную энкодерами скорость (см. раздел ПИД).
  float outL = pidLeft.update(vL, measuredVL, dt);
  float outR = pidRight.update(vR, measuredVR, dt);
#else
  (void)dt;
  // Разомкнутый контур: скорость колеса считаем линейной по заполнению ШИМ.
  float outL = vL / MAX_WHEEL_SPEED * PWM_MAX;
  float outR = vR / MAX_WHEEL_SPEED * PWM_MAX;
#endif

  // Если одно колесо упёрлось в потолок, масштабируем ОБА, сохраняя отношение.
  // Иначе на быстрой дуге насыщается внешнее колесо, соотношение скоростей
  // ломается, и робот уезжает не по той кривой, которую заказали.
  float peak = max(fabsf(outL), fabsf(outR));
  if (peak > PWM_MAX) {
    outL *= PWM_MAX / peak;
    outR *= PWM_MAX / peak;
  }

  pwmLeft  = applyDeadband(outL);
  pwmRight = applyDeadband(outR);

  motorWrite(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, LEDC_CH_LEFT,  pwmLeft);
  motorWrite(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, LEDC_CH_RIGHT, pwmRight);
}


// ============================================================================
//  Разбор команд
// ============================================================================

// Читает до n чисел, разделённых пробелами, начиная с позиции after.
// Возвращает, сколько удалось прочитать.
static int parseFloats(const String& s, int from, float* out, int n) {
  int count = 0;
  int i = from;
  while (count < n && i < (int)s.length()) {
    while (i < (int)s.length() && s[i] == ' ') i++;
    if (i >= (int)s.length()) break;
    int j = i;
    while (j < (int)s.length() && s[j] != ' ') j++;
    out[count++] = s.substring(i, j).toFloat();
    i = j;
  }
  return count;
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String verb = (sp < 0) ? line : line.substring(0, sp);
  verb.toUpperCase();
  int argsAt = (sp < 0) ? line.length() : sp + 1;

  if (verb == "TWIST") {
    float a[2] = {0.0f, 0.0f};
    if (parseFloats(line, argsAt, a, 2) == 2) {
      targetV = a[0];
      targetW = a[1];
      lastCmdMs = millis();
    }

  } else if (verb == "STOP") {
    targetV = 0.0f;
    targetW = 0.0f;
    lastCmdMs = millis();
    motorsStop();

  } else if (verb == "PID") {
    float a[3];
    if (parseFloats(line, argsAt, a, 3) == 3) {
      pidLeft.kp = pidRight.kp = a[0];
      pidLeft.ki = pidRight.ki = a[1];
      pidLeft.kd = pidRight.kd = a[2];
      // Накопленный интеграл считался по старым коэффициентам — с новыми он
      // означает уже не то же самое и даст рывок. Сбрасываем.
      pidLeft.reset();
      pidRight.reset();
    }

  } else if (verb == "ARM") {
    float a[1];
    if (parseFloats(line, argsAt, a, 1) == 1) {
      armed = (a[0] != 0.0f);
      if (!armed) motorsStop();
    }

  } else if (verb == "PING") {
    if (tcpClient && tcpClient.connected()) tcpClient.println("PONG");
  }
  // Неизвестная команда молча игнорируется: на общей линии мусор случается,
  // и падать из-за него робот не должен.
}

static void sendTelemetry() {
  if (!tcpClient || !tcpClient.connected()) return;

  char buf[224];
  snprintf(buf, sizeof(buf),
           "t=%lu,rf_l=%d,rf_c=%d,rf_r=%d,vbat=%.2f,v=%.3f,w=%.3f,"
           "pwm_l=%d,pwm_r=%d,armed=%d,kp=%.3f,ki=%.3f,kd=%.3f",
           (unsigned long)millis(), rfLeft, rfCenter, rfRight, vbat,
           targetV, targetW, pwmLeft, pwmRight, armed ? 1 : 0,
           pidLeft.kp, pidLeft.ki, pidLeft.kd);
  tcpClient.println(buf);
}


// ============================================================================
//  Кнопки
// ============================================================================

// Конструктор, а не инициализация по месту: ESP32-core 2.x собирает скетчи как
// C++11, где структура с умолчаниями у полей перестаёт быть агрегатом, и
// запись вида Button b{PIN} просто не компилируется.
struct Button {
  uint8_t pin;
  bool lastLevel;             // HIGH = отпущена (подтяжка к питанию)
  uint32_t lastChangeMs;
  bool pressedEdge;

  explicit Button(uint8_t p)
    : pin(p), lastLevel(true), lastChangeMs(0), pressedEdge(false) {}
};

Button btnUp(PIN_BTN_UP), btnDown(PIN_BTN_DOWN), btnOk(PIN_BTN_OK), btnBack(PIN_BTN_BACK);

const uint32_t BTN_DEBOUNCE_MS = 30;

static void pollButton(Button& b) {
  b.pressedEdge = false;
  bool level = digitalRead(b.pin);
  uint32_t now = millis();
  if (level != b.lastLevel && now - b.lastChangeMs > BTN_DEBOUNCE_MS) {
    b.lastChangeMs = now;
    b.lastLevel = level;
    if (level == LOW) b.pressedEdge = true;   // нажатие — это фронт вниз
  }
}

static void pollButtons() {
  pollButton(btnUp);
  pollButton(btnDown);
  pollButton(btnOk);
  pollButton(btnBack);
}


// ============================================================================
//  Сеть
// ============================================================================

static void connectWifi(int index, bool showProgress);

static void renderConnecting(const char* ssid, int dots) {
  if (!displayPresent) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Connecting..."));
  display.println(ssid);
  display.setCursor(0, 24);
  for (int i = 0; i < dots; i++) display.print('.');
  display.display();
}

static void connectWifi(int index, bool showProgress) {
  if (index < 0 || index >= WIFI_NETWORK_COUNT) return;
  currentNetwork = index;

  motorsStop();          // на время переподключения робот стоит
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  // Энергосбережение радио выключено по той же причине, что и в esp32gateway:
  // засыпающий модем добавляет задержки и рвёт соединение, а у нас по этому
  // каналу едет управление в реальном времени.
  WiFi.setSleep(false);
  WiFi.begin(WIFI_NETWORKS[index].ssid, WIFI_NETWORKS[index].pass);

  uint32_t start = millis();
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);   // единственное место с delay: робот ещё не управляется
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    if (showProgress) renderConnecting(WIFI_NETWORKS[index].ssid, (++dots) % 16);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_LED, HIGH);
    Serial.printf("\nConnected to %s, IP %s\n",
                  WIFI_NETWORKS[index].ssid, WiFi.localIP().toString().c_str());
    prefs.putInt("net", index);
    // end() перед begin() обязателен: после переподключения к WiFi слушающий
    // сокет остаётся привязан к СТАРОМУ адресу, а begin() у уже «слушающего»
    // сервера просто выходит, и робот навсегда перестаёт принимать соединения.
    tcpServer.end();
    tcpServer.begin();
  } else {
    digitalWrite(PIN_LED, LOW);
    Serial.printf("\nFailed to connect to %s\n", WIFI_NETWORKS[index].ssid);
    // Не достучались — показываем меню, пусть человек выберет другую сеть.
    uiScreen = SCREEN_MENU;
    menuCursor = index;
  }
}

// Накопитель незавершённой строки. Вынесен из функции, потому что его нужно
// очищать при подключении нового клиента: обрывок команды от прошлой сессии
// иначе приклеится к первой команде следующей и испортит её разбор.
String rxBuffer;

static void serviceTcp() {
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient incoming = tcpServer.available();
    if (incoming) {
      tcpClient = incoming;
      tcpClient.setNoDelay(true);   // низкая задержка важнее утилизации канала
      Serial.println("Client connected");
      rxBuffer = "";
      // Уставку с прошлой сессии не наследуем: новый клиент не заказывал
      // движение, а робот бы поехал.
      targetV = 0.0f;
      targetW = 0.0f;
      lastCmdMs = millis();
    }
    return;
  }

  // Строки читаем побайтно и накапливаем: readStringUntil() блокирует loop()
  // на таймаут, а нам нельзя — управляющий цикл встанет вместе с ним.
  while (tcpClient.available()) {
    char c = tcpClient.read();
    if (c == '\n') {
      handleLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      if (rxBuffer.length() < 128) rxBuffer += c;   // защита от бесконечной строки
      else rxBuffer = "";
    }
  }
}


// ============================================================================
//  Отрисовка
// ============================================================================

static void renderStatus() {
  display.clearDisplay();
  display.setCursor(0, 0);

  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  bool linkUp = tcpClient && tcpClient.connected();

  display.print(WIFI_NETWORKS[currentNetwork].ssid);
  display.println(wifiUp ? F(" +") : F(" -"));
  display.println(wifiUp ? WiFi.localIP().toString() : String("no ip"));

  display.print(linkUp ? F("LINK ") : F("---- "));
  display.println(armed ? F("ARMED") : F("SAFE"));

  if (SCREEN_HEIGHT >= 64) {
    display.printf("BAT %.2fV\n", vbat);
    display.printf("L%3d C%3d R%3d\n", rfLeft, rfCenter, rfRight);
    display.printf("v%.2f w%.2f\n", targetV, targetW);
  }
  display.display();
}

static void renderMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Select network:"));
  int rows = (SCREEN_HEIGHT >= 64) ? 6 : 3;
  // Окно прокрутки: курсор держим внутри видимых строк, чтобы список длиннее
  // экрана всё равно листался.
  int first = menuCursor - rows / 2;
  if (first < 0) first = 0;
  if (first > WIFI_NETWORK_COUNT - rows) first = WIFI_NETWORK_COUNT - rows;
  if (first < 0) first = 0;

  for (int i = first; i < WIFI_NETWORK_COUNT && i < first + rows; i++) {
    display.print(i == menuCursor ? F(">") : F(" "));
    display.print(i == currentNetwork ? F("*") : F(" "));
    display.println(WIFI_NETWORKS[i].ssid);
  }
  display.display();
}

static void renderUi() {
  if (!displayPresent) return;
  if (uiScreen == SCREEN_MENU) renderMenu();
  else                         renderStatus();
}

static void handleUiButtons() {
  if (uiScreen == SCREEN_STATUS) {
    if (btnUp.pressedEdge || btnOk.pressedEdge) {
      uiScreen = SCREEN_MENU;
      menuCursor = currentNetwork;
    }
    if (btnBack.pressedEdge) {
      // Локальный «выключатель силы». Полезно, когда робот на столе и его
      // трогают руками, а сервер продолжает слать уставки.
      armed = !armed;
      if (!armed) motorsStop();
    }
  } else {  // SCREEN_MENU
    if (btnUp.pressedEdge   && menuCursor > 0)                      menuCursor--;
    if (btnDown.pressedEdge && menuCursor < WIFI_NETWORK_COUNT - 1) menuCursor++;
    if (btnOk.pressedEdge) {
      uiScreen = SCREEN_STATUS;
      connectWifi(menuCursor, true);
    }
    if (btnBack.pressedEdge) uiScreen = SCREEN_STATUS;
  }
}


// ============================================================================
//  setup / loop
// ============================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_OK,   INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);

  // Полная шкала АЦП: выход Sharp доходит до ~3.1 В, делитель батареи до ~2.7 В.
  analogSetPinAttenuation(PIN_RF_LEFT,   ADC_11db);
  analogSetPinAttenuation(PIN_RF_CENTER, ADC_11db);
  analogSetPinAttenuation(PIN_RF_RIGHT,  ADC_11db);
  analogSetPinAttenuation(PIN_VBAT,      ADC_11db);

  motorsBegin();
  motorsStop();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  displayPresent = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (displayPresent) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.display();
  } else {
    // Экран — это удобство, а не условие работы. Без него робот всё равно
    // поднимется на сохранённой сети и будет управляем.
    Serial.println("SSD1306 not found, running headless");
  }

  prefs.begin("microbot", false);
  int saved = prefs.getInt("net", 0);
  if (saved < 0 || saved >= WIFI_NETWORK_COUNT) saved = 0;

  lastCmdMs = millis();
  connectWifi(saved, true);
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastControl = 0, lastSensors = 0, lastTelemetry = 0, lastDisplay = 0;

  pollButtons();
  handleUiButtons();

  // Сеть отвалилась — переподключаемся к той же сети. Моторы при этом не
  // трогаем явно: дедман в controlStep() обнулит уставку сам, потому что
  // команды перестанут приходить.
  if (WiFi.status() != WL_CONNECTED && uiScreen != SCREEN_MENU) {
    static uint32_t lastRetry = 0;
    if (now - lastRetry > 5000) {
      lastRetry = now;
      digitalWrite(PIN_LED, LOW);
      connectWifi(currentNetwork, false);
    }
  } else {
    serviceTcp();
  }

  if (now - lastSensors >= SENSOR_PERIOD_MS) {
    lastSensors = now;
    readSensors();
  }

  if (now - lastControl >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControl) / 1000.0f;
    lastControl = now;
    controlStep(dt);
  }

  if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
    lastTelemetry = now;
    sendTelemetry();
  }

  if (now - lastDisplay >= DISPLAY_PERIOD_MS) {
    lastDisplay = now;
    renderUi();
  }

  yield();   // отдаём процессор стеку WiFi и системному watchdog
}
