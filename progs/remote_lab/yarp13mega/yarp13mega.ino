/*
 * YARP13
 * Arduino Mega
 *
 * input:  msg_yy::cmd, geometry_msgs::Twist
 * output: msg_yy::sens
 * Motion, Serv, Encoders
 *
 * Подключение по i2c:
 * компасы и сервер УЗД
 * 
 * -- Магнитометр на основе TQM5883/AltIMU-10
 * -- Гироскопический компас GY-521 MPU6050 (9 Axis Gyroscope) + DMP (Цифровой процессор движения, Digital Motion Processor)
 *    GY-521 инициализируется очень медленно (до 10 с.)
 *
 * ExtIOStream (Serial1) (9600)  -- подключение сервера лазерных дальномеров VL53L0X
 * LogStream (Serial)  (9600)    -- порт для общения с внешним миром (отладка)
 * ROSStream (Serial2) (57600)   -- режим pseudo-ROS 
 * 
 * 06.03.2021, 20.12.2023, 21.02.2024, 24.07.2024, 23.01.2025
 * 
 * V 2.7
 *
 * LP 21.04.2025
 */

/**
 * Имя, определяющее режим эмуляции rosserial 
 */
//#define _PSEUDO_ROS_
//#define _TEST_MOTORS_

#include <ros.h>
#include <ros/time.h>

#include <msg_yy/sens.h>
#include <msg_yy/cmd.h>
#include <geometry_msgs/Twist.h>

#include <Servo.h>
#include <Encoder.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "robofob.h"
#include "i2cwctl.h"

#include "yfunc.h"
#include "y13cmd.h"
#include "compass.h"
#include "vl53l0x.h"

#include "pidlib.h"

#define ExtIOStream Serial1   // Связь с внешним миром
#define LogStream   Serial    // Диагностика

#ifdef _PSEUDO_ROS_
  #define ROSStream   Serial2
#else
  #define ROSStream   Serial
#endif


/**
 *  Подключение эмулятора и замена области имен
 */
#include <psros.h>
#ifdef _PSEUDO_ROS_
#define ros pseudoros
#endif

const char* Title = "Y-13 2.7";
int _RID_ = 1;
char *TOPIC_YY_SENSORS = "yy_sensors";
char *TOPIC_CMD_VEL    = "cmd_vel";
char *TOPIC_YY_COMMAND = "yy_command";

//-------------------------------------------------------------------
// Период опроса датчиков, мс.
// Он же - квант времени для ПИД-регулятора.
// Энкодерные скорости отсчитываются от него.
// Может зависеть от используемого компаса (некоторые очень медленные).
// На самом деле эффективная частота составляет порядка 10 Гц.
long TTT_Sensors = 50;

//-------------------------------------------------------------------
//
// Компас №1 на основе магнитометра
//
#define _TQM5883Compass_   // Для YARP 13.1, 13.3, 13.4 (YARP-6)
//#define _TALTIMUCompass_ // Для YARP 13.2

#ifdef _TQM5883Compass_
  TQM5883Compass compass_mag(1);
#endif

#ifdef _TALTIMUCompass_
  TALTIMUCompass compass_mag(1);
#endif

//
// Компас №2 на основе гироскопа
//
const int8_t PIN_COMP_MPU6050_INTERRUPT = -1; // Нога прерывания компаса. Если <0, то прерывание не используется

TMPU6050DMPCompass compass_gyro;

// Ноги индикаторов
#define PIN_DIAG_RECV_EVENT PIN_LED1
#define PIN_DIAG_SEND_DATA  PIN_LED2
#define PIN_DIAG_MAIN_LOOP  PIN_LED3

//-------------------------------------------------------------------
// Set the LCD address to 0x20 or 0x27 for a 16 chars and 2 line display
// Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
// Ноги A4 (SDA) и A5 (SCL) заняты
// Обратите внимание на адрес устройства - 0x20, 0x27, 0x3f
//byte I2CADDR = 0x3f;
byte I2CADDR = 0x27;
LiquidCrystal_I2C lcd(I2CADDR,16,2);

//-------------------------------------------------------------------
// То, что касается RC5/IRA
//-------------------------------------------------------------------
namespace RC5IRA
{
  unsigned long RxData[RC5DSRV::NUMRECV] = {0,0,0,0}; // Прнинимаемые данные
  unsigned long TxCode = 0x10101000; // Передаваемый код RC5
  byte numdigits = 32; // Количество разрядов в пакете RC5 (не более 32)
  byte freq = 5;       // Частота генерации пакетов, Гц 
};
//-------------------------------------------------------------------

// Параметры робота, собранные воедино для внешнего доступа
struct TParams
{
  TParams():Kp0(0.5),Ki0(0.02),Kd0(0.2), 
            refl_min_voltage(11.0), calibr_rot_speed(40), calibr_max_cnt(600),
            ratio_left(1.0), ratio_right(1.0), KLPF(1.0), usePIDCtl(true) { }
  float Kp0, Ki0, Kd0;     // Параметры ПИД-ругулятора
  float refl_min_voltage;  // Рефлекс. Минимальное бортовое напряжение
  float calibr_rot_speed;  // Скорость вращения при калибровке
  float calibr_max_cnt;    // Количество тактов калибровки
  // Конечные масштабные множители для согласования характеристик двигателей
  float ratio_left;
  float ratio_right;
  float KLPF;              // Коэффициент ФНЧ для скоростей движения
  bool usePIDCtl;          // Режим управления: ПИД или напрямую
};

TParams rconfig;

//-------------------------------------------------------------------
//  Hardware
//-------------------------------------------------------------------

// Servo
#define SRV_NUM 3
byte PIN_SRV[SRV_NUM] = {14, 15, 16};

//
// Компас
//
int calibr_step = 0;      // Текущий шаг калибровки

//-------------------------------------------------------------------

//ros::NodeHandle  nh;
//ros::NodeHandle_<HardwareType, MAX_PUBLISHERS=25, MAX_SUBSCRIBERS=25, IN_BUFFER_SIZE=1024, OUT_BUFFER_SIZE=512> nh;
ros::NodeHandle_<ArduinoHardware, 2, 2, 512, 1024> nh;

const int8_t sensors_msg_adc_length = 10;
const int8_t sensors_msg_data_length = 8;
msg_yy::sens sensors_msg;

//
// Статус
//
#define STAT_NO_CMD     0x0001
#define STAT_POWER      0x0002
#define STAT_RF_LEFT    0x0004
#define STAT_RF_RIGHT   0x0008

ros::Publisher pub_sensors(TOPIC_YY_SENSORS, &sensors_msg);

int CMD_CNT = 0;
float acc_voltage = 0;        // Бортовое напряжение
int REFLEX_DISTANCE[3] = {20, 20, 20};    // Рефлекс. Минимальное расстояние до препятствия, см. Центральный, левый, правый

long Pred_cmd_time = 0;           // Время последнего прихода команды
const long Wait_cmd_time = 2000;  // Рефлекс. Сколько времени ожидать команду, мс

// Servo angles
int angles[SRV_NUM] = {90, 90, 90};
Servo *srv[SRV_NUM];

//-------------------------------------------------------------------
//
// Датчики УЗД
//
//-------------------------------------------------------------------

I2CUSonicServerM8::USonicData ussData;

/// Получение данных от сервера
int ReadUSSData(byte addr, unsigned int size, byte buff[])
{
  Wire.requestFrom(addr, size);  // request size bytes from slave device #addr
  memset(buff, 0, size);
  byte n = 0;
  while(Wire.available())        // slave may send less than requested
  {
    buff[n]= Wire.read();
    if(n<size)
      n++;
  }
  return n;
}

//-------------------------------------------------------------------
//
// Датчики vl53l0x
// Показания лежат в vl53::Distances[vl53::MAX_NUM_LOX]
//
//-------------------------------------------------------------------

// Коэффициент ФНЧ для датчиков
float K_SHARP = 0.9;
// Передние дальномеры
TSharp SharpFwdCenter(K_SHARP);
TSharp SharpFwdLeft(K_SHARP);
TSharp SharpFwdRight(K_SHARP);

// Боковые дальномеры
TSharp SharpSide_left_fwd(K_SHARP);
TSharp SharpSide_right_fwd(K_SHARP);
TSharp SharpSide_left_bck(K_SHARP);
TSharp SharpSide_right_bck(K_SHARP);
// Задний
TSharp SharpBckCenter(K_SHARP);

// Command, Joystisk Buttons etc
#define ARG_NUM 3
float args[ARG_NUM] = {0, 0, 0};
unsigned long arg_da[4] = {0, 0, 0, 0};
int Command = 0;

byte SWITCHES[SWITCHES_NUM];
// Режимы
// Включение/отключение рефлекторных реакций
// SWITCHES[0] - питание
// SWITCHES[1] - препятствия
// SWITCHES[2] - потеря связи
// SWITCHES[3] - автокалибровка компаса
// SWITCHES[4] - использовать дальномеры Sharp/Ultrasonic
// SWITCHES[5] - выбор типа компаса - магнитного или гироскопического
// SWITCHES[6] - сброс ПИД
// SWITCHES[7] - резерв

#define SW_POWER           0 // питание
#define SW_OBSTACLE        1 // препятствия
#define SW_CONNECT         2 // потеря связи
#define SW_COMPASS_CALIBR  3 // автокалибровка компаса
#define SW_SHARP           4 // использовать дальномеры Sharp/Ultrasonic
#define SW_COMPASS_MAG     5 // использовать магнитный компас
#define SW_RESET_PID       6 // Сбрасывать ПИД-регуляторы при изменении задающей величины

void LogPrint(char *s)
{
#ifdef _PSEUDO_ROS_
  LogStream.print(s);
#endif  
}
void LogPrintLn(char *s)
{
#ifdef _PSEUDO_ROS_
  LogStream.println(s);
#endif  
}

void messageCbCmd(const msg_yy::cmd &msg)
{
  //LogPrintLn(msg.getType());
  LogPrintLn("cmd");
  digitalWrite(PIN_DIAG_RECV_EVENT, HIGH-digitalRead(PIN_DIAG_RECV_EVENT));   // blink the led
  CMD_CNT++;
  Command = msg.command;
  
  byte n;
  n = (msg.arg_length>3) ? 3: msg.arg_length;
  for(byte i=0;i<n;i++)
    args[i] = msg.arg[i];
 
  n = (msg.angle_length>3) ? 3 : msg.angle_length;
  for(byte i=0;i<n;i++)
    angles[i] = msg.angle[i];
  arg_da[0] = msg.da[0];
  arg_da[1] = msg.da[1];
  arg_da[2] = msg.da[2];
  arg_da[3] = msg.da[3];

  Pred_cmd_time = millis();
}

//-------------------------------------------------------------------
// Геометрия
//-------------------------------------------------------------------
const float MAX_PWM = 250.0;    // Максимальное подаваемое значение ШИМ
const float MAX_RPM =  76.0;    // Максимальная частота вращения, об/мин

// *** Геометрия колесной базы
const float wheel_rad = 0.0325; // Радиус колеса, м
const float wheel_sep = 0.17;   // Расстояние между колесами, м

//-------------------------------------------------------------------

const float MAX_W_FREQ = MAX_RPM/60;              // Максимальная частота вращения, Гц
const float MAX_LIN_SPEED = MAX_W_FREQ*wheel_rad; // Максимальная линейная скорость, м/с

//
// Скорости вращения колес, -255..255
// Задаются как функции от speed_lin, speed_ang, так и напрямую
// Эти скорости отрабатываются ПИД-регулятором
float w_r = 0, w_l = 0;

// Линейная и угловая скорости, [-1..+1]
float _speed_lin = 0;
float _speed_ang = 0;

void messageCbVel(const geometry_msgs::Twist &msg)
{
  //LogPrintLn(msg.getType());
  LogPrintLn("vel");
  static float pred_w_l = 0;
  static float pred_w_r = 0;

  rconfig.usePIDCtl = true;

  digitalWrite(PIN_DIAG_RECV_EVENT, HIGH-digitalRead(PIN_DIAG_RECV_EVENT));   // blink the led

  // Низкочастотная фильтрация
  _speed_ang = (1.0-rconfig.KLPF)*_speed_ang + rconfig.KLPF*msg.angular.z;
  _speed_lin = (1.0-rconfig.KLPF)*_speed_lin + rconfig.KLPF*msg.linear.x;

  // Если сработали рефлексы, то не будем реагировать на команды управления движением
  if((sensors_msg.status & STAT_POWER)) return;
  /* Раньше было так, но, видимо, так некорректно
  if((sensors_msg.status & STAT_RF_LEFT) || (sensors_msg.status & STAT_RF_RIGHT) ||
     (sensors_msg.status & STAT_POWER)) return;
  */

  // Предполагаем, что speed_lin - величина от -1 до 1
  // Частота вращения, Гц
  float freq_l = (_speed_lin*MAX_LIN_SPEED/wheel_rad) - ((_speed_ang*wheel_sep)/(2.0*wheel_rad));
  float freq_r = (_speed_lin*MAX_LIN_SPEED/wheel_rad) + ((_speed_ang*wheel_sep)/(2.0*wheel_rad));

  w_l = fmap(freq_l, -MAX_W_FREQ, MAX_W_FREQ, -MAX_PWM, MAX_PWM);
  w_r = fmap(freq_r, -MAX_W_FREQ, MAX_W_FREQ, -MAX_PWM, MAX_PWM);

  // Вновь рефлексы. Проверка на возможность движения
  if((sensors_msg.status & STAT_RF_LEFT) && w_l > 0) w_l = 0;
  if((sensors_msg.status & STAT_RF_RIGHT) && w_r > 0) w_r = 0;

  if(w_l>MAX_PWM) w_l = MAX_PWM;
  if(w_l<-MAX_PWM) w_l = -MAX_PWM;
  if(w_r>MAX_PWM) w_r = MAX_PWM;
  if(w_r<-MAX_PWM) w_r = -MAX_PWM;

  // Попробуем сбросить ПИД-регуляторы
  // Сбрасывать будем, если что-то поменялось, т.к. команды на робота идут постоянно
  if((pred_w_l!=w_l || pred_w_r!=w_r) && SWITCHES[SW_RESET_PID])
  {
    pred_w_l = w_l;
    pred_w_r = w_r;
    PID_LEFT.Reset();
    PID_RIGHT.Reset();
  }

  Pred_cmd_time = millis();
}

//-------------------------------------------------------------------

ros::Subscriber<geometry_msgs::Twist> sub_vel(TOPIC_CMD_VEL, messageCbVel);
ros::Subscriber<msg_yy::cmd> sub_cmd(TOPIC_YY_COMMAND, messageCbCmd);

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------

void setup()
{
#ifdef _PSEUDO_ROS_
  // Порт для общения с внешним миром (отладка)
  LogStream.begin(9600);
  LogPrint(Title);
  LogPrintLn(" (pseudoROS)");
#endif

  Wire.begin();
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.home();

  pinMode(PIN_DIAG_RECV_EVENT, OUTPUT);
  pinMode(PIN_DIAG_SEND_DATA,  OUTPUT);
  pinMode(PIN_DIAG_MAIN_LOOP,  OUTPUT);

  pinMode(IND_REFL_POWER, OUTPUT);
  pinMode(IND_OBST_LEFT, OUTPUT);
  pinMode(IND_OBST_RIGHT, OUTPUT);
  pinMode(IND_OBST_CENTER, OUTPUT);

  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_GUN, OUTPUT);

  // Бамперы
  pinMode(PIN_BUMP_L, INPUT_PULLUP);
  pinMode(PIN_BUMP_C, INPUT_PULLUP);
  pinMode(PIN_BUMP_R, INPUT_PULLUP);

  // Переключатели
  for(byte i=0;i<SWITCHES_NUM;i++)
    pinMode(PIN_SWITCHES[i], INPUT_PULLUP);

  Motors_init();

  PID_LEFT.Init(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0, -MAX_PWM, MAX_PWM);
  PID_RIGHT.Init(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0, -MAX_PWM, MAX_PWM);

  // Максимальная "энкодерная скорость"
  // Определяется в ходе экспериментов TestMotors (значение выводится на lcd)
  pidlib::MAX_ENC_SPEED = 350.0;
  /* Test Motors */
#ifdef _TEST_MOTORS_
  int EncSpeed;
  while(1)
  {
    LogStream.begin(9600);
    
    lcd.setCursor(0,0);
    lcd.print("Test motors 50% ");
    LogStream.println("Test motors 50% ");
    
    EncSpeed = TestMotors(MAX_PWM/2, TTT_Sensors);
    lcd.setCursor(0,1);
    lcd.print(EncSpeed);
    lcd.print("   ");
    LogStream.println(EncSpeed);
    Beep(1);

    lcd.setCursor(0,0);
    lcd.print("Test motors 100%");
    LogStream.println("Test motors 100% ");
    EncSpeed = TestMotors(MAX_PWM, TTT_Sensors);
    lcd.setCursor(0,1);
    lcd.print(EncSpeed);
    lcd.print("   ");
    LogStream.println(EncSpeed);
    Beep(1);
  }
#endif

  // Инициализация полей сообщений
  memset(sensors_msg.data, 0, sizeof(sensors_msg.data));
  
  // Сервомашинки
  for(byte n=0;n<SRV_NUM;n++)
  {
    srv[n] = new Servo;
    srv[n]->attach(PIN_SRV[n]);
  }

  // Переключатели
  for(byte i=0;i<SWITCHES_NUM;i++)
    SWITCHES[i] = !digitalRead(PIN_SWITCHES[i]);

  // Режим калибровки
  if(SWITCHES[SW_COMPASS_CALIBR]) calibr_step = 0; else calibr_step = rconfig.calibr_max_cnt;

  lcd.print(Title);
#ifdef _PSEUDO_ROS_
  lcd.print(" *");
#endif

  // *** Соединяемся, настраиваемся
  lcd.setCursor(10,0);
  lcd.print(" Wait");

  /**
   * 
   * Настройка порта. Отличие в возможности указания самого порта и скорости
   */
#ifdef _PSEUDO_ROS_
  nh.initNode(&ROSStream, 57600);
#else
  nh.initNode();
#endif

  nh.advertise(pub_sensors);

  nh.subscribe(sub_cmd);
  nh.subscribe(sub_vel);

  LogPrintLn("Wait for connection...");
#ifdef _PSEUDO_ROS_
  nh.WaitForConnection();
#else
  while (!nh.connected())
    nh.spinOnce();

  //if(!nh.getParam("rid", &_RID_)) _RID_ = 1;
    
#endif
  LogPrintLn("Ok.");

  //ros::Rate rate(100); // ROS Rate at 100Hz  

  // Генерация сигналов RC5 маяком
  RC5DSRV::SetBeacon(RC5DSRV::CMD_GEN_RC5, RC5IRA::TxCode, RC5IRA::numdigits, RC5IRA::freq);

  //
  // Инициализация компасов
  //
  lcd.setCursor(10,0);
  lcd.print(" Cm..");

  LogPrintLn("Init compass...");

  compass_mag.begin(); // На основе магнитометра
  compass_gyro.begin(PIN_COMP_MPU6050_INTERRUPT);
  
  // Инициализация датчиков VL53L0X
  lcd.print(" vl53..");
  LogPrintLn("Init VL53L0X...");

  ExtIOStream.begin(9600);
  vl53::ClientInit(ExtIOStream);

  LogPrintLn("Ok");

  //
  // Готово
  //
  Beep(1);

  lcd.setCursor(10,0);
  lcd.print("R    ");
  Pred_cmd_time = millis();
}

char sout[60];

void ShowStatus(void)
{
  static byte tcnt = 0;
  static int16_t pred_status = 0xffffffff;
  static float pred_volt = -1;
  static int16_t pred_compass = -1;

  tcnt++;
  if(tcnt>10)
  {
    tcnt = 0;
  }
  else return;

  if((int)(pred_volt*10)!=(int)(acc_voltage*10))
  {
    pred_volt=acc_voltage;
    dtostrf(acc_voltage, 4, 1, sout);
    lcd.setCursor(0, 1);
    lcd.print(sout);
  }
  if(pred_status!=sensors_msg.status)
  {
    pred_status=sensors_msg.status;
    for(byte i = 0; i < 8; i++) 
      sout[7-i] = (sensors_msg.status & (1 << i)) ? '1' : '-';
    sout[8] = 0;
    lcd.setCursor(5, 1);
    lcd.print(sout);
  }
  if(pred_compass!=sensors_msg.compass)
  {
    pred_compass = sensors_msg.compass;
    sprintf(sout, "%3d", sensors_msg.compass);
    lcd.setCursor(13, 0);
    lcd.print(sout);
  }
}

// Текущие энекодерные скорости
long speed_left = 0;
long speed_right = 0;

int spinfunc(void) { return nh.spinOnce(); }

void loop()
{
  static long range_timer = 0;
  static long pred_enc_left = 0;
  static long pred_enc_right = 0;

  // Publish the range value every TTT_Sensors milliseconds
  // since it takes that long for the sensor to stabilize
  if((millis()-range_timer) > TTT_Sensors)
  {
    //range_timer = millis();
    //
    // *** Работа с сенсорами
    //
    sensors_msg.tm = nh.now();

    for(byte n=0;n<sensors_msg_adc_length;n++)
      sensors_msg.adc[n] = analogRead(A0+n);

    //
    // Переключатели
    //
    for(byte n=0;n<SWITCHES_NUM;n++)
    {
      SWITCHES[n] = !digitalRead(PIN_SWITCHES[n]);
      sensors_msg.switches[n] = SWITCHES[n];
    }
    //
    // Энкодеры
    //
    sensors_msg.enc_left = encoder_left.read();
    sensors_msg.enc_right = encoder_right.read();

    //
    // Компасы
    //
    if(SWITCHES[SW_COMPASS_MAG])  // На основе магнитометра
    {
      sensors_msg.compass = compass_mag.read();
      compass_mag.read_pitch_roll(sensors_msg.cpitch, sensors_msg.croll);
    }
    else  // На основе гироскопа
    {
      sensors_msg.compass = compass_gyro.read();
      compass_gyro.read_pitch_roll(sensors_msg.cpitch, sensors_msg.croll);
    }

    // Напряжение питания
    acc_voltage = fmap(sensors_msg.adc[0], 0, 1023, 0, 25);
    sensors_msg.acc_voltage = acc_voltage;

    //
    // УЗД/VL53L0X
    //
    ReadUSSData(I2CUSonicServerM8::ADDR, sizeof(ussData.rawdata), ussData.rawdata);
    /* sic
    for(byte n=0;n<I2CUSonicServerM8::USONIC_NUM;n++)
      sensors_msg.uss[n] = ussData.dist[n];
    */

    //
    // Дальномеры
    //
    int rf_center, rf_left, rf_right;
    int rf_side_left_fwd, rf_side_right_fwd, rf_side_left_bck, rf_side_right_bck, rf_bck_center;
    if(SWITCHES[SW_SHARP])
    {
      //
      // Sharp
      //
      #define MAX_SHARP_DIST 80
      // *** Передние
      rf_center = constrain(SharpFwdCenter.GetDist(sensors_msg.adc[3]), 0, MAX_SHARP_DIST);
      rf_left   = constrain(SharpFwdLeft.GetDist(sensors_msg.adc[2]), 0, MAX_SHARP_DIST);
      rf_right  = constrain(SharpFwdRight.GetDist(sensors_msg.adc[4]), 0, MAX_SHARP_DIST);
      // *** Боковые
      rf_side_left_fwd  = constrain(SharpSide_left_fwd.GetDist(sensors_msg.adc[1]), 0, MAX_SHARP_DIST);
      rf_side_right_fwd = constrain(SharpSide_right_fwd.GetDist(sensors_msg.adc[5]), 0, MAX_SHARP_DIST);
      rf_side_left_bck  = constrain(SharpSide_left_bck.GetDist(sensors_msg.adc[6]), 0, MAX_SHARP_DIST);
      rf_side_right_bck = constrain(SharpSide_right_bck.GetDist(sensors_msg.adc[7]), 0, MAX_SHARP_DIST);
      // Задний
      rf_bck_center = constrain(SharpBckCenter.GetDist(sensors_msg.adc[8]), 0, MAX_SHARP_DIST);

      //
      // *** Дальномеры VL53L0X
      //
      int n = vl53::RequestSensors(ExtIOStream);

      // *** Передние
      rf_center = vl53::Distances[0];
      rf_left   = vl53::Distances[1];
      rf_right  = vl53::Distances[2];
      // *** Боковые
      rf_side_left_fwd  = vl53::Distances[3];
      rf_side_right_fwd = vl53::Distances[4];
      rf_side_left_bck  = vl53::Distances[5];
      rf_side_right_bck = vl53::Distances[6];
      // Задний
      rf_bck_center = vl53::Distances[7];
    }
    else
    {
      // *** Передние
      rf_center = ussData.dist[0];
      rf_left   = ussData.dist[1];
      rf_right  = ussData.dist[2];
      // *** Боковые
      rf_side_left_fwd  = ussData.dist[3];
      rf_side_right_fwd = ussData.dist[4];
      rf_side_left_bck  = ussData.dist[5];
      rf_side_right_bck = ussData.dist[6];
      // Задний
      rf_bck_center = ussData.dist[7];
    }

    sensors_msg.rf_center = rf_center;
    sensors_msg.rf_left   = rf_left;
    sensors_msg.rf_right  = rf_right;
    sensors_msg.rf_side_left_fwd  = rf_side_left_fwd;
    sensors_msg.rf_side_right_fwd = rf_side_right_fwd;
    sensors_msg.rf_side_left_bck  = rf_side_left_bck;
    sensors_msg.rf_side_right_bck = rf_side_right_bck;
    sensors_msg.rf_bck_center = rf_bck_center;

    //
    // Прочие данные
    //
    speed_left = sensors_msg.enc_left - pred_enc_left;
    speed_right = sensors_msg.enc_right - pred_enc_right;

    pred_enc_left = sensors_msg.enc_left;
    pred_enc_right = sensors_msg.enc_right;

    sensors_msg.data[0] = CMD_CNT;
    sensors_msg.data[1] = digitalRead(PIN_BUMP_L) | (digitalRead(PIN_BUMP_C)<<1) | (digitalRead(PIN_BUMP_R)<<2);

    sensors_msg.data[2] = speed_left;
    sensors_msg.data[3] = speed_right;

    sensors_msg.data[4] = w_l;
    sensors_msg.data[5] = w_r;

    //
    // Получение данных от сервера RC5
    //
    RC5DSRV::RcvGetData(RC5IRA::RxData);
    for(byte i=0;i<RC5DSRV::NUMRECV;i++)
      sensors_msg.irdata[i] = RC5IRA::RxData[i];

     //
    // *** Важные проверки (рефлексы)
    //
    // ** Напряжение питания
    if((acc_voltage<rconfig.refl_min_voltage)  && SWITCHES[SW_POWER])
    {
      sensors_msg.status |= STAT_POWER;
      Beep(1);
      w_l = w_r = 0; // Останавливаемся
    }
    else
      sensors_msg.status &= ~STAT_POWER;

    /** Дальномеры
    При срабатывании центрального: тормозим оба двигателя
    При срабатывании левого: тормозим правый двигатель
    При срабатывании правого: тормозим левый двигатель
    **/
    if(((rf_left<REFLEX_DISTANCE[1]) || (rf_center<REFLEX_DISTANCE[0]))  && SWITCHES[SW_OBSTACLE])
    {
      sensors_msg.status |= STAT_RF_LEFT;
      if(w_r>0) w_r = 0; // Останавливаем двигатель
    }
    else
      sensors_msg.status &= ~STAT_RF_LEFT;
    if(((rf_right<REFLEX_DISTANCE[2]) || (rf_center<REFLEX_DISTANCE[0]))  && SWITCHES[SW_OBSTACLE])
    {
      sensors_msg.status |= STAT_RF_RIGHT;
      if(w_l>0) w_l = 0; // Останавливаем двигатель
    }
    else
      sensors_msg.status &= ~STAT_RF_RIGHT;
    // *** Потеря связи. Статус ожидания команды
    if((range_timer-Pred_cmd_time>Wait_cmd_time)  && SWITCHES[SW_CONNECT]) // Слишком долго ждем
    {
      sensors_msg.status |= STAT_NO_CMD;
      w_l = w_r = 0; // Останавливаемся
    }
    else
      sensors_msg.status &= ~STAT_NO_CMD;

    ShowStatus();
    digitalWrite(PIN_DIAG_SEND_DATA, HIGH-digitalRead(PIN_DIAG_SEND_DATA));   // blink the led
    pub_sensors.publish(&sensors_msg);

    //*****************************
    // Калибровка магнитного компаса
    //*****************************
    if(calibr_step<rconfig.calibr_max_cnt)
    {
      Pred_cmd_time = millis(); // Чтобы не приставали с рефлексами
      lcd.setCursor(10,0);
      lcd.print("C");
      lcd.setCursor(12,0);
      lcd.print((int)(100.0*calibr_step/rconfig.calibr_max_cnt)+1);

      compass_mag.calibr_step();
      if(calibr_step<rconfig.calibr_max_cnt/2)
      {
        w_l = -rconfig.calibr_rot_speed;
        w_r = rconfig.calibr_rot_speed;
      }
      else
      {
        w_l = rconfig.calibr_rot_speed;
        w_r = -rconfig.calibr_rot_speed;
      }
      calibr_step++;
      if(calibr_step>=rconfig.calibr_max_cnt)
      {
        w_l = w_r = 0;
        _motorL(0);
        _motorR(0);
        // Сигнализируем о конце калибровки
        Beep(3);
        lcd.setCursor(10,0);
        lcd.print("R    ");
      }
    }
    range_timer = millis();
  }

  //
  // Motors
  //
  if(rconfig.usePIDCtl)
  {
    MotorL(w_l, (float)speed_left);
    MotorR(w_r, (float)speed_right);
  }
  else
  {
    _motorL(w_l);
    _motorR(w_r);
  }

  // Отработка команд  
  if(Command)
  {
    switch(Command)
    {
      case Y13::CMD_BEEP_ON:  digitalWrite(PIN_BEEP, HIGH); break;
      case Y13::CMD_BEEP_OFF: digitalWrite(PIN_BEEP, LOW); break;

      case Y13::CMD_GUN_ON:   digitalWrite(PIN_GUN, HIGH); break;
      case Y13::CMD_GUN_OFF:  digitalWrite(PIN_GUN, LOW); break;

      case Y13::CMD_SET_ENC: // Аргументы лежат в msg.arg[0], msg.arg[1])
        encoder_left.write((int)(args[0]));
        encoder_right.write((int)(args[1]));
        break;

      case Y13::CMD_SET_ANG: // Аргументы лежат в angles[0]-angles[2] (msg.ang[0]-msg.ang[2])
        for(byte n=0;n<SRV_NUM;n++)
          srv[n]->write(angles[n]);
        break;

      case Y13::CMD_SET_REFL_DIST_LEFT:
        // Установка дистанций срабатывания рефлексов, см. (центральный, левый, правый)
        // Аргументы лежат в msg.arg[0]-msg.arg[2]
        REFLEX_DISTANCE[0] = (int)(args[0]);
        REFLEX_DISTANCE[1] = (int)(args[1]);
        REFLEX_DISTANCE[2] = (int)(args[2]);
        break;

      case Y13::CMD_SET_PID:
        // Установка параметров ПИД-регуляторов (обоих сразу).
        // Аргументы лежат в msg.arg[0]-msg.arg[2]: p, i, d соответственно
        rconfig.Kp0 = args[0];
        rconfig.Ki0 = args[1];
        rconfig.Kd0 = args[2];
        PID_LEFT.SetParams(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0);
        PID_RIGHT.SetParams(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0);
        break;

       case Y13::CMD_SET_PID_LEFT:
        // Установка параметров ПИД-регулятора левого двигателя.
        // Аргументы лежат в msg.arg[0]-msg.arg[2]: p, i, d соответственно
        rconfig.Kp0 = args[0];
        rconfig.Ki0 = args[1];
        rconfig.Kd0 = args[2];
        PID_LEFT.SetParams(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0);
        break;

       case Y13::CMD_SET_PID_RIGHT:
        // Установка параметров ПИД-регулятора правого двигателя.
        // Аргументы лежат в msg.arg[0]-msg.arg[2]: p, i, d соответственно
        rconfig.Kp0 = args[0];
        rconfig.Ki0 = args[1];
        rconfig.Kd0 = args[2];
        PID_RIGHT.SetParams(rconfig.Kp0, rconfig.Ki0, rconfig.Kd0);
        break;

      case Y13::CMD_COMPASS_CALIBR:
        // Калибровка компаса
        calibr_step = 0;
       break;

      case Y13::CMD_SET_CALIBR_SPEED:
        // Скорость поворота при калибровке и количество шагов
        // Аргументы лежат в args[0], args[1] (msg.arg[0], msg.arg[1]) соответственно
        rconfig.calibr_rot_speed = args[0];  // Скорость вращения при калибровке
        rconfig.calibr_max_cnt   = args[1];  // Количество тактов калибровки
        calibr_step = rconfig.calibr_max_cnt;
        break;

      case Y13::CMD_SET_MOTORS_RATIO:
        // Масштабные множители для согласования характеристик двигателей
        // Значения лежат в msg.arg[0] (левый двигатель) и msg.arg[1] (правый двигатель)
        RATIO_LEFT = rconfig.ratio_left = args[0];
        RATIO_RIGHT = rconfig.ratio_right = args[1];
        break;

      case Y13::CMD_USR: // Общая пользовательская команда
          if(arg_da[0]==Y13::subcmdSET_RC5) // Параметры маяка RC5
          {
            RC5IRA::TxCode = (unsigned long)arg_da[1]; // Передаваемый код RC5
            RC5IRA::numdigits = (byte)arg_da[2];       // Количество разрядов в пакете RC5 (не более 32)
            RC5IRA::freq = (byte)arg_da[3];            // Частота генерации пакетов, Гц 
            RC5DSRV::SetBeacon(RC5DSRV::CMD_GEN_RC5, RC5IRA::TxCode, RC5IRA::numdigits, RC5IRA::freq);
            sprintf(sout, "Y13::CMD_USR: %08x %d %d", RC5IRA::TxCode, RC5IRA::numdigits, RC5IRA::freq);
            LogPrintLn(sout);
          }
          if(arg_da[0]==Y13::subcmdSET_KLPF) // Значение коэффициента ФНЧ
          {
            rconfig.KLPF = args[0];
          } 
        break;

      case Y13::CMD_DCTL: // Прямое задание управления двигателями: ШИМ -255..+255 для левого и правого соответственно
        // Аргументы лежат в args[0], args[1] (msg.arg[0], msg.arg[1]) соответственно
        // ПИД-регулятор отключается
        w_l = args[0];
        w_r = args[1];
        rconfig.usePIDCtl = false; // Отключаем ПИД-регулятор
        break;
      
      case Y13::CMD_PIDCTL: // Задание скоростей вращения двигателей: -255..+255 для левого и правого соответственно
        // Аргументы лежат в args[0], args[1] (msg.arg[0], msg.arg[1]) соответственно
        // Используется ПИД-регулятор
        w_l = args[0];
        w_r = args[1];
        rconfig.usePIDCtl = true; // Включаем ПИД-регулятор
        break;
    }
    Command = 0;
  }

  // Задержка на 10 ms
  pseudoros::usr_delay_ms(10, spinfunc);
  digitalWrite(PIN_DIAG_MAIN_LOOP, !digitalRead(PIN_DIAG_MAIN_LOOP));   // blink the led
}
