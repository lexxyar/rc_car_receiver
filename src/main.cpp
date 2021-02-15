#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <Servo.h>
#include <EEPROM.h>

/**
 * RF24 ->  NANO
 * -------------
 * VCC      VCC
 * GND      GND
 * CE       D7
 * CSN      D8
 * SCK      D13
 * MOSI     D11
 * MISO     D12
 * 
 * MTR  ->  NANO
 * -------------
 * VCC      VCC
 * GND      GND
 * A-1A     D4
 * A-1B     D5
 * 
 * SERV ->  NANO
 * -------------
 * VCC      VCC
 * GND      GND
 * SIG      D6
 * */

#define PIN_RF24_CE 7
#define PIN_RF24_CSN 8
#define PIN_MOTOR_SPEED 5
#define PIN_MOTOR_DIRECTION 4
#define PIN_SERVO 6

// #define ADDR_CALIBRATED 0
#define ADDR_LEFT 1
#define ADDR_RIGHT 2
#define ADDR_MIDDLE 3
#define ADDR_MAXX_H 4
#define ADDR_MAXX_L 5
#define ADDR_MAXY_H 6
#define ADDR_MAXY_L 7
#define ADDR_MINX_H 8
#define ADDR_MINX_L 9
#define ADDR_MINY_H 10
#define ADDR_MINY_L 11
#define ADDR_MIDX_H 12
#define ADDR_MIDX_L 13
#define ADDR_MIDY_H 14
#define ADDR_MIDY_L 15
#define JOYSTIC_MIDDLE_DEADZONE 10

// Tx/Rx common part
#define RF_CHANNEL 0x6f
// #define RF24_ADDRESS "00001"
const byte RF24_ADDRESS[6] = "00001";
struct RFPackage{
  uint8_t mode; // 0 - drive, 1 - config
  uint16_t j1x;
  uint16_t j1y;
  uint16_t pot;
  uint8_t btn;
} rfPackage;
// --------------

struct Joystic
{
  uint16_t x;
  uint16_t y;
  uint16_t maxX = 0;
  uint16_t maxY = 0;
  uint16_t minX = 1023;
  uint16_t minY = 1023;
  uint16_t midX = 500;
  uint16_t midY = 500;
  int16_t posX = 90;
  int16_t posY = 90;
} j1;

struct Config
{
  uint16_t left = 0;
  uint16_t right = 0;
  uint16_t middle = 0;
  uint16_t maxX = 0;
  uint16_t maxY = 0;
  uint16_t minX = 1023;
  uint16_t minY = 1023;
  uint16_t midX = 500;
  uint16_t midY = 500;
} config;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
Servo myservo;  
uint16_t minMotorPuls = 100;
uint8_t radioHasData = 0;

void rf24Setup();
void readConfig();
void turnServo();
void driveMotor();
void constrainValues();
void printRadio();
void readRadio();

void setup() {
  Serial.begin(115200);

  // Настройка радиомодуля
  rf24Setup();
  
  // Подключаем сервопривод
  myservo.attach(PIN_SERVO);
  
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT); //direction
  pinMode(PIN_MOTOR_SPEED, OUTPUT); //speed
  
  readConfig();

  radioHasData = 0;

  delay(15);
}

void loop() {

  // Если в буфере имеются принятые данные
  readRadio();
  // if(radio.available()){ 
    if(radioHasData == 1){ 
      radioHasData = 0;

  //   // Читаем данные из буфера, указывая сколько всего байт может поместиться                            
  //   radio.read(&rfPackage, sizeof(rfPackage));
    
  //   printRadio();

    j1.x = rfPackage.j1x;
    j1.y = rfPackage.j1y;

    // Если обычный режим
    if(rfPackage.mode == 0){
      constrainValues();
      turnServo();
      driveMotor();
    }
    // Если режим настройки
    else if(rfPackage.mode == 1){
      Serial.println("Calibrating mode....");
    }   

  } 
}

void readRadio(){
  // Если в буфере имеются принятые данные
  if(radio.available()){  

    // Читаем данные из буфера, указывая сколько всего байт может поместиться                            
    radio.read(&rfPackage, sizeof(rfPackage));
    
    printRadio();

    radioHasData = 1;
  }
}

void printRadio(){
  Serial.print(rfPackage.mode);
  Serial.print("\t");
  Serial.print(rfPackage.j1x);
  Serial.print("\t");
  Serial.print(rfPackage.j1y);
  Serial.print("\t");
  Serial.print(rfPackage.pot);
  Serial.print("\t");
  Serial.println(rfPackage.btn);
}

void driveMotor(){
  if(j1.posX == 0){
    digitalWrite(PIN_MOTOR_DIRECTION, LOW);
    analogWrite(PIN_MOTOR_SPEED, LOW);
  }else{
    uint16_t direction = j1.posX > 0 ? LOW : HIGH;
    digitalWrite(PIN_MOTOR_DIRECTION, direction);
    analogWrite(PIN_MOTOR_SPEED, abs(j1.posX));
  }
}

void rf24Setup(){

  // Инициируем работу nRF24L01+
  radio.begin();

  // Указываем мощность передатчика 
  // (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.setPALevel(RF24_PA_LOW);

  // Указываем канал передачи данных (от 0 до 125)
  radio.setChannel(RF_CHANNEL);

  // Указываем скорость передачи данных 
  // (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setDataRate(RF24_250KBPS);                        

  // Открываем 1 трубу с адресом 1 передатчика, для приема данных
  radio.openReadingPipe(1, RF24_ADDRESS);
  
  // Ставим размер пакета в байтах
  radio.setPayloadSize(32);

  // Режим подтверждения приема (0 - выклб 1 - вкл)
  radio.setAutoAck(1);

  // Время между попытками постучаться 
  radio.setRetries(0, 15);

  // Включаем питание
  radio.powerUp();

  // Включаем приемник, начинаем прослушивать открытые трубы
  radio.startListening();
}

void readConfig(){
  config.left = EEPROM.read(ADDR_LEFT);
  config.right = EEPROM.read(ADDR_RIGHT);
  config.middle = EEPROM.read(ADDR_MIDDLE);
  config.maxX = word(EEPROM.read(ADDR_MAXX_H), EEPROM.read(ADDR_MAXX_L));
  config.maxY = word(EEPROM.read(ADDR_MAXY_H), EEPROM.read(ADDR_MAXY_L));
  config.minX = word(EEPROM.read(ADDR_MINX_H), EEPROM.read(ADDR_MINX_L));
  config.minY = word(EEPROM.read(ADDR_MINY_H), EEPROM.read(ADDR_MINY_L));
  config.midX = word(EEPROM.read(ADDR_MIDX_H), EEPROM.read(ADDR_MIDX_L));
  config.midY = word(EEPROM.read(ADDR_MIDY_H), EEPROM.read(ADDR_MIDY_L));
}

void constrainValues(){

  // Map joystic X value to motor pulse
  j1.x = constrain(j1.x, config.minX, config.maxX);
  j1.posX = config.middle;
  if (j1.x < config.midX - JOYSTIC_MIDDLE_DEADZONE)
  {
    j1.posX = map(j1.x, config.minX, config.midX, 255, minMotorPuls);
  }else if (j1.x >= config.midX + JOYSTIC_MIDDLE_DEADZONE)
  {
    j1.posX = map(j1.x, config.midX, config.maxX, minMotorPuls, 255);
    j1.posX *= -1;
  }else if (j1.x < config.midX + JOYSTIC_MIDDLE_DEADZONE && j1.x >= config.midX - JOYSTIC_MIDDLE_DEADZONE){
    j1.posX = 0;
  }

  // Map joystic Y value to servo angle
  j1.y = constrain(j1.y, config.minY, config.maxY);
  j1.posY = config.middle;
  if (j1.y < config.midY - JOYSTIC_MIDDLE_DEADZONE)
  {
    j1.posY = map(j1.y, config.minY, config.midY, config.left, config.middle);
  }else if (j1.y >= config.midY + JOYSTIC_MIDDLE_DEADZONE)
  {
    j1.posY = map(j1.y, config.midY, config.maxY, config.middle, config.right);
  }
}

void turnServo(){
  myservo.write(j1.posY);
  delay(15);
}

void calibrateServoPos(uint8_t currentMode, uint8_t nextMode, uint16_t eepromAddr, char* pos){
  int mode = currentMode;

  Serial.print("Set ");
  Serial.print(pos);
  Serial.println(" sevo position and press button");

  while(mode == currentMode){

    readRadio();
    // if(millis() - startTime > 600){
    //   toggleRedLed();
    //   startTime = millis();
    // }

    if (radioHasData == 1)
    {
      radioHasData = 0;
      uint16_t val = rfPackage.pot;
      val = map(val, 0, 1023, 0, 180);
      myservo.write(val);                  
      delay(15); 
    }
    

    
    
    /*
    uint16_t b = digitalRead(PIN_BUTTON);
    
    if (b == HIGH)
    {
      mode = nextMode;
      EEPROM.write(eepromAddr, val);
      #if USE_SERIAL == true
      Serial.print(pos);
      Serial.print(" value is: ");
      Serial.println(val);
      #endif
      break;
    }
    */
  }
}