
#include <PID_v1.h>
//#include <Wire.h>
#include <max6675.h>
#include <EEPROM.h>
#include <UTFT.h>

UTFT myGLCD(CTE40, 38, 39, 40, 41);
//UTFT myGLCD(CTE32HR, 38, 39, 40, 41);

extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t BigFontRus[]; //Кирилица
extern uint8_t SevenSegNumFont[];

int Xgr, XTgr=2; // x координата графика
byte Cgr,CTgr=0;
int er1 = 1;
int er2 = 1;
int reg1;
int reg2;
int pwr1;
int pwr2;
double Secs, prev_millis=0;

boolean out1;
boolean out2;

byte min_pwr_BOTTOM;              //минимальная мощность нижнего нагревателя
byte max_pwr_BOTTOM;              //максимальная мощность нижнего нагревателя
byte min_pwr_TOPStep[3];       //минимальная мощность верхнего нагревателя
byte max_pwr_TOPStep[3];       //максимальная мощность верхнего нагревателя

//RelayPin "1"-ВЕРХНИЙ нагреватель
//RelayPin "2"-НИЖНИЙ нагреватель
#define RelayPin1 6  //назначаем пин "ВЕРХНЕГО" нагревателя
#define RelayPin2 7  //назначаем пин "НИЖНЕГО" нагревателя

// Выходы реле
#define P1_PIN 9  //назначаем пин реле 1
#define P2_PIN 10         //назначаем пин реле 2
#define P3_PIN 11         //назначаем пин реле 3
#define P4_PIN 12  //назначаем пин реле 4

byte tableSize;
//int zero1;

int buzzerPin = 8;
int time = 0;

//--------настройка кнопок управления------------------------------------

//#define A_PINS_BASE 100 // номер с которого начинается нумерация наших "псевдо-кнопок".
  
 
#define PIN_RIGHT 100
#define PIN_UP 101
#define PIN_DOWN 102
#define PIN_LEFT 103
#define PIN_SELECT 104
 
struct A_PIN_DESC{ // определяем  структуру которой будем описывать какое значение мы ожидаем для каждого псевдо-пина
   byte pinNo; // номер пина
   int expectedValue;// ожидаемое значение
};
 
A_PIN_DESC expected_values[]={ // ожидаемые значения для псевдо-кнопок
   { PIN_RIGHT,1023},
   { PIN_UP,0},
   { PIN_DOWN,141},
   { PIN_LEFT,325},
   { PIN_SELECT,494}
};
 
#define A_PINS_COUNT sizeof(expected_values)/sizeof(A_PIN_DESC) // вычисляем сколько у нас всего "псевдо-кнопок" заданно.
#define A_POSSIBLE_ABERRATION 50 // допустимое отклонение analogRead от ожидаемого значения, при котором псевдо кнопка считается нажатой
 
 

bool digitalReadA(byte pinNo){
   
  for(byte i=0;i<A_PINS_COUNT;i++){ // ищем описание нашего всевдо-пина
     A_PIN_DESC pinDesc=expected_values[i];// берем очередное описание
     if(pinDesc.pinNo==pinNo){ // нашли описание пина?
        int value=analogRead(A0); // производим чтение аналогово входа
         
        return (abs(value-pinDesc.expectedValue)<A_POSSIBLE_ABERRATION); // возвращаем HIGH если отклонение от ожидаемого не больше чем на A_POSSIBLE_ABERRATION
     }
  }
   
  return LOW; // если не нашли описания - считаем что пин у нас LOW
   
}

//Назначаем пины кнопок управления
int upSwitchPin = PIN_UP;
int downSwitchPin = PIN_DOWN;
int cancelSwitchPin = PIN_LEFT;
int okSwitchPin = PIN_SELECT;

//состояние кнопок по умолчанию
int upSwitchState = 0;
int downSwitchState = 0;
int cancelSwitchState = 0;
int okSwitchState = 0;

//переменные для кнопок
byte Hselect = 0;
long ms_button = 0;
boolean  button_state = false;
boolean  button_long_state = false;

//profile stuff
byte currentProfile = 1;
byte currentStep = 1;
byte profileSteps;

//profile name
byte profileName;

//current editing step pointer
byte editStep = 0;

byte dwellTimerStep[3];
byte rampRateStep[3];
byte TimerStep;

byte kp1;
byte ki1;
byte kd1;
byte kp2;
byte ki2;
byte kd2;

double startTemp;

byte setpointRamp;
boolean TopStart = false;
int flag = 0;  //флаг для фиксации стартовой температуры
int x=1;  //переменная для перехода на нужный шаг при горячей плате
byte temperatureStep[3];
int eepromAddress = 0;//начало eepromaddress

long previousMillis; //это для счетчиков

double counter;

//these are the different states of the sketch. We call different ones depending on conditions
// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_MENU_STEPS,
  REFLOW_STATE_MENU_TABLE_SIZE,
  REFLOW_STATE_MENU_BOTTOM_HEAT,
  REFLOW_STATE_MENU_BOTTOM_PWR_MIN,
  REFLOW_STATE_MENU_BOTTOM_PWR_MAX,
  REFLOW_STATE_MENU_STEP_RAMP,
  REFLOW_STATE_MENU_STEP_TARGET,
  REFLOW_STATE_MENU_TOP_PWR_MIN,
  REFLOW_STATE_MENU_TOP_PWR_MAX,
  REFLOW_STATE_MENU_STEP_DWELL,
  REFLOW_STATE_MENU_BOTTOM_P,
  REFLOW_STATE_MENU_BOTTOM_I,
  REFLOW_STATE_MENU_BOTTOM_D,
  REFLOW_STATE_MENU_TOP_P,
  REFLOW_STATE_MENU_TOP_I,
  REFLOW_STATE_MENU_TOP_D,
  REFLOW_STATE_STEP_RAMP,
  REFLOW_STATE_TABLE_SIZE,
  REFLOW_STATE_STEP,
  REFLOW_STATE_STEP_DWELL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR
}
reflowState_t;

typedef enum REFLOW_STATUS //this is simply to check if reflow should be running or not
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
}
reflowStatus_t;

#define SENSOR_SAMPLING_TIME 250 //read tc every second
reflowStatus_t reflowStatus;
// Reflow oven controller state machine state variable
reflowState_t reflowState;

//TC read timer variables
//unsigned long nextCheck1;
unsigned long nextRead1; //переменная для обновления текущей температуры
//PID stufff

byte Setpoint1;
float Input_f1, Input_f2, Input_ft1, Input_ft2;
double Input1, Output1, Setpoint1d;
//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1d, kp1, ki1, kd1,P_ON_E, DIRECT);
unsigned long windowStartTime;
//PID stuff
byte Setpoint2;
double Input2, Output2, Setpoint2d;
//Specify the links and initial tuning parameters
PID myPID2(&Input2, &Output2, &Setpoint2d, kp2, ki2, kd2,P_ON_E, DIRECT);

//Alarm state boolean
boolean alarmOn = false;

//Update whole screen boolean
boolean updateScreen = true;

//назначаем пины усилителя термопары MAX6675 "ВЕРХНЕГО" нагревателя   clk=sck cs=cs do=so
int thermoCLK = 17;  //=sck
int thermoCS = 18;   //=cs
int thermoDO = 19;   //=so
//назначаем пины усилителя термопары MAX6675 "НИЖНЕГО" нагревателя clk=sck cs=cs do=so
int thermoCLK2 = 14;  //=sck
int thermoCS2 = 15;   //=cs
int thermoDO2 = 16;   //=so

// переменные для калмана "ВЕРХНЕГО" нагревателя
float varTerm1 = 2.65;  // среднее отклонение (ищем в excel)
float varProcess1 = 0.06; // скорость реакции на изменение (подбирается вручную)
float Pc1 = 0.0;
float G1 = 0.0;
float P1 = 1.0;
float Xp1 = 0.0;
float Zp1 = 0.0;
float Xe1 = 0.0;

// переменные для калмана "НИЖНЕГО" нагревателя
float varTerm2 = 2.65;  // среднее отклонение (ищем в excel)
float varProcess2 = 0.06; // скорость реакции на изменение (подбирается вручную)
float Pc2 = 0.0;
float G2 = 0.0;
float P2 = 1.0;
float Xp2 = 0.0;
float Zp2 = 0.0;
float Xe2 = 0.0;
// переменные для калмана
int tc1;
int tc2;

MAX6675 thermocouple1(thermoCLK, thermoCS, thermoDO);//термопара "ВЕРХНЕГО" нагревателя
MAX6675 thermocouple2(thermoCLK2, thermoCS2, thermoDO2);//термопара "НИЖНЕГО" нагревателя

void loadProfile()//this function loads whichever profile currentProfile variable is set to
{
  profileSteps = EEPROM.read((currentProfile - 1) * 29);
  Setpoint2 = EEPROM.read((currentProfile - 1) * 29 + 1);
  Setpoint2d = Setpoint2;
  for (int i = 0; i < 3; i + 1) {
    rampRateStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 2);
    i++;
  }
  for (int i = 0; i < 3; i + 1) {
    dwellTimerStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 5) * 5;
    i++;
  }
  for (int i = 0; i < 3; i + 1) {
    temperatureStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 8);
    i++;
  }
  for (int i = 0; i < 3; i + 1) {
    min_pwr_TOPStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 11);
    i++;
  }
  for (int i = 0; i < 3; i + 1) {
    max_pwr_TOPStep[i] = EEPROM.read((currentProfile - 1) * 29 + i + 14);
    i++;
  }
  kp1 = EEPROM.read((currentProfile - 1) * 6 + 172);
  ki1 = EEPROM.read((currentProfile - 1) * 6 + 173);
  kd1 = EEPROM.read((currentProfile - 1) * 6 + 174);
  kp2 = EEPROM.read((currentProfile - 1) * 6 + 175);
  ki2 = EEPROM.read((currentProfile - 1) * 6 + 176);
  kd2 = EEPROM.read((currentProfile - 1) * 6 + 177);
  tableSize = EEPROM.read((currentProfile - 1) * 6 + 202);
  min_pwr_BOTTOM = EEPROM.read((currentProfile - 1) * 6 + 203);
  max_pwr_BOTTOM = EEPROM.read((currentProfile - 1) * 6 + 204);
      myPID1.SetMode(AUTOMATIC);
      myPID2.SetMode(AUTOMATIC); 
      myPID1.SetTunings(kp1/10.0, ki1/1000.0, kd1/10.0);
      myPID2.SetTunings(kp2/10.0, ki2/1000.0, kd2/10.0);
      myPID1.SetOutputLimits(min_pwr_TOPStep[currentStep - 1], max_pwr_TOPStep[currentStep - 1]);        //установка минимального и максимального значения (от 0 до 100)
      myPID2.SetOutputLimits(min_pwr_BOTTOM, max_pwr_BOTTOM);                                            //которое будет передано на управление диммером
  return;
}
 

void setup()
{
    //for (int i = 0; i < 8192; i++)
    //EEPROM.write(i, 1);
    Serial.begin(9600);
  
//setup reley pins as outputs
  pinMode(P1_PIN, OUTPUT);
//digitalWrite(P1_PIN, HIGH); //подключаем подтягивающий резистор
  pinMode(P2_PIN, OUTPUT);
//digitalWrite(P2_PIN, HIGH); //подключаем подтягивающий резистор
  pinMode(P3_PIN, OUTPUT);
//digitalWrite(P3_PIN, HIGH); //подключаем подтягивающий резистор
  pinMode(P4_PIN, OUTPUT);
//digitalWrite(P4_PIN, HIGH); //подключаем подтягивающий резистор
  
  myGLCD.InitLCD();
  myGLCD.clrScr();
  for (byte i=1; i<5; i++)
  {
 // myGLCD.clrScr();
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255-(i*i/2), 255-(i*i/2), 255-(i*i/2));
  myGLCD.print("*----------------------*",CENTER, 40+i*2);
  myGLCD.setColor(255-(i*i/2),255-(i*i/2),000);
  myGLCD.print("|   Burov  Corporation   |",CENTER, 70+i*2);
  myGLCD.setColor(255-(i*i/2),255-(i*i/2),255-(i*i/2));
  myGLCD.print("|    Infrared STATION    |",CENTER, 100+i*2);
  myGLCD.setColor(000, 255-(i*i/2), 000);
  myGLCD.print("         V3-4_2         ",CENTER, 130+i*2);
  myGLCD.setColor(255-(i*i/2), 255-(i*i/2), 255-(i*i/2));
  myGLCD.print("*----------------------*",CENTER, 160+i*2);
  }
  // подождите, пока чипы MAX будут стабилизироваться и закрасить экран
  //delay(50);
  {
  //Мелодия приветствия
  //Марио
  tone(buzzerPin,1318,150);
  delay(150);
  tone(buzzerPin,1318,300);
  delay(300);
  tone(buzzerPin,1318,150);
  delay(300);
  tone(buzzerPin,1046,150);
  delay(150);
  tone(buzzerPin,1318,300);
  delay(300);
  tone(buzzerPin,1568,600);
  delay(600);
  tone(buzzerPin,784,600);
  delay(600); 

  }
  myGLCD.clrScr();
  
//setup ssr pins as outputs
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);  

  //windowStartTime = millis()
  nextRead1 = millis();

  //myPID1.SetMode(AUTOMATIC);
  //myPID2.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(300); //задает частоту расчета выходного сигнала в милисекундах
  myPID2.SetSampleTime(300);
  }

int i = 0;
char buf[64];
void loop()
{
  //Считываем состояние кнопок управления
  upSwitchState = digitalReadA(upSwitchPin);
  downSwitchState = digitalReadA(downSwitchPin);
  cancelSwitchState = digitalReadA(cancelSwitchPin);
  okSwitchState = digitalReadA(okSwitchPin);
  //----------------------------------------------------------------------------------
/*  if (upSwitchState == HIGH) {
    upSwitchState = LOW;
  } else {
    upSwitchState = HIGH;
   
  }
  if (downSwitchState == HIGH) {
    downSwitchState = LOW;
  } else {
    downSwitchState = HIGH;
  }
  if (cancelSwitchState == HIGH) {
    cancelSwitchState = LOW;
  } else {
    cancelSwitchState = HIGH;
  }
  if (okSwitchState == HIGH) {
    okSwitchState = LOW;
  } else {
    okSwitchState = HIGH; 
  }*/
//-------------------------------------------------------------------------------

  //
  unsigned long currentMillis = millis();
      
  int SP1 = Setpoint1;
  int SP2 = Setpoint2;

//включение бузера
  if (upSwitchState == HIGH || downSwitchState == HIGH || cancelSwitchState == HIGH || okSwitchState == HIGH)
  {
     tone(buzzerPin, 1045);
     delay(100);
     noTone(buzzerPin);
  }
  if (reflowState == REFLOW_STATE_COMPLETE || alarmOn) {
    if (i < 2 && cancelSwitchState == LOW) {
      alarmOn = true;
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      i++;
    }
    else {
      i = 0;
      alarmOn = false;
    }
  }

  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
     TopStart = false;
     currentStep = 1;
     counter = 0;
     setpointRamp = 0;
     //editStep = 1;
     x = 1;             //устанавливаем переменную в исходное состояние
     flag = 0;         //после остановки профиля сбрасываем флаг
     myPID1.SetMode(MANUAL);
     myPID2.SetMode(MANUAL);
     Output1=0;
     Output2=0;

      if (millis() > nextRead1)
      {
        // Read thermocouples next sampling period
        nextRead1 =millis()+ SENSOR_SAMPLING_TIME;

        Input1 = filter1(thermocouple1.readCelsius());
        Input2 = filter2(thermocouple2.readCelsius());
        
	Serial.print(Input1);
        Serial.print(" ");
        Serial.print(Input2);

        tc1 = Input1;
        tc2 = Input2;
        if(tc1 <= 0){tc1 = 0;}
        if(tc2 <= 0){tc2 = 0;}
      //sprintf (buf, "OK%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2); // БУФЕР
        if (Input1 <= -0) { //Изменил если температура меньше нуля писать ошибку
          myGLCD.setColor(VGA_BLACK);
          myGLCD.drawRoundRect(340,100,460,180);
          myGLCD.setFont(BigFont);
          myGLCD.setColor(VGA_RED);
          myGLCD.print("ERROR",360, 140);
        } else {
          myGLCD.setFont(SevenSegNumFont);
          myGLCD.setColor(VGA_SILVER);
          myGLCD.printNumI(tc1,345, 120,3,'0');
        }
        if (Input2 <= -0) { //Изменил если температура меньше нуля писать ошибку
          myGLCD.setFont(BigFont);
          myGLCD.setColor(VGA_RED);
          myGLCD.print("ERROR",360, 250);
        } else {
          myGLCD.setFont(SevenSegNumFont);
          myGLCD.setColor(VGA_SILVER);
          myGLCD.printNumI(tc2,345, 232,3,'0');
        }
      }

    //Настройка экрана Рабочий режим
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("`",155, 115);
        myGLCD.textRus("`/с",175, 133);
        myGLCD.textRus("`/с",175, 245);
        myGLCD.textRus("`",155, 227);
        myGLCD.textRus("`",450, 123);
        myGLCD.textRus("`",450, 235);
		myGLCD.setColor(100,100,100);    // "вкладка" 
		myGLCD.drawLine(195,108,210,78); //   для
		myGLCD.drawLine(210,78,475,78);  // мощности
		myGLCD.drawLine(475,78,475,108); //  верха
		myGLCD.drawLine(195,106,475,106);//
		
		myGLCD.setColor(100,100,100);    // "вкладка" 
		myGLCD.drawLine(195,221,210,191); //   для
		myGLCD.drawLine(210,191,475,191);  // мощности
		myGLCD.drawLine(475,191,475,221); //   низа
		myGLCD.drawLine(197,219,475,219);//
		myGLCD.setColor(30,30,30);
		myGLCD.textRus("График Температуры",90, 20);
        myGLCD.setColor(VGA_SILVER);
		myGLCD.textRus("Мощность      %",230, 84);
		myGLCD.textRus("Мощность      %",230, 198);
        myGLCD.drawRoundRect(3,108,478,179);
        myGLCD.drawRoundRect(3,221,478,291);
		myGLCD.setColor(40,40,40);      // верхний график 
        myGLCD.drawRect(1,1,479,72);    //температуры
        //myGLCD.drawRect(224,221,478,291);
        myGLCD.setColor(VGA_SILVER);
		myGLCD.drawLine(224,108,224,179); //разделитель
		myGLCD.drawLine(224,221,224,291); //
        myGLCD.textRus("ВЕРХ->",5, 115);
        myGLCD.textRus("НИЗ ->",5, 227);
		myGLCD.textRus("Рост=",5, 133);
        myGLCD.textRus("Рост=",5, 245);
       // myGLCD.textRus("ДАТЧИК",235, 153);
       // myGLCD.textRus("ДАТЧИК",235, 266);
        updateScreen = false;
      }
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(currentProfile,465, 300);
        myGLCD.setFont(BigFont);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.printNumI(temperatureStep[editStep],100, 115,3,'0');
        myGLCD.printNumI(SP2,100, 227,3,'0');
        windowStartTime = millis();

       if (currentProfile == 1) profileName = 1;
       if (currentProfile == 2) profileName = 2;
       if (currentProfile == 3) profileName = 3;
       if (currentProfile == 4) profileName = 4;
       if (currentProfile == 5) profileName = 5;
       
        if (profileName == 1){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СНЯТИЕ БЕССВИНЕЦ",200, 300);
        }
        if (profileName == 2){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("   СНЯТИЕ СВИНЕЦ",200, 300);
        }
        if (profileName == 3){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УС-ВКА БЕССВИНЕЦ",200, 300);
        }
        if (profileName == 4){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УСТАНОВКА СВИНЕЦ",200, 300);
        }
        if (profileName == 5){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("НИЖНИЙ ПОДОГРЕВ",200, 300);
      }
      
      if (upSwitchState == HIGH && ( millis() - ms_button)>500)//if up switch is pressed go to next profile
      {
        ms_button =  millis();
        currentProfile = currentProfile + 1;
        if (currentProfile >= 6)//if currentProfile = 5 and up is pressed go back to profile 1
        {
          currentProfile = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>500)//same as above just go down one profile
      {
        ms_button =  millis();
        currentProfile = currentProfile - 1;
        if (currentProfile <= 0)
        {
          currentProfile = 5;
        }
      }
      loadProfile();//call the loadProfile function to load from eeprom

//фиксируем момент нажатия кнопки "ОК" + защита от дребезга
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
        button_long_state = false;
      }
//держим "ОК" 3+ секунды и заходим в меню настроек      
      if (okSwitchState == HIGH && !button_long_state && ( millis() - ms_button)>1500)
      {
        button_long_state = true;
        button_state = false;
        reflowState = REFLOW_STATE_MENU_STEPS;
        //update next screen
        updateScreen = true;
		
      }
//включаем пайку, кнопка сработает после отпускания "ОК"       
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        tone(buzzerPin, 1045, 500);  //звуковой сигнал при старте профиля
        //update next screen
        updateScreen = true;
        reflowStatus = REFLOW_STATUS_ON;
        reflowState = REFLOW_STATE_TABLE_SIZE;
      }
      break;
//устанавливаем количество шагов профиля
    case REFLOW_STATE_MENU_STEPS:
      if (updateScreen){
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("ПР-ЛЬ",0, 20);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ",90, 60);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("К-ВО ШАГОВ",10, 110);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("РАЗМЕР НИЗА",260, 110);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ",5, 180);
        myGLCD.textRus("МОЩНОСТЬ НИЗ",280, 180);
        myGLCD.textRus("МИН",320, 220);
        myGLCD.textRus("МАХ",320, 260);
        myGLCD.textRus("`",180, 223);
        myGLCD.textRus("%",430, 220);
        myGLCD.textRus("%",430, 260);
        myGLCD.printNumI(tableSize,450, 110);
        myGLCD.printNumI(min_pwr_BOTTOM,380, 220,3,'0');
        myGLCD.printNumI(max_pwr_BOTTOM,380, 260,3,'0');
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(SP2,75, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(5,103,200,131);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(currentProfile,88, 20);
        myGLCD.printNumI(profileSteps,180, 110);
        updateScreen = false;
      }
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(profileSteps,180, 110);

        if (profileName == 1){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СНЯТИЕ ЧИПА БЕССВИНЕЦ",120, 20);
        }
        if (profileName == 2){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СНЯТИЕ ЧИПА СВИНЕЦ",120, 20);
        }
        if (profileName == 3){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УС-ВКА ЧИПА БЕССВИНЕЦ",120, 20);
        }
        if (profileName == 4){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УС-ВКА ЧИПА СВИНЕЦ",120, 20);
        }
        if (profileName == 5){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("НИЖНИЙ ПОДОГРЕВ",120, 20);
      }

      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        profileSteps = profileSteps + 1;
        if (profileSteps >= 4)
        {
          profileSteps = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        profileSteps = profileSteps - 1;
        if (profileSteps <= 0) 
        {
          profileSteps = 3;
        }
      }
      
      if (okSwitchState == LOW && button_long_state) button_long_state = false; //чтобы после входа в меню не кнопка "ОК" не считалась нажатой
      
      if (okSwitchState == HIGH && !button_state && !button_long_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write((currentProfile - 1) * 29, profileSteps);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TABLE_SIZE;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
//устанавливаем размер стола
    case REFLOW_STATE_MENU_TABLE_SIZE:
      if (updateScreen){
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(5,103,200,131);
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("РАЗМЕР НИЗА",260, 110);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("К-ВО ШАГОВ",10, 110);
        myGLCD.printNumI(profileSteps,180, 110);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(250,103,475,131);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(tableSize,450, 110);
        updateScreen = false;
      }
        myGLCD.printNumI(tableSize,450, 110);

      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        tableSize = tableSize + 1;
        if (tableSize >= 4)
        {
          tableSize = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        tableSize = tableSize - 1;
        if (tableSize <= 0) 
        {
          tableSize = 3;
        }
      }      
      if (okSwitchState == HIGH && !button_state && !button_long_state && ( millis() - ms_button)>120)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 202), tableSize);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_HEAT;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
//устанавливаем температуру "Нижнего Нагревателя"
    case REFLOW_STATE_MENU_BOTTOM_HEAT:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(250,103,475,131);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("РАЗМЕР НИЗА",260, 110);
        myGLCD.printNumI(tableSize,450, 110);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ",5, 180);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2,175,258,203);
        updateScreen = false;
      }
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(SP2,75, 220,3,'0');

      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        Setpoint2 = Setpoint2 + 1;
        if (Setpoint2 >= 250)
        {
          Setpoint2 = 250;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        Setpoint2 = Setpoint2 - 1;
        if (Setpoint2 <= 100)
        {
          Setpoint2 = 100;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write((currentProfile - 1) * 29 + 1, Setpoint2);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MIN;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
 //устанавливаем минимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"     
     case REFLOW_STATE_MENU_BOTTOM_PWR_MIN:    
      if (updateScreen){
        myGLCD.setFont(BigFontRus);        
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2,175,258,203);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ НИЗ",280, 180);
        myGLCD.textRus("МИН",320, 220);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ",5, 180);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(SP2,75, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(275,175,475,203);
        updateScreen = false;
      }
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(min_pwr_BOTTOM,380, 220,3,'0');
      
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        min_pwr_BOTTOM = min_pwr_BOTTOM + 1;
        if (min_pwr_BOTTOM >= 100)
        {
          min_pwr_BOTTOM = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        min_pwr_BOTTOM = min_pwr_BOTTOM - 1;
        if (min_pwr_BOTTOM <= 0)
        {
         min_pwr_BOTTOM = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 203), min_pwr_BOTTOM);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MAX;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>50)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

 //устанавливаем максимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"     
     case REFLOW_STATE_MENU_BOTTOM_PWR_MAX:    
      if (updateScreen){
        myGLCD.setFont(BigFontRus);        
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2,175,258,203);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ НИЗ",280, 180);
        myGLCD.textRus("МАХ",320, 260);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МИН",320, 220);
        myGLCD.printNumI(min_pwr_BOTTOM,380, 220,3,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(275,175,475,203);
        updateScreen = false;
      }
        myGLCD.printNumI(max_pwr_BOTTOM,380, 260,3,'0');
      
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        max_pwr_BOTTOM = max_pwr_BOTTOM + 1;
        if (max_pwr_BOTTOM >= 100)
        {
          max_pwr_BOTTOM = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        max_pwr_BOTTOM = max_pwr_BOTTOM - 1;
        if (max_pwr_BOTTOM <= 0)
        {
         max_pwr_BOTTOM = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 204), max_pwr_BOTTOM);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_STEP_RAMP;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>50)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
      
//устанавливаем скорость нагрева "Верхним Нагревателем"
    case REFLOW_STATE_MENU_STEP_RAMP:
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("ПР-ЛЬ",0, 20);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ",120, 55);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА ВЕРХОМ",15, 90);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ШАГ:",0, 55);
        myGLCD.textRus("ТЕМП-РА ВЕРХ",5, 140);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ",260, 140);
        myGLCD.textRus("МИН",310, 180);
        myGLCD.textRus("МАХ",310, 220);
        myGLCD.printNumI(min_pwr_TOPStep[editStep],370, 180,3,'0');
        myGLCD.printNumI(max_pwr_TOPStep[editStep],370, 220,3,'0');
        myGLCD.textRus("ВРЕМЯ ПЕРЕХОДА НА СЛЕД ШАГ",5, 270);
        myGLCD.textRus("с",460, 90);
        myGLCD.textRus("с",462, 270);
        myGLCD.textRus("`",160, 183);
        myGLCD.textRus("%",420, 180);
        myGLCD.textRus("%",420, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(12,85,390,112);
        myGLCD.printNumI(currentProfile,88, 20);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');
        myGLCD.setFont(BigFont);
        myGLCD.printNumI(dwellTimerStep[editStep],427, 270,2,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1,70, 55);
        updateScreen = false;
      }
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumF(rampRateStep[editStep] * 0.1,0, 405, 90);
        
        if (profileName == 1){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СНЯТИЕ ЧИПА БЕССВИНЕЦ",120, 20);
        }
        if (profileName == 2){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СНЯТИЕ ЧИПА СВИНЕЦ",120, 20);
        }
        if (profileName == 3){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УС-ВКА ЧИПА БЕССВИНЕЦ",120, 20);
        }
        if (profileName == 4){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("УС-ВКА ЧИПА СВИНЕЦ",120, 20);
        }
        if (profileName == 5){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("НИЖНИЙ ПОДОГРЕВ",120, 20);
        myGLCD.setColor(250, 180, 000);
        
      }
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        rampRateStep[editStep] = rampRateStep[editStep] + 1.;
        if (rampRateStep[editStep] >= 30.)
        {
          rampRateStep[editStep] = 30.;  //максимальная скорость роста температуры от 0.1с. до 3с.
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        rampRateStep[editStep] = rampRateStep[editStep] - 1.;
        if (rampRateStep[editStep] <= 1.)
        {
          rampRateStep[editStep] = 1.;  //минимальная скорость роста температуры от 3с. до 0.1с.
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
      /*  if (editStep + 1 == profileSteps) {
          editStep = 0;*/
          reflowState = REFLOW_STATE_MENU_STEP_TARGET;
       /* }
        else {
          editStep++;
        } */
        for (int i = 0; i < 3; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 2), rampRateStep[i]);
          i++;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      
      break;
 //устанавливаем температуру "Верхнего Нагревателя"
    case REFLOW_STATE_MENU_STEP_TARGET:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(12,85,390,112);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА ВЕРХОМ",15, 90);
        myGLCD.textRus("с",455, 90);
        myGLCD.printNumF(rampRateStep[editStep] * 0.1,0, 405, 90);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ТЕМП-РА ВЕРХ",5, 140); 
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2,131,200,160);
        myGLCD.printNumI(editStep + 1,70, 55);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');       
        updateScreen = false;
      }
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');
        
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        temperatureStep[editStep] = temperatureStep[editStep] + 1;
        if (temperatureStep[editStep] >= 250)
        {
          temperatureStep[editStep] = 250;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        temperatureStep[editStep] = temperatureStep[editStep] - 1;
        if (temperatureStep[editStep] <= 0)
        {
          temperatureStep[editStep] = 0;
        }
        if (temperatureStep[editStep] <= 99)
        {
         myGLCD.printNumI(temperatureStep[editStep],180, 160);
        }
        if (temperatureStep[editStep] <= 9)
        {
         myGLCD.printNumI(temperatureStep[editStep],180, 160);
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
     /*   if (editStep + 1 == profileSteps) {
          editStep = 0; */
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MIN;
      /*  }
        else {
          editStep++;
        } */
        for (int i = 0; i < 3; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 8), temperatureStep[i]);
          i++;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
 //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"      
     case REFLOW_STATE_MENU_TOP_PWR_MIN:      
      if (updateScreen)
      {
        myGLCD.setFont(BigFontRus);        
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2,131,200,160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ТЕМП-РА ВЕРХ",5, 140);
        myGLCD.printNumF(rampRateStep[editStep] * 0.1,0, 405, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ",260, 140);
        myGLCD.textRus("МИН",310, 180);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1,70, 55);
        myGLCD.drawRoundRect(255,131,470,160);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(min_pwr_TOPStep[editStep],370, 180,3,'0');
        updateScreen = false;
      }
        myGLCD.printNumI(min_pwr_TOPStep[editStep],370, 180,3,'0');
      
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        min_pwr_TOPStep[editStep] = min_pwr_TOPStep[editStep] + 1;
        if (min_pwr_TOPStep[editStep] >= 100)
        {
          min_pwr_TOPStep[editStep] = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        min_pwr_TOPStep[editStep] = min_pwr_TOPStep[editStep] - 1;
        if (min_pwr_TOPStep[editStep] <= 0)
        {
         min_pwr_TOPStep[editStep] = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
     /*   if (editStep + 1 == profileSteps) {
        editStep = 0;*/
        reflowState = REFLOW_STATE_MENU_TOP_PWR_MAX;
       /* }
        else {
          editStep++;
        }*/
        for (int i = 0; i < 3; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 11), min_pwr_TOPStep[i]);
          i++;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

 //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"      
     case REFLOW_STATE_MENU_TOP_PWR_MAX:      
      if (updateScreen)
      {
        myGLCD.setFont(BigFontRus);        
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2,131,200,160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МИН",310, 180);
        myGLCD.printNumI(min_pwr_TOPStep[editStep],370, 180,3,'0');
        myGLCD.printNumF(rampRateStep[editStep] * 0.1,0, 405, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МАХ",310, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1,70, 55);
        myGLCD.drawRoundRect(255,131,470,160);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(max_pwr_TOPStep[editStep],370, 220,3,'0');
        updateScreen = false;
      }
        myGLCD.printNumI(max_pwr_TOPStep[editStep],370, 220,3,'0');
      
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        max_pwr_TOPStep[editStep] = max_pwr_TOPStep[editStep] + 1;
        if (max_pwr_TOPStep[editStep] >= 100)
        {
          max_pwr_TOPStep[editStep] = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        max_pwr_TOPStep[editStep] = max_pwr_TOPStep[editStep] - 1;
        if (max_pwr_TOPStep[editStep] <= 0)
        {
         max_pwr_TOPStep[editStep] = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
     /*   if (editStep + 1 == profileSteps) {
        editStep = 0;*/
        reflowState = REFLOW_STATE_MENU_STEP_DWELL;
       /* }
        else {
          editStep++;
        }*/
        for (int i = 0; i < 3; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 14), max_pwr_TOPStep[i]);
          i++;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
      
//устанавливаем время перехода на следующий шаг
    case REFLOW_STATE_MENU_STEP_DWELL:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(255,131,470,160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ",260, 140);
        myGLCD.textRus("МАХ",310, 220);
        myGLCD.printNumI(max_pwr_TOPStep[editStep],370, 220,3,'0');
        myGLCD.printNumF(rampRateStep[editStep] * 0.1,0, 405, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(temperatureStep[editStep],55, 180,3,'0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ВРЕМЯ ПЕРЕХОДА НА СЛЕД ШАГ",5, 270);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2,263,420,290);
        myGLCD.printNumI(editStep + 1 ,70, 55);
        myGLCD.printNumI(dwellTimerStep[editStep],427, 270,2,'0');
        updateScreen = false;
       }
        myGLCD.printNumI(dwellTimerStep[editStep],427, 270,2,'0');

      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        dwellTimerStep[editStep] = dwellTimerStep[editStep] + 5;
        if (dwellTimerStep[editStep] >= 90)
        {
          dwellTimerStep[editStep] = 90;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        dwellTimerStep[editStep] = dwellTimerStep[editStep] - 5;
        if (dwellTimerStep[editStep] <= 0)
        {
          dwellTimerStep[editStep] = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        for (int i = 0; i < 3; i + 1) {
          EEPROM.write(((currentProfile - 1) * 29 + i + 5), dwellTimerStep[i] / 5);
          i++;
        }
        updateScreen = true;
        if (editStep + 1 == profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_BOTTOM_P;
        }
        else {
          editStep++;
          reflowState = REFLOW_STATE_MENU_STEP_RAMP;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
//настройка "ПИД" нижнего нагревателя
    case REFLOW_STATE_MENU_BOTTOM_P:
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("НАСТРОЙКА ПИД",130, 5);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ",90, 35);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ",85, 170);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(ki2,200, 100,3,'0');
        myGLCD.printNumI(kd2,375, 100,3,'0');
        myGLCD.setFont(BigFont);
        myGLCD.print("I=",230, 70);
        myGLCD.print("D=",405, 70);
        myGLCD.print("P=",55, 205);
        myGLCD.print("I=",230, 205);
        myGLCD.print("D=",405, 205);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("P=",55, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.drawRoundRect(80,28,390,55);
        myGLCD.printNumI(kp2,25, 100,3,'0');
        updateScreen = false;
      }
        myGLCD.printNumI(kp2,25, 100,3,'0');
        
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kp2 = kp2 + 1;
        if (kp2 >= 255)
        {
          kp2 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kp2 = kp2 - 1;
        if (kp2 <= 0)
        {
          kp2 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 175), kp2);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_I;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_BOTTOM_I:
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("I=",230, 70);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("P=",55, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(kp2,25, 100,3,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(ki2,200, 100,3,'0');       
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        ki2 = ki2 + 1;
        if (ki2 >= 255)
        {
          ki2 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        ki2 = ki2 - 1;
        if (ki2 <= 0)
        {
          ki2 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 176), ki2);
        reflowState = REFLOW_STATE_MENU_BOTTOM_D;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_BOTTOM_D:
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("D=",405, 70);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("I=",230, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(ki2,200, 100,3,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(kd2,375, 100,3,'0');
        
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kd2 = kd2 + 1;
        if (kd2 >= 255)
        {
          kd2 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kd2 = kd2 - 1;
        if (kd2 <= 0)
        {
          kd2 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 177), kd2);
        reflowState = REFLOW_STATE_MENU_TOP_P;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
//настройка "ПИД" верхнего нагревателя
    case REFLOW_STATE_MENU_TOP_P:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("P=",55, 205);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ",85, 170);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ",90, 35);
        myGLCD.setFont(BigFont);
        myGLCD.print("D=",405, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(kd2,375, 100,3,'0');
        myGLCD.printNumI(ki1,200, 240,3,'0');
        myGLCD.printNumI(kd1,375, 240,3,'0');
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(80,28,390,55);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(80,163,394,190);
        myGLCD.printNumI(kp1,25, 240,3,'0');
        updateScreen = false;
      }
        myGLCD.printNumI(kp1,25, 240,3,'0');
        
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kp1 = kp1 + 1;
        if (kp1 >= 255)
        {
          kp1 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kp1 = kp1 - 1;
        if (kp1 <= 0)
        {
          kp1 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 172), kp1);
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_I;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_TOP_I:
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("I=",230, 205);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("P=",55, 205);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(kp1,25, 240,3,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(ki1,200, 240,3,'0');
        
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        ki1 = ki1 + 1;
        if (ki1 >= 255)
        {
          ki1 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        ki1 = ki1 - 1;
        if (ki1 <= 0)
        {
          ki1 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 173), ki1);
        reflowState = REFLOW_STATE_MENU_TOP_D;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_TOP_D:
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("D=",405, 205);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("I=",230, 205);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(ki1,200, 240,3,'0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(kd1,375, 240,3,'0');
      if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kd1 = kd1 + 1;
        if (kd1 >= 255)
        {
          kd1 = 255;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        kd1 = kd1 - 1;
        if (kd1 <= 0)
        {
          kd1 = 0;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button)>100)
      {        
        ms_button =  millis();
        button_state = false;
        EEPROM.write(((currentProfile - 1) * 6 + 174), kd1);        
        myGLCD.clrScr();
        reflowState = REFLOW_STATE_IDLE;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
      {
        ms_button =  millis();
        myGLCD.clrScr();
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
      
      //фиксируем размер стола
    case REFLOW_STATE_TABLE_SIZE:
      if (tableSize == 1) {
       digitalWrite(P1_PIN, HIGH);
       }
      if (tableSize == 2) {
       digitalWrite(P1_PIN, HIGH);
       digitalWrite(P2_PIN, HIGH);
       }
      if (tableSize == 3) {
       digitalWrite(P1_PIN, HIGH);
       digitalWrite(P2_PIN, HIGH);
       digitalWrite(P3_PIN, HIGH);
       }
       reflowState = REFLOW_STATE_STEP_RAMP;
      break;
      
//"Старт и процесс пайки", рост температуры с заданной скоростью
    case REFLOW_STATE_STEP_RAMP:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
		myGLCD.setColor(VGA_GREEN);
        myGLCD.textRus("ШАГ",5, 300);
        myGLCD.textRus("П-",90, 300);
		myGLCD.setFont(BigFont);
        myGLCD.print("P",228, 232);
        myGLCD.print("I",228, 250);
        myGLCD.print("D",228, 268);
		myGLCD.print("kD",135, 268);

		
		myGLCD.print("P",228, 119);
		myGLCD.setColor(VGA_LIME);
        myGLCD.print("I",228, 137);
		myGLCD.setColor(VGA_GREEN);
        myGLCD.print("D",228, 155);
		myGLCD.print("kD",135, 155);

        //myGLCD.textRus("НАГРЕВ",235, 153);
        //myGLCD.textRus("НАГРЕВ",235, 266);
        myGLCD.setFont(BigFont);
		myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(temperatureStep[editStep],100, 115,3,'0');
        myGLCD.printNumI(SP2,100, 227,3,'0');
        updateScreen = false;
      }
        if (profileName == 1){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0,255,50);
        myGLCD.textRus("СНЯТИЕ БЕССВИНЕЦ",200, 300);
        }
        if (profileName == 2){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0,255,50);
        myGLCD.textRus(" СНЯТИЕ СВИНЕЦ   ",200, 300);
        }
        if (profileName == 3){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0,255,50);
        myGLCD.textRus("УС-ВКА БЕССВИНЕЦ",200, 300);
        }
        if (profileName == 4){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0,255,50);
        myGLCD.textRus("УС-ВКА СВИНЕЦ",200, 300);
        }
        if (profileName == 5){
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0,255,50);
        myGLCD.textRus("НИЖНИЙ ПОДОГРЕВ",200, 300);
		
      }
       
      if (tc2 >= SP2-2 && !TopStart) TopStart = true;  //если температура низа вышла на уставку включаем верхний нагреватель
      
      if (TopStart == true){   // включен верхний нагреватель      
      if (flag == 0)           //фиксируем стартовую температуру
       {
        startTemp = tc1;
        flag = 1;
       }
//устанавливаем нужный шаг, до которой нагрета плата      
      if (startTemp > temperatureStep[currentStep -1]){
        for (x=1; startTemp > temperatureStep[currentStep - 1]; currentStep++){
          x++;
       }
       }      
      if (currentStep > x && flag == 1){
        flag = 0;
        startTemp = temperatureStep[currentStep - 2];        
        flag = 1;
       }
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(currentStep,60, 300);
        //myGLCD.setFont(SevenSegNumFont);
       // myGLCD.printNumI(temperatureStep[currentStep - 1],90, 120);
      
//счётчик скорости роста температуры             
      if ((currentMillis - previousMillis) > 1000 * rampRateStep[currentStep - 1] * 0.1) //скорость роста температуры от 0.1с. до 3с.
       {
        previousMillis = currentMillis;
        counter = counter + 1;
        setpointRamp = counter + startTemp;
        //myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        //myGLCD.printNumI(setpointRamp,420, 300,3,'0');
        myGLCD.printNumI(setpointRamp,100, 115,3,'0');
        Setpoint1 = setpointRamp;
		Setpoint1d = Setpoint1;
       }
       } //закрывам скобку
 
      if (setpointRamp >= temperatureStep[currentStep - 1] - 2) //если достигли нужной температуры
       {
       // myGLCD.setColor(VGA_RED);
       // myGLCD.printNumI(temperatureStep[currentStep - 1],420, 300);
        reflowState = REFLOW_STATE_STEP;
       }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>50)
       {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
        updateScreen = true;
       }
      break;
      
    case REFLOW_STATE_STEP:
      Setpoint1 = temperatureStep[currentStep - 1];
	  Setpoint1d = Setpoint1;
     if (Input1 >= temperatureStep[currentStep - 1] - 2)
       {
        counter = 0;
        reflowState = REFLOW_STATE_STEP_DWELL;        
       }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
       {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        updateScreen = true;
        
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
       }
      break;
//считаем время перехода на следующий шаг
    case REFLOW_STATE_STEP_DWELL:  
      if (currentMillis - previousMillis > 1000) 
       {        
        previousMillis = currentMillis;
        dwellTimerStep[currentStep - 1] = dwellTimerStep[currentStep - 1] - 1;
        TimerStep = dwellTimerStep[currentStep - 1];
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(TimerStep,120, 300,2,'0');
       }
      if (dwellTimerStep[currentStep - 1] == 0) //если счётчик равен установленному времени
       {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(VGA_RED);
        myGLCD.printNumI(TimerStep,120, 300,2,'0');
        //Serial.println(TimerStep);
        tone(buzzerPin, 1045, 500);  //звуковой сигнал
        //counter = 0;
        setpointRamp = 0;
        
      if (profileSteps == currentStep) //если достигли последнего шага 
       {
          currentStep = 1;
          x = 1;      //устанавливаем переменную в исходное состояние
          flag = 0;   //после завершения профиля сбрасываем флаг
          reflowState = REFLOW_STATE_COMPLETE;
       }
      else //если шаг не последний
       {          
          currentStep++; //переходим на следующий шаг
          myPID1.SetOutputLimits(min_pwr_TOPStep[currentStep - 1], max_pwr_TOPStep[currentStep - 1]);
          reflowState = REFLOW_STATE_STEP_RAMP;          
       }
        }
      if (cancelSwitchState == HIGH && ( millis() - ms_button)>60)
       {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        updateScreen = true;
        
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
       }
      break;
//завершение пайки      
    case REFLOW_STATE_COMPLETE:
       digitalWrite(P1_PIN, LOW);
       digitalWrite(P2_PIN, LOW);
       digitalWrite(P3_PIN, LOW);
       digitalWrite(P4_PIN, LOW);
                  
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;      
      updateScreen = true;
      break;
     }
//включение нагревателей
    if (reflowStatus == REFLOW_STATUS_ON)
     {
    if (millis() > nextRead1)
     {
      Input1 = Input_f1 = filter1(thermocouple1.readCelsius());
      Input2 = Input_f2 = filter2(thermocouple2.readCelsius());
      tc1 = Input1;
      tc2 = Input2;
	  myGLCD.setColor(VGA_YELLOW);
	  myGLCD.setFont(BigFontRus);
	  myGLCD.printNumF((Input_f1 - Input_ft1) * 1000 / (millis()-prev_millis),2,92,133,',',5);
	  myGLCD.printNumF((Input_f2 - Input_ft2) * 1000 / (millis()-prev_millis),2,92,245,',',5);
	  //Serial.println();
		prev_millis = millis();
	  Input_ft1 = Input_f1;
	  Input_ft2 = Input_f2;
	  Icontrol();
      
      //sprintf (buf, "%03d %03d %03f %03f\r\n", int(Output1), int(Output2), tc1, tc2);
      //Serial.println(filter2(thermocouple2.readCelsius()));
      myGLCD.setColor(255,80,000);

	  myGLCD.setFont(BigFontRus); 
	  myGLCD.printNumI((Input_f2 - floor(Input_f2))*100,443, 268, 2,'0');
	  myGLCD.printNumI((Input_f1 - floor(Input_f1))*100,443, 155, 2,'0');

	 //--------------------------------
      myGLCD.setColor(VGA_YELLOW);
	 //   myGLCD.textRus("НАГРЕВ",235, 153);
     //   myGLCD.textRus("НАГРЕВ",235, 266);

	  myGLCD.printNumF(myPID2.proportional,2,241, 232,',',6,' ');
	  myGLCD.printNumF(myPID2.integral,2,241, 250,',',6,' ');
	  myGLCD.printNumF(myPID2.differential,2,241, 268,',', 6,' ');
	  myGLCD.printNumI(myPID1.GetKd()*10,170, 155,3,'0');
	  myGLCD.printNumF(myPID1.proportional,2,241, 119,',',6,' ');
	  myGLCD.printNumF(myPID1.integral,2,241, 137,',',6,' ');
	  myGLCD.printNumF(myPID1.differential,2,241, 155,',', 6,' ');
	  myGLCD.printNumI(myPID2.GetKd()*10,170, 268,3,'0');
	  myGLCD.printNumF(Output2,1,365, 198,',',5,' ');
	  myGLCD.printNumF(Output1,1,365, 84,',',5,' ');
	  
	  // графики---------------------------------------
	  sprintf(buf, "$%d %d;", int(Input_f2*10), int(Input_f1*10)); // график ПК
	 

	  if (Cgr == 1) //скорость графика в условии
	  {
		  myGLCD.setColor(0,0,0);
		  myGLCD.drawLine(Xgr, 186, Xgr, 219);
  		  myGLCD.drawLine(Xgr, 73, Xgr, 106);
		  myGLCD.setColor(30,30,30);
		  myGLCD.drawPixel(Xgr, 203);
		  myGLCD.drawPixel(Xgr, 90);
		  myGLCD.setColor(0,0,255);
		  myGLCD.drawPixel(Xgr, 219-round(Output2 / 3));
		  myGLCD.drawPixel(Xgr, 106-round(Output1 / 3));

		  myGLCD.setColor(20,110,50);
		  myGLCD.drawLine(Xgr+1, 186, Xgr+1, 219);
  		  myGLCD.drawLine(Xgr+1, 73, Xgr+1, 106);

		  Xgr++;
		  if (Xgr >= 195) 
		  {
			  Xgr = 2;
	  		  myGLCD.setColor(0,0,0);

			  myGLCD.drawLine(195, 186, 195, 219);
			  myGLCD.drawLine(195, 73, 195, 106);
		  }
		  
		  Cgr=0;
	  } Cgr++;
	  
	  if (CTgr == 1) //скорость графика в условии
	  {
		  myGLCD.setColor(0,0,0);
  		  myGLCD.drawLine(XTgr, 2, XTgr, 71);
		  myGLCD.setColor(30,30,30);
		  myGLCD.drawPixel(XTgr, 12);
		  myGLCD.drawPixel(XTgr, 22);
		  myGLCD.drawPixel(XTgr, 32);
		  myGLCD.drawPixel(XTgr, 42);
		  myGLCD.drawPixel(XTgr, 52);
		  myGLCD.drawPixel(XTgr, 62);
		  myGLCD.setColor(40,150,0);
		  myGLCD.drawPixel(XTgr, 71-round((tc2-15) / 3.4));
		  myGLCD.setColor(150,0,40);		  
		  myGLCD.drawPixel(XTgr, 71-round((tc1-15) / 3.4));
		  myGLCD.setColor(40,50,0);
  		  myGLCD.drawLine(XTgr+1, 2, XTgr+1, 71);

		  XTgr++;
		  if (XTgr >= 477) 
		  {
			  XTgr = 2;
	  		  myGLCD.setColor(0,0,0);
			  myGLCD.drawLine(477, 2, 477, 71);
		  }
		  
		  CTgr=0;
	  } CTgr++; 
	  //============================================================
      nextRead1 += SENSOR_SAMPLING_TIME;
 
    if (isnan(Input1)) {
       myGLCD.setFont(BigFont);
       myGLCD.setColor(VGA_RED);
       myGLCD.print("ERROR",360, 140);
     } else {
       myGLCD.setFont(SevenSegNumFont);
       myGLCD.setColor(VGA_RED);
       myGLCD.printNumI(tc1,345, 120,3,'0');
     }
    if (isnan(Input2)) {
       myGLCD.setFont(BigFont);
       myGLCD.setColor(VGA_RED);
       myGLCD.print("ERROR",360, 250);
     } else {
       myGLCD.setFont(SevenSegNumFont);
       myGLCD.setColor(VGA_RED);
       myGLCD.printNumI(tc2,345, 232,3,'0');
     }
     }

      myPID1.Compute();
      myPID2.Compute();
      attachInterrupt(0, Dimming, RISING); // настроить порт прерывания(0 или 1) 2й или 3й цифровой пин
      // Serial.println(reg2);
      if (tc1 == SP2-5){
       digitalWrite(P4_PIN, LOW);
       tone(buzzerPin, 1045, 500);  //звуковой сигнал
       } 
      if (tc1 >= 250)
       {
       digitalWrite(P1_PIN, LOW);
       digitalWrite(P2_PIN, LOW);
       digitalWrite(P3_PIN, LOW);
       digitalWrite(P4_PIN, LOW);
       tone(buzzerPin, 1045, 400);  //звуковой сигнал
       
       reflowStatus = REFLOW_STATUS_OFF;
       reflowState = REFLOW_STATE_IDLE;    
       updateScreen = true;
       }       
      if (!TopStart) Output1=0;    
       }
      else
       {
       digitalWrite(RelayPin1, LOW);
       digitalWrite(RelayPin2, LOW);
       }
       }
      
void Dimming()
{
  OutPWR_TOP();
  OutPWR_BOTTOM();

     // Serial.print(Secs); // ВЫВОД        
	 // Serial.println(' '); 

	 // Serial.print(out2);
     // Serial.println(' '); 
 
  if (Secs >= 100)
  {
    Serial.println(buf);
	  Secs=0;
  } else Secs++;
  
}

void Icontrol()
	{
	  if (Hselect==0) 
	  {
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        myPID1.integral = myPID1.integral + 1;
        if (myPID1.integral >= 100)
        {
          myPID1.integral = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        myPID1.integral = myPID1.integral - 1;
        if (myPID1.integral <= 0)
        {
         myPID1.integral = 0;
        }
       }
	  } 
	  
	  if (Hselect==1) {
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        myPID2.integral = myPID2.integral + 1;
        if (myPID2.integral >= 100)
        {
          myPID2.integral = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
        ms_button =  millis();
        myPID2.integral = myPID2.integral - 1;
        if (myPID2.integral <= 0)
        {
         myPID2.integral = 0;
        }
       }		  
		  
	  }
	
	  if (Hselect==2) {
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
	    if (myPID1.GetKd()*10 < 254)
        {
        ms_button =  millis();
        myPID1.SetTunings(myPID1.GetKp(), myPID1.GetKi(),(myPID1.GetKd()*10+1)/10.0);

        
        }
      }
       if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
	    if (myPID1.GetKd() > 0)
        {
        ms_button =  millis();
        myPID1.SetTunings(myPID1.GetKp(), myPID1.GetKi(),(myPID1.GetKd()*10-1)/10.0);

        
        }
      }		  
		  
	  }
	  if (Hselect==3) {
       if (upSwitchState == HIGH && ( millis() - ms_button)>100)
      {
	    if (myPID2.GetKd()*1 < 254)
        {
        ms_button =  millis();
        myPID2.SetTunings(myPID2.GetKp(), myPID2.GetKi(),(myPID2.GetKd()*10+1)/10.0);

        
        }
      }
       if (downSwitchState == HIGH && ( millis() - ms_button)>100)
      {
	    if (myPID2.GetKd() > 0)
        {
        ms_button =  millis();
        myPID2.SetTunings(myPID2.GetKp(), myPID2.GetKi(),(myPID2.GetKd()*10-1)/10.0);

        
        }
      }		  
		  
	  }	  
	  
	if (okSwitchState == HIGH)
      {        
        myGLCD.setFont(BigFont);
        
       // button_state = false;
        if (Hselect==0) 
		{
		myGLCD.setColor(VGA_GREEN);
        myGLCD.print("I",228, 137);
		myGLCD.setColor(VGA_LIME);        
		myGLCD.print("I",228, 250);

        }
		
        if (Hselect==1) 
		{
		myGLCD.setColor(VGA_GREEN);
        myGLCD.print("I",228, 250);
		myGLCD.setColor(VGA_LIME);        
		myGLCD.print("kD",135, 155);  
		  
        }
        if (Hselect==2) 
		{
		myGLCD.setColor(VGA_GREEN);
        myGLCD.print("kD",135, 155);
		myGLCD.setColor(VGA_LIME);        
		myGLCD.print("kD",135, 268);		    
        }
		
        if (Hselect>=3) 
		{
		myGLCD.setColor(VGA_LIME);
        myGLCD.print("I",228, 137);
		myGLCD.setColor(VGA_GREEN);       
		myGLCD.print("kD",135, 268);		    
        }
        Hselect++;
		if (Hselect==4) Hselect=0;
      }
	}

void OutPWR_TOP(){
       reg1 = round(Output1) + er1; //pwr- задание выходной мощности в %,в текущем шаге профиля, er- ошибка округления   
        if (reg1 < 50){
          out1 = LOW;
          er1 = reg1; // reg- переменная для расчетов
          }
        else {
          out1 = HIGH;
          er1 = reg1-100;
        }
       digitalWrite(RelayPin1,out1);//пин через который осуществляется дискретное управление   
       }
       
void OutPWR_BOTTOM(){
       reg2 = round(Output2) + er2; //pwr- задание выходной мощности в %, er- ошибка округления
        if (reg2 < 50){
          out2 = LOW;
          er2 = reg2; // reg- переменная для расчетов
        }
        else {
          out2 = HIGH;
          er2 = reg2-100;
        }
       digitalWrite(RelayPin2,out2);//пин через который осуществляется дискретное управление
       }
       
  float filter1(float val_1) {  //функция фильтрации показений "Верхней" термопары
  Pc1 = P1 + varProcess1;
  G1 = Pc1/(Pc1 + varTerm1);
  P1 = (1-G1)*Pc1;
  Xp1 = Xe1;
  Zp1 = Xp1;
  Xe1 = G1*(val_1-Zp1)+Xp1; // "фильтрованное" значение
  return(Xe1);
  }
  float filter2(float val_2) {  //функция фильтрации показений "Нижней" термопары
  Pc2 = P2 + varProcess2;
  G2 = Pc2/(Pc2 + varTerm2);
  P2 = (1-G2)*Pc2;
  Xp2 = Xe2;
  Zp2 = Xp2;
  Xe2 = G2*(val_2-Zp2)+Xp2; // "фильтрованное" значение
  return(Xe2);
  }
  
