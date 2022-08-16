#include "ssd1306h.h"   //Библиотека дисплея
#include "MAX30102.h"   //Библиотека датчика
#include "Pulse.h"      //Библиотека для измерения пульса
#include <avr/pgmspace.h>  //Работа со строками, хранимыми во flash – памяти
#include <EEPROM.h>     //Библиотека памяти
#include <avr/sleep.h>  //Управление питанием и режимом сна

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

SSD1306 oled; 
MAX30102 sensor; //Определние переменных
Pulse pulseIR;
Pulse pulseRed;
//Фильтр для пульса
MAFilter bpm;

#define LED LED_BUILTIN //Определение светодиода
#define BUTTON 3    //Определение кнопки
#define OPTIONS 7

bool filter_for_graph = false;
bool draw_Red = false;
bool isTemp = false;
bool led_on = false;
byte pulseLED = 11; //Пин PWM
byte readLED = 13; //Мерцание при каждом чтении данных
float temp;
int  beatAvg;
int lm335=0; //Присвоение A0 для выхода термодатчика
int  SPO2, SPO2f;  //Определение переменных
int  voltage;       
long lastBeat = 0;  //Время последнего удара 
long displaytime = 0; //Время последнего обновления дисплея
short displayNumber = 0; // Присвоение номера дисплею 
uint8_t pcflag =0;  //Флаг
uint8_t istate = 0;           
uint8_t sleep_counter = 0;    //Счётчик для сна
const uint8_t MAXWAVE = 72;

static const uint8_t heart_bits[] PROGMEM = { 0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0xfe, 0xff, 
                                        0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0, 0x0f,
                                        0xc0, 0x07, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00 }; //Размещение данных пульса во флэш-памяти

//spo2_table аппроксимируется как  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t spo2_table[184] PROGMEM =
        { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
          99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
          100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
          97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
          90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
          80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
          66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
          49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
          28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
          3, 2, 1 } ; //Размещение данных уроаня кислорода во флэш-памяти

int getVCC() 
{
  //считывает внутреннее опорное напряжение 1V1 относительно VCC
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(2); // Подождите, пока Vref установится
  ADCSRA |= _BV(ADSC); // Конвертировать
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL;
  unsigned int val = (ADCH << 8) | low;
  //отмена предыдущего результата
  ADCSRA |= _BV(ADSC); // Конвертировать
  while (bit_is_set(ADCSRA, ADSC));
  low = ADCL;
  val = (ADCH << 8) | low;
  
  return (((long)1024 * 1100) / val)/100;  
}

void print_digit(int x, int y, long val, char c=' ', uint8_t field = 3,const int BIG = 2)
    {  
    uint8_t ff = field;
    do { 
        char ch = (val!=0) ? val%10+'0': c;
        oled.drawChar( x+BIG*(ff-1)*6, y, ch, BIG);
        val = val/10; 
        --ff;
    } while (ff>0);
}

/*
 * Запись, масштабирование и отображение PPG Waveform
 */
class Waveform
{
  public:
  Waveform(void) 
  {
    wavep = 0;
  }
  void record(int waveval)
  {
    // масштабирование для размещения в байте
    waveval = waveval/8; 
    //сдвиг, чтобы введенная форма волны была +ve          
    waveval += 128;              
    waveval = waveval<0? 0 : waveval;
    waveform[wavep] = (uint8_t) (waveval>255)?255:waveval; 
    wavep = (wavep+1) % MAXWAVE;
  }

  //Функция масштабированрия
  void scale()  
  {
    uint8_t maxw = 0;
    uint8_t minw = 128;
    for (int i=0; i<MAXWAVE; i++) 
    { 
      maxw = waveform[i]>maxw?waveform[i]:maxw;
      minw = waveform[i]<minw?waveform[i]:minw;
    }
    //масштаб * 8 для сохранения точности
    uint8_t scale8 = (maxw-minw)/4 + 1;  
    uint8_t index = wavep;
    for (int i=0; i<MAXWAVE; i++)
    {
      disp_wave[i] = 31-((uint16_t)(waveform[index]-minw)*8)/scale8;
      index = (index + 1) % MAXWAVE;
    }
  }

void draw(uint8_t X) 
{
  for (int i=0; i<MAXWAVE; i++) 
  {
    uint8_t y = disp_wave[i];
    oled.drawPixel(X+i, y);
    if (i<MAXWAVE-1) 
    {
      uint8_t nexty = disp_wave[i+1];
      if (nexty>y) 
      {
        for (uint8_t iy = y+1; iy<nexty; ++iy)  
        oled.drawPixel(X+i, iy);
      } 
      else if (nexty<y) 
      {
        for (uint8_t iy = nexty+1; iy<y; ++iy)  
        oled.drawPixel(X+i, iy);
      }
    }
  } 
}
private:
    uint8_t waveform[MAXWAVE];
    uint8_t disp_wave[MAXWAVE];
    uint8_t wavep = 0;
} 
wave;

//Функция кнопки
void button(void) 
{
  isTemp = !isTemp;
  Serial.println(isTemp);
}

//Функция для пятого дисплея
void Display_5() 
{
draw_oled(5);
}

void go_sleep()
{
  oled.fill(0);
  oled.off();
  delay(10);
  sensor.off();
  delay(10);
  cbi(ADCSRA, ADEN);  // отключение ADC
  delay(10);
  pinMode(0,INPUT);
  pinMode(2,INPUT);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // выключение до нажатия кнопки     
  sleep_mode();
  // Вызывает перезапуск
  setup();
}

 //Функция дисплеев
void draw_oled(int msg)
{
  oled.firstPage();
  do
  {
    switch(msg)
    {
              //Если не подключено устройство
    case 0:  oled.drawStr(10,0,F("Device error"),1); 
                 break;

              //"Приложите ваш палец"
    case 1:  oled.drawStr(0,0,F("PLACE YOUR"),2);  
             oled.drawStr(25,18,F("FINGER"),2);    
             break;
             
              //Пульс и уровень кислорода
    case 2:  print_digit(86,0,beatAvg);            
             oled.drawStr(0,3,F("Pulse"),1);
             oled.drawStr(3,17,F("SpO2"),1);
             oled.drawStr(0,25,F("level"),1);
             print_digit(73,16,SPO2f,' ',3,2);
             oled.drawChar(116,16,'%',2);
             break;
             
              //Заставка при запуске   
    case 3:  oled.drawStr(33,0,F("Pulse"),2); 
             oled.drawStr(17,15,F("Oximeter"),2);
             break;
              
              //Отсчёт до сна
    case 4:  oled.drawStr(28,12,F("OFF IN"),1); 
             oled.drawChar(76,12,10-sleep_counter/10+'0');
             oled.drawChar(82,12,'s');
             break;
             
              //Вывод температуры
    case 5:  oled.drawStr(0,16,F("Temerature"),1); 
             print_digit(60,12,temp);
             oled.drawStr(93,12,F(","),2);             
             oled.drawStr(106,12,F("C"),2); 
             break;
    }
  } 
  while (oled.nextPage());
}

void setup(void) 
{
    //Скорость порта
  Serial.begin(115200);
    //Установка скорости I2C на 400 кГц    
  Wire.setClock(400000);  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  filter_for_graph = EEPROM.read(OPTIONS);
  draw_Red = EEPROM.read(OPTIONS+1);
  oled.init();
  oled.fill(0x00);
  draw_oled(3);
  delay(3000);
   
  if (!sensor.begin())  
  {
    draw_oled(0);
    while (1);
  }
  sensor.setup(); 
    //Прерывание
  attachInterrupt(digitalPinToInterrupt(BUTTON),button, RISING); 
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
}

void loop()
{
  sensor.check();
   //время начала данного цикла
  long now = millis();  
  if (!sensor.available()) return;
  uint32_t irValue = sensor.getIR(); 
  uint32_t redValue = sensor.getRed();
  sensor.nextSample();
  if (irValue<5000) 
  {
    voltage = getVCC();
      // finger not down message
    draw_oled(sleep_counter<=50 ? 1 : 4); 
    delay(200);
    ++sleep_counter;
    if (sleep_counter>100)
    {
      go_sleep(); 
      sleep_counter = 0;
    }
  } 
  else
  {
    sleep_counter = 0;
    //Удаление компонентов постоянного тока
    int16_t IR_signal, Red_signal;
    bool beatRed, beatIR;
    if (!filter_for_graph) 
    {//графические фильтры
      IR_signal =  pulseIR.dc_filter(irValue) ;
      Red_signal = pulseRed.dc_filter(redValue);
      beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
      beatIR =  pulseIR.isBeat(pulseIR.ma_filter(IR_signal));        
    } 
    else
    {
      IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue)) ;
      Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
      beatRed = pulseRed.isBeat(Red_signal);
      beatIR =  pulseIR.isBeat(IR_signal);
    }
    // инвертирование формы волны для получения классической формы волны ВР
    wave.record(draw_Red ? -Red_signal : -IR_signal ); 
    // проверка ИК или красный цвет на наличие сердцебиения     
    if (draw_Red ? beatRed : beatIR)
    {
      long btpm = 60000/(now - lastBeat);
      if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
      lastBeat = now; 
      digitalWrite(LED, HIGH); 
      led_on = true;
      
      // рассчитать соотношение SpO2
      long numerator   = (pulseRed.avgAC() * pulseIR.avgDC())/256;
      
      long denominator = (pulseRed.avgDC() * pulseIR.avgAC())/256;
      int RX100 = (denominator>0) ? (numerator * 100)/denominator : 999;
      // использование формулы
      SPO2f = (10400 - RX100*17+50)/100;  
      // из таблицы 

        // Чтение данных с аналогового вывода             
      double val = analogRead(lm335);
        // Преобразование значения в напряжение   
      double voltage = val*5.0/1024; 
        // Преобразование напряжения в температру в цельсиях   
      temp = voltage*100 - 273.15; 
      Serial.println (temp);
      
      if(isTemp)
      { 
        //Указание, какой дисплей использовать
        Display_5();
      }
      else
      {
      draw_oled(2);
      }
    }
    // обновление дисплея каждые 50 мс при падении пальца
    if (now-displaytime>50)
    {
      displaytime = now;
      wave.scale(); 
    }
    
    if(isTemp)
    {       
      Display_5();
    }
    else
    {
      draw_oled(2);
    }
  }
  // вспышка светодиода в течение 25 мс
  if (led_on && (now - lastBeat)>25)
  {
    digitalWrite(LED, LOW);
    led_on = false;
  }       
}
