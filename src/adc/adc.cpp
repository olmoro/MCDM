/*
  Версия от 27 мая 2022г.
  Измерители тока и напряжения дифференциальные.
  Датчики опрашиваются с частотой 500Hz. В перспективе 1кГц.
  По обоим датчикам максимально общие настройки. 
  Сглаживание - скользящее среднее, коэффициенты задаются.
  Три уровня сглаживания - для защиты, регулирования и отображения (не реализовано).
  Приборные смещение и фактор пересчета в физические величины задаются.

  Для справки:
  D:\Users\MORO\.platformio\packages\framework-arduino-samd\bootloaders\sofia\Bootloader_D21_Sofia_V2.1\src\ASF\sam0\utils\cmsis
  http://microsin.net/programming/avr/what-is-asf-atmel.html
  enum gains      { GAIN_1X = 0x00, GAIN_2X, GAIN_4X, GAIN_8X, GAIN_16X, GAIN_DIV2 };
  enum references { INTREF  = 0x00, INTVCC0,INTVCC1, AREFA, AREFB };    
  Internal Bandgap Reference, 1/1.48 VDDANA, 1/2 VDDANA, External Reference A, External Reference B
*/

#include <Arduino.h>
#include "atsamd21_adc.h"
#include "adc/adc.h"
#include "power/power_reg.h"
#include "board/mpins.h"
#include "board/mboard.h"
#include "merrors.h"
#include "stdint.h"

constexpr uint16_t measurement_period = 1000UL;                   // Период запуска измерителя в микросекундах
constexpr uint16_t pid_period = 100000UL / measurement_period;  // Период запуска pid-регулятора в тактах измерителя
//uint32_t ts;                                              // таймер отсчета времени одного слота nu

#ifdef DEBUG_ADC_TIME
  unsigned long oldTime;                                    // Таймер проверки периода запуска ПИД-регулятора
#endif

  // Начальное смещение ADC установленного субмодуля SAMD21 MINI
#ifdef COM4
  constexpr int16_t adc_offset_default = 2;   //  2022.05.27 (COM4) - восстановленный SAMD21 MINI
#endif
#ifdef COM7
  constexpr int16_t adc_offset_default = 10;  //  2022.04.6 (COM7)
#endif
int16_t adcOffset = adc_offset_default;

// Данные аппаратной поддержки измерителя напряжения:
  constexpr float R77     =    39.0;   // кОм (верхнее плечо входного делителя напряжения)
  constexpr float R78     =     4.7;   // кОм (нижнее плечо)
  constexpr float AREF    =  2495.0;   // mV   (внешний источник опорного напряжения)
  constexpr float GAINU   =     1.0;   // усиление
  constexpr float ADCMAXU =  2048.0;   // (для дифференциального режима)
  constexpr float SHL10U  =  1024.0;   // множитель для целочисленных вычислений 2^10

// Ожидаемый коэффициент преобразования в милливольты        
//constexpr int16_t factor_default_u = int16_t(((R77+R78)/R78) * (AREF/ADCMAXU) * SHL10U);  // 0x2D4F
// Фактическое значение для платы V53 с учетом погрешностей комплектующих
#ifdef COM4
  constexpr int16_t factor_default_u = 0x2D7D;    //  2022.05.27 (калибровано на 4,47в )
#endif
#ifdef COM7
  constexpr int16_t factor_default_u = 0x2DE0;    //  2022.04.16
#endif
int16_t factorU = factor_default_u;

constexpr uint16_t smooth_default_u = 3;          // параметр сглаживания для измерителя напряжения
uint8_t smoothU = smooth_default_u;

// Данные аппаратной поддержки измерителя тока:
  constexpr float KSHUNT  =  1000.0/20.0;         // mA/mV (1A/20mV) параметр шунта (два по R04 в параллель )
  constexpr float INTREF  =  1000.0;              // mV  (внутренний источник опорного напряжения)
  constexpr float GAINI   =     2.0;              // усиление
  constexpr float ADCMAXI =  2048.0;              // (для дифференциального режима)
  constexpr float SHL4I  =     16.0;              // множитель для целочисленных вычислений 2^4

// Ожидаемый коэффициент преобразования в миллиамперы        
//constexpr int16_t factor_default_i = int16_t((INTREF * GAINI/ADCMAXI) * KSHUNT * SHL4I);  // 0x030D

// Фактическое значение для платы V53 с учетом погрешностей комплектующих
#ifdef COM4
  constexpr uint16_t factor_default_i = 0x030C;   //  2022.05.27 ( неуточненное значение )
#endif
#ifdef COM7
  constexpr uint16_t factor_default_i = 0x030C;
#endif
uint16_t factorI = factor_default_i;

constexpr uint8_t smooth_default_i = 3;           // параметр сглаживания для измерителя тока
uint8_t smoothI = smooth_default_i;

  // приборные смещения
#ifdef COM4
  constexpr int16_t offset_default_u = 0x0000;    //  ( неуточненное значение )
  constexpr int16_t offset_default_i = -135;      //  ( неуточненное значение )
#endif
#ifdef COM7
  constexpr int16_t offset_default_u = 0x0000;
  constexpr int16_t offset_default_i = -135;      //  2022.04.16
#endif
int16_t offsetU = offset_default_u;
int16_t offsetI = offset_default_i;

  // Данные АЦП без фильтрации с компенсацией смещения
int16_t adcVoltage = 0x0000;  
int16_t adcCurrent = 0x0000;
int16_t adcCelsius = 0x0000;

  // Пересчитанные в физические величины - mV, mA (как пример)
int16_t mvVoltage  = 0x0064;                      //  0.10V
int16_t maCurrent  = 0xfc17;                      // -1.00A

  // Пороги отключения: мВ, мА  (имитация аппаратной поддержки)
int16_t winLtU = win_less_default_u;
int16_t winUpU = win_up_default_u;
int16_t winLtI = win_less_default_i;
int16_t winUpI = win_up_default_i;

// Максимальное значение ADC после автоматической обработки
// при выборе 16-битного разрешения (adcBits=0x01)
//constexpr uint16_t maxVal = 4096;

//int16_t measureU();
//int16_t measureI();

  // Инициализация измерений
void initMeasure()
{
  // sets the ADC clock relative to the peripheral clock. pg 864
  analogPrescaler(2);   //25...30µs                       //     (uint8_t val) 0 - /4 ... 7 - /512 (0.43ms, default)

  analogGain( 0x00 );
  analogReadConfig( 0x00, 0x00, 0x00 );                   // uint8_t bits, uint8_t samples, uint8_t divider
  analogReferenceCompensation(0);                         // автокомпенсация начального смещения выключена

  smoothU = SetSmoothDefaultU();
  smoothI = SetSmoothDefaultI();
  adcOffset = adc_offset_default;
  factorU = SetFactorDefaultU();
  factorI = SetFactorDefaultI();
  offsetU = SetOffsetDefaultU();
  offsetI = SetOffsetDefaultI();
}

  // Инициализация и преобразование по входу датчика напряжения с учетом смещения АЦП
int16_t adcU()
{
  analogGain( 0x00 );           // +5us
  analogReference2( 0x03 );    // выбор опорного REFA
  // если analogReferenceCompensation(1) то без -adcOffset
  adcVoltage = analogDifferentialRaw( MPins::bat_plus_mux, MPins::bat_minus_mux ) - adcOffset;    // Входы 4, 5
  return adcVoltage;
}
  // Инициализация и преобразование по входу датчика тока с учетом смещения АЦП
int16_t adcI()
{
  analogGain( 0x01 );
  analogReference2( 0x00 );    // выбор опорного INTREF 
  // если analogReferenceCompensation(1) то без -adcOffset
  adcCurrent = analogDifferentialRaw( MPins::shunt_plus_mux, MPins::shunt_minus_mux ) - adcOffset;  // Входы 6, 7
  return adcCurrent;
}

  // Накопление преобразований по входу датчика напряжения и вычисление среднего
int16_t averageAdcU(uint8_t smooth)
{
  static int32_t collect;

  collect -= collect >> (smooth + 4);
  collect += adcU();
  return collect >> smooth;
} 
  // Накопление преобразований по входу датчика тока и вычисление среднего
int16_t averageAdcI(uint8_t smooth)
{
  static int32_t collect;

  collect -= collect >> (smooth + 4);
  collect += adcI();
  return collect >> smooth ;
} 

  // Преобразование в милливольты
int16_t getVoltage(int16_t avg)
{
  // int16 * int16 = int32
  int32_t val = avg * factorU;
  int16_t mv = int16_t( val >> 14 );
  return mv - offsetU;
}
  // Преобразование в миллиамперы
int16_t getCurrent(int16_t avg)
{
  // int16 * int16 = int32
  int32_t val = avg * factorI;
  int16_t ma = int16_t( val >> 10 );      //
  return ma - offsetI;
}


void measure()
{
  static uint32_t ts = 0;                       // таймер отсчета времени
  static uint16_t pp = 0;                       // счетчик отсчета периода запуска пид-регулятора

//  if( millis() - ts >= period )
  if( micros() - ts >= measurement_period )
  { 
    ts += measurement_period;

    #ifdef TEST_MEASURE
      test1On();     // Метка для осциллографа
    #endif
    mvVoltage = getVoltage( averageAdcU( smoothU ) );
    tresholdUpU(mvVoltage);                     // отключения по перенапряжению (взять быстрое)
    #ifdef TEST_MEASURE
      test1Off();     // Метка для осциллографа
    #endif

    #ifdef TEST_MEASURE
      test1On();     // Метка для осциллографа
    #endif
    maCurrent = getCurrent( averageAdcI( smoothI ) );
    tresholdUpI(maCurrent);                     // Перегрузка по току  (взять надо бы побыстрее)
    #ifdef TEST_MEASURE
      test1Off();     // Метка для осциллографа
    #endif

    pp++;
    if( pp >= pid_period )
    {
      pp = 0;
      #ifdef TEST_PID
        test2Off();    // Метка для осциллографа
      #endif

        doPid( mvVoltage, maCurrent );    // fbU, fbI

      #ifdef TEST_PID
        test2On();     // Метка для осциллографа
      #endif
    }
  }
}
  
void tresholdUpU(int16_t valU)
{
  if( valU > winUpU )
  {
    // отключить нагрузку и преобразователь
    powerFailure( MErrors::voltage_out_of_win );    // или сбросить ПИД??? вместо паузы, 

    #ifdef DEBUG_ADC_U
      SerialUSB.println(" +V-ОТКЛЮЧЕНО");
    #endif
  }
}

void tresholdLTU(int16_t valU)     // Переполюсовка
{
  if( valU < winLtU )
  {
    // отключить нагрузку и преобразователь
    powerFailure( MErrors::voltage_negative );

    #ifdef DEBUG_ADC_U
      SerialUSB.println(" -V-ОБРАТНОЕ");
    #endif
  } 
}

void tresholdUpI(int16_t valI)     // Перегрузка по току заряда
{
  if( valI > winUpI )
  {
    // отключить нагрузку и преобразователь
    powerFailure( MErrors::current_out_of_win );

    #ifdef DEBUG_ADC_I
      SerialUSB.println(" +I-ОТКЛЮЧЕНО");
    #endif
  }
}

void tresholdLtI(int16_t valI)     // Перегрузка по току разряда
{
  if( valI < winLtI )
  {
    // отключить нагрузку и цепь разряда
    powerFailure( MErrors::disCurrent_out_of_win );

    #ifdef DEBUG_ADC_I
      SerialUSB.println(" -I-ОТКЛЮЧЕНО");
    #endif
  }
}


void setFactoryConfiguration(uint8_t prb)
{

}

uint16_t SetFactorDefaultU() { return factor_default_u; }
uint16_t SetFactorDefaultI() { return factor_default_i; }
int16_t  SetOffsetDefaultU() { return offset_default_u; }
int16_t  SetOffsetDefaultI() { return offset_default_i; }
uint8_t  SetSmoothDefaultU() { return smooth_default_u; }
uint8_t  SetSmoothDefaultI() { return smooth_default_i; }

// My DAC
// https://electronics.stackexchange.com/questions/409136/samd21-dac-example/411040
void syncDAC()
{
  while (1 == DAC->STATUS.bit.SYNCBUSY);
}

void dacInit()
{
  DAC->CTRLB.reg = 0x40;  // use AVCC as the reference
  syncDAC();
}

void dacWrite10bit(const uint16_t value) 
{
  DAC->CTRLB.reg = 0x40;              // DAC off
  DAC->CTRLA.reg = DAC_CTRLA_ENABLE;  // Enable DAC
  syncDAC();
  DAC->DATA.reg  = 0x3FF & value;
  syncDAC();
  DAC->CTRLB.reg = 0x43;              // DAC on
}
