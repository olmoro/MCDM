/*
  
 * Версия от 23 октября 2020г.
*/

#include "board/mboard.h"
#include "board/mpins.h"
#include "adc/adc.h"    // DAC
#include <Arduino.h>

  // state1 - состояния, true - включено:
extern bool switchStatus;         // коммутатор нагрузки ( sw_pin   )
extern bool powerStatus;          // преобразователь     (  )
// extern bool currentControlStatus;  // регулирование по току
extern bool voltageControlStatus;  // регулирование по напряжению
extern bool chargeStatus;         // заряд               (  )
extern bool dischargeStatus;      // разряд (тот же вывод, !chargeStatus )
// extern bool pauseStatus;           // пауза
extern bool pidStatus;             // pid-управление регулятором

  // Включение силовых ключей
void swPinOn()
{
  digitalWrite( MPins::sw_pin, LOW );
  switchStatus = true;
}

  // Выключение силовых ключей
void swPinOff()
{
  digitalWrite( MPins::sw_pin, HIGH );
  switchStatus = false;
}

void test1On()  { digitalWrite( MPins::test1, LOW  ); }  // Включение тестового вывода
void test1Off() { digitalWrite( MPins::test1, HIGH ); }  // Отключение  тестового вывода

void test2Low()  { digitalWrite( MPins::test2, LOW  ); }  // Включение тестового вывода
void test2High() { digitalWrite( MPins::test2, HIGH ); }  // Отключение  тестового вывода


  // Инициализация дискретных портов
void portsInit()
{
  pinMode( MPins::test1,  OUTPUT);  
  pinMode( MPins::test2,  OUTPUT);
  pinMode( MPins::sw_pin, OUTPUT);
   
  #ifdef WEMOS    // using pcb SAMD21 MINI
    pinMode( MPins::led_rx, OUTPUT);  // led_rx   = 25   no   PB03/LED1 (LED_BUILTIN, LED_RX)
    pinMode( MPins::led_tx, OUTPUT);  // led_tx   = 26   no   PA27/LED2 (LED_TX)
  #endif

  swPinOff();               // Силовые ключи нагрузки отключены
  test1Off();
  test2Low();               //
  dacInit();                // Set reference
  dacWrite10bit( 0x0000 );  //
}

  // Конфигурация режимов
void configMode(uint8_t mode)
{
  switch ( mode )
  {
  case 1:                             // U - регулирование по напряжению
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = true;               // преобразователь включить
//    powPinOn();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  case 2:                             // I - регулирование по току заряда
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = true;               // преобразователь включить
//    powPinOn();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  case 3:                             // D - регулирование по току разряда
    //dischargeStatus = true;           // коммутатор на заряд
    //chargeStatus = !dischargeStatus;
    powerStatus = false;              // преобразователь выключить
//    powPinOff();
    switchStatus = true;              // к клеммам подключить
    swPinOn();
  break;

  default:    // OFF
    //chargeStatus = true;              // коммутатор на заряд
    //dischargeStatus = !chargeStatus;
    powerStatus = false;              // преобразователь выключить
//    powPinOff();
    switchStatus = false;             // от клемм отключить
    swPinOff();
  break;
  }
}
