/*
  https://manualzz.com/doc/11587480/using-atsamd21-sercom-for-more-spi--i2c-and-serial-ports
  Using ATSAMD21 SERCOM for more SPI, I2C and Serial ports

  View detail for Atmel AT11628: SAM D21 SERCOM I2C Configuration

  Вариант с UART
  05 март 2022
  Platform: 7.1.0
  VS: 1.65.0
  pcb: micD21.v51
*/

#include "board/mpins.h"
#include "wake/wake.h"
#include "commands/commands.h"
#include "power/power_reg.h"
#include "adc/adc.h"
    //#include "power/power_pwm.h"
    #include "power/mpwm.h"

#include <Arduino.h>            // N. порядок не нарушать!
#include "wiring_private.h"     // N=1.


void setup() 
{
  // инициализация UART порта обмена с ESP32 ( D0:PA11/UART-RX, D1:PA10/UART-TX )
  Serial1.begin(115200);            // 
  while(!Serial1);

  portsInit();
  wakeInit( 0x00, 500 );            // обмен без адреса, время ожидания 500 ms
//initAdc(0);                       // не обязательно
  initMeasure();
  initPids();                       // 
  initState1();
  initState2();


  //  writePwm(0x0800);         // test
  

}

void loop() 
{
    // Измерения и регулирование
    // считать, преобразовать, задать следующее и запустить
  measure();
    // Обслуживание интерфейса
  if( Serial1.available() )
  {                             // В буфере приема есть принятые байты. Не факт, что пакет полный.
    wakeRead();                 // Пока не принят весь пакет, время ожидания ограничено (пока 1с).
  }

  doState1();
  doState2();
  doCommand();                  // Если ненулевая, будет исполнена.
}
