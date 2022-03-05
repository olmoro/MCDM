/*
  // https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/  ??
  // Output PWM 24Khz on digital pin PA16 (D11 SAMD21 MINI)
  // SAMD21 pin 11
  // using timer TCC2 (10-bit resolution), channel CH0
  // Версия от 23 октября 2020г.

VER190:
  https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
  // Note: Uses pin 16 as the out_pin   = 11;    // D11  PA16 ШИМ на силовой преобразователь                Ch 7 ?
  // Note: Uses pin 14 as the pa14_pin  =  2; //4?;    // D4   PA14 Зарезервирован для вентилятора           Ch 5 ?
*/

#include "power/mpwm.h"
#include "board/mpins.h"
#include "SAMD21turboPWM.h"
#include <Arduino.h>

TurboPWM pwm;

  // nu
  constexpr uint32_t pwm_period = 48 - 1;       // 
  uint32_t           pwmPeriod  = pwm_period;   // 1/Freq
  constexpr bool     pwm_invert = false;        //
  bool pwmInvert                = pwm_invert;   // 0 - активный уровень

void initPwm()
{
  pwm.setClockDivider(1, true);     // Input clock is divided by 1 and sent to Generic Clock, Turbo is On
  pwm.timer(2, 256, 40000, false);  // Timer 2 is set to Generic Clock divided by 256, resolution is 40000, phase-correct aka dual-slope PWM 
  pwm.timer(1, 1, 250, true);       // Timer 1 is set to Generic Clock divided by 1, resolution is 250, normal aka fast aka single-slope PWM

 //pwm.analogWrite(MPins::pa14_pin, 500);

 pwm.analogWrite(MPins::out_pin, 400);
  SerialUSB.print("PWM frequency: "); SerialUSB.print(pwm.frequency(1)); SerialUSB.println("Hz");
  SerialUSB.println("Duty cycle: 100/1000\n");
  //delay(2000);


}

void writePwm(uint16_t value)
{
  // REG_TCC2_CC0 = value;                           // TCC2 CC0 - on D11 - PWM signalling
  // while (TCC2->SYNCBUSY.bit.CC3);                 // Wait for synchronization
}
