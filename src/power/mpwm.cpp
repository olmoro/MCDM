/*
  // https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/  - отменено

  https://translated.turbopages.org/proxy_u/en-ru.ru.14bc2e5e-6224d5e4-354108d9-74722d776562/https/github.com/ocrdu/Arduino_SAMD21_turbo_PWM

  Версия май 2022
  Оба канала на 190кГц

*/

#include "power/mpwm.h"
#include "board/mpins.h"
#include "board/mboard.h"
#include "SAMD21turboPWM.h" /* const unsigned int _maxDutyCycle = 0x01FF;   // исправлено 20220507, было 1000 */
#include "adc/adc.h"    // DAC
#include <Arduino.h>

TurboPWM pwm;

  // // nu - исторические, до замены
  // constexpr uint32_t pwm_period = 48 - 1;       // 
  // uint32_t           pwmPeriod  = pwm_period;   // 1/Freq
  // constexpr bool     pwm_invert = false;        //
  // bool pwmInvert                = pwm_invert;   // 0 - активный уровень

  // Новые параметры настройки
  constexpr bool                    pwm_turbo       = true;   // turbo on/off для обоих таймеров
  constexpr unsigned int            pwm_tccdiv_out  = 1;      // делитель для таймера 0 (1,2,4,8,16,64,256,1024)
  constexpr unsigned int            pwm_tccdiv_cool = 1;      // делитель для таймера 2 (1,2,4,8,16,64,256,1024)
  constexpr unsigned long long int  pwm_steps_out   = 0x01FF; // разрешение для таймера 0 (2 to counter_size)
  constexpr unsigned long long int  pwm_steps_cool  = 0x01FF; // разрешение для таймера 2 (2 to counter_size)

  bool         pwmTurbo       = pwm_turbo;
  unsigned int pwmTccdivOut   = pwm_tccdiv_out;
  unsigned int pwmTccdivCool  = pwm_tccdiv_cool;
  unsigned int pwmStepsOut    = pwm_steps_out;
  unsigned int pwmStepsCool   = pwm_steps_cool;

  extern bool  powerStatus;

void initPwm()
{
  pwm.setClockDivider(1, pwm_turbo);           // Input clock is divided by 1 and sent to Generic Clock, Turbo is On/Off
  pwm.timer(0, pwm_tccdiv_out,  pwm_steps_out,  true);  // (OUT)  T0, divider, resolution (подстройка частоты), single-slope PWM
  pwm.timer(2, pwm_tccdiv_cool, pwm_steps_cool, true);  // (COOL) T2, divider, resolution (подстройка частоты), single-slope PWM


  //pwm.analogWrite(MPins::out_pin, 0x01FF - 0x000F);    // test 
  writePwmOut( 0x01F4 );       // test        125 = 12,5в при 3А

  pwm.analogWrite(MPins::cool_pin, 0x00B0);    // test

  swPinOn();                 // Включение нагрузки test

//  dacWrite10bit( 0x0200 );  // test 12.4v: 0x0280 -> -1.8A
}

void goPwmOut()
{
  pwm.setClockDivider(1, pwmTurbo);               // Input clock is divided by 1 and sent to Generic Clock, Turbo is On/Off
  pwm.timer(0, pwmTccdivOut, pwmStepsOut,  true); // (OUT)  T0, divider, resolution (подстройка частоты), single-slope PWM
  pwm.analogWrite(MPins::out_pin, pwmStepsOut);   // test
}

void goPwmCool()
{

}


void writePwmOut(uint32_t value)
{
  if( value == 0 )
  {
    powerStatus = false;
  }
  else
  {
    powerStatus = true;
  }

  #ifdef MIC4420
    value = 0x01FF - value;                       // Заменяет инвертирование в настройках вывода
  #endif
  pwm.analogWrite(MPins::out_pin, value);
}

void writePwmCool(uint32_t value)
{
  #ifdef MIC4420
    value = 0x01FF - value;
  #endif
  pwm.analogWrite(MPins::cool_pin, value);
}

int  enable(unsigned int timerNumber)
{
  return pwm.enable(timerNumber, true);
}
