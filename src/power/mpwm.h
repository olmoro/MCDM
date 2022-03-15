/*
  Версия для платы v57 с подачей ШИМ на вывод PA16
  и micD21.v41, micD21.v51    - февраль 2022
*/

#ifndef _MPWM_H_
#define _MPWM_H_

#include "stdint.h"

void initPwm();
void goPwmOut();
void goPwmCool();
void writePwmOut(uint32_t value);
void writePwmCool(uint32_t value);
int  enable(unsigned int timerNumber);
//int  frequency(unsigned int timerNumber);

#endif  //!_MPWM_H_