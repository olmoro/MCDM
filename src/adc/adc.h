/*

  Версия от 27 мая 2022г.
*/

#ifndef _ADC_H_
#define _ADC_H_

#include "stdint.h"

void initMeasure();
void measure();                     // new
void tresholdUpU(int16_t valU);     // Перенапряжение
void tresholdLTU(int16_t valU);     // Переполюсовка

void tresholdUpI(int16_t valI);     // Перегрузка по току заряда
void tresholdLtI(int16_t valI);     // Перегрузка по току разряда

void setFactoryConfiguration(uint8_t prb);

int16_t  SetAdcOffsetDefault();

uint8_t  SetSmoothDefaultU();
uint8_t  SetSmoothDefaultI();
uint16_t SetFactorDefaultU();     // коэффициент преобразования в милливольты
uint16_t SetFactorDefaultI();     // коэффициент преобразования в миллиамперы
int16_t  SetOffsetDefaultU();
int16_t  SetOffsetDefaultI();

void dacInit();
void dacWrite10bit(const uint16_t value);

// Пороги отключения: мВ, мА  (имитация аппаратной поддержки)
constexpr int16_t win_less_default_u  =  -200;   // при переплюсовке
constexpr int16_t win_up_default_u    = 18000;   // исполняется конструктивно
constexpr int16_t win_less_default_i  = -1500;   // максимальный ток разряда
constexpr int16_t win_up_default_i    =  6000;   // максимальный ток заряда

#endif  //!_ADC_H_
