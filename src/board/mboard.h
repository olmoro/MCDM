#ifndef _MBOARD_H_
#define _MBOARD_H_

/*
 *
 * Версия от 23 октября 2020г.
 */

#include "stdint.h"

  // Управление дискретными портами
void swPinOn();                 // Включение нагрузки
void swPinOff();                // Отключение нагрузки

void test1On();                // Включение (для осциллографирования)
void test1Off();               // Отключение

void test2On();                // Включение (для осциллографирования)
void test2Off();               // Отключение

void portsInit();               // Инициализация дискретных портов

void configMode(uint8_t mode);  // Конфигурация режимов

#endif  //_MBOARD_H_
