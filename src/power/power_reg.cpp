/*
  Вдохновение от:

  https://github.com/mike-matera/FastPID/tree/master/examples/VoltageRegulator
  FastPID: A fast 32-bit fixed-point PID controller for Arduino

  Подбор коэффициентов ПИД-регулятора
  
  версия от 21 июня 2022г.
*/

#include <Arduino.h>
#include "board/mboard.h"
#include "adc/adc.h"      // DAC
#include "wake/wake.h"
//#include "eeprom/drvData.h"
#include "power/mpwm.h"
#include "power/power_reg.h"
#include "commands/commands.h"
#include "power/mpid.h"
#include "stdint.h"

  // Переменные протокола Wake 
extern uint8_t  rxNbt;              // принятое количество байт в пакете
extern uint8_t  rxDat[frame];       // массив принятых данных
extern uint8_t  command;            // код команды на выполнение
extern uint8_t  txNbt;              // количество байт данных в пакете
extern uint8_t  txDat[frame];       // массив данных для передачи

extern uint8_t state1;  // state1 - состояния и управление, true - включено(включить):
extern bool switchStatus;           // коммутатор нагрузки ( sw_pin PA15 )
extern bool powerStatus;            // управление преобразователем
extern bool currentControlStatus;   // регулирование по току
extern bool voltageControlStatus;   // регулирование по напряжению
extern bool chargeStatus;           // заряд
extern bool dischargeStatus;        // разряд (тот же вывод, !chargeStatus )
extern bool pauseStatus;            // пауза
extern bool pidStatus;              // pid-управление регулятором

extern uint8_t state2;  // state2
// extern bool overHeatingStatus;     // перегрев
// extern bool overloadStatus;        // перегрузка
// extern bool powerLimitationStatus; // ограничение мощности
// extern bool reversePolarityStatus; // обратная полярность
// extern bool shortCircuitStatus;    // короткое замыкание
// extern bool calibrationStatus;     // калибровка
// extern bool upgradeStatus;         // обновление
// extern bool reserve2Status;        // резерв 2

uint8_t errorCode = 0x00;

  // Пересчитанные в физические величины - mV, mA
extern int16_t mvVoltage;
extern int16_t maCurrent;

  // Приборные диапазоны задания напряжения и токов ??? согласовать 
  // с параметрами отключения (adc.cpp L80)?
  // Предварительно, для тестирования
constexpr int16_t volt_min       =  2000;  //  2.0 в
constexpr int16_t volt_max       = 16200;  // 16.2 в
constexpr int16_t curr_ch_min    =    50;  //  0.05 А
constexpr int16_t curr_ch_max    =  6000;  //  6.0 А
constexpr int16_t curr_disch_min =    50;  //  0.05 А
constexpr int16_t curr_disch_max =  3000;  //  3.0 А
constexpr float   hz             = 10.0f;  //  всегда 10

// Дефолтные параметры регулирования для всех режимов (v53)
// Это тестовые значения - задавать через целочисленные значения, используя согласованный множитель
// Вычисленные по методу Циглера-Никольса ( K=0.08  T=1 ) с уточнением.
constexpr uint16_t kp_def   =    0.06f         * MPid::param_mult;   // 15.36 0x000D
constexpr uint16_t ki_def   =  ( 0.20f / hz )  * MPid::param_mult;   //  5.12 0x0005  
constexpr uint16_t kd_def   =  ( 0.00f * hz )  * MPid::param_mult;   //  0.00 0x0000
// bits и sign заданы жестко в отличие от прототипа.

// Ограничения на output приборные, вводятся setOutputRange(min,max),
// будут в инициализации? 
constexpr int16_t min_pwm   = 0x0008;   // май 2022
constexpr int16_t max_pwm   = 0x01F4;   // 
constexpr int16_t min_dac   = 0x0020;
constexpr int16_t max_dac   = 0x03FF;   //

// Варианты настройки для разных режимов (modes) регулирования 
// напряжения, тока заряда, тока разряда (дефолтные значения)
// Для разряда (режим D) используется другой экземпляр регулятора
// Разрядность (10бит) и опорное (AVCC) DAC заданы жестко, как и частота регулирования (hz=10Hz) 

enum mode { MODE_OFF = 0, MODE_U, MODE_I, MODE_D };

// Массивы параметров настройки ПИД-регуляторов (Общие настройки)
//             mode =  MODE_OFF   MODE_U   MODE_I   MODE_D
uint16_t kP[]       = {       0,  kp_def,  kp_def,  kp_def };  
uint16_t kI[]       = {       0,  ki_def,  ki_def,  ki_def };
uint16_t kD[]       = {       0,  kd_def,  kd_def,  kd_def };
int16_t minOut[]    = { min_pwm, min_pwm, min_pwm, min_dac };
int16_t maxOut[]    = { max_pwm, max_pwm, max_pwm, max_dac };

// Заданный уровень регулирования - мВ или мА
int16_t setpoint[]  = {  0x0000,  0x0000,  0x0000,  0x0000 };

int16_t output  = 0x0000;

// Параметры компенсация всплеска напряжения минимальной подгрузкой преобразователя
int16_t surgeVoltage = 250;       // Милливольты превышения
int16_t surgeCurrent = 0x200;     // Ток в коде DAC

int16_t idleCurrent = 250;        // Миллиамперы, меньше которых необходима дополнительная нагрузка 
int16_t idleDac     = 0x240;      // Ток в коде DAC (не проверено)

// Выбор режима регулирования (не путать с датчиками)
uint8_t pidMode = MODE_OFF;   // OFF-U-I-D: выкл, задать напряжение, ток заряда или ток разряда

// Сохраненные регистры регуляторов
int16_t sLastSpU;
int64_t sSumU;
int32_t sLastErrU;

int16_t sLastSpI;
int64_t sSumI;
int32_t sLastErrI;

// Вариант с общим регулятором по напряжению и току
MPid MyPid ( kP[MODE_U], kI[MODE_U], kD[MODE_U], minOut[MODE_U], maxOut[MODE_U] );  // Common Voltage and Current control
MPid MyPidD( kP[MODE_D], kI[MODE_D], kD[MODE_D], minOut[MODE_D], maxOut[MODE_D] );  // Discharge current control


void  initPids()
{
  MyPid.configure ( kP[MODE_U], kI[MODE_U], kD[MODE_U], minOut[MODE_U], maxOut[MODE_U] );
  MyPidD.configure( kP[MODE_D], kI[MODE_D], kD[MODE_D], minOut[MODE_D], maxOut[MODE_D] );
  initPwm();

  pidMode = MODE_U;   // Test
} 

// Запуск и выбор регулятора производится выбором pidMode: MODE_OFF, MODE_U, modeI, modeD
// powerStatus = true      - Преобразователь включить (включен)
// pidStatus   = true      - Регулятор включить (включен) - оменено, включение pidMode != MODE_OFF

void doPid( int16_t fbU, int16_t fbI )
{
  //unsigned long before = micros();  // Раскомментировать 3 строки для вывода времени исполнения

  //int16_t fbU = mvVoltage;  // feedback U
  //int16_t fbI = maCurrent;
  int16_t outU; // = 0x0100;      // test
  int16_t outI; //= 0x0040;      // test
  int16_t outD; //= 0x0200;      // test 12.4v: 0x0280 -> -1.8A

  // setpoint[MODE_U] = 12000;    // test 10v
  // setpoint[MODE_I] =  800;    // test
  // pidMode          = MODE_U;

  setpoint[MODE_U] = 12000;    // test
  setpoint[MODE_I] =   610;    // test 0.61A 
  //pidMode          = MODE_I;
  //swPinOn();

  //outU = MyPid.step( setpoint[MODE_U], fbU );  // коррекция 
  //writePwmOut( MyPid.step( setpoint[MODE_U], fbU ) );

  //writePwmOut( 0x0100 ); 

  switch ( pidMode )
  {
  case MODE_OFF:
    // Выход из регулирования с отключением всего
    swPinOff();                       // отключить от выходных клемм
    writePwmOut( 0x0000 );            // преобразователь выключить
    currentControlStatus  = false;    // регулирование по току отключено
    voltageControlStatus  = false;    // регулирование по напряжению отключено
    chargeStatus          = false;    // заряд отключен
    pidStatus = false;                 // pid-регулятор выключен

      // Зарезервированная функциональность
      // dacWrite10bit( surgeCurrent );  // разрядить выходной фильтр 
      // dischargeStatus       = false;  // разряд отключен
      // pauseStatus           = false;  // пауза отключена nu
      // idleLoad();
    break;  //case MODE_OFF

  case MODE_U:
    // Если при регулировании по напряжению ток ниже заданного, то продолжать.
    // Иначе перейти к регулированию по току.
   if( fbI < (setpoint[MODE_I] - 10) )        // если ток менее заданного, но не разряд)) 
   {
      // Режим регулирования по напряжению подтвержден
      swPinOn();                        // подключение к силовым клеммам
      voltageControlStatus = true;      // регулирование по напряжению включено
      currentControlStatus = false;     // регулирование по току отключено
      dischargeStatus      = false;     // разряд отключен
      chargeStatus         = true;      // режим заряда включен
      pidStatus            = true;      // pid-регулятор включен
      outU = MyPid.step( setpoint[MODE_U], fbU );  // коррекция 
      writePwmOut( outU );
        // powerStatus          = true;           // преобразователь включен
        // pauseStatus           = false;          // пауза отключена nu
      
      // Резерв
      // surgeCompensation( -(MyPid.getLastErr()) );    // Компенсация всплеска напряжения
      // idleLoad();
    }
    else
    {
      // ток выше предела - перейти к регулированию по току
      if( pidMode )                           // если не отключено pid-регулирование
      {
        // #ifdef OSC 
        test2Off();                        // Метка для осциллографа
        // #endif
        saveState(MODE_U);                         // Сохранить регистры регулятора
        restoreState(MODE_I);                      // Перейти к регулированию по току
          MyPid.setCoefficients( kP[MODE_I], kI[MODE_I], kD[MODE_I] );
        //             //MyPid.replaceConfig( kP[MODE_I], kI[MODE_I], kD[MODE_I], minOut[MODE_I], maxOut[MODE_I]);
        //             //MyPid.configure( kP[MODE_I], kI[MODE_I], kD[MODE_I], minOut[MODE_I], maxOut[MODE_I]);
        pidMode = MODE_I;
        // #ifdef OSC 
        test2On();                         // Метка для осциллографа
        // #endif
      }
    }
    break; //case MODE_U

  case MODE_I:
    // Если при регулировании по току напряжение ниже заданного, то продолжать.
    // Иначе перейти к регулированию по напряжению.
    if( fbI >= setpoint[MODE_I] )                  // если то более или равен заданному, иначе перейти...
    {
      // Режим pid-регулирования по току
      swPinOn();           // коммутатор включен
      currentControlStatus = true;      // регулирование по току включено
      voltageControlStatus = false;     // регулирование по напряжению выключено
      chargeStatus         = true;      // заряд включен

      outI = MyPid.step( setpoint[MODE_I], fbI );
      writePwmOut( outI );
        //    powerStatus = true;          // преобразователь включен
                
      dischargeStatus      = false;      // разряд отключен
        // pauseStatus     = false;      // пауза отключена nu
      pidStatus            = true;       // регулятор включен

        //         idleLoad(); 
    }
    else
    {
      // иначе перейти к регулированию по напряжению
      if( pidMode )                           // если не отключено 
      {
        //           #ifdef OSC 
        //             tstPinOff();                        // Метка для осциллографа
        //           #endif
        saveState(MODE_I);
        restoreState(MODE_U);
        MyPid.setCoefficients( kP[MODE_U], kI[MODE_U], kD[MODE_U] );
        //                 //MyPid.replaceConfig( kP[MODE_U], kI[MODE_U], kD[MODE_U], minOut[MODE_U], maxOut[MODE_U]);
        //                 //MyPid.configure( kP[MODE_U], kI[MODE_U], kD[MODE_U], minOut[MODE_U], maxOut[MODE_U]);
        pidMode = MODE_U;
        //           #ifdef OSC 
        //             tstPinOn();                         // Метка для осциллографа
        //           #endif
      }
    }
    break;  //case MODE_I

  case MODE_D:
    // Регулирование тока разряда                             !!! ( НЕ ПРОВЕРЕНО ) !!!
    swPinOn();   // батарея подключена (не факт))

    writePwmOut( 0x0000 );
        //powerStatus           = false;  // преобразователь выключен

    currentControlStatus  = false;  // регулирование по току выключено
    voltageControlStatus  = false;  // регулирование по напряжению выключено
    chargeStatus          = false;  // заряд выключен

      outD = MyPidD.step( setpoint[MODE_I], fbI );  // коррекция ( откорректировать полярности )
    dacWrite10bit( outD );  // test 12.4v: 0x0280 -> -1.8A

    dischargeStatus       = true;   // разряд включен с регулированием по току
    //       pauseStatus           = false;  // пауза отключена
    pidStatus             = true;   // регулятор включен

    break;

  default:
    break;
  } //switch(pidMode)
  // unsigned long after = micros();
  // SerialUSB.print("runtime,us: "); SerialUSB.println((uint16_t)(after - before));
} //doPid()

// Сохранение и восстановление регистров регулятора для корректного перехода
void saveState( int mode )
{  
  switch (mode)
  {
    case MODE_U:
      sLastSpU  = MyPid.getLastSp();
//      sSumU     = MyPid.getSum();
      sLastErrU = MyPid.getLastErr();
      break;

    case MODE_I: 
      sLastSpI  = MyPid.getLastSp();
//      sSumI     = MyPid.getSum();
      sLastErrI = MyPid.getLastErr();
      break;

    default: break;
  } //
} //

void restoreState( int mode )
{
  switch (mode)
  {
    case MODE_U:
      MyPid.setLastSp( sLastSpU );
//      MyPid.setSum( sSumU );            // По невыясненным причинам выполняется некорректно
      MyPid.setLastErr( sLastErrU );
      break;

    case MODE_I: 
      MyPid.setLastSp( sLastSpI );
//      MyPid.setSum( sSumI );
      MyPid.setLastErr( sLastErrI );
      break;

    default: break;
  }
}

//   // Компенсация всплеска напряжения
// void surgeCompensation( int16_t uErr )
// {
//   uint16_t val;
//   if( uErr > surgeVoltage )
//   {
//     val = surgeCurrent;
//   }
//   else
//   {
//     val = 0x0000;
//   }
  
//   // #ifdef DEBUG_POWER
//   //   SerialUSB.println( val, HEX);
//   // #endif

//   dacWrite10bit( val );
// }

void idleLoad()
{
  if( maCurrent < idleCurrent ) 
  {   //ток мал
    dacWrite10bit( idleDac );
  }
  else
  { 
    dacWrite10bit( 0 );
  }
}



  // Отключение, в том числе и аварийное
void powerFailure(uint8_t err)
{
  swPinOff();                     // Отключение нагрузки
  switchStatus = false;
  powerStatus = false;
  //chargeStatus = false;
  //dischargeStatus = false;
  errorCode = err;
}

// Перевод в безопасное состояние
void powerStop()
{
  pidMode = MODE_OFF;       // При включенном регуляторе отключение автоматическое, ниже - дублирование
  swPinOff();               // Коммутатор отключен      (switchStatus = false;)
  writePwmOut( 0x0000 );    // Преобразователь выключен (powerStatus = false;)
  dacWrite10bit( 0x0000 );  // Разрядная цепь отключена (dischargeStatus = false;) }
}

// ======================= Команды управления процессами ======================

// 0x20 Старт заряда с заданными максимальными U и I,
// оно же и для режима источника питания.
// ПИД-регулятор(ы) должны быть сконфигурированы и инициализированы ранее.
void doPowerGo()
{
  if( rxNbt == 5 )
  {
    setpoint[MODE_U] = get16(0);  // U
    setpoint[MODE_I] = get16(2);  // I
    pidMode = MODE_U;             // U - начать с установки напряжения
//    pidStatus = true;             // Разрешить регулирование
    swPinOn();                    // Коммутатор включен      (switchStatus = true;)
    txReplay( 1, 0 );             // Команда исполнена (0x00)
  }
  else txReplay(1, err_tx);       // Сообщение об ошибке приема команды (0x01)
}

// 0x21 Стоп заряд или разряд
void doPowerStop()
{
  if( rxNbt == 0 )
  {
    powerStop();              // Перевод в безопасное состояние
    txReplay(1, 0);           // Команда исполнена (0x00)
  }
  else txReplay(1, err_tx);   // Сообщение об ошибке приема команды (0x01)
}

  // 0x22 пока не реализована
void  doSetPid()
{
  if( rxNbt == 5 )
  {
    uint8_t _id   = rxDat[0];
    //int32_t _par  = getI32(1);
    
//SerialUSB.print("  0: 0x"); SerialUSB.println( _id, HEX );

    txReplay( 1, _id );
  }
    else txReplay(1, err_tx);      // Ошибка протокола
} 

// 0x40 Тестовая. Конфигурирование пид-регулятора
void doPidConfigure()
{
  //uint8_t err = 0x00;

  if( rxNbt == 11 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);
    minOut[m] = get16(7);
    maxOut[m] = get16(9);

    switch ( m )
    {
    case MODE_U: case MODE_I: MyPid.configure( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    case MODE_D:        MyPidD.configure( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
      //SerialUSB.print(" sign: "); SerialUSB.println( signOut[m] );
    #endif
    txReplay( 1, 0 );  
  }
  else  txReplay(1, err_tx);
}

// 0x41 Тестовая: ввод коэффициентов kp, ki, kd для заданного режима
void doPidSetCoefficients()
{
  if( rxNbt == 7 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);

    switch ( m )
    {
    case MODE_U: case MODE_I: MyPid.setCoefficients( kP[m], kI[m], kD[m] );      break;
    case MODE_D:        MyPidD.setCoefficients( kP[m], kI[m], kD[m] );      break;
    default:       break;
    }

    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
    #endif

    txReplay( 1, 0 );             // только подтверждение
  }
  else txReplay(1, err_tx);       // Ошибка протокола
}

// 0x42 Тестовая: ввод диапазона вывода
void doPidOutputRange()
{
  if( rxNbt == 5 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( MODE_OFF, U, I, D )
    pidMode = m;
    minOut[m] = get16(1);
    maxOut[m] = get16(3);

    switch ( m )
    {
    case MODE_U: case MODE_I: MyPid.setOutputRange( minOut[m], maxOut[m] );      break;
    case MODE_D:        MyPidD.setOutputRange( minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    txReplay( 1, 0 );
  }
  else txReplay(1, err_tx);
}

// 0x43 Тестовая: set as 0x40 w/o clear
void doPidReconfigure()
{
  if( rxNbt == 10 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( MODE_OFF, U, I, D )
    pidMode = m;
    kP[m] = get16(1);
    kI[m] = get16(3);
    kD[m] = get16(5);
    minOut[m] = get16(7);
    maxOut[m] = get16(9);
      // Это та же процедура, только без очистки регистров регулятора:
    //replaceConfigure( m, kP[m], kI[m], kD[m], minOut[m], maxOut[m] );
    switch ( m )
    {
    case MODE_U: case MODE_I: MyPid.replaceConfig( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    case MODE_D:              MyPidD.replaceConfig( kP[m], kI[m], kD[m], minOut[m], maxOut[m] );      break;
    default:       break;
    }
    
    #ifdef DEBUG_PID
      SerialUSB.print(" mode: "); SerialUSB.println( m );
      SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
      SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
      SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
      //SerialUSB.print(" sign: "); SerialUSB.println( signOut[m] );
    #endif
    txReplay( 1, 0 );  
  }
  else  txReplay(1, err_tx);
}

// 0x44 Очистка регистров регулятора
void doPidClear()
{
  if( rxNbt == 1 )
  {
    uint8_t m = rxDat[0] & 0x03;   // Выбор режима ( MODE_OFF, U, I, D )

    switch ( m )
    {
    case MODE_U: case MODE_I: MyPid.clear();      break;
    case MODE_D:        MyPidD.clear();      break;
    default:       break;
    }

    txReplay(1, 0);
  }
  else txReplay(1, err_tx);
}

  // 0x46 Тестовая. Тест пид-регулятора
  // Задает ПИД-регулятору режим регулирования U,I или D и задает уровень.
  // В режиме MODE_OFF ПИД-регулятор отключен, но схема скоммутирована как для регулирования 
  // по напряжению. Уровень предназначен для подачи непосредственно на PWM с осторожностью. 
void doPidTest()
{
  if( rxNbt == 3 )
  {
    uint8_t   m = rxDat[0] & 0x03;  // 0-1-2-3 - выкл или задать напряжение, ток заряда или ток разряда

    pidMode = m;                    // выбор канала регулирования

    if( pidMode == 0 )
    {
      // включать как регулятор напряжения
      setpoint[1] = get16(1);  
      configMode(MODE_U);
//      pidStatus = false;            // PID-регулятор выключен
    }
    else
    {
      setpoint[m] = get16(1);  
      configMode(m);
//      pidStatus = true;             // PID-регулятор включен
  //  }

    #ifdef DEBUG_PID
      SerialUSB.print("mode: ");  SerialUSB.println( m );
      SerialUSB.print(" sp: 0x"); SerialUSB.println( setpoint[m], HEX );
      //SerialUSB.print("min: 0x"); SerialUSB.println( minOut[m], HEX );
      //SerialUSB.print("max: 0x"); SerialUSB.println( maxOut[m], HEX );
    #endif
    }
    txReplay( 1, 0 );               // только подтверждение
  
  }
  else txReplay(1, err_tx);         // Ошибка протокола     
}

//   // 0x47 Конфигурирование pwm-регулятора Out (C_47_pwmConf.wak)
// void doPwmOut()
// {
//   uint8_t err = 0x00;

//   if( rxNbt == 4 )
//   {
//     pwmTurbo      = (bool)get08(0);          // t - False for 48MHz clock, true for 96MHz clock
//     pwmTccdivOut  = (unsigned int)get08(1);  // d - задать делитель
//     pwmStepsOut   = get16(2);                // s - задать steps (частоту)
//     goPwmOut();

//     txReplay( 1, err );  
//   }
//   else txReplay(1, err_tx);
// }

// 0x48 Возвращает параметры текущего режима регулирования
void doPidGetConfigure()
{
  if( rxNbt == 0 )
  {
    int id = 1;
    id = replyU08( id, pidMode );
    id = replyU16( id, kP[pidMode] );
    id = replyU16( id, kI[pidMode] );
    id = replyU16( id, kD[pidMode] );
    id = replyU16( id, (uint16_t)minOut[pidMode] );
    id = replyU16( id, (uint16_t)maxOut[pidMode] );

    txReplay( id, 0 );   // всего байт, в нулевом - сообщение об ошибках (подтверждение)
  }
  else txReplay(1, err_tx);    // ошибка протокола (пакет не полный)
}

// 0x49 Задает максимальный интеграл при вычислении шага регулирования
void doPidSetMaxSum()
{
  if( rxNbt == 12 )
  {
    //integ_min = get64(0);       // Лучше задавать число знаков и сдвигами вычислять мин и макс
    //integ_max = get64(7);  //знак - !!!

    // #ifdef DEBUG_PID
    //   SerialUSB.print(" mode: "); SerialUSB.println( m );
    //   SerialUSB.print(" kp: ");   SerialUSB.println( (float)kP[m]/MPid::param_mult, 2 );
    //   SerialUSB.print(" ki: ");   SerialUSB.println( (float)kI[m] * 10 / MPid::param_mult, 2 );
    //   SerialUSB.print(" kd: ");   SerialUSB.println( (float)((kD[m] / MPid::param_mult) / 10), 2 );
    // #endif

    txReplay( 1, 0 );             // только подтверждение
  }
  else txReplay(1, err_tx);       // Ошибка протокола
}

// // 0x4A Конфигурирование pwm-регулятора Cool (C_4A_pwmConfCool.wak)
// void doPwmCool()
// {
//   uint8_t err = 0x00;

//   if( rxNbt == 4 )
//   {
//     pwmTurbo      = (bool)get08(0);          // t - False for 48MHz clock, true for 96MHz clock
//     pwmTccdivCool  = (unsigned int)get08(1);  // d - задать делитель
//     pwmStepsCool   = get16(2);                // s - задать steps (частоту)
//     goPwmCool();

//     txReplay( 1, err );  
//   }
//   else txReplay(1, err_tx);
// }

// 0x4F Задать скорость вентилятора
void doCooler()                      
{
  if( rxNbt == 2 )
  {
    writePwmCool(get16(0));         // 0...1000
    txReplay( 1, 0x00 );            // подтверждение
  }
  else txReplay(1, err_tx);        // ошибка протокола
}


  // 0x5B задать параметры компенсации перенапряжения - отменено
void doSurgeCompensation()
{
  if( rxNbt == 4 )
  {
    surgeVoltage = get16(0);        // Милливольты превышения
    surgeCurrent = get16(2);        // Ток в коде DAC

    txReplay( 1, 0 );
  }
  else txReplay(1, err_tx);                    // ошибка протокола
}

  // 0x5C задать параметры доп. нагрузки на ХХ
void doIdleLoad()
{
  if( rxNbt == 4 )
  {
    idleCurrent  = get16(0);        // Минимальный ток, при котором не нужна дополнительная нагрузка 
    idleDac      = get16(2);        // Ток в коде DAC
    txReplay( 1, 0 );
  }
  else txReplay(1, err_tx);                    // ошибка протокола
}



// ============================ Команды тестирования ===========================

// Команда 0x54. Управление ключами подключения нагрузки.
// MINI: закомментировать powerFailure() или замкнуть D8,9 и A3,4 
void doSwPin()
{
  if( rxNbt == 1 )
  {
    bool sw = (bool)(rxDat[0] & 0x01);

    if( !sw )
    {
      // При отключении нагрузки снять питание
      // и отключить цепь разряда

      pidMode = MODE_OFF;      // При включенном регуляторе отключение автоматическое, ниже - дублирование

      swPinOff();
      switchStatus          = false;  // коммутатор отключен

//      writePwm( 0x0000 );
      writePwmOut( 0x0000 );
//      powerStatus           = false;  // преобразователь выключен
      chargeStatus          = false;
      
      dacWrite10bit( 0x0000 );
      dischargeStatus = false;
    }
    else
    {
      swPinOn();
      switchStatus          = true;  // коммутатор включен
    }

    txReplay( 1, 0 );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола  
}

  // Команда 0x56. Для проверки пределов регулирования преобразователя снизу. 
  // Использовать с осторожностью, только для проверки низковольтной схемы.
  // ПИД-регулятор отключается, коммутатор включен, преобразователь включен
void setPower()
{
  if( rxNbt == 4 )
  {
    pidMode = MODE_OFF;      // При включенном регуляторе всё отключится
//    pidStatus = false;

    swPinOn();
    switchStatus          = true;   // коммутатор включен

    uint16_t val = get16(0);
//    writePwm( val );           //     
    writePwmOut( val );           //     

    if( val )  powerStatus = true;   // преобразователь включен
      else     powerStatus = false;  // преобразователь выключен

    val = get16(2); 
    dacWrite10bit( val );

    if( val ) dischargeStatus = true; // Схема разряда как нагрузка включена
      else    dischargeStatus = false;
    
    #ifdef DEBUG_POWER
      // Реальные значения будут только в следующем запросе.
      // Ток разрядной цепи не учитывается. 
      SerialUSB.print( "mV: " );    SerialUSB.print( mvVoltage );
      SerialUSB.print( "  mA: " );  SerialUSB.println( maCurrent );
    #endif

    txReplay( 1, 0 );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола
}

// Команда 0x57 - проверка управления цепью разряда.
// Пользоваться с осторожностью, выставив порог отключения
void setDischg()
{
  if( rxNbt == 1 )
  {
    uint8_t err = 0x00;         // зарезервировано
    uint16_t proc = rxDat[0];
    if( proc > 100 ) proc = 100;

    if( proc )  // Включить если не 0%
    {
      proc = 1023 - ( proc * 1023 / 100 );
      dacWrite10bit( proc );

      //dischargeStatus = true;   // коммутатор на разряд
      //chargeStatus = !dischargeStatus;
      powerStatus = false;      // преобразователь выключить
      switchStatus = true;      // к клеммам подключить
    }
    else        // Выключить если 0%
    {
      proc = 1023;
      dacWrite10bit( proc );    //

      //dischargeStatus = true;   // оставить подключенным на разряд
      //chargeStatus = !dischargeStatus;
      powerStatus = false;      // преобразователь выключить
      switchStatus = true;      // от нагрузки не отключать
    }

    #ifdef DEBUG_POWER
//      SerialUSB.println( proc );
    #endif

    txReplay( 1, err );         // Подтверждение
  }
  else txReplay(1, err_tx);   // ошибка протокола
}

  // 0x58 Включение и поддержание заданного напряжение в мВ
void doSetVoltage()
{
  if( rxNbt == 3 )
  {
    uint8_t _mode = rxDat[0] & 0x03;    // MODE_OFF-U-I-D - выкл или задать напряжение, ток заряда или ток разряда
    int16_t sp    = (int16_t)get16(1);  // Заданная величина в mV или mA

    switch (_mode)
    {
      case MODE_U :
      {
        if(sp < volt_min) sp = volt_min;           // Если за пределом - задать минимум
        if(sp > volt_max) sp = volt_max;           // Если за пределом - задать максимум


        setpoint[MODE_U] = sp;       // милливольты

//      SerialUSB.print("x65_U ");  SerialUSB.println(setpoint[U]);


        // Задать условия, установить напряжение 
        //chargeStatus    = true;               // заряд, иное невозможно
        //dischargeStatus = !chargeStatus;

        swPinOn();                            // sw_pin D5 PA15 - силовые ключи (нагрузку) включить
        switchStatus = true; 
        
//        powPinOn();
        powerStatus = true;                   //         !pwr_pin D4 PA14 - преобразователь включить

//        pidStatus = true;   //        _pidStatus;
        voltageControlStatus  = true;   //_pidStatus;   // регулирование по напряжению

        //initPid();  // ****

//        SerialUSB.print("kP[U]... ");  SerialUSB.println(kP[U]);

          MyPid.configure( kP[MODE_U], kI[MODE_U], kD[MODE_U], minOut[MODE_U], maxOut[MODE_U] );
          //writePwm( sp );           // запустить это тест на 20 ... 100мс
      }
      break;

      case MODE_I:
      {
        int16_t sp = (int16_t)get16(0);        // Заданное напряжение в милливольтах

        if(sp < curr_ch_min) sp = curr_ch_min;           // Если за пределом - задать минимум
        if(sp > curr_ch_max) sp = curr_ch_max;           // Если за пределом - задать максимум


        setpoint[MODE_I] = sp;       // миллиамперы

//      SerialUSB.print("x65_I ");  SerialUSB.println(setpoint[MODE_I]);

      // ...

      }
      break;

    default:
      // Выключить регулятор
        swPinOff();                            // sw_pin D5 PA15 - силовые ключи (нагрузку) выключить
        switchStatus = false; 
        
//        powPinOff();
        powerStatus = false;                   //      !pwr_pin D4 PA14 - преобразователь выключить

//        pidStatus = false;      // 
//      SerialUSB.print("x65_OFF ");  SerialUSB.println(setpoint[OFF]);

      break;
    }
  
    // Подготовить 3 байта ответа: 0 - нет ошибок и setpoint
    int id = 1;
    id =  replyU16(id, sp);
    txReplay( id, txDat[0] ); 
  }
  else txReplay(1, err_tx);                   // ошибка протокола
} // !doSetVoltage()

  // 0x59 задать ток в мА и включить
void doSetCurrent()
{
  if( rxNbt == 5 )
  {
    txDat[0] = 0x00;                      // Очистить сообщение об ошибках

    bool _pidStatus    = rxDat[0] & 0x01; // Регулятор отключить или включить - отменено 202206
    uint16_t _setpoint = get16(1);        // Заданный ток в миллиамперах
    uint16_t _factor   = get16(3);        // Коэффициент преобразования в код ADC
    
    if(_pidStatus)  // Если задаются миллиамперы
    {
      if(_setpoint <= curr_ch_min)         // Если за пределом
      {
        _setpoint = curr_ch_min;          // Задать минимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }

      if(_setpoint >= curr_ch_max)         // Если за пределом
      {
        _setpoint = curr_ch_max;          // Задать максимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }
    } 
    else
    {
      if(_setpoint <= min_pwm)                // Если за пределом
      {
        _setpoint = min_pwm;                 // Задать минимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      }

      if(_setpoint >= max_pwm)                // Если за пределом
      {
        _setpoint = max_pwm;                 // Задать максимум
        txDat[0] = 0x01;                  // и сообщить об ошибке
      } 
    }

    uint16_t _value = _setpoint / _factor;

    // Задать условия, установить ток 
    //chargeStatus    = true;               // заряд, иное невозможно
    //dischargeStatus = !chargeStatus;
    switchStatus    = true;               // коммутатор включить     ( foff_pin = 21 D21 PA23 ) 
    powerStatus = true;               // преобразователь включить ( pwr_pin =  2 D4  PA14 )
//    pidStatus = _pidStatus;

//    if(_pidStatus)
//    {
      setpoint[MODE_I] = _value;
//      // запустить
//    }
//    else
//    {
//      dacWrite10bit( _value );            // Задать код
//    }

    // Подготовить 3 байта ответа: 0 - нет ошибок и код, который ушел в ADC или setpoint
    txDat[1] = ( _value >> 8) & 0xFF;      // Hi
    txDat[2] =   _value & 0xFF;            // Lo
    txNbt = 3;
    txReplay( txNbt, txDat[0] ); 
  }
  else
  {
    txReplay(1, err_tx);                   // ошибка протокола
  }
} // !doSetCurrent()

  // 0x5A задать код DAC или ток разряда в мА и включить
void doSetDiscurrent()
{
  if( rxNbt == 3 )
  {
    uint8_t   m = rxDat[0] & 0x03;  // MODE_OFF - задать код, иначе ток разряда в миллиамперах
    pidMode = m;                    // выбор канала регулирования

    if( m == 0 )
    {
      // Задается код
      pidMode = m = MODE_D;              // выбор канала регулирования 
      configMode(m);
//      pidStatus = false;            // PID-регулятор выключен
      setpoint[m] = get16(1); 

      if( setpoint[m] >= 0x0400 )  setpoint[m] = 0x3ff; // Если за пределами 10 разрядов
    
      dacWrite10bit( setpoint[m] ); // Задать код

      #ifdef DEBUG_DAC
//        SerialUSB.print("mode: ");  SerialUSB.print( m );
//        SerialUSB.print(" sp: 0x"); SerialUSB.println( setpoint[m], HEX );
      #endif
    }
    else
    {
      // Задается ток в миллиамперах
      pidMode = m = MODE_D;              // выбор канала регулирования
      configMode(m);
//      pidStatus = true;             // PID-регулятор включен
      setpoint[m] = get16(1);       // Вводится абсолютное значение
    }
  
    txReplay( 1, 0 ); 
  }
  else  txReplay(1, err_tx);                    // ошибка протокола
}
