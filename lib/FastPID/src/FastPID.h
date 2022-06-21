#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>

#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
#define DERIV_MAX    (INT16_MAX)
#define DERIV_MIN    (INT16_MIN)

#define PARAM_SHIFT  8
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT) 
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT)) 

/*
  A fixed point PID controller with a 32-bit internal calculation pipeline.
*/
class FastPid 
{

public:

  // Константы - должны быть синхронизированы с ведущим контроллером
  // для корректного преобразования (предвычисления) параметров.

  static constexpr int32_t  integ_max   = (INT32_MAX);         //0x007FFFFF;   //0x007FFFFF
  static constexpr int32_t  integ_min   = (INT32_MIN);         //0xFF800000;   //0xFF800000
  static constexpr int16_t  deriv_max   = (INT16_MAX);                  //0x7FFF;
  static constexpr int16_t  deriv_min   = (INT16_MIN);                  //0x8000;

  static constexpr uint8_t  param_shift =  8;
  static constexpr uint8_t  param_bits  = 16;
  static constexpr uint16_t param_max   = (((0x1ULL << param_bits)-1) >> param_shift);              // 0xFF
  static constexpr uint16_t param_mult  = (((0x1ULL << param_bits)) >> (param_bits - param_shift)); // 0x100
  static constexpr uint16_t hz = 10;



  FastPid() 
  {
    clear();
  }

  //FastPid(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)
  FastPid(float kp, float ki, float kd, float hz, int bits=10, bool sign=false)  // 202206
  {
    configure(kp, ki, kd, hz, bits, sign);
  }

  ~FastPid();

  bool setCoefficients(float kp, float ki, float kd, float hz);
  bool setOutputConfig(int bits, bool sign);
  bool setOutputRange(int16_t min, int16_t max);
  void clear();
  //bool configure(float kp, float ki, float kd, float hz, int bits=16, bool sign=false);
  bool configure(float kp, float ki, float kd, float hz, int bits=10, bool sign=false);
//  bool replaceConfig(float kp, float ki, float kd, float hz, int bits, bool sign);    // moro 

  int16_t step(int16_t sp, int16_t fb);

  bool err() {
    return _cfg_err;
  }

private:

  uint32_t floatToParam(float); 
  void setCfgErr(); 

private:

  // Configuration
  uint32_t _p, _i, _d;
  int64_t _outmax, _outmin; 
  bool _cfg_err; 
  
  // State
  int16_t _last_sp, _last_out;
  int64_t _sum;
  int32_t _last_err;
};

#endif
