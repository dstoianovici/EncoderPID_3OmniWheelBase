#ifndef ANALOG_POT_H
#define ANALOG_POT_H

#include <Arduino.h>

class Analog_Pot
{
  public:
    Analog_Pot(int pin, int range_min, int range_max);
    int GetVal();
    int GetVal2PWM();


  private:
    int _pin;
    int _range_min;
    int _range_max;
    int _val;
    int _norm_val;

};

#endif
