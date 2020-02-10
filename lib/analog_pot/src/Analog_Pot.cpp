#include <Analog_Pot.h>

Analog_Pot::Analog_Pot(int pin, int range_min, int range_max){
  _pin = pin;
  _range_max = range_max;
  _range_min = range_min;
  pinMode(_pin,INPUT);
  //val = analogRead(pin);
}


//////////Methods////////////////

//Return potentiometer value
Analog_Pot::GetVal(void){
  _val = analogRead(_pin);
  return _val;
}

//For testing controlling motor from pot
Analog_Pot::GetVal2PWM(void){
  _val = analogRead(_pin);
  _norm_val = map(_val,_range_min,_range_max,-255,255); //Map to PWM range of arduino

  return _norm_val;
}
