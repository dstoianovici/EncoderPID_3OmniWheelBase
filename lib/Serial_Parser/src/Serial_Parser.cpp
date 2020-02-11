#include <Serial_Parser.h>

struct {
  int param_flag = 0;
  int range_flag = 0;
} output_flags;

///////////Constructor////////////
Serial_Parser::Serial_Parser(char delimiter, int range_M, int range_m){
  _delimiter = delimiter;
  _range_M = range_M;
  _range_m = range_m;
  //Serial.println("parser created");
}

//////////Methods//////////////////
void Serial_Parser::GetParams(int* nums, int* param_check){
  String ser_data = "/n";
  String sub_string = "/n";
  int index = 0; //Index of the delimiter
  int i = 0; // itterator for nums array
  int endstring_flag = 1; //Flag for end of string object processing
  //param_check : Slot 0 = number of read params, Slot1 = 1 if param out of range

  if (Serial.available() > 0){
    ser_data = Serial.readString();

    while(endstring_flag == 1){
      index = ser_data.indexOf(_delimiter); //Find Index of Delimiter

      if(index == -1){//No more delimiter
        nums[i] = ser_data.toInt();
        if(_range_M < nums[i] ||nums[i] < _range_m) param_check[1] = 1;
        i++;
        //Serial.println(i);
        endstring_flag = 0;
      }

      else{ //Serpate string at delimiter, turn into int check range.
        sub_string = ser_data.substring(0,index);
        ser_data.remove(0,index+1);
        nums[i] = sub_string.toInt();
        if(_range_M < nums[i] || nums[i] < _range_m)  param_check[1] = 1;
        i++;
        //Serial.println(i);
        endstring_flag = 1;
      }
    }
  }
  else{
    i = 0;
    param_check[0] = i;
    param_check[1] = 0;

    return;
  }

  param_check[0] = i;

  return;

  }
