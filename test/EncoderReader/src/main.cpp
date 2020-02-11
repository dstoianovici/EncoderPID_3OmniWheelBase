#include <Arduino.h>
#include <SPI.h>
#include <Encoder_Buffer.h>


const uint8_t cs1 = A1; //Chip Select Pins
const uint8_t cs2 = A2;
const uint8_t cs3 = A3;

signed long enc1_cnt = 0;
signed long enc2_cnt = 0;
signed long enc3_cnt = 0;

Encoder_Buffer enc1(cs1);
Encoder_Buffer enc2(cs2);
Encoder_Buffer enc3(cs3);

void setup() {
  Serial.begin(115200);
  SPI.begin();
  enc1.initEncoder();
  enc2.initEncoder();
  enc3.initEncoder();
}


void loop() {

enc1_cnt = enc1.readEncoder();
enc2_cnt = enc2.readEncoder();
enc3_cnt = enc3.readEncoder();

Serial.print("ecn1_cnt: ");
Serial.println(enc1_cnt);
Serial.print("ecn2_cnt: ");
Serial.println(enc2_cnt);
Serial.print("ecn3_cnt: ");
Serial.println(enc3_cnt);
Serial.println();

delay(500);


}
