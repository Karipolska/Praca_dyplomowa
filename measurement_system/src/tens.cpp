#include "tens.hpp"

long get_scale(HX711 amplifier, uint8_t frequency){
  amplifier.tare();
  
  uint8_t temp = 10;
  Serial.println(temp);

  delay(5000);
  
  long before = amplifier.read_average(5*frequency);
  uint8_t temp_2 = 20;
  Serial.println(temp_2);

  delay(5000);
  
  long after = amplifier.read_average(5*frequency);
  long weight = 18; //znana waga na belce
  if (after < 0){
  Serial.println(-100);
  }
  long computed_scale = -(before - after) / weight;

  delay(1000);
  uint8_t temp_3 = 30;
  Serial.println(temp_3);

  delay(2000);

  return computed_scale;
}