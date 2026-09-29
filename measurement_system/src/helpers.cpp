#include "helpers.hpp"

float milgToSi(float mg){
  /* 1 milg = 0.00981 m/s2 */
  /* 1 m/s2 = 102.94 milg */
  return mg * 0.00981F;
}

float gToSi(float g){
  return g * 9.81F;
}

int32_t floatToInt32Prec(float value, uint8_t prec){
  return static_cast<int32_t>(std::round(value * pow(10, prec)));
}