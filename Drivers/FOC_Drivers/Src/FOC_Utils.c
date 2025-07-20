#include <stdio.h>
#include <math.h>
#include "FOC_Utils.h"
#include "main.h"


__attribute__((weak)) float _sinf(float a){
  // 16bit integer array for sine lookup. interpolation is used for better precision
  // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
  // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
  static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  int32_t t1, t2;
  unsigned int i = (unsigned int)(a * (64*4*256.0f/M_2PIF));
  int frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) {
    t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
  }
  else if(i < 128) {
    t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
  }
  else if(i < 192) {
    t1 = -(int32_t)sine_array[-128 + i]; t2 = -(int32_t)sine_array[-127 + i];
  }
  else {
    t1 = -(int32_t)sine_array[256 - i]; t2 = -(int32_t)sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
__attribute__((weak)) float _cosf(float a){
  float a_sin = a + M_PI_2F;
  a_sin = a_sin > M_2PIF ? a_sin - M_2PIF : a_sin;
  return _sinf(a_sin);
}
  
  
// fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
// Via Odrive project
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
// The origin for Odrive atan2 is public domain. Thanks to Odrive for making
// it easy to borrow.
__attribute__((weak)) float _atan2(float y, float x) {
  float abs_y = fabsf(y);
  float abs_x = fabsf(x);
  float a = fminf(abs_x, abs_y) / (fmaxf(abs_x, abs_y));
  float s = a * a;
  float r =
      ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (abs_y > abs_x) r = 1.57079637f - r;
  if (x < 0.0f) r = 3.14159274f - r;
  if (y < 0.0f) r = -r;

  return r;
}


//Normalize angle to [0, 2*PI]
void normalize_angle(float *angle){
  while (*angle > M_2PIF) *angle -= M_2PIF;
  while (*angle <= 0) *angle += M_2PIF;
}

//Normalize angle to [-PI, PI]
void normalize_angle2(float *angle){
  while (*angle > M_PI) *angle -= M_2PIF;
  while (*angle <= -M_PI) *angle += M_2PIF;
}

/**
  * @brief Constrains a float to be within a specified range
  * @param value The value to constrain
  * @param min The minimum value
  * @param max The maximum value
  * @retval The constrained value
  */
float constrainf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}



#define TEMP_MIN 10       // Minimum temperature in °C
#define TEMP_MAX 100      // Maximum temperature in °C
#define LUT_SIZE (TEMP_MAX - TEMP_MIN + 1)

#define T0 298.15f        // Reference temperature in Kelvin (25°C)
#define B 3950.0f         // Beta constant
#define R0 100000.0f      // Reference resistance at T0 (Ohms)

static float resistance_LUT[LUT_SIZE];
static float temperature_LUT[LUT_SIZE];

/**
  * @brief Generates the lookup table for the LUT based NTC temperature conversion
  * @note This function needs to be called once at the start of the program if GetNtcTemperature is used.
  * @param None
  * @retval None
  */
void GenerateNtcLut(){
    for (int i = 0; i < LUT_SIZE; i++) {
        float tempC = TEMP_MIN + i;             // Temperature in °C
        temperature_LUT[i] = tempC;
        
        float tempK = tempC + 273.15f;          // Convert °C to K
        float exponent = B * (1.0f / tempK - 1.0f / T0);
        resistance_LUT[i] = R0 * expf(exponent);
    }
}



/**
  * @brief Convert the NTC resistance to temperature, using a LUT
  * @note The LUT needs to be generated first using GenerateNtcLut(). The temperature deviates less than 0.1°C from the actual value.
  * @param ntc_resistance
  * @retval The temperature in °C
  */
float GetNtcTemperature(float ntc_resistance){
  if (ntc_resistance >= resistance_LUT[0])
        return temperature_LUT[0];
    if (ntc_resistance <= resistance_LUT[LUT_SIZE - 1])
        return temperature_LUT[LUT_SIZE - 1];

    // Find interval for interpolation
    int i = 0;
    while (i < LUT_SIZE - 1 && !(ntc_resistance <= resistance_LUT[i] && ntc_resistance > resistance_LUT[i + 1])) {
        i++;
    }

    // Linear interpolation
    float t1 = temperature_LUT[i];
    float t2 = temperature_LUT[i + 1];
    float r1 = resistance_LUT[i];
    float r2 = resistance_LUT[i + 1];

    float temp = t1 + (t2 - t1) * (ntc_resistance - r1) / (r2 - r1);

    // float temp1 =  (1.0f / ((1.0f / 298.15f) + (1.0f / 3950.0f) * log(ntc_resistance / 100e3f)) - 273.15f); //takes too long
    return temp;
  
}