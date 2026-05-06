#include <stdio.h>
#include <math.h>
#include "FOC_Utils.h"


//Normalize angle to [0, 2*PI]
void normalize_angle_0_2pi(float *angle){
  while (*angle > M_2PIF) *angle -= M_2PIF;
  while (*angle <= 0) *angle += M_2PIF;
}

//Normalize angle to [-PI, PI]
void normalize_angle_pm_pi(float *angle){
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

/** 
 * @brief Counts the number of set bits in a 32-bit integer
 * @param n The integer to count bits from
 * @return The number of set bits
 */
uint8_t countbits(uint32_t n) {
    uint8_t count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}


#define TEMP_MIN -10       // Minimum temperature in °C
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