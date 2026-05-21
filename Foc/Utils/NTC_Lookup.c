#include "NTC_Lookup.h"
#include <math.h>

/**
 * @brief Generates a lookup table for NTC thermistor resistance to temperature conversion
 * @param hntclut Pointer to the NTC_LUT_TypeDef structure to store the generated LUT
 * @param temp_min Minimum temperature in °C for the LUT
 * @param temp_max Maximum temperature in °C for the LUT
 * @param beta Beta constant of the NTC thermistor
 * @param r0 Resistance of the NTC thermistor at the reference temperature T0
 * @param t0 Reference temperature in Kelvin (usually 25°C + 273.15K)
 */
void GenerateNtcLut(NTC_LUT_TypeDef *hntclut, float temp_min, float temp_max, float beta, float r0, float t0){
    if(hntclut == NULL) return;

    float stepC = (temp_max - temp_min) / (NTC_LUT_SIZE - 1);

    for (int i = 0; i < NTC_LUT_SIZE; i++) {
        float tempC = temp_min + i * stepC;
        float tempK = tempC + 273.15f;
        
        float exponent = beta * (1.0f / tempK - 1.0f / t0);

        hntclut->temperature_LUT[i] = tempC;
        hntclut->resistance_LUT[i] = r0 * expf(exponent);
    }
}



/**
 * @brief Gets the temperature corresponding to a given NTC resistance using the LUT
 * @param hntclut Pointer to the NTC_LUT_TypeDef structure containing the LUT
 * @param ntc_resistance The resistance of the NTC thermistor for which to get the temperature
 * @return The corresponding temperature in °C, or -1.0f if the input is invalid
 */
float GetNtcTemperature(NTC_LUT_TypeDef *hntclut, float ntc_resistance){
    if(hntclut == NULL) return -1.0f;

    if (ntc_resistance >= hntclut->resistance_LUT[0])
        return hntclut->temperature_LUT[0];
    if (ntc_resistance <= hntclut->resistance_LUT[NTC_LUT_SIZE - 1])
        return hntclut->temperature_LUT[NTC_LUT_SIZE - 1];

    // Find interval for interpolation
    int i = 0;
    while (i < NTC_LUT_SIZE - 1 && !(ntc_resistance <= hntclut->resistance_LUT[i] && ntc_resistance > hntclut->resistance_LUT[i + 1])) {
        i++;
    }

    // Linear interpolation
    float t1 = hntclut->temperature_LUT[i];
    float t2 = hntclut->temperature_LUT[i + 1];
    float r1 = hntclut->resistance_LUT[i];
    float r2 = hntclut->resistance_LUT[i + 1];

    float temp = t1 + (t2 - t1) * (ntc_resistance - r1) / (r2 - r1);

    // float temp1 =  (1.0f / ((1.0f / 298.15f) + (1.0f / 3950.0f) * log(ntc_resistance / 100e3f)) - 273.15f); //takes too long
    return temp;
  
}