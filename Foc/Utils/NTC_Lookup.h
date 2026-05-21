#ifndef NTC_LOOKUP_H
#define NTC_LOOKUP_H

#define NTC_LUT_SIZE 128

typedef struct {
    float resistance_LUT[NTC_LUT_SIZE];
    float temperature_LUT[NTC_LUT_SIZE];
} NTC_LUT_TypeDef;

void GenerateNtcLut(NTC_LUT_TypeDef *hntclut, float temp_min, float temp_max, float beta, float r0, float t0);
float GetNtcTemperature(NTC_LUT_TypeDef *hntclut, float ntc_resistance);

#endif // NTC_LOOKUP_H