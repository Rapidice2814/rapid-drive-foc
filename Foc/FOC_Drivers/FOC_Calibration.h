#ifndef FOC_CALIBRATION_H
#define FOC_CALIBRATION_H
#include <stdint.h>
typedef struct FOC_Handle FOC_HandleTypeDef;
/* Kind 0: encoder radians; kind 1: forward/reverse compensation amperes. */
#define FOC_CALIBRATION_CHUNK 12U
uint16_t FOC_CalibrationSize(uint8_t kind);
uint8_t FOC_CalibrationSafe(FOC_HandleTypeDef *h);
uint8_t FOC_CalibrationBegin(FOC_HandleTypeDef *h, uint8_t kind, uint16_t total);
uint8_t FOC_CalibrationWrite(FOC_HandleTypeDef *h, uint8_t kind, uint16_t offset, const float *values, uint8_t count);
uint8_t FOC_CalibrationCommit(FOC_HandleTypeDef *h, uint8_t kind, uint32_t crc);
uint8_t FOC_CalibrationEnable(FOC_HandleTypeDef *h, uint8_t kind, uint8_t enabled);
float FOC_CalibrationValue(FOC_HandleTypeDef *h, uint8_t kind, unsigned index);
#endif
