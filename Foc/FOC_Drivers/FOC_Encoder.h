#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H
#include <stdint.h>

#define FOC_ENCODER_LUT_SIZE 128U
#define FOC_ENCODER_SOURCE_MAX 8U
#ifndef FOC_ENCODER_SOURCE_LIST
#define FOC_ENCODER_SOURCE_LIST(X) X(0, "AS5047P timer", FOC_EncoderInitTimer, FOC_EncoderReadTimer, 1)
#endif
typedef struct FOC_Handle FOC_HandleTypeDef;

/* Stable source IDs are persisted. Add a descriptor/reader, not plotter code.
   Readers provide absolute mechanical radians in [0, 2pi), independent of zero.
   Electrical-only estimators (e.g. HFI) need an explicit mechanical adapter. */
typedef uint8_t (*FOC_EncoderReader)(FOC_HandleTypeDef *, float *);
typedef struct {
    uint8_t id;
    const char *name;
    uint8_t (*initialize)(FOC_HandleTypeDef *);
    FOC_EncoderReader read;
    uint8_t calibratable;
} FOC_EncoderSourceTypeDef;

typedef struct {
    float raw_angle;
    uint8_t healthy, history_valid, bypass_correction, initialized;
} FOC_EncoderRuntimeTypeDef;

const FOC_EncoderSourceTypeDef *FOC_EncoderSources(unsigned *count);
const FOC_EncoderSourceTypeDef *FOC_EncoderSource(uint8_t id);
uint8_t FOC_EncoderReadTimer(FOC_HandleTypeDef *hfoc, float *angle);
uint8_t FOC_EncoderInitTimer(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderInitialize(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderReadRaw(FOC_HandleTypeDef *hfoc, float *angle);
float FOC_EncoderLUT(const float *table, float angle);
uint8_t FOC_EncoderValidateLUT(const float *table);
float FOC_EncoderCorrect(FOC_HandleTypeDef *hfoc, float raw);
uint8_t FOC_EncoderUpdate(FOC_HandleTypeDef *hfoc, float frequency);
uint8_t FOC_EncoderSetZero(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderSelect(FOC_HandleTypeDef *hfoc, uint8_t source);
uint8_t FOC_EncoderEnableCorrection(FOC_HandleTypeDef *hfoc, uint8_t enabled);
void FOC_EncoderResetHistory(FOC_HandleTypeDef *hfoc);
#endif
