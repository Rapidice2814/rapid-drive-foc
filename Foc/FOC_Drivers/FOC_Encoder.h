#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H
#include <stdint.h>

#define FOC_ENCODER_LUT_SIZE 128U
#define FOC_ENCODER_SOURCE_MAX 8U
#ifndef FOC_ENCODER_SOURCE_LIST
#define FOC_ENCODER_SOURCE_LIST(X) \
    X(0, "Internal AS5047P quadrature", FOC_EncoderInitTimer, FOC_EncoderReadTimer, 1) \
    X(1, "Internal AS5047P SPI DMA", FOC_EncoderInitSPI, FOC_EncoderReadSPI, 1) \
    X(2, "External AS5047P SPI DMA", FOC_EncoderInitExternalSPI, FOC_EncoderReadSPI, 1) \
    X(3, "External AS5047P quadrature", FOC_EncoderInitExternalTimer, FOC_EncoderReadTimer, 1)
#endif
typedef struct FOC_Handle FOC_HandleTypeDef;

/* Stable source IDs are persisted. Add a descriptor/reader, not plotter code.
   Readers provide absolute mechanical radians in [0, 2pi), independent of zero.
   All sources share the same calibration, unwrapping and speed pipeline. */
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
    float speed_pll_angle; /* Wrapped prediction; never used for commutation. */
    uint8_t healthy, history_valid, bypass_correction, initialized;
} FOC_EncoderRuntimeTypeDef;

const FOC_EncoderSourceTypeDef *FOC_EncoderSources(unsigned *count);
const FOC_EncoderSourceTypeDef *FOC_EncoderSource(uint8_t id);
uint8_t FOC_EncoderReadTimer(FOC_HandleTypeDef *hfoc, float *angle);
uint8_t FOC_EncoderInitTimer(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderInitSPI(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderInitExternalSPI(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderInitExternalTimer(FOC_HandleTypeDef *hfoc);
uint8_t FOC_EncoderReadSPI(FOC_HandleTypeDef *hfoc, float *angle);
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
