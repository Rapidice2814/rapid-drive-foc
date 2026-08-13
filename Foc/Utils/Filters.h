#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

typedef struct {
    uint8_t is_initialized;
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} BiquadFilter;

typedef enum {
    BIQUAD_LOW_PASS,
    BIQUAD_HIGH_PASS,
    BIQUAD_BAND_PASS
} BiquadType;

void biquad_filter_init(BiquadFilter *filter, BiquadType type, float cutoff_or_center_hz, float sampling_frequency_hz, float q);
float biquad_filter_update(BiquadFilter *filter, float input);
void biquad_filter_reset(BiquadFilter *filter, float state);
#endif // FILTERS_H