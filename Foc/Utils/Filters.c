#include "Filters.h"
#include "Utils.h"

#include "math.h"

/**
 * @brief Initializes a biquad filter with the specified parameters
 * @param filter Pointer to the BiquadFilter structure to initialize
 * @param type The type of biquad filter (low-pass, high-pass, band-pass)
 * @param cutoff_or_center_hz The cutoff frequency (for low-pass and high-pass) or center frequency (for band-pass) in Hz
 * @param sampling_frequency_hz The sampling frequency in Hz
 * @param q The quality factor of the filter
 * @retval None
 */
void biquad_filter_init(BiquadFilter *filter, BiquadType type, float cutoff_or_center_hz, float sampling_frequency_hz, float q){
    if (filter == 0 || sampling_frequency_hz <= 0.0f || cutoff_or_center_hz <= 0.0f || cutoff_or_center_hz >= 0.5f * sampling_frequency_hz || q <= 0.0f){
        if (filter != 0){
            filter->is_initialized = 0U;
        }
        return;
    }

    const float w0 = 2.0f * M_PIF * cutoff_or_center_hz / sampling_frequency_hz;
    const float sin_w0 = sinf(w0);
    const float cos_w0 = cosf(w0);
    const float alpha = sin_w0 / (2.0f * q);
    const float a0 = 1.0f + alpha;

    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    switch (type){
    case BIQUAD_LOW_PASS:
        b0 = (1.0f - cos_w0) * 0.5f;
        b1 = 1.0f - cos_w0;
        b2 = b0;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;

    case BIQUAD_HIGH_PASS:
        b0 = (1.0f + cos_w0) * 0.5f;
        b1 = -(1.0f + cos_w0);
        b2 = b0;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;

    case BIQUAD_BAND_PASS:
        /* Constant peak gain: unity gain at the center frequency. */
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alpha;
        break;

    default:
        filter->is_initialized = 0U;
        return;
    }

    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->is_initialized = 1U;
}

/**
 * @brief Resets a biquad filter to a specified state
 * @param filter Pointer to the BiquadFilter structure to reset
 * @param state The initial state for the filter
 * @retval None
 */
void biquad_filter_reset(BiquadFilter *filter, float state){
    if (filter == 0){
        return;
    }

    filter->x1 = state;
    filter->x2 = state;
    filter->y1 = state;
    filter->y2 = state;
}

/**
 * @brief Updates a biquad filter with a new input sample
 * @param filter Pointer to the BiquadFilter structure to update
 * @param input The new input sample
 * @return The filtered output sample
 */
float biquad_filter_update(BiquadFilter *filter, float input){
    if (filter == 0 || !filter->is_initialized){
        return input;
    }

    const float output =
        filter->b0 * input
        + filter->b1 * filter->x1
        + filter->b2 * filter->x2
        - filter->a1 * filter->y1
        - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}