#include <math.h>
#include "FOC_Encoder.h"
#include "FOC_Handle.h"
#include "Utils.h"

uint8_t FOC_EncoderInitSimulator(FOC_HandleTypeDef *h){
    h->encoder_runtime.simulator = (AngleSimulator){0, 1.0f};
    return 1;
}
uint8_t FOC_EncoderReadSimulator(FOC_HandleTypeDef *h, float *angle){
    *angle = h->encoder_runtime.simulator.angle;
    return isfinite(*angle);
}
/* Temporary control-frame override: physical encoder selection/feedback stay intact. */
uint8_t FOC_EncoderUseSimulatedCommutation(FOC_HandleTypeDef *h, uint8_t enabled){
    if(enabled > 1 || (enabled && (!h->flash_data.motor.pole_pairs ||
        !isfinite(h->encoder_runtime.simulator.angle)))) return 0;
    h->encoder_runtime.simulated_commutation = enabled;
    return 1;
}
void FOC_EncoderControlFrame(FOC_HandleTypeDef *h, float *angle, float *d, float *q){
    if(!h->encoder_runtime.simulated_commutation) return;
    *angle = AngleSimulator_Electrical(&h->encoder_runtime.simulator, h->flash_data.motor.pole_pairs);
    /* Commands are in the generated field's coordinates, independent of sensor direction. */
    *d = h->dq_current_setpoint.d;
    *q = h->dq_current_setpoint.q;
}

/* Add hardware readers here. IDs must remain stable across firmware updates.
   Do not advertise an electrical-only estimator as a mechanical encoder. */
#define DECLARE_READER(id, name, init, reader, calibratable) uint8_t init(FOC_HandleTypeDef *); uint8_t reader(FOC_HandleTypeDef *, float *);
FOC_ENCODER_SOURCE_LIST(DECLARE_READER)
#undef DECLARE_READER
#define SOURCE_ROW(id, name, init, reader, calibratable) {id, name, init, reader, calibratable},
static const FOC_EncoderSourceTypeDef sources[] = {FOC_ENCODER_SOURCE_LIST(SOURCE_ROW)};
#undef SOURCE_ROW
_Static_assert(sizeof(sources) / sizeof(sources[0]) <= FOC_ENCODER_SOURCE_MAX, "Too many encoder sources");

/** @brief Returns driver-supported sources for runtime host discovery. */
const FOC_EncoderSourceTypeDef *FOC_EncoderSources(unsigned *count){
    *count = sizeof(sources) / sizeof(sources[0]);
    return sources;
}
/** @brief Looks up a stable source identifier, rejecting unavailable sources. */
const FOC_EncoderSourceTypeDef *FOC_EncoderSource(uint8_t id){
    unsigned count;
    const FOC_EncoderSourceTypeDef *list = FOC_EncoderSources(&count);
    for(unsigned i = 0; i < count; i++) if(list[i].id == id) return &list[i];
    return NULL;
}
/** @brief Reads the existing absolute-initialized quadrature timer backend. */
uint8_t FOC_EncoderReadTimer(FOC_HandleTypeDef *hfoc, float *angle){
    if(hfoc->pencoder_count == NULL) return 0;
    *angle = (float)(*hfoc->pencoder_count % ENCODER_PULSES_PER_ROTATION) * (M_2PIF / ENCODER_PULSES_PER_ROTATION);
    return 1;
}
/** @brief Returns a completed DMA angle without blocking or starting a transfer. */
uint8_t FOC_EncoderReadSPI(FOC_HandleTypeDef *hfoc, float *angle){
    uint16_t count;
    if(AS5047P_GetAngleDMA(&hfoc->has5047p, &count) != AS5047P_OK) return 0;
    *angle = count * (M_2PIF / 16384.0f);
    return 1;
}
/** @brief Validates and normalizes a backend's absolute mechanical angle. */
uint8_t FOC_EncoderReadRaw(FOC_HandleTypeDef *hfoc, float *angle){
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(hfoc->flash_data.encoder.source_id);
    if(!hfoc->encoder_runtime.initialized || source == NULL || !source->read(hfoc, angle) || !isfinite(*angle)) return 0;
    normalize_angle_0_2pi(angle);
    return 1;
}
/** @brief Initializes only the selected backend, including after source/load changes. */
uint8_t FOC_EncoderInitialize(FOC_HandleTypeDef *hfoc){
    AS5047P_StopAngleDMA(&hfoc->has5047p);
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(hfoc->flash_data.encoder.source_id);
    hfoc->encoder_runtime.healthy = hfoc->encoder_runtime.initialized = 0;
    FOC_EncoderResetHistory(hfoc);
    if(source == NULL || !source->initialize(hfoc)){
        hfoc->encoder_runtime.healthy = 0;
        return 0;
    }
    hfoc->encoder_runtime.initialized = 1;
    float raw;
    if(!FOC_EncoderReadRaw(hfoc, &raw)){
        hfoc->encoder_runtime.healthy = hfoc->encoder_runtime.initialized = 0;
        return 0;
    }
    hfoc->encoder_runtime.raw_angle = raw;
    hfoc->encoder_runtime.healthy = 1;
    return 1;
}
/** @brief Interpolates a periodic table indexed by uncorrected absolute angle. */
float FOC_EncoderLUT(const float *table, float angle){
    normalize_angle_0_2pi(&angle);
    float position = angle * (FOC_ENCODER_LUT_SIZE / M_2PIF);
    unsigned index = (unsigned)position;
    if(index >= FOC_ENCODER_LUT_SIZE) index = 0;
    float fraction = position - (unsigned)position;
    return table[index] + fraction * (table[(index + 1) % FOC_ENCODER_LUT_SIZE] - table[index]);
}
/** @brief Rejects excessive/nonfinite correction and nonmonotonic mapping. */
uint8_t FOC_EncoderValidateLUT(const float *table){
    const float interval = M_2PIF / FOC_ENCODER_LUT_SIZE;
    for(unsigned i = 0; i < FOC_ENCODER_LUT_SIZE; i++){
        float value = table[i], next = table[(i + 1) % FOC_ENCODER_LUT_SIZE];
        float increment = interval + next - value;
        if(!isfinite(value) || fabsf(value) > 0.1f || !isfinite(increment) ||
           increment < interval * 0.1f || increment > interval * 2) return 0;
    }
    return 1;
}
/** @brief Applies only a valid enabled table belonging to the selected source. */
float FOC_EncoderCorrect(FOC_HandleTypeDef *hfoc, float raw){
    if(!hfoc->encoder_runtime.bypass_correction && hfoc->flash_data.encoder.correction_enabled &&
       hfoc->flash_data.encoder.correction_valid &&
       hfoc->flash_data.encoder.correction_source == hfoc->flash_data.encoder.source_id){
        raw += FOC_EncoderLUT(hfoc->flash_data.encoder.correction, raw);
    }
    normalize_angle_0_2pi(&raw);
    return raw;
}
/** @brief Resets position/speed history when the angle coordinate changes. */
void FOC_EncoderResetHistory(FOC_HandleTypeDef *hfoc){
    hfoc->encoder_runtime.history_valid = 0;
    hfoc->encoder_runtime.speed_pll_angle = 0;
    hfoc->encoder_full_rotations = 0;
    hfoc->encoder_speed_mechanical = hfoc->encoder_speed_electrical = 0;
}
/** @brief Updates mechanical/electrical estimates from the selected source. */
uint8_t FOC_EncoderUpdate(FOC_HandleTypeDef *hfoc, float frequency){
    float raw;
    if(hfoc->flash_data.encoder.source_id == FOC_ENCODER_SOURCE_SIMULATOR && isfinite(frequency) && frequency > 0 &&
       !AngleSimulator_Advance(&hfoc->encoder_runtime.simulator, 1.0f / frequency)){
        hfoc->encoder_runtime.healthy = 0;
        return 0;
    }
    if(!isfinite(frequency) || frequency <= 0 || !FOC_EncoderReadRaw(hfoc, &raw)){
        hfoc->encoder_runtime.healthy = 0;
        return 0;
    }
    hfoc->encoder_runtime.raw_angle = raw;
    hfoc->encoder_runtime.healthy = 1;
    float angle = FOC_EncoderCorrect(hfoc, raw) - hfoc->flash_data.encoder.mechanical_offset;
    normalize_angle_0_2pi(&angle);
    float delta = 0;
    if(!hfoc->encoder_runtime.history_valid){
        hfoc->encoder_runtime.speed_pll_angle = angle;
        hfoc->encoder_speed_mechanical = 0;
    }else{
        /* Critically damped alpha-beta angle tracker. The rational pole avoids
           an exponential in the current ISR and stays stable as dt changes.
           alpha=1-p*p, beta=(1-p)^2 give two identical discrete poles p. */
        float step = M_2PIF * ENCODER_SPEED_PLL_FREQUENCY_HZ / frequency;
        float gain = step / (1.0f + step);
        float alpha = gain * (2.0f - gain);
        float beta = gain * gain;
        float prediction = hfoc->encoder_runtime.speed_pll_angle + hfoc->encoder_speed_mechanical / frequency;
        normalize_angle_0_2pi(&prediction);
        float error = angle - prediction;
        if(error > M_PIF) error -= M_2PIF;
        else if(error < -M_PIF) error += M_2PIF;
        hfoc->encoder_runtime.speed_pll_angle = prediction + alpha * error;
        normalize_angle_0_2pi(&hfoc->encoder_runtime.speed_pll_angle);
        hfoc->encoder_speed_mechanical += beta * frequency * error;
    }
    if(hfoc->encoder_runtime.history_valid){
        delta = angle - hfoc->encoder_angle_mechanical_wrapped_prev;
        if(delta > M_PIF){hfoc->encoder_full_rotations--; delta -= M_2PIF;}
        else if(delta < -M_PIF){hfoc->encoder_full_rotations++; delta += M_2PIF;}
    }
    hfoc->encoder_runtime.history_valid = 1;
    hfoc->encoder_angle_mechanical_wrapped = angle;
    hfoc->encoder_angle_mechanical_wrapped_prev = angle;
    hfoc->encoder_angle_mechanical_unwrapped = angle + hfoc->encoder_full_rotations * M_2PIF;
    hfoc->encoder_angle_electrical = angle * hfoc->flash_data.motor.pole_pairs;
    normalize_angle_0_2pi(&hfoc->encoder_angle_electrical);
    hfoc->encoder_speed_electrical = hfoc->encoder_speed_mechanical * hfoc->flash_data.motor.pole_pairs;
    return 1;
}
/** @brief Stores zero in both raw and corrected coordinates for the active source. */
uint8_t FOC_EncoderSetZero(FOC_HandleTypeDef *hfoc){
    float raw;
    if(!FOC_EncoderReadRaw(hfoc, &raw)) return 0;
    hfoc->flash_data.encoder.raw_zero = raw;
    hfoc->flash_data.encoder.mechanical_offset = FOC_EncoderCorrect(hfoc, raw);
    hfoc->flash_data.encoder.offset_valid = 1;
    FOC_EncoderResetHistory(hfoc);
    return 1;
}
/** @brief Checks that a coordinate change cannot occur during motor actuation. */
static uint8_t encoder_change_allowed(FOC_HandleTypeDef *hfoc){
    return (hfoc->state == FOC_STATE_IDLE || hfoc->state == FOC_STATE_ERROR) &&
           isfinite(hfoc->encoder_speed_mechanical) && fabsf(hfoc->encoder_speed_mechanical) <= 0.1f;
}
/** @brief Selects an available source while stopped, invalidating old calibration. */
uint8_t FOC_EncoderSelect(FOC_HandleTypeDef *hfoc, uint8_t source){
    if(!encoder_change_allowed(hfoc) || FOC_EncoderSource(source) == NULL) return 0;
    if(hfoc->flash_data.encoder.source_id == source){
        return hfoc->encoder_runtime.healthy ? 1 : FOC_EncoderInitialize(hfoc);
    }
    hfoc->flash_data.encoder.source_id = source;
    hfoc->flash_data.encoder.offset_valid = 0;
    hfoc->flash_data.encoder.correction_valid = hfoc->flash_data.encoder.correction_enabled = 0;
    hfoc->flash_data.controller.anticogging_data_valid = 0;
    hfoc->encoder_runtime.healthy = 0;
    FOC_EncoderResetHistory(hfoc);
    return FOC_EncoderInitialize(hfoc);
}
/** @brief Toggles correction and rebases zero without manufacturing a speed spike. */
uint8_t FOC_EncoderEnableCorrection(FOC_HandleTypeDef *hfoc, uint8_t enabled){
    if(enabled > 1 || !encoder_change_allowed(hfoc)) return 0;
    if(enabled && (!hfoc->flash_data.encoder.correction_valid ||
       hfoc->flash_data.encoder.correction_source != hfoc->flash_data.encoder.source_id ||
       !FOC_EncoderValidateLUT(hfoc->flash_data.encoder.correction))) return 0;
    if(enabled == hfoc->flash_data.encoder.correction_enabled) return 1;
    hfoc->flash_data.encoder.correction_enabled = enabled;
    hfoc->flash_data.encoder.mechanical_offset = FOC_EncoderCorrect(hfoc, hfoc->flash_data.encoder.raw_zero);
    hfoc->flash_data.controller.anticogging_data_valid = 0;
    FOC_EncoderResetHistory(hfoc);
    return 1;
}
