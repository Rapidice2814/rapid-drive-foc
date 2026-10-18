#include <math.h>
#include <string.h>
#include "FOC_Handle.h"
#include "FOC_Calibration.h"

/** @brief Returns the flattened float count; anti-cogging channels are interleaved. */
uint16_t FOC_CalibrationSize(uint8_t kind){
    return kind == 0 ? FOC_ENCODER_LUT_SIZE : kind == 1 ? 2 * NUMBER_OF_ANTICOG_MEASUREMENTS : 0;
}
/** @brief Allows table transfers only while stationary with no routine running. */
uint8_t FOC_CalibrationSafe(FOC_HandleTypeDef *h){
    return (h->state == FOC_STATE_IDLE || h->state == FOC_STATE_ERROR) &&
        !h->encoder_calibration.runtime.active && !h->anticogging.runtime.active &&
        isfinite(h->encoder_speed_mechanical) && fabsf(h->encoder_speed_mechanical) <= 0.1f;
}
/** @brief Reads one accepted entry in wire order. */
float FOC_CalibrationValue(FOC_HandleTypeDef *h, uint8_t kind, unsigned index){
    return kind == 0 ? h->flash_data.encoder.correction[index] : h->flash_data.controller.anticogging_array[index % 2][index / 2];
}
/** @brief Opens an upload; anti-cogging is invalidated in place to conserve RAM. */
uint8_t FOC_CalibrationBegin(FOC_HandleTypeDef *h, uint8_t kind, uint16_t total){
    if(!FOC_CalibrationSafe(h) || h->calibration_upload.active || !total || total != FOC_CalibrationSize(kind)) return 0;
    h->calibration_upload = (FOC_CalibrationUploadTypeDef){.active=1, .kind=kind, .source=h->flash_data.encoder.source_id, .crc=0xFFFFFFFFU};
    if(kind == 1){
        h->flash_data.controller.anticogging_data_valid = 0;
        h->flash_data.controller.anticogging_FF_enabled = 0;
    }
    return 1;
}
/** @brief Validates a complete sequential chunk before writing any of its entries. */
uint8_t FOC_CalibrationWrite(FOC_HandleTypeDef *h, uint8_t kind, uint16_t offset, const float *values, uint8_t count){
    FOC_CalibrationUploadTypeDef *u = &h->calibration_upload;
    if(!FOC_CalibrationSafe(h) || !u->active || u->kind != kind || offset != u->received ||
       u->source != h->flash_data.encoder.source_id || !count || count > FOC_CALIBRATION_CHUNK ||
       offset + count > FOC_CalibrationSize(kind)) return 0;
    float limit = kind == 0 ? 0.1f : h->flash_data.limits.max_dq_current;
    if(!isfinite(limit) || limit <= 0) return 0;
    for(unsigned i=0; i<count; i++) if(!isfinite(values[i]) || fabsf(values[i]) > limit) return 0;
    for(unsigned i=0; i<count; i++){
        unsigned index = offset+i;
        if(kind == 0) h->encoder_calibration.forward[index] = values[i];
        else h->flash_data.controller.anticogging_array[index % 2][index / 2] = values[i];
    }
    const uint8_t *bytes = (const uint8_t *)values;
    for(unsigned i=0; i<4U*count; i++){
        u->crc ^= bytes[i];
        for(unsigned bit=0; bit<8; bit++) u->crc = (u->crc >> 1) ^ (0xEDB88320U & (0U - (u->crc & 1U)));
    }
    u->received += count;
    return 1;
}
/** @brief Commits a complete CRC-checked table, disabled until explicitly enabled. */
uint8_t FOC_CalibrationCommit(FOC_HandleTypeDef *h, uint8_t kind, uint32_t crc){
    FOC_CalibrationUploadTypeDef *u = &h->calibration_upload;
    if(!FOC_CalibrationSafe(h) || !u->active || u->kind != kind || u->received != FOC_CalibrationSize(kind) ||
       u->source != h->flash_data.encoder.source_id || ~u->crc != crc) return 0;
    if(kind == 0){
        if(!h->flash_data.encoder.offset_valid || !FOC_EncoderValidateLUT(h->encoder_calibration.forward)) return 0;
        memcpy(h->flash_data.encoder.correction, h->encoder_calibration.forward, sizeof(h->flash_data.encoder.correction));
        h->flash_data.encoder.correction_valid = 1;
        h->flash_data.encoder.correction_source = u->source;
        h->flash_data.encoder.correction_enabled = 0;
        h->flash_data.encoder.mechanical_offset = h->flash_data.encoder.raw_zero;
        h->flash_data.controller.anticogging_data_valid = 0;
        FOC_EncoderResetHistory(h);
    }else{
        float limit = h->flash_data.limits.max_dq_current;
        if(!isfinite(limit) || limit <= 0) return 0;
        for(unsigned i=0; i<FOC_CalibrationSize(kind); i++)
            if(!isfinite(FOC_CalibrationValue(h, kind, i)) || fabsf(FOC_CalibrationValue(h, kind, i)) > limit) return 0;
        h->flash_data.controller.anticogging_data_valid = 1;
    }
    u->active = 0;
    return 1;
}
/** @brief Changes compensation only in safe states and with a valid table. */
uint8_t FOC_CalibrationEnable(FOC_HandleTypeDef *h, uint8_t kind, uint8_t enabled){
    if(!FOC_CalibrationSafe(h) || h->calibration_upload.active || kind > 1 || enabled > 1) return 0;
    if(kind == 0) return FOC_EncoderEnableCorrection(h, enabled);
    if(enabled){
        if(!h->flash_data.controller.anticogging_data_valid || !isfinite(h->flash_data.limits.max_dq_current) || h->flash_data.limits.max_dq_current <= 0) return 0;
        for(unsigned i=0; i<FOC_CalibrationSize(kind); i++)
            if(!isfinite(FOC_CalibrationValue(h, kind, i)) || fabsf(FOC_CalibrationValue(h, kind, i)) > h->flash_data.limits.max_dq_current) return 0;
    }
    h->flash_data.controller.anticogging_FF_enabled = enabled;
    return 1;
}
