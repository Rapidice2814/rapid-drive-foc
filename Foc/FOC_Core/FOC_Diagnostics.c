#include "FOC_Diagnostics.h"
#include "FOC_States.h"
#include "FOC_Handle.h"

void FOC_CheckErrors(FOC_HandleTypeDef *hfoc){
    uint32_t active_errors = 0;

    if(DRV8323_CheckFault(&hfoc->hdrv8323)) active_errors |= FOC_ERROR_DRIVER_FAULT;
    
    if(hfoc->adc_values.motor_temp > hfoc->flash_data.limits.motor_temp_trip_level) active_errors |= FOC_ERROR_MOTOR_OT;
    if(hfoc->adc_values.motor_temp < 0.0f) active_errors |= FOC_ERROR_MOTOR_UT;

    if(hfoc->adc_values.mosfet_temp > hfoc->flash_data.limits.mosfet_temp_trip_level) active_errors |= FOC_ERROR_MOSFET_OT;
    if(hfoc->adc_values.mosfet_temp < 0.0f) active_errors |= FOC_ERROR_MOSFET_UT;

    if(hfoc->adc_values.vbus > hfoc->flash_data.limits.vbus_overvoltage_trip_level) active_errors |= FOC_ERROR_VBUS_OV;
    if(hfoc->adc_values.vbus < hfoc->flash_data.limits.vbus_undervoltage_trip_level) active_errors |= FOC_ERROR_VBUS_UV;
    if(hfoc->ibus > hfoc->flash_data.limits.ibus_overcurrent_trip_level) active_errors |= FOC_ERROR_IBUS_OC;

    hfoc->active_errors = active_errors;
    hfoc->latched_errors |= active_errors;

    if(hfoc->latched_errors){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

void FOC_ClearLatchedErrors(FOC_HandleTypeDef *hfoc){
    hfoc->latched_errors = 0;
}

void FOC_SetError(FOC_HandleTypeDef *hfoc, uint32_t error){
    hfoc->active_errors |= error;
    hfoc->latched_errors |= error;
    FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
}

uint32_t FOC_GetActiveErrors(FOC_HandleTypeDef *hfoc){
    return hfoc->active_errors;
}

uint32_t FOC_GetLatchedErrors(FOC_HandleTypeDef *hfoc){
    return hfoc->latched_errors;
}