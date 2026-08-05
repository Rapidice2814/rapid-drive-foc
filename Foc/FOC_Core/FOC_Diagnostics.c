#include "FOC_Diagnostics.h"
#include "FOC_States.h"
#include "FOC_Handle.h"


#define FOC_ERROR_INITIALIZING (1u << 0)
#define FOC_ERROR_SYSTEM_FAULT (1u << 1)
#define FOC_ERROR_TIMING_VIOLATION (1u << 2)
#define FOC_ERROR_INVALID_CONFIGURATION (1u << 3)

#define FOC_ERROR_DRIVER_FAULT (1u << 4)

#define FOC_ERROR_MOTOR_OT (1u << 5)
#define FOC_ERROR_MOTOR_UT (1u << 6)
#define FOC_ERROR_MOTOR_DISCONNECTED (1u << 7)
#define FOC_ERROR_MOTOR_STALL (1u << 8)

#define FOC_ERROR_MOSFET_OT (1u << 9)
#define FOC_ERROR_MOSFET_UT (1u << 10)

#define FOC_ERROR_VBUS_OV (1u << 11)
#define FOC_ERROR_VBUS_UV (1u << 12)

#define FOC_ERROR_IBUS_OC (1u << 13)

#define FOC_ERROR_ENCODER_FAULT (1u << 14)





void FOC_CheckErrors(FOC_HandleTypeDef *hfoc){
    uint32_t active_errors = 0;

    if(DRV8323_CheckFault(&hfoc->hdrv8323)) active_errors |= FOC_ERROR_DRIVER_FAULT;

    
    if(hfoc->adc_values.motor_temp > hfoc->flash_data.limits.motor_temp_trip_level) active_errors |= FOC_ERROR_MOTOR_OT;
    if(hfoc->adc_values.motor_temp < 0.0f) active_errors |= FOC_ERROR_MOTOR_UT;

    if(hfoc->adc_values.mosfet_temp > hfoc->flash_data.limits.mosfet_temp_trip_level) active_errors |= FOC_ERROR_MOSFET_OT;
    if(hfoc->adc_values.mosfet_temp < 0.0f) active_errors |= FOC_ERROR_MOSFET_UT;

    if(hfoc->adc_values.vbus > hfoc->flash_data.limits.vbus_overvoltage_trip_level) active_errors |= FOC_ERROR_VBUS_OV;
    if(hfoc->adc_values.vbus < hfoc->flash_data.limits.vbus_undervoltage_trip_level) active_errors |= FOC_ERROR_VBUS_UV;

    if(active_errors){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}