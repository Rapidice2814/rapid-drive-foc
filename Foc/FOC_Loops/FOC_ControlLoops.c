#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include <math.h>
#include "Utils.h"
#include "FOC_Config.h"
#include "FOC_USB_Debug.h"

#include "FOC_HFI.h"


void Current_Loop(FOC_HandleTypeDef *hfoc){
    float encoder_angle_electrical, setpoint_q, setpoint_d;

    if(hfoc->flash_data.motor.direction == 1){
        encoder_angle_electrical = 2 * M_PI - hfoc->encoder_angle_electrical;
        setpoint_q = -hfoc->dq_current_setpoint.q;
        setpoint_d = -hfoc->dq_current_setpoint.d;
    }else{
        encoder_angle_electrical = hfoc->encoder_angle_electrical;
        setpoint_q = hfoc->dq_current_setpoint.q;
        setpoint_d = hfoc->dq_current_setpoint.d;
    }


    if(hfoc->flash_data.controller.anticogging_FF_enabled == 1 && hfoc->flash_data.controller.anticogging_data_valid == 1){ //apply the anticog LUT to the setpoint
        float encoder_angle_mechanical_wrapped = hfoc->encoder_angle_mechanical_wrapped;
        normalize_angle_0_2pi(&encoder_angle_mechanical_wrapped);
        uint16_t anticog_index = (uint16_t)(encoder_angle_mechanical_wrapped / ANTICOG_ANGLE_STEP); //TODO: put this in a function
        uint8_t anticog_dir = (hfoc->encoder_speed_mechanical >= 0) ? 0 : 1;
        setpoint_q += hfoc->flash_data.controller.anticogging_array[anticog_dir][anticog_index];
    }
    
    setpoint_d = constrainf(setpoint_d, -hfoc->flash_data.limits.max_dq_current, hfoc->flash_data.limits.max_dq_current);
    setpoint_q = constrainf(setpoint_q, -hfoc->flash_data.limits.max_dq_current, hfoc->flash_data.limits.max_dq_current);


    hfoc->ab_current = Clarke_transform(hfoc->adc_values.phase_current);
    hfoc->dq_current = Park_transform(hfoc->ab_current, encoder_angle_electrical);

    hfoc->dq_current_filtered.d = biquad_filter_update(&hfoc->dq_current_d_filter, hfoc->dq_current.d);
    hfoc->dq_current_filtered.q = biquad_filter_update(&hfoc->dq_current_q_filter, hfoc->dq_current.q);

    // hfoc->dq_voltage.d = PID_Update(&hfoc->pid_current_d, setpoint_d, hfoc->dq_current_filtered.d);
    // hfoc->dq_voltage.q = PID_Update(&hfoc->pid_current_q, setpoint_q, hfoc->dq_current_filtered.q);
    
    hfoc->dq_voltage.d = PID_Update(&hfoc->pid_current_d, setpoint_d, hfoc->dq_current.d);
    hfoc->dq_voltage.q = PID_Update(&hfoc->pid_current_q, setpoint_q, hfoc->dq_current.q);

    if(hfoc->flash_data.hfi.hfi_enabled == 1){
        hfoc->dq_voltage.d += FOC_HFI_GetInjectedVoltage(hfoc);
    }

    if(hfoc->flash_data.controller.current_PID_FF_enabled == 1){
        if(hfoc->flash_data.motor.torque_constant != 0 && 
            hfoc->flash_data.motor.pole_pairs != 0 && 
            hfoc->flash_data.motor.phase_inductance != 0 &&
            hfoc->flash_data.motor.phase_resistance != 0){

                float lambda = (2.0f/3.0f) * hfoc->flash_data.motor.torque_constant / (float)hfoc->flash_data.motor.pole_pairs;
                hfoc->dq_voltage.d += -hfoc->encoder_speed_electrical * hfoc->flash_data.motor.phase_inductance * hfoc->dq_current.q;
                hfoc->dq_voltage.q +=  hfoc->encoder_speed_electrical * (hfoc->flash_data.motor.phase_inductance * hfoc->dq_current.d + lambda);
                
            } 
    }


    hfoc->ab_voltage = InvPark_transform(hfoc->dq_voltage, encoder_angle_electrical);
    PhaseVoltagesTypeDef phase_voltage = InvClarke_transform(hfoc->ab_voltage);
    FOC_SetPhaseVoltages(hfoc, phase_voltage);

}

void Speed_Loop(FOC_HandleTypeDef *hfoc){
    
    if(hfoc->flash_data.controller.control_mode == CONTROL_MODE_SPEED){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_speed, hfoc->speed_setpoint, hfoc->encoder_speed_mechanical);
    }

    if(hfoc->flash_data.controller.control_mode == CONTROL_MODE_POSITION){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_position, hfoc->angle_setpoint, hfoc->encoder_angle_mechanical_unwrapped);
    }
}

uint8_t FOC_SetControlMode(FOC_HandleTypeDef *hfoc, ControlModeTypeDef mode){
    switch(mode){
        case CONTROL_MODE_OPENLOOP:
            hfoc->flash_data.controller.control_mode = CONTROL_MODE_OPENLOOP;
            hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
            Debug_SendTextResponse("Disabled both speed and position PID\n");
            break;
        case CONTROL_MODE_SPEED:
            hfoc->flash_data.controller.control_mode = CONTROL_MODE_SPEED;
            hfoc->speed_setpoint = 0.0f;
            Debug_SendTextResponse("Enabled speed PID, disabled position PID\n");
            break;
        case CONTROL_MODE_POSITION:
            hfoc->flash_data.controller.control_mode = CONTROL_MODE_POSITION;
            hfoc->angle_setpoint = 0.0f;
            Debug_SendTextResponse("Enabled position PID, disabled speed PID\n");
            break;
        default:
            Debug_SendTextResponse("Invalid control mode: %d\n", mode);
            return 1; // invalid mode
    }
    return 0;
}

ControlModeTypeDef FOC_GetControlMode(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.controller.control_mode;
}