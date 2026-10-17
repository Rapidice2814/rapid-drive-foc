#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include <math.h>
#include "Utils.h"
#include "FOC_Config.h"
#include "FOC_USB_Debug.h"

#include "FOC_HFI.h"


void Current_Loop(FOC_HandleTypeDef *hfoc){
    float encoder_angle_electrical, speed_electrical, setpoint_q, setpoint_d;

    // angles and speeds below are in the phase frame, setpoints are given in the encoder direction
    // only q flips with the direction, the d-axis (magnet flux) is the same in both frames
    setpoint_d = hfoc->dq_current_setpoint.d;
    if(hfoc->flash_data.motor.direction == 1){
        encoder_angle_electrical = 2 * M_PI - hfoc->encoder_angle_electrical;
        speed_electrical = -hfoc->encoder_speed_electrical;
        setpoint_q = -hfoc->dq_current_setpoint.q;
    }else{
        encoder_angle_electrical = hfoc->encoder_angle_electrical;
        speed_electrical = hfoc->encoder_speed_electrical;
        setpoint_q = hfoc->dq_current_setpoint.q;
    }


    if(hfoc->flash_data.controller.anticogging_FF_enabled == 1 && hfoc->flash_data.controller.anticogging_data_valid == 1){ //apply the anticog LUT to the setpoint
        float encoder_angle_mechanical_wrapped = hfoc->encoder_angle_mechanical_wrapped;
        normalize_angle_0_2pi(&encoder_angle_mechanical_wrapped);
        uint16_t anticog_index = (uint16_t)(encoder_angle_mechanical_wrapped / ANTICOG_ANGLE_STEP); //TODO: put this in a function
        if(anticog_index >= NUMBER_OF_ANTICOG_MEASUREMENTS) anticog_index = 0; // the angle can be exactly 2pi
        uint8_t anticog_dir = (hfoc->encoder_speed_mechanical >= 0) ? 0 : 1;
        setpoint_q += hfoc->flash_data.controller.anticogging_array[anticog_dir][anticog_index];
    }

    hfoc->ab_current = Clarke_transform(hfoc->adc_values.phase_current);

    float control_angle_electrical = encoder_angle_electrical;
    ABCurrentsTypeDef ab_current_control = hfoc->ab_current;
    uint8_t hfi_active = FOC_HFI_IsActive(hfoc);

    if(hfi_active){
        ab_current_control = FOC_HFI_Update(hfoc, hfoc->ab_current, encoder_angle_electrical); //removes the carrier from the controlled current

        if(hfoc->flash_data.hfi.mode == HFI_MODE_SENSORLESS){
            control_angle_electrical = hfoc->hfi.angle;
            speed_electrical = hfoc->hfi.speed;
            if(!FOC_HFI_IsLocked(hfoc)){ //no torque until the angle and polarity are known
                setpoint_d = 0.0f;
                setpoint_q = 0.0f;
            }
        }

        FOC_HFI_ApplyBiasCurrent(hfoc, control_angle_electrical, &setpoint_d, &setpoint_q);
    } else {
        hfoc->hfi.state = HFI_STATE_RESET;
    }

    setpoint_d = constrainf(setpoint_d, -hfoc->flash_data.limits.max_dq_current, hfoc->flash_data.limits.max_dq_current);
    setpoint_q = constrainf(setpoint_q, -hfoc->flash_data.limits.max_dq_current, hfoc->flash_data.limits.max_dq_current);

    hfoc->dq_current = Park_transform(ab_current_control, control_angle_electrical);

    hfoc->dq_current_filtered.d = biquad_filter_update(&hfoc->dq_current_d_filter, hfoc->dq_current.d);
    hfoc->dq_current_filtered.q = biquad_filter_update(&hfoc->dq_current_q_filter, hfoc->dq_current.q);

    // hfoc->dq_voltage.d = PID_Update(&hfoc->pid_current_d, setpoint_d, hfoc->dq_current_filtered.d);
    // hfoc->dq_voltage.q = PID_Update(&hfoc->pid_current_q, setpoint_q, hfoc->dq_current_filtered.q);
    
    hfoc->dq_voltage.d = PID_Update(&hfoc->pid_current_d, setpoint_d, hfoc->dq_current.d);
    hfoc->dq_voltage.q = PID_Update(&hfoc->pid_current_q, setpoint_q, hfoc->dq_current.q);

    if(hfi_active && FOC_HFI_OverridesCurrentControl(hfoc)){ //HFI polarity voltage pulses, the controller must not react
        hfoc->dq_voltage = (DQVoltagesTypeDef){0.0f, 0.0f};
        PID_Reset(&hfoc->pid_current_d);
        PID_Reset(&hfoc->pid_current_q);
    } else if(hfoc->flash_data.controller.current_PID_FF_enabled == 1){
        if(hfoc->flash_data.motor.torque_constant != 0 && 
            hfoc->flash_data.motor.pole_pairs != 0 && 
            hfoc->flash_data.motor.phase_inductance != 0 &&
            hfoc->flash_data.motor.phase_resistance != 0){

                float lambda = (2.0f/3.0f) * hfoc->flash_data.motor.torque_constant / (float)hfoc->flash_data.motor.pole_pairs;
                hfoc->dq_voltage.d += -speed_electrical * hfoc->flash_data.motor.phase_inductance * hfoc->dq_current.q;
                hfoc->dq_voltage.q +=  speed_electrical * (hfoc->flash_data.motor.phase_inductance * hfoc->dq_current.d + lambda);

            }
    }


    // the voltage is applied during the next PWM period, by then the rotor has moved on
    const float output_angle_electrical = control_angle_electrical + speed_electrical * (CURRENT_LOOP_OUTPUT_DELAY / CURRENT_LOOP_FREQUENCY);
    hfoc->ab_voltage = InvPark_transform(hfoc->dq_voltage, output_angle_electrical);
    if(hfi_active){
        FOC_HFI_AddInjectionVoltage(hfoc, &hfoc->ab_voltage);
    }
    PhaseVoltagesTypeDef phase_voltage = InvClarke_transform(hfoc->ab_voltage);
    FOC_SetPhaseVoltages(hfoc, phase_voltage);

}

void Speed_Loop(FOC_HandleTypeDef *hfoc){
    float speed_mechanical = hfoc->encoder_speed_mechanical;

    if(FOC_HFI_IsActive(hfoc) && hfoc->flash_data.hfi.mode == HFI_MODE_SENSORLESS){
        if(!FOC_HFI_IsLocked(hfoc) || hfoc->flash_data.motor.pole_pairs == 0){ //don't wind up while the current loop ignores the setpoint
            PID_Reset(&hfoc->pid_speed);
            PID_Reset(&hfoc->pid_position);
            return;
        }
        speed_mechanical = hfoc->hfi.speed / (float)hfoc->flash_data.motor.pole_pairs; //phase frame, convert to the encoder direction
        if(hfoc->flash_data.motor.direction == 1) speed_mechanical = -speed_mechanical;
    }

    if(hfoc->flash_data.controller.control_mode == CONTROL_MODE_SPEED){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_speed, hfoc->speed_setpoint, speed_mechanical);
    }

    if(hfoc->flash_data.controller.control_mode == CONTROL_MODE_POSITION){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_position, hfoc->angle_setpoint, hfoc->encoder_angle_mechanical_unwrapped);
    }
}

/**
 * @brief Sets the control mode of the FOC.
 * @param hfoc: Pointer to the FOC handle
 * @param mode: The new control mode
 * @return 0 if successful, 1 if an error occurred
 */
uint8_t FOC_SetControlMode(FOC_HandleTypeDef *hfoc, ControlModeTypeDef mode){
    if(hfoc->flash_data.controller.control_mode == mode) return 0; // no change needed

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

/**
 * @brief Retrieves the current control mode of the FOC.
 * @param hfoc: Pointer to the FOC handle
 * @return ControlModeTypeDef, the current control mode
 */
ControlModeTypeDef FOC_GetControlMode(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.controller.control_mode;
}