#include "FOC_Loops.h"
#include <math.h>
#include "FOC_Utils.h"
#include "FOC_Config.h"


void Current_Loop(FOC_HandleTypeDef *hfoc){
    float encoder_angle_electrical, setpoint_q, setpoint_d;

    if(hfoc->flash_data.motor_direction_swapped_flag == 1){
        encoder_angle_electrical = 2 * M_PI - hfoc->encoder_angle_electrical;
        setpoint_q = -hfoc->dq_current_setpoint.q;
        setpoint_d = -hfoc->dq_current_setpoint.d;
    }else{
        encoder_angle_electrical = hfoc->encoder_angle_electrical;
        setpoint_q = hfoc->dq_current_setpoint.q;
        setpoint_d = hfoc->dq_current_setpoint.d;
    }


    if(hfoc->flash_data.anticogging_enabled_flag == 1 && hfoc->flash_data.anticogging_data_valid_flag == 1){ //apply the anticog LUT to the setpoint
        float encoder_angle_mechanical = hfoc->encoder_angle_mechanical;
        normalize_angle(&encoder_angle_mechanical);
        uint16_t anticog_index = (uint16_t)(encoder_angle_mechanical / ANTICOG_ANGLE_STEP);
        uint8_t anticog_dir = (hfoc->encoder_speed_mechanical >= 0) ? 0 : 1;
        setpoint_q += hfoc->flash_data.anticogging_measurements[anticog_dir][anticog_index];
    }

    hfoc->ab_current = FOC_Clarke_transform(hfoc->phase_current);
    hfoc->dq_current = FOC_Park_transform(hfoc->ab_current, encoder_angle_electrical);
    
    hfoc->dq_voltage.d = PID_Update(&hfoc->pid_current_d, setpoint_d, hfoc->dq_current.d);
    hfoc->dq_voltage.q = PID_Update(&hfoc->pid_current_q, setpoint_q, hfoc->dq_current.q);
    
    if(hfoc->flash_data.current_PID_FF_enabled_flag == 1){
        float lambda = (2.0f/3.0f) * hfoc->flash_data.motor_torque_constant * (1/hfoc->flash_data.motor_pole_pairs);
        hfoc->dq_voltage.d += -hfoc->encoder_speed_electrical * hfoc->flash_data.motor_stator_inductance * hfoc->dq_current.q;
        hfoc->dq_voltage.q +=  hfoc->encoder_speed_electrical * (hfoc->flash_data.motor_stator_inductance * hfoc->dq_current.d + lambda);
    }



    hfoc->ab_voltage = FOC_InvPark_transform(hfoc->dq_voltage, encoder_angle_electrical);
    PhaseVoltagesTypeDef phase_voltages = FOC_InvClarke_transform(hfoc->ab_voltage);
    FOC_SetPhaseVoltages(hfoc, phase_voltages);

}

void Speed_Loop(FOC_HandleTypeDef *hfoc){
    
    if(hfoc->flash_data.speed_PID_enabled_flag == 1){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_speed, hfoc->speed_setpoint, hfoc->encoder_speed_mechanical);
    }

    if(hfoc->flash_data.position_PID_enabled_flag == 1){
        hfoc->dq_current_setpoint.q = PID_Update(&hfoc->pid_position, hfoc->angle_setpoint, hfoc->encoder_angle_mechanical);
    }
}