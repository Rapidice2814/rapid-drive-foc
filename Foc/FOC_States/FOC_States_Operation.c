#include "FOC_States.h"
#include "FOC_Loops.h"
#include <math.h>

/* FOC_STATE_RUN */

FOC_StateTransitionTypeDef FOC_StateRun_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->adc_calibrated != 1){
        USB_printf("Cannot enter RUN state because ADC is not calibrated!\n");
        return FOC_STATETRANSITION_DENIED; // can only enter RUN state if ADC is calibrated
    }
    if(hfoc->flash_data.encoder.offset_valid != 1){
        USB_printf("Cannot enter RUN state because encoder is not aligned!\n");
        return FOC_STATETRANSITION_DENIED; // can only enter RUN state if encoder is aligned
    }
    if(hfoc->flash_data.controller.current_PID_gains_valid != 1){
        USB_printf("Cannot enter RUN state because current PID gains are not valid!\n");
        return FOC_STATETRANSITION_DENIED; // can only enter RUN state if current PID gains are valid
    }

    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;

    PID_Reset(&hfoc->pid_current_d);
    PID_Reset(&hfoc->pid_current_q);
    PID_Reset(&hfoc->pid_speed);
    PID_Reset(&hfoc->pid_position);

    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    DRV8323_ExitHighImpedance(&hfoc->hdrv8323);

    return FOC_STATETRANSITION_OK;
}

void FOC_StateRun(FOC_HandleTypeDef* hfoc){
    Current_Loop(hfoc);
    static uint8_t current_loop_counter = 0;
    if(++current_loop_counter >= SPEED_LOOP_CLOCK_DIVIDER){//runs at CURRENT_LOOP_FREQUENCY / CURRENT_LOOP_CLOCK_DIVIDER
        current_loop_counter = 0;
        Speed_Loop(hfoc); 
    }
}


/* FOC_STATE_STOP */

FOC_StateTransitionTypeDef FOC_StateStop_Transition(FOC_HandleTypeDef* hfoc){
    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    return FOC_STATETRANSITION_OK;
}

void FOC_StateStop(FOC_HandleTypeDef* hfoc){
    Current_Loop(hfoc);
    if(fabsf(hfoc->encoder_speed_mechanical) > 0.1f) return; //wait until the motor is stopped before transitioning to IDLE state
    FOC_SetState(hfoc, FOC_STATE_IDLE, FOC_STATE_NONE);
}

/* FOC_STATE_IDLE */

FOC_StateTransitionTypeDef FOC_StateIdle_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_STOP){
        USB_printf("Can only enter IDLE state from STOP state! Current state: %d\n", hfoc->state);
        return FOC_STATETRANSITION_DENIED; // can only enter IDLE state from STOP state
    }
    if(fabsf(hfoc->encoder_speed_mechanical) > 0.1f){
        USB_printf("Cannot enter IDLE state because motor is not stopped! Speed: %dmRad/s\n", (int)(hfoc->encoder_speed_mechanical * 1000.0f));
        return FOC_STATETRANSITION_DENIED; // can only enter IDLE state if motor is stopped
    }

    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;

    PID_Reset(&hfoc->pid_current_d);
    PID_Reset(&hfoc->pid_current_q);
    PID_Reset(&hfoc->pid_speed);
    PID_Reset(&hfoc->pid_position);

    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);

    return FOC_STATETRANSITION_OK;
}

void FOC_StateIdle(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
}

/* FOC_STATE_OPENLOOP */

FOC_StateTransitionTypeDef FOC_StateOpenLoop_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateOpenLoop(FOC_HandleTypeDef* hfoc){
    FOC_OpenLoop(hfoc, hfoc->speed_setpoint, 1.0f, CURRENT_LOOP_FREQUENCY);
}