#include "FOC_Statecontroller.h"
#include "FOC_Loops.h"

#include "Sounds.h"
#include "Lights.h"

static void FOC_StateInit(FOC_HandleTypeDef* hfoc){
    if(hfoc->flash_data.contains_data == 1){
        USB_printf("Flash data loaded!: Offset: %d\n", (int)(hfoc->flash_data.encoder.mechanical_offset * 1000));
    }else{
        USB_printf("Flash data is missing!\n");
    }
    FOC_SetState(hfoc, FOC_STATE_RESET, FOC_STATE_NONE);
}

static void FOC_StateReset(FOC_HandleTypeDef* hfoc){
    USB_printf("Resetting FOC ...\n");
    
    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;

    DRV8323_ExitHighImpedance(&hfoc->hdrv8323);

    FOC_SetState(hfoc, FOC_STATE_BOOTUP_SOUND, FOC_STATE_NONE);
}

static void FOC_StateBootupSound(FOC_HandleTypeDef* hfoc){
    if(SoundEngine_PlaySequence(hfoc, bootup_sound_array, sizeof(bootup_sound_array)/sizeof(bootup_sound_array[0]), 0.8f, CURRENT_LOOP_FREQUENCY) != SOUND_IN_PROGRESS){
        hfoc->state = FOC_STATE_CURRENT_SENSOR_CALIBRATION;
    }
    LightEngine_PlaySequence(bootup_light_array, sizeof(bootup_light_array)/sizeof(bootup_light_array[0]));

    // if(SoundEngine_PlaySequence(&hfoc, super_mario_sound_array, sizeof(super_mario_sound_array)/sizeof(super_mario_sound_array[0]), 0.4f, CURRENT_LOOP_FREQUENCY) != SOUND_IN_PROGRESS){
    //     hfoc->state = FOC_STATE_CURRENT_SENSOR_CALIBRATION;
    // }
    // LightEngine_PlaySequence(super_mario_light_array, sizeof(super_mario_light_array)/sizeof(super_mario_light_array[0]));
}

static void FOC_StateCurrentSensorCalibration(FOC_HandleTypeDef* hfoc){
    if(FOC_CurrentSensorCalibration(hfoc) == FOC_LOOP_COMPLETED){
        hfoc->adc_calibrated = 1;
        hfoc->state = FOC_STATE_CHECKLIST;
    }
}

static void FOC_StateChecklist(FOC_HandleTypeDef* hfoc){
    if(hfoc->flash_data.encoder.offset_valid != 1){
        hfoc->state = FOC_STATE_ALIGNMENT;
    }else if(hfoc->flash_data.motor.phase_resistance_valid != 1 || hfoc->flash_data.motor.phase_inductance_valid != 1){
        hfoc->state = FOC_STATE_IDENTIFY;
    }else if(hfoc->flash_data.controller.current_PID_gains_valid != 1){
        hfoc->state = FOC_STATE_PID_AUTOTUNE;
    } else {
        hfoc->state = FOC_STATE_RUN;
    }
}

static void FOC_StateGeneralTest(FOC_HandleTypeDef* hfoc){
    hfoc->state = FOC_STATE_RUN;
}

static void FOC_StateCalibration(FOC_HandleTypeDef* hfoc){
    hfoc->state = FOC_STATE_RUN;
}

static void FOC_StateIdentify(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_MotorIdentification(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.current_PID_gains_valid = 0;
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        USB_printf("Motor identification failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    } 
}

static void FOC_StatePIDAutotune(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_PIDAutotune(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        hfoc->flash_data.controller.current_PID_gains_valid = 1;
        hfoc->flash_data.controller.current_PID_FF_enabled = 0;
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        USB_printf("PID autotune failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

static void FOC_StateAlignment(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_Alignment(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        hfoc->state = FOC_STATE_ALIGNMENT_TEST;
    } else if(ret == FOC_LOOP_ERROR){
        USB_printf("Alignment failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

static void FOC_StateAlignmentTest(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef  ret = Alignment_Test_Loop(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

static void FOC_StateAntiCogging(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_AntiCoggingMeasurement(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 1;
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        USB_printf("Anti-cogging measurement failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}


static void FOC_StateRun(FOC_HandleTypeDef* hfoc){
    Current_Loop(hfoc);
    static uint8_t current_loop_counter = 0;
    if(++current_loop_counter >= SPEED_LOOP_CLOCK_DIVIDER){//runs at CURRENT_LOOP_FREQUENCY / CURRENT_LOOP_CLOCK_DIVIDER
        current_loop_counter = 0;
        Speed_Loop(hfoc); 
    }
}

static void FOC_StateStop(FOC_HandleTypeDef* hfoc){
    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    Current_Loop(hfoc);
    if(hfoc->encoder_speed_mechanical < 0.1f){
        FOC_SetState(hfoc, FOC_STATE_IDLE, FOC_STATE_NONE);
    }
}

static void FOC_StateIdle(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
}

static void FOC_StateError(FOC_HandleTypeDef* hfoc){
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});

    if(LightEngine_PlaySequence(error_light_array, sizeof(error_light_array)/sizeof(error_light_array[0])) != LIGHT_IN_PROGRESS){
        __NOP();
    }
}

static void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc){
    USB_printf("Saving flash data ...\n");
    if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    USB_printf("Flash data saved!\n");
    hfoc->state = hfoc->next_state;
}

static void FOC_StateFlashLoad(FOC_HandleTypeDef* hfoc){
    USB_printf("Loading flash data ...\n");
    if(FOC_FLASH_ReadData(&hfoc->flash_data) != FLASH_OK){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    USB_printf("Flash data loaded!\n");
    hfoc->state = hfoc->next_state;
}

static void FOC_StateOpenLoop(FOC_HandleTypeDef* hfoc){
    FOC_OpenLoop(hfoc, hfoc->speed_setpoint, 1.0f, CURRENT_LOOP_FREQUENCY);
}




FOC_StatusTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef state, FOC_StateTypeDef next_state){
    if(next_state != FOC_STATE_NONE){
        hfoc->next_state = next_state;
    }
    
    switch(state){
        case FOC_STATE_NONE:
            return FOC_OK;
            break;
        case FOC_STATE_INIT:
            break;
        case FOC_STATE_RESET:
            break;
        case FOC_STATE_BOOTUP_SOUND:
            break;
        case FOC_STATE_CURRENT_SENSOR_CALIBRATION:
            break;
        case FOC_STATE_IDENTIFY:
            break;
        case FOC_STATE_ANTICOGGING:
            break;
        case FOC_STATE_CHECKLIST:
            break;
        case FOC_STATE_PID_AUTOTUNE:
            break;
        case FOC_STATE_GENERAL_TEST:
            break;
        case FOC_STATE_ERROR:
            break;
        case FOC_STATE_CALIBRATION:
            break;
        case FOC_STATE_ALIGNMENT:
            break;
        case FOC_STATE_ALIGNMENT_TEST:
            break;
        case FOC_STATE_RUN:
            break;
        case FOC_STATE_STOP:
            break;
        case FOC_STATE_IDLE:
            if(hfoc->state != FOC_STATE_STOP){
                USB_printf("Can only enter IDLE state from STOP state! Current state: %d\n", hfoc->state);
                return FOC_ERROR; // can only enter IDLE state from STOP state
            }
            if(hfoc->encoder_speed_mechanical > 0.1f){
                USB_printf("Cannot enter IDLE state because motor is not stopped! Speed: %dmRad/s\n", (int)(hfoc->encoder_speed_mechanical * 1000.0f));
                return FOC_ERROR; // can only enter IDLE state if motor is stopped
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

            break;
        case FOC_STATE_FLASH_SAVE:
            if(hfoc->state != FOC_STATE_IDLE){
                USB_printf("Can only enter FLASH_SAVE from IDLE state! Current state: %d\n", hfoc->state);
                return FOC_ERROR; // can only save flash from IDLE state
            }
            break;
        case FOC_STATE_FLASH_LOAD:
            if(hfoc->state != FOC_STATE_IDLE){
                USB_printf("Can only enter FLASH_LOAD from IDLE state! Current state: %d\n", hfoc->state);
                return FOC_ERROR; // can only load flash from IDLE state
            }
            break;
        case FOC_STATE_OPENLOOP:
            break;
    }
    
    
    
    
    hfoc->previous_state = hfoc->state;
    hfoc->state = state;
    return FOC_OK;
}

void FOC_StateLoop(FOC_HandleTypeDef *hfoc){
    switch(hfoc->state){
        case FOC_STATE_NONE: return; break;
        case FOC_STATE_INIT: FOC_StateInit(hfoc); break;
        case FOC_STATE_RESET: FOC_StateReset(hfoc); break;
        case FOC_STATE_BOOTUP_SOUND: FOC_StateBootupSound(hfoc); break;
        case FOC_STATE_CURRENT_SENSOR_CALIBRATION: FOC_StateCurrentSensorCalibration(hfoc); break;
        case FOC_STATE_CHECKLIST: FOC_StateChecklist(hfoc); break;
        case FOC_STATE_GENERAL_TEST: FOC_StateGeneralTest(hfoc); break;
        case FOC_STATE_CALIBRATION: FOC_StateCalibration(hfoc); break;
        case FOC_STATE_IDENTIFY: FOC_StateIdentify(hfoc); break;
        case FOC_STATE_PID_AUTOTUNE: FOC_StatePIDAutotune(hfoc); break;
        case FOC_STATE_ALIGNMENT: FOC_StateAlignment(hfoc); break;
        case FOC_STATE_ALIGNMENT_TEST: FOC_StateAlignmentTest(hfoc); break;
        case FOC_STATE_ANTICOGGING: FOC_StateAntiCogging(hfoc); break;
        case FOC_STATE_RUN: FOC_StateRun(hfoc); break;
        case FOC_STATE_STOP: FOC_StateStop(hfoc); break;
        case FOC_STATE_IDLE: FOC_StateIdle(hfoc); break;
        case FOC_STATE_ERROR: FOC_StateError(hfoc); break;
        case FOC_STATE_FLASH_SAVE: FOC_StateFlashSave(hfoc); break;
        case FOC_STATE_FLASH_LOAD: FOC_StateFlashLoad(hfoc); break;
        case FOC_STATE_OPENLOOP: FOC_StateOpenLoop(hfoc); break;
    }
}