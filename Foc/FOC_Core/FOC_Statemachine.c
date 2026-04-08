#include "FOC_Statemachine.h"
#include "FOC_Loops.h"

#include "Sounds.h"
#include "Lights.h"



extern volatile uint8_t debug_loop_flag;


static void FOC_StateInit(FOC_HandleTypeDef* hfoc){
    if(hfoc->flash_data.contains_data == 1){
        Log_printf("Flash data loaded!: Offset: %d\n", (int)(hfoc->flash_data.encoder.mechanical_offset * 1000));
    }else{
        Log_printf("Flash data is missing!\n");
    }
    hfoc->state = FOC_STATE_RESET;
}

static void FOC_StateReset(FOC_HandleTypeDef* hfoc){
    Log_printf("Resetting FOC ...\n");
    DRV8323_ExitHighImpedance(&hfoc->hdrv8323);

    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;

    hfoc->state = FOC_STATE_BOOTUP_SOUND;
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
        Log_printf("Motor identification failed!\n");
        hfoc->state = FOC_STATE_ERROR;
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
        Log_printf("PID autotune failed!\n");
        hfoc->state = FOC_STATE_ERROR;
    }
}

static void FOC_StateAlignment(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_Alignment(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        hfoc->state = FOC_STATE_ALIGNMENT_TEST;
    } else if(ret == FOC_LOOP_ERROR){
        Log_printf("Alignment failed!\n");
        hfoc->state = FOC_STATE_ERROR;
    }
}

static void FOC_StateAlignmentTest(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef  ret = Alignment_Test_Loop(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        hfoc->state = FOC_STATE_ERROR;
    }
}

static void FOC_StateAntiCogging(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_AntiCoggingMeasurement(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 1;
        hfoc->state = FOC_STATE_CHECKLIST;
    } else if(ret == FOC_LOOP_ERROR){
        Log_printf("Anti-cogging measurement failed!\n");
        hfoc->state = FOC_STATE_ERROR;
    }
}

static void FOC_StateRun(FOC_HandleTypeDef* hfoc){
    Current_Loop(hfoc);
    static uint8_t current_loop_counter = 0;
    if(++current_loop_counter >= SPEED_LOOP_CLOCK_DIVIDER){//runs at CURRENT_LOOP_FREQUENCY / CURRENT_LOOP_CLOCK_DIVIDER
        current_loop_counter = 0;
        Speed_Loop(hfoc); 
    }

    if(debug_loop_flag){
        Log_printf("Vq:%d, Iq:%d, Iq_set:%d, Mspeed:%d, Ibus:%d, Vbus:%d, Temp:%d, Mang:%d\n",
        (int)(hfoc->dq_voltage.q * 1000), (int)(hfoc->dq_current.q * 1000), 
        (int)(hfoc->dq_current_setpoint.q * 1000), (int)(hfoc->encoder_speed_mechanical * 1000),
        (int)(hfoc->ibus * 1000), (int)(hfoc->adc_values.vbus * 10), (int)(hfoc->adc_values.motor_temp * 10),
        (int)(hfoc->encoder_angle_mechanical * 1000));

        // Log_printf("Time: %d, ADC1 Time: %d, ADC2 Time: %d, Log Time: %d, Count:%d\n",
        //     (int)max_execution_time, (int)max_adc1_time, (int)max_adc2_time, (int)max_log_time);

        debug_loop_flag = 0;
    }
}

static void FOC_StateError(FOC_HandleTypeDef* hfoc){
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});

    if(LightEngine_PlaySequence(error_light_array, sizeof(error_light_array)/sizeof(error_light_array[0])) != LIGHT_IN_PROGRESS){
        __NOP();
    }
}

static void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc){
    if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
        hfoc->state = FOC_STATE_ERROR;
    }
    hfoc->state = FOC_STATE_RUN;
}

static void FOC_StateOpenLoop(FOC_HandleTypeDef* hfoc){
    FOC_OpenLoop(hfoc, hfoc->speed_setpoint, 1.0f, CURRENT_LOOP_FREQUENCY);
}

void FOC_StateLoop(FOC_HandleTypeDef *hfoc){
    switch(hfoc->state){
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
        case FOC_STATE_ERROR: FOC_StateError(hfoc); break;
        case FOC_STATE_FLASH_SAVE: FOC_StateFlashSave(hfoc); break;
        case FOC_STATE_OPENLOOP: FOC_StateOpenLoop(hfoc); break;
    }
}