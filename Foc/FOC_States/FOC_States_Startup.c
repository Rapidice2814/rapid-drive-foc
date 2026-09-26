#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB.h"
#include "FOC_USB_Debug.h"
#include "Bootloader.h"

#include "BootupSounds.h"
#include "BootupLights.h"

/* FOC_STATE_BOOTLOADER */

FOC_StateTransitionTypeDef FOC_StateBootloader_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        Debug_SendTextResponse("Can only enter FOC_STATE_BOOTLOADER from FOC_STATE_IDLE! Current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_DENIED;
    }

    WS2812b_SetAllColor(0, 0, 25);
    WS2812b_Send();

    return FOC_STATETRANSITION_OK;
}

void FOC_StateBootloader(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    HAL_Delay(100);
    JumpToBootloader(); 
    
    while(1); // MCU will reset and jump to the bootloader, so no need to set next state
}

/* FOC_STATE_INIT */

FOC_StateTransitionTypeDef FOC_StateInit_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateInit(FOC_HandleTypeDef* hfoc){
    if(hfoc->flash_data.contains_data == 1){
        Debug_SendTextResponse("Flash data loaded!: Offset: %d\n", (int)(hfoc->flash_data.encoder.mechanical_offset * 1000));
    }else{
        Debug_SendTextResponse("Flash data is missing!\n");
    }
    FOC_SetState(hfoc, FOC_STATE_RESET, FOC_STATE_NONE);
}

/* FOC_STATE_RESET */

FOC_StateTransitionTypeDef FOC_StateReset_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateReset(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Resetting FOC ...\n");
    
    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;

    DRV8323_ExitHighImpedance(&hfoc->hdrv8323);

    FOC_SetState(hfoc, FOC_STATE_BOOTUP_SOUND, FOC_STATE_CHECKLIST);
}

/* FOC_STATE_BOOTUP_SOUND */

FOC_StateTransitionTypeDef FOC_StateBootupSound_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateBootupSound(FOC_HandleTypeDef* hfoc){
    if(SoundEngine_PlaySequence(hfoc, bootup_sound_array, sizeof(bootup_sound_array)/sizeof(bootup_sound_array[0]), 0.8f, CURRENT_LOOP_FREQUENCY) != SOUND_IN_PROGRESS){
        FOC_NextState(hfoc);
    }
    LightEngine_PlaySequence(bootup_light_array, sizeof(bootup_light_array)/sizeof(bootup_light_array[0]));

    // if(SoundEngine_PlaySequence(&hfoc, super_mario_sound_array, sizeof(super_mario_sound_array)/sizeof(super_mario_sound_array[0]), 0.4f, CURRENT_LOOP_FREQUENCY) != SOUND_IN_PROGRESS){
    //     hfoc->state = FOC_STATE_CURRENT_SENSOR_CALIBRATION;
    // }
    // LightEngine_PlaySequence(super_mario_light_array, sizeof(super_mario_light_array)/sizeof(super_mario_light_array[0]));
}