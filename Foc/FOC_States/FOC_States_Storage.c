#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB.h"
#include "FOC_USB_Debug.h"
#include "FOC_Diagnostics.h"

/**
 * @brief Completes storage without clearing faults or resuming motion from ERROR.
 * @param hfoc FOC handle containing the requested next state.
 */
static void FOC_StateStorage_Complete(FOC_HandleTypeDef* hfoc){
    FOC_StateTypeDef next_state = hfoc->next_state;
    if(hfoc->active_errors || hfoc->latched_errors){
        next_state = FOC_STATE_ERROR;
    }else if(hfoc->previous_state == FOC_STATE_ERROR || next_state == FOC_STATE_NONE){
        next_state = FOC_STATE_STOP;
    }
    hfoc->next_state = FOC_STATE_NONE;
    if(FOC_SetState(hfoc, next_state, FOC_STATE_NONE) != FOC_STATETRANSITION_OK){
        Debug_SendTextResponse("Cannot enter %s after flash operation; entering STOP.\n", FOC_StateToString(next_state));
        FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
    }
}

/* FOC_STATE_FLASH_SAVE */

FOC_StateTransitionTypeDef FOC_StateFlashSave_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE && hfoc->state != FOC_STATE_ERROR){
        Debug_SendTextResponse("FLASH_SAVE requires IDLE or ERROR; current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_MUST_BE_IDLE;
    }
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Saving flash data ...\n");
    if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
        Debug_SendTextResponse("Flash save failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
        return;
    }
    Debug_SendTextResponse("Flash data saved!\n");
    FOC_StateStorage_Complete(hfoc);
}

/* FOC_STATE_FLASH_LOAD */

FOC_StateTransitionTypeDef FOC_StateFlashLoad_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE && hfoc->state != FOC_STATE_ERROR){
        Debug_SendTextResponse("FLASH_LOAD requires IDLE or ERROR; current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_MUST_BE_IDLE;
    }
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashLoad(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Loading flash data ...\n");
    if(FOC_FLASH_ReadData(&hfoc->flash_data) != FLASH_OK){
        Debug_SendTextResponse("Flash load failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
        return;
    }
    if(!FOC_EncoderInitialize(hfoc)){
        Debug_SendTextResponse("Encoder initialization failed after flash load.\n");
        FOC_SetError(hfoc, FOC_ERROR_ENCODER_FAULT);
        return;
    }
    Debug_SendTextResponse("Flash data loaded!\n");
    FOC_StateStorage_Complete(hfoc);
}

/* FOC_STATE_FLASH_CLEAR */

FOC_StateTransitionTypeDef FOC_StateFlashClear_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE && hfoc->state != FOC_STATE_ERROR){
        Debug_SendTextResponse("FLASH_CLEAR requires IDLE or ERROR; current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_MUST_BE_IDLE;
    }
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashClear(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Clearing flash data ...\n");
    if(FOC_FLASH_ClearData() != FLASH_OK){
        Debug_SendTextResponse("Flash clear failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
        return;
    }
    Debug_SendTextResponse("Flash data cleared!\n");
    FOC_StateStorage_Complete(hfoc);
}
