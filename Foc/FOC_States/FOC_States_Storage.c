#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB.h"
#include "FOC_USB_Debug.h"

/**
 * @brief Leaves a completed storage operation, falling back to STOP if denied.
 * @param hfoc FOC handle containing the requested next state.
 */
static void FOC_StateStorage_Complete(FOC_HandleTypeDef* hfoc){
    FOC_StateTypeDef next_state = hfoc->next_state;
    hfoc->next_state = FOC_STATE_NONE;
    if(FOC_SetState(hfoc, next_state, FOC_STATE_NONE) != FOC_STATETRANSITION_OK){
        Debug_SendTextResponse("Cannot enter %s after flash operation; entering STOP.\n", FOC_StateToString(next_state));
        FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
    }
}

/* FOC_STATE_FLASH_SAVE */

FOC_StateTransitionTypeDef FOC_StateFlashSave_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        Debug_SendTextResponse("Can only enter FOC_STATE_FLASH_SAVE from FOC_STATE_IDLE! Current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_DENIED; // can only save flash from IDLE state
    }
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Saving flash data ...\n");
    if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
        Debug_SendTextResponse("Flash save failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
        return;
    }
    Debug_SendTextResponse("Flash data saved!\n");
    FOC_StateStorage_Complete(hfoc);
}

/* FOC_STATE_FLASH_LOAD */

FOC_StateTransitionTypeDef FOC_StateFlashLoad_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        Debug_SendTextResponse("Can only enter FOC_STATE_FLASH_LOAD from FOC_STATE_IDLE! Current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_DENIED; // can only load flash from IDLE state
    }
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashLoad(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Loading flash data ...\n");
    if(FOC_FLASH_ReadData(&hfoc->flash_data) != FLASH_OK){
        Debug_SendTextResponse("Flash load failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
        return;
    }
    Debug_SendTextResponse("Flash data loaded!\n");
    FOC_StateStorage_Complete(hfoc);
}

/* FOC_STATE_FLASH_CLEAR */

FOC_StateTransitionTypeDef FOC_StateFlashClear_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        Debug_SendTextResponse("Can only enter FOC_STATE_FLASH_CLEAR from FOC_STATE_IDLE! Current state: %s\n", FOC_StateToString(hfoc->state));
        return FOC_STATETRANSITION_DENIED; // can only clear flash from IDLE state
    }
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashClear(FOC_HandleTypeDef* hfoc){
    Debug_SendTextResponse("Clearing flash data ...\n");
    if(FOC_FLASH_ClearData() != FLASH_OK){
        Debug_SendTextResponse("Flash clear failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
        return;
    }
    Debug_SendTextResponse("Flash data cleared!\n");
    FOC_StateStorage_Complete(hfoc);
}
