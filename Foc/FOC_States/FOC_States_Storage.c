#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB.h"
#include "FOC_USB_Debug.h"

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
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    Debug_SendTextResponse("Flash data saved!\n");
    FOC_NextState(hfoc);
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
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    Debug_SendTextResponse("Flash data loaded!\n");
    FOC_NextState(hfoc);
}
