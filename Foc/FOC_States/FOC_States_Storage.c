#include "FOC_States.h"
#include "FOC_USB.h"

/* FOC_STATE_FLASH_SAVE */

FOC_StateTransitionTypeDef FOC_StateFlashSave_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        USB_printf("Can only enter FLASH_SAVE from IDLE state! Current state: %d\n", hfoc->state);
        return FOC_STATETRANSITION_DENIED; // can only save flash from IDLE state
    }
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc){
    USB_printf("Saving flash data ...\n");
    if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    USB_printf("Flash data saved!\n");
    hfoc->state = hfoc->next_state;
}

/* FOC_STATE_FLASH_LOAD */

FOC_StateTransitionTypeDef FOC_StateFlashLoad_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE){
        USB_printf("Can only enter FLASH_LOAD from IDLE state! Current state: %d\n", hfoc->state);
        return FOC_STATETRANSITION_DENIED; // can only load flash from IDLE state
    }
    return FOC_STATETRANSITION_OK;
}

void FOC_StateFlashLoad(FOC_HandleTypeDef* hfoc){
    USB_printf("Loading flash data ...\n");
    if(FOC_FLASH_ReadData(&hfoc->flash_data) != FLASH_OK){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
    USB_printf("Flash data loaded!\n");
    hfoc->state = hfoc->next_state;
}
