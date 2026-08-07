#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_Loops.h"
#include "FOC_USB_Debug.h"
#include "WS2812b_Driver.h"


/*FOC_STATE_CURRENT_SENSOR_CALIBRATION*/

FOC_StateTransitionTypeDef FOC_StateCurrentSensorCalibration_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateCurrentSensorCalibration(FOC_HandleTypeDef* hfoc){
    if(FOC_CurrentSensorCalibration(hfoc) == FOC_LOOP_COMPLETED){
        hfoc->adc_calibrated = 1;
        FOC_NextState(hfoc);
    }
}

/*FOC_STATE_IDENTIFY*/

FOC_StateTransitionTypeDef FOC_StateIdentify_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);

    WS2812b_SetAllColor(20, 15, 0);
    WS2812b_Send();

    return FOC_STATETRANSITION_OK;
}

void FOC_StateIdentify(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_MotorIdentification(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        FOC_NextState(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Motor identification failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    } 
}

/*FOC_STATE_PID_AUTOTUNE*/

FOC_StateTransitionTypeDef FOC_StatePIDAutotune_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StatePIDAutotune(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_PIDAutotune(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        hfoc->flash_data.controller.current_PID_FF_enabled = 0;
        FOC_NextState(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("PID autotune failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

/*FOC_STATE_ALIGNMENT*/

FOC_StateTransitionTypeDef FOC_StateAlignment_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);

    WS2812b_SetAllColor(20, 15, 0);
    WS2812b_Send();

    return FOC_STATETRANSITION_OK;
}

void FOC_StateAlignment(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_Alignment(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        FOC_NextState(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Alignment failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

/*FOC_STATE_ALIGNMENT_TEST*/

FOC_StateTransitionTypeDef FOC_StateAlignmentTest_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);

    WS2812b_SetAllColor(20, 15, 0);
    WS2812b_Send();
    
    return FOC_STATETRANSITION_OK;
}

void FOC_StateAlignmentTest(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef  ret = Alignment_Test_Loop(hfoc, 1.0f);
    if(ret == FOC_LOOP_COMPLETED){
        FOC_NextState(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}

/*FOC_STATE_ANTICOGGING*/

FOC_StateTransitionTypeDef FOC_StateAntiCogging_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);

    WS2812b_SetAllColor(20, 15, 0);
    WS2812b_Send();

    return FOC_STATETRANSITION_OK;
}

void FOC_StateAntiCogging(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_AntiCoggingMeasurement(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 1;
        FOC_NextState(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Anti-cogging measurement failed!\n");
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }
}