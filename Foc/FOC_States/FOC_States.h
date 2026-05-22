#ifndef FOC_STATES_H
#define FOC_STATES_H

#include "FOC_Utils.h"

typedef enum {
    FOC_STATETRANSITION_OK,
    FOC_STATETRANSITION_INVALID,
    FOC_STATETRANSITION_DENIED,
    FOC_STATETRANSITION_ERROR
} FOC_StateTransitionTypeDef;


/* State Functions */
FOC_StateTransitionTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef state, FOC_StateTypeDef next_state);
void FOC_NextState(FOC_HandleTypeDef *hfoc);
void FOC_StateLoop(FOC_HandleTypeDef *hfoc);


/* Startup States */
FOC_StateTransitionTypeDef FOC_StateInit_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateInit(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateReset_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateReset(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateBootupSound_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateBootupSound(FOC_HandleTypeDef* hfoc);

/* Calibration States */
FOC_StateTransitionTypeDef FOC_StateCurrentSensorCalibration_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateCurrentSensorCalibration(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateIdentify_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateIdentify(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StatePIDAutotune_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StatePIDAutotune(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateAlignment_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateAlignment(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateAlignmentTest_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateAlignmentTest(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateAntiCogging_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateAntiCogging(FOC_HandleTypeDef* hfoc);

/* Operation States */
FOC_StateTransitionTypeDef FOC_StateRun_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateRun(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateStop_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateStop(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateIdle_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateIdle(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateOpenLoop_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateOpenLoop(FOC_HandleTypeDef* hfoc);

/* Storage States */
FOC_StateTransitionTypeDef FOC_StateFlashSave_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateFlashSave(FOC_HandleTypeDef* hfoc);
FOC_StateTransitionTypeDef FOC_StateFlashLoad_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateFlashLoad(FOC_HandleTypeDef* hfoc);

/* Fault State */
FOC_StateTransitionTypeDef FOC_StateError_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateError(FOC_HandleTypeDef* hfoc);



#endif // FOC_STATES_H