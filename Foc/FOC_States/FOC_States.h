#ifndef FOC_STATES_H
#define FOC_STATES_H

typedef enum {
    FOC_STATETRANSITION_OK,
    FOC_STATETRANSITION_INVALID,
    FOC_STATETRANSITION_DENIED,
    FOC_STATETRANSITION_ERROR
} FOC_StateTransitionTypeDef;

/**
 * @param state_enum: The enumerated value representing the state.
 * @param transition_fn: The function that is executed when the state is entered.
 * @param loop_fn: The function that is called repeatedly while the system is in this state
 */
#define FOC_STATE_TABLE(X) \
    X(FOC_STATE_INIT,                       FOC_StateInit_Transition,                       FOC_StateInit) \
    X(FOC_STATE_RESET,                      FOC_StateReset_Transition,                      FOC_StateReset) \
    X(FOC_STATE_BOOTUP_SOUND,               FOC_StateBootupSound_Transition,                FOC_StateBootupSound) \
    X(FOC_STATE_CURRENT_SENSOR_CALIBRATION, FOC_StateCurrentSensorCalibration_Transition,   FOC_StateCurrentSensorCalibration) \
    X(FOC_STATE_IDENTIFY,                   FOC_StateIdentify_Transition,                   FOC_StateIdentify) \
    X(FOC_STATE_ANTICOGGING,                FOC_StateAntiCogging_Transition,                FOC_StateAntiCogging) \
    X(FOC_STATE_CHECKLIST,                  FOC_StateTransition_AlwaysOk,                   FOC_StateChecklist) \
    X(FOC_STATE_PID_AUTOTUNE,               FOC_StatePIDAutotune_Transition,                FOC_StatePIDAutotune) \
    X(FOC_STATE_ERROR,                      FOC_StateError_Transition,                      FOC_StateError) \
    X(FOC_STATE_ALIGNMENT,                  FOC_StateAlignment_Transition,                  FOC_StateAlignment) \
    X(FOC_STATE_ALIGNMENT_TEST,             FOC_StateAlignmentTest_Transition,              FOC_StateAlignmentTest) \
    X(FOC_STATE_RUN,                        FOC_StateRun_Transition,                        FOC_StateRun) \
    X(FOC_STATE_STOP,                       FOC_StateStop_Transition,                       FOC_StateStop) \
    X(FOC_STATE_IDLE,                       FOC_StateIdle_Transition,                       FOC_StateIdle) \
    X(FOC_STATE_FLASH_SAVE,                 FOC_StateFlashSave_Transition,                  FOC_StateFlashSave) \
    X(FOC_STATE_FLASH_LOAD,                 FOC_StateFlashLoad_Transition,                  FOC_StateFlashLoad) \
    X(FOC_STATE_BOOTLOADER,                 FOC_StateBootloader_Transition,                 FOC_StateBootloader) \
    X(FOC_STATE_OPENLOOP,                   FOC_StateOpenLoop_Transition,                   FOC_StateOpenLoop)

typedef enum
{
    FOC_STATE_NONE = 0,

#define FOC_STATE_ENUM(state, transition_function, run_function) \
    state,

    FOC_STATE_TABLE(FOC_STATE_ENUM)

#undef FOC_STATE_ENUM

    FOC_STATE_COUNT
} FOC_StateTypeDef;

static const char *const FOC_StateNames[FOC_STATE_COUNT] =
{
    [FOC_STATE_NONE] = "FOC_STATE_NONE",

#define FOC_STATE_NAME(state, transition_function, run_function) \
    [state] = #state,

    FOC_STATE_TABLE(FOC_STATE_NAME)

#undef FOC_STATE_NAME
};

typedef struct FOC_Handle FOC_HandleTypeDef;

/* State String Conversion */
const char *FOC_StateToString(FOC_StateTypeDef state);

/* State Functions */
FOC_StateTransitionTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef state, FOC_StateTypeDef next_state);
void FOC_NextState(FOC_HandleTypeDef *hfoc);
void FOC_StateLoop(FOC_HandleTypeDef *hfoc);


/* Startup States */
FOC_StateTransitionTypeDef FOC_StateBootloader_Transition(FOC_HandleTypeDef* hfoc);
void FOC_StateBootloader(FOC_HandleTypeDef* hfoc);
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