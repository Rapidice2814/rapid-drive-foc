#ifndef FOC_LOOPS_H
#define FOC_LOOPS_H

#include "main.h"
#include "FOC_Routine.h"

typedef enum {
    CONTROL_MODE_OPENLOOP = 0,
    CONTROL_MODE_POSITION,
    CONTROL_MODE_SPEED
} ControlModeTypeDef;

typedef struct FOC_Handle FOC_HandleTypeDef;

void FOC_EncoderCalibration_Start(FOC_HandleTypeDef *hfoc);
void FOC_MotorProbe_Start(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_MotorProbe_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_MotorProbe_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
FOC_LoopStatusTypeDef FOC_EncoderCalibration_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_EncoderCalibration_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);

void FOC_MotorIdentification_Start(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_MotorIdentification_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_MotorIdentification_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
void FOC_OpenLoop_Reset(FOC_HandleTypeDef *hfoc);
uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency);
void FOC_Alignment_Start(FOC_HandleTypeDef *hfoc, float magnitude);
FOC_LoopStatusTypeDef FOC_Alignment_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_Alignment_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
void FOC_PIDAutotune_Start(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_PIDAutotune_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_PIDAutotune_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
void FOC_CurrentSensorCalibration_Start(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
void FOC_AlignmentTest_Start(FOC_HandleTypeDef *hfoc, float magnitude);
FOC_LoopStatusTypeDef FOC_AlignmentTest_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_AlignmentTest_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);
void FOC_AntiCogging_Start(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_AntiCogging_Update(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_AntiCogging_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result);

void Current_Loop(FOC_HandleTypeDef *hfoc);
void Speed_Loop(FOC_HandleTypeDef *hfoc);

uint8_t FOC_SetControlMode(FOC_HandleTypeDef *hfoc, ControlModeTypeDef mode);
ControlModeTypeDef FOC_GetControlMode(FOC_HandleTypeDef *hfoc);

#endif // FOC_LOOPS_H
