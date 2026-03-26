#ifndef FOC_LOOPS_H
#define FOC_LOOPS_H

#include "main.h"
#include "FOC_Driver.h"
#include "logging.h"

typedef enum{
	FOC_LOOP_IN_PROGRESS,
	FOC_LOOP_COMPLETED,
    FOC_LOOP_ERROR
}FOC_LoopStatusTypeDef;

FOC_LoopStatusTypeDef FOC_BootupSound(FOC_HandleTypeDef *hfoc, float loop_frequency);
FOC_LoopStatusTypeDef FOC_MotorIdentification(FOC_HandleTypeDef *hfoc);
uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency);
FOC_LoopStatusTypeDef FOC_Alignment(FOC_HandleTypeDef *hfoc, float magnitude);
FOC_LoopStatusTypeDef FOC_PIDAutotune(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration(FOC_HandleTypeDef *hfoc);
FOC_LoopStatusTypeDef Alignment_Test_Loop(FOC_HandleTypeDef *hfoc, float magnitude);
FOC_LoopStatusTypeDef FOC_AntiCoggingMeasurement(FOC_HandleTypeDef *hfoc);

void Current_Loop(FOC_HandleTypeDef *hfoc);
void Speed_Loop(FOC_HandleTypeDef *hfoc);

#endif // FOC_LOOPS_H