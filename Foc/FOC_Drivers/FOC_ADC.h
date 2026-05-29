#ifndef FOC_ADC_H
#define FOC_ADC_H

#include "FOC_Utils.h"


typedef enum{
    FOC_ADC_MEASUREMENT_COMPLETE,
	FOC_ADC_MEASUREMENT_PENDING
}FOC_ADC_StatusTypeDef;

typedef struct {
    PhaseCurrentsTypeDef phase_current;            //measured phase currents [A]
    
    float vbus; //bus voltage [V]

    float mosfet_temp; //mosfet temperature [C]
    float motor_temp; //motor temperature [C]
} FOC_ADC_ValuesTypeDef;


void FOC_ADC_Setup();
FOC_ADC_StatusTypeDef FOC_ADC_Measure(FOC_ADC_ValuesTypeDef *hadc_values);
void FOC_ADC_SetPhaseCurrentOffsets(PhaseCurrentsTypeDef *offset_values);

#endif /* FOC_ADC_H */