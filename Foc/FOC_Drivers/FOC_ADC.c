#include "FOC_ADC.h"
#include "FOC_Config.h"
#include "NTC_Lookup.h"
#include "DRV8323_Driver.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* flags, used for the interrupts*/
static volatile uint8_t adc1_complete_flag = 0;
static volatile uint8_t adc1_half_complete_flag = 0;

static volatile uint8_t adc2_complete_flag = 0;
static volatile uint8_t adc2_half_complete_flag = 0;

static FOC_ADC_ValuesTypeDef adc_offset_values = {0};


/* ADC gains*/
#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / (CURRENT_SENSE_RESISTANCE * CSA_GAIN_VALUE))
#define VOLTAGE_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / VBUS_VOLTAGE_DIVIDER_RATIO)

/* ADC buffers */
static volatile uint16_t adc1_buffer[ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2] = {0};
static volatile uint16_t adc2_buffer[ADC2_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2] = {0};

NTC_LUT_TypeDef hntclut;

/**
 * @brief Initializes the ADCs for current and voltage measurements. Starts the ADC in DMA mode. Triggered by the PWM timer.
 * @param None
 */
void FOC_ADC_Setup(){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2); //start adc in dma mode for the current and vbus
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC2_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2); //adc for temp sensors
    GenerateNtcLut(&hntclut, NTC_TEMP_MIN, NTC_TEMP_MAX, NTC_B, NTC_R0, NTC_T0);
}


/**
 * @brief Checks if ADC has available measurements. 
 * @param hadc_values Pointer to the ADC values structure, which will be updated if a new measurement is available.
 * @retval FOC_ADC_StatusTypeDef
 */
FOC_ADC_StatusTypeDef FOC_ADC_Measure(FOC_ADC_ValuesTypeDef *hadc_values){
    static uint8_t foc_adc1_measurement_flag = 0;
    static uint8_t foc_adc2_measurement_flag = 0;

    static FOC_ADC_ValuesTypeDef adc_values = {0};

    if(adc1_half_complete_flag || adc1_complete_flag){ //This loop runs at (PWM frequency / CURRENT_LOOP_CLOCK_SCALER)

        int32_t acc_phase_current_a = 0;
        int32_t acc_phase_current_b = 0;
        int32_t acc_phase_current_c = 0;
        int32_t acc_vbus = 0;

        int start_index = adc1_half_complete_flag ? 0 : CURRENT_LOOP_CLOCK_DIVIDER * ADC1_CHANNELS;
        int end_index = start_index + CURRENT_LOOP_CLOCK_DIVIDER * ADC1_CHANNELS;

        for (int i = start_index; i < end_index; i += ADC1_CHANNELS) {
            int32_t raw_a = (int32_t)adc1_buffer[i + 0] - 2048;
            int32_t raw_b = (int32_t)adc1_buffer[i + 1] - 2048;
            int32_t raw_c = (int32_t)adc1_buffer[i + 2] - 2048;
            int32_t raw_v = (int32_t)adc1_buffer[i + 3];

            acc_phase_current_a += raw_a;
            acc_phase_current_b += raw_b;
            acc_phase_current_c += raw_c;
            acc_vbus += raw_v;

            //TODO: check if saturated
            if (abs(raw_a) > 2000 || abs(raw_b) > 2000 || abs(raw_c) > 2000) {
                __NOP();
            }
        }

        adc_values.phase_current.a = (CURRENT_SENSE_CONVERSION_FACTOR * (float)acc_phase_current_a / CURRENT_LOOP_CLOCK_DIVIDER);
        adc_values.phase_current.b = (CURRENT_SENSE_CONVERSION_FACTOR * (float)acc_phase_current_b / CURRENT_LOOP_CLOCK_DIVIDER);
        adc_values.phase_current.c = (CURRENT_SENSE_CONVERSION_FACTOR * (float)acc_phase_current_c / CURRENT_LOOP_CLOCK_DIVIDER);
        adc_values.vbus = VOLTAGE_SENSE_CONVERSION_FACTOR * (float)acc_vbus / CURRENT_LOOP_CLOCK_DIVIDER;

        foc_adc1_measurement_flag = 1; //triggered after the adc conversion is complete (either first or second half of the buffer)
        adc1_half_complete_flag = 0;
        adc1_complete_flag = 0;
    }

    if(adc2_half_complete_flag || adc2_complete_flag){

        int32_t acc_ntc1_resistance = 0;
        int32_t acc_ntc2_resistance = 0;

        int start_index = adc2_half_complete_flag ? 0 : CURRENT_LOOP_CLOCK_DIVIDER * ADC2_CHANNELS;
        int end_index = start_index + CURRENT_LOOP_CLOCK_DIVIDER * ADC2_CHANNELS;

        for (int i = start_index; i < end_index; i += ADC2_CHANNELS) {
            int32_t raw_ntc1 = (int32_t)adc2_buffer[i + 0];
            int32_t raw_ntc2 = (int32_t)adc2_buffer[i + 1];

            acc_ntc1_resistance += raw_ntc1;
            acc_ntc2_resistance += raw_ntc2;
        }

        float ntc1_resistance = NTC_SERIES_RESISTOR * ((4095.0f / (float)acc_ntc1_resistance * CURRENT_LOOP_CLOCK_DIVIDER) - 1);
        float ntc2_resistance = NTC_SERIES_RESISTOR * ((4095.0f / (float)acc_ntc2_resistance * CURRENT_LOOP_CLOCK_DIVIDER) - 1);

        adc_values.mosfet_temp = GetNtcTemperature(&hntclut, ntc1_resistance);
        adc_values.motor_temp = GetNtcTemperature(&hntclut, ntc2_resistance);

        foc_adc2_measurement_flag = 1;
        adc2_half_complete_flag = 0;
        adc2_complete_flag = 0;
    }

    if(foc_adc1_measurement_flag && foc_adc2_measurement_flag){
        foc_adc1_measurement_flag = 0;
        foc_adc2_measurement_flag = 0;

        hadc_values->phase_current.a = adc_values.phase_current.a - adc_offset_values.phase_current.a;
        hadc_values->phase_current.b = adc_values.phase_current.b - adc_offset_values.phase_current.b;
        hadc_values->phase_current.c = adc_values.phase_current.c - adc_offset_values.phase_current.c;

        hadc_values->vbus = adc_values.vbus - adc_offset_values.vbus;
        hadc_values->mosfet_temp = adc_values.mosfet_temp - adc_offset_values.mosfet_temp;
        hadc_values->motor_temp = adc_values.motor_temp - adc_offset_values.motor_temp;

        return FOC_ADC_MEASUREMENT_COMPLETE;
    }

    return FOC_ADC_MEASUREMENT_PENDING;
}

/**
 * @brief Sets the phase current offsets
 * @param PhaseCurrentsTypeDef Pointer to the structure containing the phase current offsets
 * @retval None
 */
void FOC_ADC_SetPhaseCurrentOffsets(PhaseCurrentsTypeDef *offset_values){
    adc_offset_values.phase_current.a = offset_values->a;
    adc_offset_values.phase_current.b = offset_values->b;
    adc_offset_values.phase_current.c = offset_values->c;
}





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is complete
    if(hadc->Instance == ADC1){ 
        if(!adc1_complete_flag){
            adc1_complete_flag = 1;  
        }
    } else if(hadc->Instance == ADC2){
        if(!adc2_complete_flag){
            adc2_complete_flag = 1;  
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is half complete
    if(hadc->Instance == ADC1){
        if(!adc1_half_complete_flag){
            adc1_half_complete_flag = 1;
        }
    } else if(hadc->Instance == ADC2){
        if(!adc2_half_complete_flag){
            adc2_half_complete_flag = 1;
        }
    }
}