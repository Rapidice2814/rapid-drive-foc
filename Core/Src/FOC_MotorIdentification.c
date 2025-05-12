#include "FOC_Loops.h"
#include <math.h>

/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete
  */
uint8_t FOC_MotorIdentification(FOC_HandleTypeDef *hfoc){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint32_t substep_counter = 0;

    static uint8_t th_counter = 0;

    static AlphaBetaCurrents PreviousPhaseCurrents = {0.0f, 0.0f};

    static uint8_t measurement_step_counter = 0;
    static AlphaBetaCurrents AlphaBetaArray[25] = {0.0f, 0.0f};

    static AlphaBetaCurrents prevlog = {0.0f, 0.0f};
    static AlphaBetaCurrents cumlog = {0.0f, 0.0f};

    static float reference_electrical_angle = 0.0f;

    static float cumResistance = 0.0f;
    static float cumInductance = 0.0f;

    static float tempbuffer[100] = {0.0f};
    static float tempbuffer2[100] = {0.0f};
    static uint8_t tempbuffer_index = 0;


    hfoc->ab_current = FOC_Clarke_transform(hfoc->phase_current);


    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){

                hfoc->ab_voltage.alpha = 0.0f;
                hfoc->ab_voltage.beta = 0.0f;
                PreviousPhaseCurrents.alpha = 0.0f;
                PreviousPhaseCurrents.beta = 0.0f;
                measurement_step_counter = 0;

                prevlog.alpha = 0.0f;
                prevlog.beta = 0.0f;
                cumlog.alpha = 0.0f;
                cumlog.beta = 0.0f;

                cumResistance = 0.0f;
                cumInductance = 0.0f;

                for(int i = 0; i < 25; i++){
                    AlphaBetaArray[i].alpha = 0.0f;
                    AlphaBetaArray[i].beta = 0.0f;
                }


                for(int i = 0; i < 100; i++){
                    tempbuffer[i] = 0.0f;
                    tempbuffer2[i] = 0.0f;
                }
                tempbuffer_index = 0;

                step++;
                next_step_time = HAL_GetTick() + 10; //wait before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){
                

                float magnitude = 0.5f;

                hfoc->ab_voltage.alpha = magnitude * cosf(reference_electrical_angle);
                hfoc->ab_voltage.beta = magnitude * sinf(reference_electrical_angle);


                AlphaBetaArray[measurement_step_counter].alpha = hfoc->ab_current.alpha;
                AlphaBetaArray[measurement_step_counter].beta = hfoc->ab_current.beta;
                

                if((fabsf(hfoc->ab_current.alpha - PreviousPhaseCurrents.alpha) < 0.05f && fabsf(hfoc->ab_current.beta - PreviousPhaseCurrents.beta) < 0.05f)){
                    th_counter++;
                    if(th_counter > 5){
                        th_counter = 0;


                        for(int i = 0; i < measurement_step_counter-5; i++){
                            if(fabsf(hfoc->ab_voltage.alpha) > 0.1f){
                                float currlog = logf(1.0f - (AlphaBetaArray[i].alpha / AlphaBetaArray[measurement_step_counter].alpha)); 
                                tempbuffer[i] = currlog - prevlog.alpha;
                                if(i>2){
                                    cumlog.alpha += currlog - prevlog.alpha;
                                }

                                prevlog.alpha = currlog;
                            } 
                            
                            if(fabsf(hfoc->ab_voltage.beta) > 0.1f){
                                float currlog = logf(1.0f - (AlphaBetaArray[i].beta / AlphaBetaArray[measurement_step_counter].beta));
                                tempbuffer2[i] = currlog - prevlog.beta;
                                if(i>2){
                                    cumlog.beta += currlog - prevlog.beta;
                                }

                                prevlog.beta = currlog;
                            }
                            
                        }
                        
                        float ResistanceA = 0.0f;
                        float ResistanceB = 0.0f;

                        float InductanceA = 0.0f;
                        float InductanceB = 0.0f;

                        if(fabsf(hfoc->ab_voltage.alpha) > 0.1f){
                            ResistanceA = hfoc->ab_voltage.alpha / AlphaBetaArray[measurement_step_counter].alpha;
                            float slope = ((cumlog.alpha * CURRENT_LOOP_FREQUENCY) / (measurement_step_counter - 5 - 2));
                            float tau = -1.0f / slope;
                            InductanceA = tau * ResistanceA;
                        } else{
                            ResistanceA = 0.0f;
                            InductanceA = 0.0f;
                        }
                        if(fabsf(hfoc->ab_voltage.beta) > 0.1f){
                            ResistanceB = hfoc->ab_voltage.beta / AlphaBetaArray[measurement_step_counter].beta;
                            float slope2 = ((cumlog.beta * CURRENT_LOOP_FREQUENCY) / (measurement_step_counter - 5 - 2));
                            float tau2 = -1.0f / slope2;
                            InductanceB = tau2 * ResistanceB;
                        } else{
                            ResistanceB = 0.0f;
                            InductanceB = 0.0f;
                        }
                        measurement_step_counter = 0;

                        float weightA = hfoc->ab_voltage.alpha / (hfoc->ab_voltage.alpha + hfoc->ab_voltage.beta);
                        float weightB = hfoc->ab_voltage.beta / (hfoc->ab_voltage.alpha + hfoc->ab_voltage.beta);


                        float Inductance_total = weightA * InductanceA + weightB * InductanceB;
                        float Resistance_total = weightA * ResistanceA + weightB * ResistanceB;

                        cumResistance += Resistance_total;
                        cumInductance += Inductance_total;
                        
                        if(substep_counter >= 3){
                            substep_counter = 0;
                            step++;
                            next_step_time = HAL_GetTick() + 10; //wait before the next step
                        } else{
                            substep_counter++;
                            reference_electrical_angle += 2.0f * M_PIF / 4.0f;
                            next_step_time = HAL_GetTick() + 2; //wait before the next substep
                        }
                        
                    } else{
                        
                    }
                }

                PreviousPhaseCurrents.alpha = hfoc->ab_current.alpha;
                PreviousPhaseCurrents.beta = hfoc->ab_current.beta;

                if(measurement_step_counter >= 25){
                    //error
                    measurement_step_counter = 0;
                    step++;
                    __NOP();
                } else{
                    measurement_step_counter++;
                }
                
            }
            break;
        default:
            if(HAL_GetTick() >= next_step_time){

                hfoc->ab_voltage.alpha = 0.0f;
                hfoc->ab_voltage.beta = 0.0f;

                float Resistance = cumResistance /4.0f;
                float Inductance = cumInductance /4.0f;



                if((Resistance < 5.0f && Resistance > 0.0f) && (Inductance < 0.1f && Inductance > 0.0f)){
                    hfoc->flash_data.motor_stator_resistance = Resistance;
                    hfoc->flash_data.motor_stator_inductance = Inductance;

                    step = 0;
                    return 1; // complete
                }


                step = 0;
                return 0; //retry      
            }
            break;
    }

    // hfoc.ab_voltage = FOC_InvPark_transform(hfoc.dq_voltage, hfoc.encoder_angle_electrical);
    PhaseVoltages phase_voltages = FOC_InvClarke_transform(hfoc->ab_voltage);
    FOC_SetPhaseVoltages(hfoc, phase_voltages);

return 0;
}

