#include "FOC_Loops.h"
#include <math.h>

static void play_frequency(FOC_HandleTypeDef *hfoc, float frequency, float magnitude, float loop_frequency);

/**
 * @brief Plays a sound array outputting voltages to the motor. Function should be called at loop_frequency
 * @param hfoc: pointer to the FOC handle
 * @param melody_array: array of NoteTypeDef structs that define the melody to play
 * @param melody_length: length of the melody array
 * @param magnitude: magnitude of the sound in volts
 * @param loop_frequency: frequency at which this function is called, used to calculate the phase increment. Should be in Hz.
 * @retval FOC_LoopStatusTypeDef
 */
FOC_LoopStatusTypeDef FOC_PlayMelody(FOC_HandleTypeDef *hfoc, const NoteTypeDef *melody_array, uint16_t melody_length, float magnitude, float loop_frequency){

    static uint16_t step = 0;
    static uint32_t next_step_time = 0;

    if(step == 0){
        play_frequency(hfoc, 0.0f, 0.0f, loop_frequency);
        step++;
        next_step_time = HAL_GetTick() + melody_array[0].duration;
    } else if(step <= melody_length){
        play_frequency(hfoc, melody_array[step-1].frequency, magnitude, loop_frequency);
        if(HAL_GetTick() >= next_step_time){
            step++;
            next_step_time = HAL_GetTick() + melody_array[step-1].duration;
        }
    } else{
        play_frequency(hfoc, 0.0f, 0.0f, loop_frequency);
        step = 0;
        return FOC_LOOP_COMPLETED;
    }

    return FOC_LOOP_IN_PROGRESS;
}



/**
* @brief Plays a sound by outputting voltages to the motor. Function should be called at loop_frequency
* @param hfoc: pointer to the FOC handle
* @param sound_frequency: frequency of the sound to play in Hz (0.0 = rest)
* @param magnitude: magnitude of the sound in volts
* @param loop_frequency: frequency at which this function is called, used to calculate the phase increment. Should be in Hz.
* @retval none
*/
static void play_frequency(FOC_HandleTypeDef *hfoc, float sound_frequency, float magnitude, float loop_frequency){
    static float phase = 0.0f;

    if(sound_frequency < 10.0f || magnitude <= 0.1f){
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
        return;
    }

    phase += (sound_frequency * M_TWOPI) / loop_frequency;
    normalize_angle_0_2pi(&phase);

    ABVoltagesTypeDef Vab = {
        .alpha = magnitude * cosf(phase),
        .beta  = magnitude * sinf(phase)
    };

    FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform(Vab));

    
}
