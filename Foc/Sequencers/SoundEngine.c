#include "SoundEngine.h"
#include "FOC_Utils.h"
#include "Cordic.h"

#include <math.h>

static void play_frequency(FOC_HandleTypeDef *hfoc, float frequency, float magnitude, float loop_frequency);

/**
 * @brief Plays a sound array outputting voltages to the motor. Function should be called at loop_frequency
 * @param hfoc: pointer to the FOC handle
 * @param sequence_array: array of NoteTypeDef structs that define the sequence to play
 * @param sequence_length: length of the sequence array
 * @param amplitude: amplitude of the sound, normalized to 1.0f. This is multiplied by the note amplitude to allow for global volume control.
 * @param loop_frequency: frequency at which this function is called, used to calculate the phase increment. Should be in Hz.
 * @retval FOC_LoopStatusTypeDef
 */
Sound_StatusTypeDef SoundEngine_PlaySequence(FOC_HandleTypeDef *hfoc, const NoteTypeDef *sequence_array, uint16_t sequence_length, float amplitude, float loop_frequency){
    static uint16_t step = 0;
    static uint32_t next_step_time = 0;
    static float magnitude = 0.0f;
    static float frequency = 0.0f;

    play_frequency(hfoc, frequency, magnitude, loop_frequency);

    if(HAL_GetTick() < next_step_time) return SOUND_IN_PROGRESS;

    if(step < sequence_length){
        magnitude = sequence_array[step].amplitude * amplitude * 1.0f; //the ampliture is normalized to 1.0, so we multiply my max voltage.
        frequency = sequence_array[step].frequency_hz;

        next_step_time = HAL_GetTick() + sequence_array[step].duration_ms;
        step++;
    } else{
        magnitude = 0.0f;
        frequency = 0.0f;
        
        step = 0;
        return SOUND_COMPLETED;
    }

    return SOUND_IN_PROGRESS;
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

    float cos_theta, sin_theta;
    Cordic_CalculateSinCos(phase, &cos_theta, &sin_theta);
    
    ABVoltagesTypeDef Vab = {
        .alpha = magnitude * cos_theta,
        .beta  = magnitude * sin_theta
    };

    FOC_SetPhaseVoltages(hfoc, InvClarke_transform(Vab));

    
}
