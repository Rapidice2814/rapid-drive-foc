#include "FOC_Loops.h"
#include <math.h>

static void play_sound(FOC_HandleTypeDef *hfoc, float frequency, float magnitude, float loop_frequency);

typedef struct {
    float frequency;     // Hz (0.0 = rest)
    uint16_t duration;   // ms
} NoteTypeDef;

static NoteTypeDef melody[] = {
    {1046.50f, 200},  // C4
    {1318.51f, 200},  // E4
    {1567.98f, 200},  // G4
    {0.0f,     300},  // Rest
    {2093.00f, 600},  // C5
};

#define NOTE_DURATION 150 // ms
#define NOTE_PAUSE 15 // ms

// static NoteTypeDef mario[] = {
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1567.98f, NOTE_DURATION}, // G6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     3*NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE}, // rest
    
//     {0.0f,     2*NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {698.46f,  NOTE_DURATION}, // F5
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {880.00f,  NOTE_DURATION}, // A5
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {987.77f,  NOTE_DURATION}, // B5
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {880.00f,  NOTE_DURATION}, // A5
//     {0.0f,     NOTE_PAUSE}, // rest
    
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1567.98f, NOTE_DURATION}, // G6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1760.00f, NOTE_DURATION}, // A6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1396.91f, NOTE_DURATION}, // F6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1567.98f, NOTE_DURATION}, // G6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {1174.66f, NOTE_DURATION}, // D6
//     {0.0f,     NOTE_PAUSE}, // rest
//     {987.77f,  NOTE_DURATION}, // B5
//     {0.0f,     NOTE_PAUSE}, // rest

//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {987.77f,  NOTE_DURATION}, // B5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {880.00f,  NOTE_DURATION}, // A5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1174.66f, NOTE_DURATION}, // D6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE},    // rest

//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1174.66f, NOTE_DURATION}, // D6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {987.77f,  NOTE_DURATION}, // B5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {880.00f,  NOTE_DURATION}, // A5
//     {0.0f,     NOTE_PAUSE},    // rest

//     {0.0f,     NOTE_DURATION}, // rest
//     {0.0f,     NOTE_PAUSE},    // rest
//     {880.00f,  NOTE_DURATION}, // A5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1318.51f, NOTE_DURATION}, // E6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1174.66f, NOTE_DURATION}, // D6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {987.77f,  NOTE_DURATION}, // B5
//     {0.0f,     NOTE_PAUSE},    // rest

//     {1046.50f, NOTE_DURATION}, // C6
//     {0.0f,     NOTE_PAUSE},    // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {784.00f,  NOTE_DURATION}, // G5
//     {0.0f,     NOTE_PAUSE},    // rest
//     {0.0f,     3 * NOTE_DURATION}, // extended rest
// };




FOC_LoopStatusTypeDef FOC_BootupSound(FOC_HandleTypeDef *hfoc, float loop_frequency){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;

    uint8_t note_count = sizeof(melody) / sizeof(melody[0]);

    if(step == 0){
        play_sound(hfoc, 0.0f, 0.0f, loop_frequency);
        step++;
        next_step_time = HAL_GetTick() + melody[0].duration;
    } else if(step >= note_count+1){
        play_sound(hfoc, 0.0f, 0.0f, loop_frequency);
        return FOC_LOOP_COMPLETED;
    } else{
        play_sound(hfoc, melody[step-1].frequency, 1.0f, loop_frequency);
        if(HAL_GetTick() >= next_step_time){
            step++;
            next_step_time = HAL_GetTick() + melody[step-1].duration; //wait before the next step
        }
    }

    return FOC_LOOP_IN_PROGRESS;
}




uint8_t FOC_BootupSound2(FOC_HandleTypeDef *hfoc, float loop_frequency){

    static float frequency = 0.0f;
    frequency += 0.1f;
    play_sound(hfoc, frequency, 0.6f, loop_frequency);
    return 0;
}

static void play_sound(FOC_HandleTypeDef *hfoc, float frequency, float magnitude, float loop_frequency){
    static float reference_electrical_angle = 0.0f;
    static float step_size = 0.0f;


    step_size = (frequency * 4) / loop_frequency;
    reference_electrical_angle += step_size;
    normalize_angle(&reference_electrical_angle);

    ABVoltagesTypeDef Vab;
    Vab.alpha = magnitude * cosf(reference_electrical_angle);
    Vab.beta = magnitude * sinf(reference_electrical_angle);
    PhaseVoltagesTypeDef phase_voltages = FOC_InvClarke_transform(Vab);

    // PhaseVoltagesTypeDef phase_voltages;
    // phase_voltages.a = magnitude * cosf(reference_electrical_angle);
    // phase_voltages.b = -magnitude * cosf(reference_electrical_angle);
    // phase_voltages.c = -magnitude * cosf(reference_electrical_angle);

    if(frequency > 100.0f){
        FOC_SetPhaseVoltages(hfoc, phase_voltages);
    } else{
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    }
}
