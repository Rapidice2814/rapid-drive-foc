#ifndef SOUNDENGINE_H
#define SOUNDENGINE_H

#include "FOC_Utils.h"

typedef enum{
	SOUND_IN_PROGRESS,
	SOUND_COMPLETED
}Sound_StatusTypeDef;

typedef struct {
    float frequency_hz;
    uint16_t duration_ms;
    float amplitude;      // 0.0 to 1.0
} NoteTypeDef;

Sound_StatusTypeDef SoundEngine_PlaySequence(FOC_HandleTypeDef *hfoc, const NoteTypeDef *sequence_array, uint16_t sequence_length, float amplitude, float loop_frequency);


#endif // SOUNDENGINE_H