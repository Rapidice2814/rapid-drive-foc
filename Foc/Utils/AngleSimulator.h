#ifndef ANGLE_SIMULATOR_H
#define ANGLE_SIMULATOR_H
#include <math.h>
#include <stdint.h>

/* Independent of FOC, encoders, timers and hardware. Angles are mechanical radians.
   Caller owns the clock: advance at any rate, or set a trajectory explicitly. */
typedef struct { float angle, speed; } AngleSimulator;
#define ANGLE_SIMULATOR_TAU 6.2831853071795864769f
static inline uint8_t AngleSimulator_SetAngle(AngleSimulator *s, float angle){
    if(!s || !isfinite(angle)) return 0;
    angle = fmodf(angle, ANGLE_SIMULATOR_TAU);
    s->angle = angle < 0 ? angle + ANGLE_SIMULATOR_TAU : angle;
    return 1;
}
static inline uint8_t AngleSimulator_SetSpeed(AngleSimulator *s, float speed){
    if(!s || !isfinite(speed)) return 0;
    s->speed = speed;
    return 1;
}
static inline uint8_t AngleSimulator_Advance(AngleSimulator *s, float dt){
    if(!s || !isfinite(dt) || dt < 0 || !isfinite(s->speed)) return 0;
    return AngleSimulator_SetAngle(s, s->angle + s->speed * dt);
}
static inline float AngleSimulator_Electrical(const AngleSimulator *s, unsigned pole_pairs){
    return fmodf(s->angle * pole_pairs, ANGLE_SIMULATOR_TAU);
}
#endif
