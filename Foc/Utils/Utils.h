#ifndef UTILS_H
#define UTILS_H

#include "main.h"


#define M_2PIF 6.28318530717958647692f
#define M_PIF 3.14159265358979323846f
#define M_PI_2F 1.57079632679489661923f
#define M_PI_3F 1.04719755119659774615f
#define M_SQRT3_2F 0.86602540378f
#define M_SQRT3F 1.73205080757f
#define M_1_SQRT3F 0.57735026919f
#define M_2_SQRT3F 1.15470053838f


void normalize_angle_0_2pi(float *angle);
void normalize_angle_pm_pi(float *angle);
float constrainf(float value, float min, float max);

void write_u16_le(uint8_t *dst, uint16_t v);
void write_u32_le(uint8_t *dst, uint32_t v);
uint8_t countbits(uint32_t n);



/**************MACROS**************/

/** 
 * Declares a fixed-size pool of type `type` with base name `name` and capacity `size`. 
 * The pool supports allocation and freeing of individual items, as well as pushing and popping items to/from a queue. 
 * Generated symbols: 
 *   - `name##_t`   : the pool struct type 
 *   - `name##_alloc`: allocates an item from the pool. Returns a pointer to the allocated item on success, 0 if no items are available or if the input is invalid.
 *   - `name##_push`: pushes the item which was allocated onto the queue. Returns 1 on success, 0 if the queue is full or if the input is invalid.
 *   - `name##_pop` : pops an item from the queue. Returns a pointer to the popped item on success, 0 if the queue is empty or if the input is invalid. Must be freed back to the pool after use.
 *   - `name##_free` : frees an item back to the pool. Returns 1 on success, 0 if the item is invalid or does not belong to the pool.
 * 
 * Usage: 
 *   DECLARE_POOL(type, name, size)
 */
#define DECLARE_POOL(type, name, size)                                        \
typedef struct{                                                               \
    uint8_t used[size];                                                       \
    type pool[size];                                                          \
    type *queue[size];                                                        \
    volatile uint8_t queue_head;                                              \
    volatile uint8_t queue_tail;                                              \
    volatile uint8_t queue_count;                                             \
} name##_t;                                                                   \
                                                                              \
static name##_t h##name = {0};                                                \
                                                                              \
static type *name##_alloc(void){                                              \
    for(uint8_t i = 0; i < (size); i++){                                      \
        if(h##name.used[i] == 0U){                                            \
            h##name.used[i] = 1U;                                             \
            return &h##name.pool[i];                                          \
        }                                                                     \
    }                                                                         \
    return NULL;                                                              \
}                                                                             \
                                                                              \
static uint8_t name##_push(type *msg){                                        \
    if(msg == NULL) return 0;                                                 \
    if(h##name.queue_count >= (size)) return 0;                               \
    h##name.queue[h##name.queue_head] = msg;                                  \
    h##name.queue_head = (uint8_t)((h##name.queue_head + 1U) % (size));       \
    h##name.queue_count++;                                                    \
    return 1;                                                                 \
}                                                                             \
                                                                              \
static type *name##_pop(void){                                                \
    type *msg;                                                                \
    if(h##name.queue_count == 0U) return NULL;                                \
    msg = h##name.queue[h##name.queue_tail];                                  \
    h##name.queue_tail = (uint8_t)((h##name.queue_tail + 1U) % (size));       \
    h##name.queue_count--;                                                    \
    return msg;                                                               \
}                                                                             \
                                                                              \
static uint8_t name##_free(type *msg){                                        \
    if(msg == NULL) return 0;                                                 \
    for(uint8_t i = 0; i < (size); i++){                                      \
        if(msg == &h##name.pool[i]){                                          \
            h##name.used[i] = 0U;                                             \
            return 1;                                                         \
        }                                                                     \
    }                                                                         \
    return 0;                                                                 \
}





#endif // UTILS_H





