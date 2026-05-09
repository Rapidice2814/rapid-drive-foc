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
uint8_t countbits(uint32_t n);

void GenerateNtcLut();
float GetNtcTemperature(float ntc_resistance);



/** 
 * Declares a ring queue for type `type` with base name `name` 
 * and capacity `size`. 
 * Generated symbols: 
 *   - `name##_t`   : the queue struct type 
 *   - `name##_push`: push a message onto the queue. Returns 1 on success, 0 if the queue is full or if the input is invalid.
 *   - `name##_pop` : pop a message from the queue. Returns 1 on success, 0 if the queue is empty or if the input is invalid.
 * 
 * Usage: 
 *   DECLARE_QUEUE(RxMsg_t, RxQueue, RX_QUEUE_SIZE) 
 */ 
#define DECLARE_QUEUE(type, name, size)                                 \
typedef struct {                                                        \
    volatile uint8_t head;                                              \
    volatile uint8_t tail;                                              \
    type items[size];                                                   \
} name##_t;                                                             \
                                                                        \
static uint8_t name##_push(name##_t *queue, const type *msg) {          \
    if (!queue || !msg) return 0;                                       \
    uint8_t next = (uint8_t)((queue->head + 1U) % (size));              \
    if (next == queue->tail) return 0;                                  \
    queue->items[queue->head] = *msg;                                   \
    queue->head = next;                                                 \
    return 1;                                                           \
}                                                                       \
                                                                        \
static uint8_t name##_pop(name##_t *queue, type *msg) {                 \
    if (!queue || !msg) return 0;                                       \
    if (queue->head == queue->tail) return 0;                           \
    *msg = queue->items[queue->tail];                                   \
    queue->tail = (uint8_t)((queue->tail + 1U) % (size));               \
    return 1;                                                           \
}

#endif // UTILS_H





