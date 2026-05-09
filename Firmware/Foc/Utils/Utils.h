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

/** 
 * Declares a ring queue with reservation support for type `type` with base name `name` 
 * and capacity `size`. 
 * Generated symbols: 
 *   - `name##_t`   : the queue struct type 
 *   - `name##_push`: push a message onto the queue. Returns 1 on success, 0 if the queue is full or if the input is invalid.
 *   - `name##_pop` : pop a message from the queue. Returns 1 on success, 0 if the queue is empty or if the input is invalid.
 *   - `name##_reserve_push`: reserve a slot for pushing a message onto the queue. Returns a pointer to the reserved slot on success, 0 if no slot is available or if the input is invalid.
 *   - `name##_commit_push`: commit the reserved push operation. Returns 1 on success, 0 if the operation failed.
 *   - `name##_reserve_pop`: reserve a slot for popping a message from the queue. Returns a pointer to the reserved slot on success, 0 if no message is available or if the input is invalid.
 *   - `name##_commit_pop`: commit the reserved pop operation. Returns 1 on success, 0 if the operation failed.
 * 
 * Usage: 
 *   DECLARE_QUEUE_RESERVE(RxMsg_t, RxQueue, RX_QUEUE_SIZE) 
 */ 
#define DECLARE_QUEUE_RESERVE(type, name, size)                               \
typedef struct {                                                              \
    volatile uint8_t head;                                                    \
    volatile uint8_t tail;                                                    \
    volatile uint8_t push_reserved;                                           \
    volatile uint8_t pop_reserved;                                            \
    type items[size];                                                         \
} name##_t;                                                                   \
                                                                              \
static uint8_t name##_push(name##_t *queue, const type *msg) {                \
    if (!queue || !msg) return 0;                                             \
    uint8_t next = (uint8_t)((queue->head + 1U) % (size));                    \
    if (next == queue->tail) return 0;                                        \
    if (queue->push_reserved) return 0;                                       \
    queue->items[queue->head] = *msg;                                         \
    queue->head = next;                                                       \
    return 1;                                                                 \
}                                                                             \
                                                                              \
static uint8_t name##_pop(name##_t *queue, type *msg) {                       \
    if (!queue || !msg) return 0;                                             \
    if (queue->head == queue->tail) return 0;                                 \
    if (queue->pop_reserved) return 0;                                        \
    *msg = queue->items[queue->tail];                                         \
    queue->tail = (uint8_t)((queue->tail + 1U) % (size));                     \
    return 1;                                                                 \
}                                                                             \
                                                                              \
static type *name##_reserve_push(name##_t *queue) {                           \
    if (!queue) return (type *)0;                                             \
    if (queue->push_reserved) return (type *)0;                               \
    uint8_t next = (uint8_t)((queue->head + 1U) % (size));                    \
    if (next == queue->tail) return (type *)0;                                \
    queue->push_reserved = 1U;                                                \
    return &queue->items[queue->head];                                        \
}                                                                             \
                                                                              \
static uint8_t name##_commit_push(name##_t *queue) {                          \
    if (!queue) return 0;                                                     \
    if (!queue->push_reserved) return 0;                                      \
    queue->push_reserved = 0U;                                                \
    queue->head = (uint8_t)((queue->head + 1U) % (size));                     \
    return 1;                                                                 \
}                                                                             \
                                                                              \
static void name##_cancel_push(name##_t *queue) {                             \
    if (!queue) return;                                                       \
    queue->push_reserved = 0U;                                                \
}                                                                             \
                                                                              \
static type *name##_reserve_pop(name##_t *queue) {                            \
    if (!queue) return (type *)0;                                             \
    if (queue->pop_reserved) return (type *)0;                                \
    if (queue->head == queue->tail) return (type *)0;                         \
    queue->pop_reserved = 1U;                                                 \
    return &queue->items[queue->tail];                                        \
}                                                                             \
                                                                              \
static uint8_t name##_commit_pop(name##_t *queue) {                           \
    if (!queue) return 0;                                                     \
    if (!queue->pop_reserved) return 0;                                       \
    queue->pop_reserved = 0U;                                                 \
    queue->tail = (uint8_t)((queue->tail + 1U) % (size));                     \
    return 1;                                                                 \
}                                                                             \
                                                                              \
static void name##_cancel_pop(name##_t *queue) {                              \
    if (!queue) return;                                                       \
    queue->pop_reserved = 0U;                                                 \
}





#endif // UTILS_H





