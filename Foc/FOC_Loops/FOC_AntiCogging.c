#include <math.h>

#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "Utils.h"
#include "FOC_Flash.h"
#include "FOC_Config.h"
#include "Cordic.h"

/* Position controller used only for the measurement, so the configured position gains are never touched.
   A light rotor needs a lot of damping: tested on the 11 pole pair gimbal motor, stable for Kd 0.7-1.5 and Kp 10-15. */
#define ANTICOG_POSITION_KP 10.0f   // [A/rad]
#define ANTICOG_POSITION_KI 200.0f  // [A/(rad*s)]
#define ANTICOG_POSITION_KD 1.0f    // [A*s/rad]
#define ANTICOG_HOLD_TICKS 10       // [speed loop ticks], the rotor has to stand still this long before a point is recorded
#define ANTICOG_MAX_POSITION_ERROR 0.005f // [rad], the rotor has to be this close to the setpoint
#define ANTICOG_LOOP_DIVIDER 8      // [-], the measurement runs at CURRENT_LOOP_FREQUENCY / ANTICOG_LOOP_DIVIDER, independent of the speed loop rate
#define ANTICOG_MAX_HARMONIC 50     // [per revolution], the table is low-passed to this, higher harmonics are measurement noise
#define ANTICOG_RECONSTRUCT_CHUNK 25 // [points], table points rebuilt per call, keeps the work per control loop short

static uint8_t step = 0;
static uint16_t substep = 0;
static uint8_t hold_counter = 0;
static uint8_t direction = 0;
static uint8_t print_direction = 0;
static uint16_t print_index = 0;
static uint32_t next_step_time = 0;
static uint32_t start_time = 0;
static float base_angle = 0.0f;     // [rad], unwrapped angle of the table start in the current revolution
static uint16_t start_index = 0;    // table index the measurement starts at, the rotor's current position
static float current = 0.0f;        // [A], filtered q current
static float last_angle = 0.0f;     // [rad], counter angle at the previous speed loop tick, to detect standstill

/* Post processing: each point is the current that holds the rotor still, anywhere in the static friction band works, so the raw
   table is noisy. The average of both directions (friction cancels) is low-passed in the harmonic domain, and the measured
   friction is added back with the sign of the direction. Measured on the test motor: speed ripple at 1 rad/s 22% -> 12%. */
static float harmonic_cos[ANTICOG_MAX_HARMONIC + 1];
static float harmonic_sin[ANTICOG_MAX_HARMONIC + 1];
static float friction = 0.0f;       // [A], half the difference between the forward and backward table
static uint16_t process_index = 0;

static PIDValuesTypeDef position_gains = {ANTICOG_POSITION_KP, ANTICOG_POSITION_KI, ANTICOG_POSITION_KD};
static PIDControllerTypeDef position_pid;

/**
 * @brief Restarts the anticogging measurement. Needs to be called when the measurement state is entered.
 * @param hfoc Handle to the FOC structure
 */
void FOC_AntiCoggingReset(FOC_HandleTypeDef *hfoc){
    UNUSED(hfoc);
    step = 0;
    substep = 0;
    hold_counter = 0;
    direction = 0;
    print_direction = 0;
    print_index = 0;
    process_index = 0;
}

/* Fourier coefficient k of the direction averaged table, the phasor is rotated instead of calling cos/sin per point */
static void AntiCogging_Coefficient(FOC_HandleTypeDef *hfoc, uint16_t k){
    float step_cos, step_sin;
    Cordic_CalculateSinCos(M_2PIF * (float)k / NUMBER_OF_ANTICOG_MEASUREMENTS, &step_cos, &step_sin);
    float phasor_cos = 1.0f, phasor_sin = 0.0f;
    float sum_cos = 0.0f, sum_sin = 0.0f, sum_difference = 0.0f;

    for(uint16_t n = 0; n < NUMBER_OF_ANTICOG_MEASUREMENTS; n++){
        const float forward = hfoc->flash_data.controller.anticogging_array[0][n];
        const float backward = hfoc->flash_data.controller.anticogging_array[1][n];
        const float average = 0.5f * (forward + backward);
        sum_cos += average * phasor_cos;
        sum_sin += average * phasor_sin;
        sum_difference += forward - backward;

        const float next_cos = phasor_cos * step_cos - phasor_sin * step_sin;
        phasor_sin = phasor_sin * step_cos + phasor_cos * step_sin;
        phasor_cos = next_cos;
    }

    const float scale = (k == 0) ? (1.0f / NUMBER_OF_ANTICOG_MEASUREMENTS) : (2.0f / NUMBER_OF_ANTICOG_MEASUREMENTS);
    harmonic_cos[k] = sum_cos * scale;
    harmonic_sin[k] = sum_sin * scale;
    if(k == 0){
        friction = 0.5f * sum_difference / NUMBER_OF_ANTICOG_MEASUREMENTS;
    }
}

/* Rebuilds table point n from the coefficients */
static void AntiCogging_Reconstruct(FOC_HandleTypeDef *hfoc, uint16_t n){
    float angle_cos, angle_sin;
    Cordic_CalculateSinCos(M_2PIF * (float)n / NUMBER_OF_ANTICOG_MEASUREMENTS, &angle_cos, &angle_sin);
    float phasor_cos = angle_cos, phasor_sin = angle_sin;
    float value = harmonic_cos[0];

    for(uint16_t k = 1; k <= ANTICOG_MAX_HARMONIC; k++){
        value += harmonic_cos[k] * phasor_cos + harmonic_sin[k] * phasor_sin;
        const float next_cos = phasor_cos * angle_cos - phasor_sin * angle_sin;
        phasor_sin = phasor_sin * angle_cos + phasor_cos * angle_sin;
        phasor_cos = next_cos;
    }

    hfoc->flash_data.controller.anticogging_array[0][n] = value + friction;
    hfoc->flash_data.controller.anticogging_array[1][n] = value - friction;
}

/**
 * @brief Measures the q current needed to hold the rotor at NUMBER_OF_ANTICOG_MEASUREMENTS positions, in both directions
 *        (friction acts in the direction of motion). Starts at the rotor's current position.
 * @param hfoc Handle to the FOC structure
 * @retval FOC_LoopStatusTypeDef
 */
FOC_LoopStatusTypeDef FOC_AntiCoggingMeasurement(FOC_HandleTypeDef *hfoc){

    Current_Loop(hfoc);

    current = current * (1 - 0.01f) + 0.01f * hfoc->dq_current.q;

    static uint8_t current_loop_counter = 0;
    if(++current_loop_counter < ANTICOG_LOOP_DIVIDER){
        return FOC_LOOP_IN_PROGRESS;
    }
    current_loop_counter = 0;

    switch(step){
        case 0:
            start_time = HAL_GetTick();
            hfoc->flash_data.controller.anticogging_data_valid = 0; // the table is being overwritten

            PID_Init(&position_pid, (1.0f/(CURRENT_LOOP_FREQUENCY / ANTICOG_LOOP_DIVIDER)), 0.01f, &hfoc->flash_data.limits.max_dq_current, &position_gains, 0);

            // start at the table entry the rotor is at, so the position loop doesn't move it around (or back several revolutions)
            base_angle = (float)hfoc->encoder_full_rotations * M_2PIF;
            start_index = (uint16_t)(hfoc->encoder_angle_mechanical_wrapped / ANTICOG_ANGLE_STEP) % NUMBER_OF_ANTICOG_MEASUREMENTS;
            hfoc->angle_setpoint = base_angle + start_index * ANTICOG_ANGLE_STEP;

            step++;
            next_step_time = HAL_GetTick() + 100;
            break;

        case 1:
            if(HAL_GetTick() < next_step_time) break;
            if(substep < NUMBER_OF_ANTICOG_MEASUREMENTS){
                float error = hfoc->angle_setpoint - hfoc->encoder_angle_mechanical_wrapped;
                normalize_angle_pm_pi(&error);

                const uint8_t standstill = (hfoc->encoder_angle_mechanical_wrapped == last_angle); // no counter change
                if(standstill && fabsf(error) < ANTICOG_MAX_POSITION_ERROR){
                    hold_counter++;
                }else{
                    hold_counter = 0;
                }
                if(hold_counter > ANTICOG_HOLD_TICKS){
                    // the rotor holds the current setpoint, store it and move to the next one
                    const uint16_t offset = (direction == 0) ? substep : (NUMBER_OF_ANTICOG_MEASUREMENTS - 1 - substep);
                    const uint16_t index = (start_index + offset) % NUMBER_OF_ANTICOG_MEASUREMENTS;
                    hfoc->flash_data.controller.anticogging_array[direction][index] = current;

                    const uint16_t next_offset = (direction == 0) ? (offset + 1 < NUMBER_OF_ANTICOG_MEASUREMENTS ? offset + 1 : offset) : (offset > 0 ? offset - 1 : 0);
                    hfoc->angle_setpoint = base_angle + (start_index + next_offset) * ANTICOG_ANGLE_STEP;

                    if(substep % 100 == 0){
                        Debug_SendTextResponse("Anticogging direction %d: %d/%d, current %dmA\n", direction, substep, NUMBER_OF_ANTICOG_MEASUREMENTS, (int)(current * 1000));
                    }
                    substep++;
                    hold_counter = 0;
                }
                last_angle = hfoc->encoder_angle_mechanical_wrapped;
            } else{
                substep = 0;
                if(direction == 0){
                    direction = 1; // the backward pass starts where the forward pass ended
                }else{
                    step++;
                    process_index = 0;
                }
            }
            break;

        case 2: // low-pass the table, coefficients first (they read the raw table), then rebuild it in chunks
            if(process_index <= ANTICOG_MAX_HARMONIC){
                AntiCogging_Coefficient(hfoc, process_index);
                process_index++;
            } else {
                const uint16_t first = (process_index - ANTICOG_MAX_HARMONIC - 1) * ANTICOG_RECONSTRUCT_CHUNK;
                for(uint16_t n = first; n < first + ANTICOG_RECONSTRUCT_CHUNK && n < NUMBER_OF_ANTICOG_MEASUREMENTS; n++){
                    AntiCogging_Reconstruct(hfoc, n);
                }
                process_index++;
                if(first + ANTICOG_RECONSTRUCT_CHUNK >= NUMBER_OF_ANTICOG_MEASUREMENTS){
                    Debug_SendTextResponse("Anticogging table low-passed to %d/rev, friction %dmA\n", ANTICOG_MAX_HARMONIC, (int)(friction * 1000));
                    step++;
                    next_step_time = HAL_GetTick() + 100;
                }
            }
            break;

        case 3: // print the table
            if(HAL_GetTick() < next_step_time) break;
            if(print_index == 0){
                Debug_SendTextResponse("Current measurements for direction %d:\n", print_direction);
                print_index++;
                next_step_time = HAL_GetTick() + 10;
            }else if(print_index <= NUMBER_OF_ANTICOG_MEASUREMENTS){
                Debug_SendTextResponse("%d,", (int)(hfoc->flash_data.controller.anticogging_array[print_direction][print_index-1] * 1000));
                print_index++;
                next_step_time = HAL_GetTick() + 2;
            }else{
                Debug_SendTextResponse("\n");
                print_index = 0;
                if(print_direction == 0){
                    print_direction = 1;
                }else{
                    Debug_SendTextResponse("Measurement completed in %ds\n", (int)(HAL_GetTick() - start_time)/1000);
                    step++;
                }
            }
            break;

        default:
            hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0.0f, 0.0f};
            FOC_AntiCoggingReset(hfoc);
            return FOC_LOOP_COMPLETED;
    }

    hfoc->dq_current_setpoint.q = PID_Update(&position_pid, hfoc->angle_setpoint, hfoc->encoder_angle_mechanical_unwrapped);
    hfoc->dq_current_setpoint.d = 0.0f;

    return FOC_LOOP_IN_PROGRESS;
}
