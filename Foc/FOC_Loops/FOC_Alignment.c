#include <math.h>

#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "Cordic.h"

#define ALIGNMENT_HOLD_TIME 1.0f                    // [s], voltage vector held at angle 0 to pull the rotor in, ramped up in the first half so it doesn't swing
#define ALIGNMENT_ATTEMPTS 3                        // [-], the sweep is repeated if the rotor slipped a pole
#define ALIGNMENT_SWEEP_SPEED (2.0f * M_2PIF)       // [rad/s], electrical speed of the sweep
#define ALIGNMENT_MIN_TRAVEL_RATIO 0.8f             // [-], measured / expected mechanical travel, outside this the pole pairs or wiring are wrong
#define ALIGNMENT_MAX_TRAVEL_RATIO 1.25f
#define ALIGNMENT_MIN_CONSISTENCY 0.9f              // [-], length of the averaged offset vector, low if the encoder skipped counts

#define ALIGNMENT_HOLD_CYCLES ((uint32_t)(ALIGNMENT_HOLD_TIME * CURRENT_LOOP_FREQUENCY))

static void Alignment_ApplyVoltage(FOC_HandleTypeDef *hfoc, float angle, float magnitude){
    float cos_angle, sin_angle;
    Cordic_CalculateSinCos(angle, &cos_angle, &sin_angle);
    FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){magnitude * cos_angle, magnitude * sin_angle}));
}

/**
* @brief Aligns the encoder zero to the motor zero (electrical angle), and detects the motor direction.
* @note The voltage vector is swept slowly forward and backward over one mechanical revolution. During the sweep the difference between
*       the commanded angle and the encoder counter angle is averaged. Friction makes the rotor lag in both directions, so the lag
*       cancels, and cogging and the once per revolution encoder error are averaged over the sweep. The counter (the same reference the control loop uses) is used, not the SPI angle.
*       The travel of the rotor gives the direction and checks the pole pairs.
* @param hfoc Handle to the FOC structure
* @param magnitude Voltage magnitude of the vector [V]
* @retval FOC_LoopStatusTypeDef
*/
FOC_LoopStatusTypeDef FOC_Alignment(FOC_HandleTypeDef *hfoc, float magnitude){

    static uint8_t step = 0;
    static uint32_t counter = 0;
    static float command_angle = 0.0f;      // [rad], electrical, not wrapped
    static float start_angle = 0.0f;        // [rad], mechanical unwrapped angle at the start of a sweep
    static float travel[2] = {0.0f};        // [rad], mechanical travel of the forward and backward sweep
    static float sum_cos[2] = {0.0f};       // offset vector sums, index = motor direction
    static float sum_sin[2] = {0.0f};
    static uint32_t samples = 0;
    static uint8_t attempt = 0;

    const float pole_pairs = (float)hfoc->flash_data.motor.pole_pairs;
    const float sweep_angle = pole_pairs * M_2PIF; // [rad], electrical angle of one mechanical revolution
    const uint32_t sweep_cycles = (uint32_t)(sweep_angle / ALIGNMENT_SWEEP_SPEED * CURRENT_LOOP_FREQUENCY);

    switch(step){
        case 0:
            if(hfoc->flash_data.motor.pole_pairs == 0){
                Debug_SendTextResponse("Alignment needs the pole pairs\n");
                return FOC_LOOP_ERROR;
            }
            uint16_t diaagc = 0;
            if(hfoc->has5047p.setup_complete != 1 || AS5047P_GetDIAAGC(&hfoc->has5047p, &diaagc) != AS5047P_OK || (diaagc & 0x0E00) != 0){ // CORDIC overflow, magnet too strong or too weak
                Debug_SendTextResponse("Alignment failed: encoder error, DIAAGC 0x%04X\n", diaagc);
                return FOC_LOOP_ERROR;
            }
            Debug_SendTextResponse("Starting alignment sweep with %dmV\n", (int)(magnitude * 1000));
            command_angle = 0.0f;
            counter = 0;
            samples = 0;
            for(int i = 0; i < 2; i++){
                sum_cos[i] = 0.0f;
                sum_sin[i] = 0.0f;
                travel[i] = 0.0f;
            }
            step++;
            break;

        case 1: // pull the rotor to angle 0
            Alignment_ApplyVoltage(hfoc, command_angle, magnitude * fminf(1.0f, (float)counter / (ALIGNMENT_HOLD_CYCLES / 2)));
            if(++counter >= ALIGNMENT_HOLD_CYCLES){
                start_angle = hfoc->encoder_angle_mechanical_unwrapped;
                counter = 0;
                step++;
            }
            break;

        case 2: // forward sweep
        case 3: { // backward sweep
            const uint8_t forward = (step == 2);
            command_angle += (forward ? ALIGNMENT_SWEEP_SPEED : -ALIGNMENT_SWEEP_SPEED) / CURRENT_LOOP_FREQUENCY;
            Alignment_ApplyVoltage(hfoc, command_angle, magnitude);

            // electrical angle = pole_pairs * (counter angle - offset) for direction 0, and its negative for direction 1
            const float counter_angle_electrical = pole_pairs * ((float)(*(hfoc->pencoder_count)) / ENCODER_PULSES_PER_ROTATION) * M_2PIF;
            const float offset_candidate[2] = {counter_angle_electrical - command_angle, counter_angle_electrical + command_angle};
            for(int i = 0; i < 2; i++){
                float cos_offset, sin_offset;
                Cordic_CalculateSinCos(offset_candidate[i], &cos_offset, &sin_offset);
                sum_cos[i] += cos_offset;
                sum_sin[i] += sin_offset;
            }
            samples++;

            if(++counter >= sweep_cycles){
                travel[forward ? 0 : 1] = hfoc->encoder_angle_mechanical_unwrapped - start_angle;
                start_angle = hfoc->encoder_angle_mechanical_unwrapped;
                counter = 0;
                step++;
            }
            break;
        }

        case 4: {
            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
            step = 0;

            const float expected_travel = sweep_angle / pole_pairs;
            const float travel_ratio = fabsf(travel[0]) / expected_travel;
            const uint8_t direction = (travel[0] >= 0.0f) ? 0 : 1;
            const float consistency = sqrtf(sum_cos[direction] * sum_cos[direction] + sum_sin[direction] * sum_sin[direction]) / (float)samples;

            Debug_SendTextResponse("Alignment travel: forward %dmRad, backward %dmRad, expected %dmRad, consistency %d/1000\n",
                (int)(travel[0] * 1000), (int)(travel[1] * 1000), (int)(expected_travel * 1000), (int)(consistency * 1000));

            const uint8_t travel_ok = travel_ratio >= ALIGNMENT_MIN_TRAVEL_RATIO && travel_ratio <= ALIGNMENT_MAX_TRAVEL_RATIO
                && fabsf(travel[0] + travel[1]) < 0.5f * expected_travel / pole_pairs; // a slipped pole shows up as a forward/backward mismatch
            if(!travel_ok || consistency < ALIGNMENT_MIN_CONSISTENCY){
                if(++attempt < ALIGNMENT_ATTEMPTS){
                    Debug_SendTextResponse("Alignment attempt %d failed, retrying\n", attempt);
                    return FOC_LOOP_IN_PROGRESS;
                }
                attempt = 0;
                if(!travel_ok){
                    Debug_SendTextResponse("Alignment failed: the rotor did not follow the sweep, check the pole pairs and the motor\n");
                } else {
                    Debug_SendTextResponse("Alignment failed: inconsistent encoder angle, the encoder may skip counts\n");
                }
                return FOC_LOOP_ERROR;
            }
            attempt = 0;

            // the offset is only defined modulo 2pi/pole_pairs, use the smallest one so the mechanical zero is the same every time
            float offset_electrical = atan2f(sum_sin[direction], sum_cos[direction]);
            normalize_angle_0_2pi(&offset_electrical);
            if(offset_electrical >= M_2PIF) offset_electrical = 0.0f;

            hfoc->flash_data.encoder.mechanical_offset = offset_electrical / pole_pairs;
            hfoc->flash_data.encoder.offset_valid = 1;
            hfoc->flash_data.motor.direction = direction;

            Debug_SendTextResponse("Encoder aligned: mechanical offset %dmRad, direction %d\n", (int)(hfoc->flash_data.encoder.mechanical_offset * 1000), direction);
            return FOC_LOOP_COMPLETED;
        }

        default:
            step = 0;
            break;
    }

    return FOC_LOOP_IN_PROGRESS;
}
