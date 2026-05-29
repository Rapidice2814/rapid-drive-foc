#include "Cordic.h"
#include "main.h"

#define CORDIC_2exp31divPI       683565275.57643158978229477811035f
#define CORDIC_2exp31            0x1p31f
#define CORDIC_2exp32            0x1p32f
#define CORDIC_2exp31m1          0x7FFFFFFFp0f

extern CORDIC_HandleTypeDef hcordic;


/**
 * @brief This function calculates the sine and cosine of the given angle using the CORDIC peripheral.
 * @param theta: The angle in radians for which to calculate the sine and cosine.
 * @param cos: Pointer to a float where the calculated cosine value will be stored.
 * @param sin: Pointer to a float where the calculated sine value will be stored.
 * @note The function caches the last calculated andgle, so if the same angle is requested again, it will return the cached values instead of recalculating them.
 */
void Cordic_CalculateSinCos(float theta, float *cos, float *sin){
    static float previous_theta = 0.0f;
    static float cos_buffer = 0.0f;
    static float sin_buffer = 0.0f;

    if(theta != previous_theta){
        int32_t cordic_input;
        int32_t cordic_output[2];
        float thetaScaled = theta * CORDIC_2exp31divPI;
        while (thetaScaled < -CORDIC_2exp31)
            thetaScaled += CORDIC_2exp32;
        while (thetaScaled > CORDIC_2exp31m1)
            thetaScaled -= CORDIC_2exp32;
        if (thetaScaled < -CORDIC_2exp31)
            cordic_input = 0x80000000;
        else
            cordic_input = (int32_t)(thetaScaled);

        if(HAL_CORDIC_Calculate(&hcordic, &cordic_input, cordic_output, 1, 1000) != HAL_OK){
            Error_Handler();
        }
        
        cos_buffer = (float)cordic_output[0] * (1.0f / CORDIC_2exp31);
        sin_buffer = (float)cordic_output[1] * (1.0f / CORDIC_2exp31);
        previous_theta = theta;
    }

    *cos = cos_buffer;
    *sin = sin_buffer;

}