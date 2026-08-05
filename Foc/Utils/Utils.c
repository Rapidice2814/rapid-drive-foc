#include "Utils.h"
#include <stdio.h>
#include <math.h>

/** 
 * @brief Writes a 16-bit value in little-endian format to a byte array
 * @param dst The destination byte array
 * @param v The 16-bit value to write
 * @retval None
 */
void write_u16_le(uint8_t *dst, uint16_t v){
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/** 
 * @brief Writes a 32-bit value in little-endian format to a byte array
 * @param dst The destination byte array
 * @param v The 32-bit value to write
 * @retval None
 */
void write_u32_le(uint8_t *dst, uint32_t v){
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/**
 * @brief Reads a 32-bit value in little-endian format from a byte array
 * @param src The source byte array
 * @return The 32-bit value
 */
uint32_t read_u32_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

/** 
 * @brief Counts the number of set bits in a 32-bit integer
 * @param n The integer to count bits from
 * @return The number of set bits
 */
uint8_t countbits(uint32_t n) {
    uint8_t count = 0;
    while (n) {
        count += n & 1;
        n >>= 1;
    }
    return count;
}


/** 
 * @brief Normalizes an angle to the range [0, 2*PI]
 * @param angle The angle to normalize
 * @retval None
 */
void normalize_angle_0_2pi(float *angle){
  while (*angle > M_2PIF) *angle -= M_2PIF;
  while (*angle <= 0) *angle += M_2PIF;
}

/** 
 * @brief Normalizes an angle to the range [-PI, PI]
 * @param angle The angle to normalize
 * @retval None
 */
void normalize_angle_pm_pi(float *angle){
  while (*angle > M_PI) *angle -= M_2PIF;
  while (*angle <= -M_PI) *angle += M_2PIF;
}

/**
  * @brief Constrains a float to be within a specified range
  * @param value The value to constrain
  * @param min The minimum value
  * @param max The maximum value
  * @retval The constrained value
  */
float constrainf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


