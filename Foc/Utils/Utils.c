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
 * @brief Reads a 16-bit value in little-endian format from a byte array
 * @param src The source byte array
 * @return The 16-bit value
 */
uint16_t read_u16_le(const uint8_t *src){
    return ((uint16_t)src[0]) | ((uint16_t)src[1] << 8);
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

void write_float_le(uint8_t *dst, float v){
    union {
        float f;
        uint32_t u;
    } conv;
    conv.f = v;
    write_u32_le(dst, conv.u);
}

/** 
 * @brief Reads a float value in little-endian format from a byte array
 * @param src The source byte array
 * @return The float value
 */
float read_float_le(const uint8_t *src){
    union {
        float f;
        uint32_t u;
    } conv;
    conv.u = read_u32_le(src);
    return conv.f;
}

/** 
 * @brief Counts the number of set bits in an array of bytes
 * @param data Pointer to the array of bytes
 * @param length Length of the array
 * @return The number of set bits
 */
uint8_t countbits_array(const uint8_t *data, uint8_t length){
    uint8_t count = 0;
    for (uint8_t i = 0; i < length; i++){
        uint8_t n = data[i];
        while (n){
            count += n & 1;
            n >>= 1;
        }
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
float constrainf(float value, float min, float max){
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

uint8_t fnv1a64(const void *data, size_t data_len, int8_t *out, size_t out_len){
    if (out_len > 8 || (data_len != 0 && data == NULL) ||
        (out_len != 0 && out == NULL)){
        return 1;
    }

    const uint8_t *bytes = (const uint8_t *)data;
    uint64_t hash = UINT64_C(14695981039346656037);

    for (size_t i = 0; i < data_len; ++i){
        hash ^= bytes[i];
        hash *= UINT64_C(1099511628211);
    }

    for (size_t i = 0; i < out_len; ++i){
        out[i] = (uint8_t)(hash >> (8U * (out_len - 1U - i)));
    }

    return 0;
}

