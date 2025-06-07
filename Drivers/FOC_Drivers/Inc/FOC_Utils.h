#ifndef FOC_UTILS_H
#define FOC_UTILS_H


#define M_2PIF 6.28318530717958647692f
#define M_PIF 3.14159265358979323846f
#define M_PI_2F 1.57079632679489661923f
#define M_PI_3F 1.04719755119659774615f
#define M_SQRT3_2F 0.86602540378f
#define M_SQRT3F 1.73205080757f
#define M_1_SQRT3F 0.57735026919f
#define M_2_SQRT3F 1.15470053838f



/* Some optimizations from SimpleFOC */

/**
 *  Function approximating the sine calculation by using fixed size array
 * - execution time ~40us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _sinf(float a);
/**
 * Function approximating cosine calculation by using fixed size array
 * - execution time ~50us (Arduino UNO)
 *
 * @param a angle in between 0 and 2PI
 */
float _cosf(float a);

/**
 * Function approximating atan2 
 * 
 */
float _atan2(float y, float x);

void normalize_angle(float *angle);
void normalize_angle2(float *angle);

void GenerateNtcLut();
float GetNtcTemperature(float ntc_resistance);

#endif // FOC_UTILS_H