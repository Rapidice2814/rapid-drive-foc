#include <math.h>
#include "FOC_Utils.h"
#include "main.h"


__attribute__((weak)) float _sinf(float a){
    // 16bit integer array for sine lookup. interpolation is used for better precision
    // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
    // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
    static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
    int32_t t1, t2;
    unsigned int i = (unsigned int)(a * (64*4*256.0f/M_2PIF));
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;
    if (i < 64) {
      t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
    }
    else if(i < 128) {
      t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
    }
    else if(i < 192) {
      t1 = -(int32_t)sine_array[-128 + i]; t2 = -(int32_t)sine_array[-127 + i];
    }
    else {
      t1 = -(int32_t)sine_array[256 - i]; t2 = -(int32_t)sine_array[255 - i];
    }
    return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
  }
  
  // function approximating cosine calculation by using fixed size array
  // ~55us (float array)
  // ~56us (int array)
  // precision +-0.005
  // it has to receive an angle in between 0 and 2PI
  __attribute__((weak)) float _cosf(float a){
    float a_sin = a + M_PI_2F;
    a_sin = a_sin > M_2PIF ? a_sin - M_2PIF : a_sin;
    return _sinf(a_sin);
  }
  
  
  // fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
  // Via Odrive project
  // https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
  // This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
  // The origin for Odrive atan2 is public domain. Thanks to Odrive for making
  // it easy to borrow.
  __attribute__((weak)) float _atan2(float y, float x) {
      // a := min (|x|, |y|) / max (|x|, |y|)
      float abs_y = fabsf(y);
      float abs_x = fabsf(x);
      // inject FLT_MIN in denominator to avoid division by zero
      float a = fminf(abs_x, abs_y) / (fmaxf(abs_x, abs_y));
      // s := a * a
      float s = a * a;
      // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
      float r =
          ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
      // if |y| > |x| then r := 1.57079637 - r
      if (abs_y > abs_x) r = 1.57079637f - r;
      // if x < 0 then r := 3.14159274 - r
      if (x < 0.0f) r = 3.14159274f - r;
      // if y < 0 then r := -r
      if (y < 0.0f) r = -r;
  
      return r;
    }



    void normalize_angle(float *angle){
        while (*angle > M_2PIF) {
            *angle -= M_2PIF;
        }
        while (*angle <= 0) {
            *angle += M_2PIF;
        }
    }

    void normalize_angle2(float *angle){
      while (*angle > M_PI) {
          *angle -= M_2PIF;
      }
      while (*angle <= -M_PI) {
          *angle += M_2PIF;
      }
  }