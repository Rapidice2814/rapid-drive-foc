#include "WS2812b_Driver.h"

static TIM_HandleTypeDef *ws2812b_timer_ptr;
static uint32_t ws2812b_channel;

#define WS2812B_PWM_LENGTH ((WS2812B_NUMBER_OF_LEDS * 24) + 50)
static uint16_t ws2812b_pwm_data[WS2812B_PWM_LENGTH] = {0};
static volatile uint8_t ws2812b_pulse_busy = 0;


void WS2812b_Setup(TIM_HandleTypeDef *htim, uint32_t channel) {
    ws2812b_timer_ptr = htim;
    ws2812b_channel = channel;

    for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++) {
        WS2812b_SetColor(i, 0, 0, 0);
    }
    WS2812b_Send();
} 

void WS2812b_SetColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t data = (g<<16) | (r<<8) | b;

    if(led >= WS2812B_NUMBER_OF_LEDS) {
        return;
    }

    for (int i = 23; i >= 0; i--) {
        if(data & (1 << i)) {
            ws2812b_pwm_data[i + led * 24] = 144; //1
        } else {
            ws2812b_pwm_data[i + led * 24] = 68; //0
        }
    }

    for(int i = 24 * WS2812B_NUMBER_OF_LEDS; i < WS2812B_PWM_LENGTH; i++) {
        ws2812b_pwm_data[i] = 0;
    }
        
}

void WS2812b_Send(){
    if(ws2812b_pulse_busy){
        return;
    }
    ws2812b_pulse_busy = 1;
    HAL_TIM_PWM_Start_DMA(ws2812b_timer_ptr, ws2812b_channel, (uint32_t *)ws2812b_pwm_data, WS2812B_PWM_LENGTH);
}

void WS2812b_PulseFinishedCallback() {
    HAL_TIM_PWM_Stop_DMA(ws2812b_timer_ptr, ws2812b_channel);
    ws2812b_pulse_busy = 0;
}