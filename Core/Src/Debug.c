#include <stdio.h>
#include "Debug.h"
#include "FOC.h"
#include "FOC_Utils.h"


extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2; //Counter used to time functions

/* uart buffers and message counters*/
#define USART2_TX_BUFFER_SIZE 200
#define USART2_RX_BUFFER_SIZE 200

static char usart2_tx_buffer[USART2_TX_BUFFER_SIZE] = {0};
static int tx_packet_length = 0;
static uint32_t usart2_tx_message_counter = 0;
static volatile uint8_t uart2_tx_free_flag = 1;

static char usart2_rx_buffer[USART2_RX_BUFFER_SIZE] = {0};
static uint16_t rx_packet_length = 0;
static volatile uint8_t uart2_rx_flag = 0;




static uint32_t debug_step_counter = 0;
extern FOC_HandleTypeDef hfoc;

void Debug_Setup(){
    HAL_UART_Transmit(&huart3, (uint8_t*)"FOC Setup Complete\n", 20, 1000);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)usart2_rx_buffer, sizeof(usart2_rx_buffer));
}

static float Vq = 0.0f;
static float Vd = 0.0f;
static float Id = 0.0f;
static float Iq = 0.0f;
static float Id_setpoint = 0.0f;
static float Iq_setpoint = 0.0f;
static float EAngle = 0.0f;
static float ESpeed = 0.0f;
static float MAngle = 0.0f;
static float Vbus = 0.0f;



void Debug_Queue(FOC_HandleTypeDef *hfoc){
    debug_step_counter = 0;
    Vq = hfoc->dq_voltage.q;
    Vd = hfoc->dq_voltage.d;
    Id = hfoc->dq_current.d;
    Iq = hfoc->dq_current.q;
    Id_setpoint = hfoc->dq_current_setpoint.d;
    Iq_setpoint = hfoc->dq_current_setpoint.q;
    EAngle = hfoc->encoder_angle_electrical;
    ESpeed = hfoc->encoder_speed_electrical;
    MAngle = hfoc->encoder_angle_mechanical;
    Vbus = hfoc->vbus;
    normalize_angle2(&EAngle);
    normalize_angle2(&MAngle);
}

extern uint8_t timeout_flag;

uint32_t debug_start_time = 0;
uint32_t debug_step_time[10] = {0};
uint32_t debug_max_time = 0;

extern FOC_State Current_FOC_State;


void Debug_Loop(){
    debug_start_time = __HAL_TIM_GET_COUNTER(&htim2);
    
    switch (debug_step_counter) {
        case 0:
            tx_packet_length = snprintf(usart2_tx_buffer, sizeof(usart2_tx_buffer), "Vq:%d,", (int)(Vq * 1000));
            debug_step_time[0] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 1:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Vd:%d,", (int)(Vd * 1000));
            debug_step_time[1] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 2:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Id:%d,", (int)(Id * 1000));
            debug_step_time[2] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 3:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Iq:%d,", (int)(Iq * 1000));
            debug_step_time[3] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 4:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Id_set:%d,", (int)(Id_setpoint * 1000));
            debug_step_time[4] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 5:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Iq_set:%d,", (int)(Iq_setpoint * 1000));
            debug_step_time[5] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 6:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "EAngle:%d,", (int)(EAngle * 1000));
            debug_step_time[6] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 7:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Espeed:%d,", (int)(ESpeed * 1));
            debug_step_time[7] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 8:
            tx_packet_length += snprintf(usart2_tx_buffer + tx_packet_length, sizeof(usart2_tx_buffer) - tx_packet_length, "Vbus:%d\n", (int)(Vbus * 10));
            debug_step_time[7] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 9:
            if(uart2_tx_free_flag){
                uart2_tx_free_flag = 0;
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)usart2_tx_buffer, tx_packet_length);
                usart2_tx_message_counter++;
                tx_packet_length = 0;
            }
            debug_step_time[8] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 10:
            if(uart2_rx_flag){

                for(int i = 0; i < rx_packet_length; i++){
                    if(usart2_rx_buffer[i] == 'D'){
                        Current_FOC_State = FOC_STATE_ALIGNMENT_TEST;
                    }
                    if(usart2_rx_buffer[i] == 'A'){
                        Current_FOC_State = FOC_STATE_ALIGNMENT;
                    }
                    if(usart2_rx_buffer[i] == 'N'){
                        Current_FOC_State = FOC_STATE_RUN;
                    }
                    if(usart2_rx_buffer[i] == 'M'){
                        Current_FOC_State = FOC_STATE_IDENTIFY;
                    }
                    if(usart2_rx_buffer[i] == 'F'){
                        Current_FOC_State = FOC_STATE_FLASH_SAVE;
                    }
                    if(usart2_rx_buffer[i] == 'E'){
                        Current_FOC_State = FOC_STATE_ENCODER_TEST;
                    }
                    if(usart2_rx_buffer[i] == 'K'){
                        hfoc.motor_disable_flag = 1;
                    }
                    if(usart2_rx_buffer[i] == 'T'){
                        timeout_flag = !timeout_flag;
                    }
                }

                if(usart2_rx_buffer[0] == 'P' && usart2_rx_buffer[1] == 'd'){
                    int Pd = 0;
                    sscanf(usart2_rx_buffer, "Pd%d", &Pd);
                    hfoc.flash_data.PID_gains_d.Kp = (float)Pd / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'P' && usart2_rx_buffer[1] == 'q'){
                    int Pq = 0;
                    sscanf(usart2_rx_buffer, "Pq%d", &Pq);
                    hfoc.flash_data.PID_gains_q.Kp = (float)Pq / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'P' && usart2_rx_buffer[1] == 's'){
                    int Ps = 0;
                    sscanf(usart2_rx_buffer, "Ps%d", &Ps);
                    // hfoc.pid_speed.K->Ki = (float)Ps / 1000000.0f;
                }
                
                if(usart2_rx_buffer[0] == 'I' && usart2_rx_buffer[1] == 'd'){
                    int Id = 0;
                    sscanf(usart2_rx_buffer, "Id%d", &Id);
                    hfoc.flash_data.PID_gains_d.Ki = (float)Id / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'I' && usart2_rx_buffer[1] == 'q'){
                    int Iq = 0;
                    sscanf(usart2_rx_buffer, "Iq%d", &Iq);
                    hfoc.flash_data.PID_gains_q.Ki = (float)Iq / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'I' && usart2_rx_buffer[1] == 's'){
                    int Is = 0;
                    sscanf(usart2_rx_buffer, "Is%d", &Is);
                    // hfoc.pid_speed.Ki = (float)Is / 1000000.0f;
                }

                if(usart2_rx_buffer[0] == 'S' && usart2_rx_buffer[1] == 'q'){
                    int Sq = 0;
                    sscanf(usart2_rx_buffer, "Sq%d", &Sq);
                    hfoc.dq_current_setpoint.q = (float)Sq / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'S' && usart2_rx_buffer[1] == 'd'){
                    int Sd = 0;
                    sscanf(usart2_rx_buffer, "Sd%d", &Sd);
                    hfoc.dq_current_setpoint.d = (float)Sd / 1000.0f;
                }
                if(usart2_rx_buffer[0] == 'S' && usart2_rx_buffer[1] == 's'){
                    int Ss = 0;
                    sscanf(usart2_rx_buffer, "Ss%d", &Ss);
                    hfoc.speed_setpoint = (float)Ss;
                }
        
                
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)usart2_rx_buffer, sizeof(usart2_rx_buffer));
                uart2_rx_flag = 0;
            }
            debug_step_time[9] = __HAL_TIM_GET_COUNTER(&htim2) - debug_start_time;
            break;
        case 11:
            {
                for (int i = 0; i < 10; i++) {
                    if (debug_step_time[i] > debug_max_time) {
                        debug_max_time = debug_step_time[i];
                    }
                }
            }
            break;
        default:
            break;
    }

    debug_step_counter++;

     

    
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == huart3.Instance){
        uart2_tx_free_flag = 1;
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance == huart3.Instance){
        rx_packet_length = Size;
        uart2_rx_flag = 1;
    }  
}