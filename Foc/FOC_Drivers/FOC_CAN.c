#include "FOC_CAN.h"
#include "FOC_Handle.h"
#include "Utils.h"
#include "FOC_Diagnostics.h"
#include <math.h>
#include <string.h>

void FOC_QueueCANMessage(FOC_HandleTypeDef *hfoc, CAN_CommandTypeDef command, uint32_t delay_ticks);

/**
 * @brief Sets the stored node ID and configures broadcast and addressed receive filters.
 * @param hfoc FOC handle containing the FDCAN peripheral and node settings.
 * @param node_id Requested node ID; callers must supply a four-bit value (0 to 15).
 * @note Returns unchanged without a peripheral. Filter IDs are masked to four bits,
 *       but the supplied byte is stored without range validation.
 */
void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id){
    if(hfoc->phfdcan == NULL) return;
    
    
    if(hfoc->phfdcan->State == HAL_FDCAN_STATE_BUSY){
        HAL_FDCAN_Stop(hfoc->phfdcan); // Stop the FDCAN peripheral if it is busy
    }
    
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterID1 = 0x000; // Accept id 0, broadcast
    sFilterConfig.FilterID2 = ID_MASK;
    if(HAL_FDCAN_ConfigFilter(hfoc->phfdcan, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = node_id & ID_MASK; // Accept node id
    sFilterConfig.FilterID2 = ID_MASK;
    if(HAL_FDCAN_ConfigFilter(hfoc->phfdcan, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    
    if(HAL_FDCAN_Start(hfoc->phfdcan) != HAL_OK){
        Error_Handler();
    }
    
    if(HAL_FDCAN_ActivateNotification(hfoc->phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }

    hfoc->flash_data.node.node_id = node_id;
    
}

/**
 * @brief Returns the node ID currently stored in the FOC configuration.
 * @param hfoc FOC handle to inspect.
 * @return Configured node-ID byte; zero is broadcast/unassigned.
 */
uint8_t FOC_GetNodeId(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.node.node_id;
}

/**
 * @brief Sets the period of one cyclic CAN reply.
 * @param hfoc FOC handle containing the cyclic-message settings.
 * @param index Valid CAN_CyclicIndexTypeDef; the caller must check the range.
 * @param rate Period in FOC ticks, or zero to disable the reply.
 */
void FOC_SetCyclicRate(FOC_HandleTypeDef *hfoc, CAN_CyclicIndexTypeDef index, uint32_t rate){
    hfoc->flash_data.node.can_msg_period_ticks[index] = rate;
}

/**
 * @brief Returns the configured period of one cyclic CAN reply.
 * @param hfoc FOC handle to inspect.
 * @param index Valid CAN_CyclicIndexTypeDef; the caller must check the range.
 * @return Period in FOC ticks, or zero when disabled.
 */
uint32_t FOC_GetCyclicRate(FOC_HandleTypeDef *hfoc, CAN_CyclicIndexTypeDef index){
    return hfoc->flash_data.node.can_msg_period_ticks[index];
}

uint32_t missed_can_messages = 0;

/**
 * @brief Builds and submits a supported reply as an 11-bit-ID CAN FD frame.
 * @param hfoc FOC handle supplying the peripheral, source node, and reply values.
 * @param command Reply command selecting the payload layout.
 * @note Unsupported commands, including CAN_ACK and CAN_ERROR, send nothing. A full
 *       transmit FIFO drops the reply and increments missed_can_messages.
 * @see docs/can-protocol.md
 */
void FOC_TransmitCANMessage(FOC_HandleTypeDef *hfoc, CAN_CommandTypeDef command){
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxHeader.Identifier = GET_CAN_ID(hfoc->flash_data.node.node_id, command);

    uint8_t TxData[64];

    switch (command) { //FOC -> PC commands
        case CAN_VERSION_REPLY:
            TxData[0] = (uint8_t)(FOC_VERSION_MAJOR);
            TxData[1] = (uint8_t)(FOC_VERSION_MINOR);
            TxData[2] = (uint8_t)(FOC_VERSION_PATCH);
            TxHeader.DataLength = FDCAN_DLC_BYTES_3;
            break;
        case CAN_ADDRESS_REPLY:
            TxData[0] = (uint8_t)(hfoc->flash_data.node.node_id);
            memcpy(&TxData[1], &hfoc->uid, 6);
            TxHeader.DataLength = FDCAN_DLC_BYTES_7;
            break;
        case CAN_STATE_REPLY:
            TxData[0] = (uint8_t)FOC_GetState(hfoc);
            TxHeader.DataLength = FDCAN_DLC_BYTES_1;
            break;
        case CAN_CONTROL_MODE_REPLY:
            TxData[0] = (uint8_t)FOC_GetControlMode(hfoc);
            TxHeader.DataLength = FDCAN_DLC_BYTES_1;
            break;
        case CAN_HEARTBEAT_REPLY:
            TxData[0] = 0xFF & hfoc->state; // current state of the FOC driver
            TxData[1] = (uint8_t)fminf(fmaxf(hfoc->adc_values.motor_temp, 0.0f), 255.0f); // Send the temperature as the second byte, in C
            uint16_t timestamp = (uint16_t)(FOC_GetTick(hfoc) & 0xFFFF); // Get the current timestamp
            memcpy(&TxData[2], &timestamp, sizeof(uint16_t)); //byte 2-3
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            break;
        case CAN_ENCODER_ESTIMATES_REPLY:
            write_float_le(&TxData[0], hfoc->encoder_angle_mechanical_unwrapped); //byte 0-3
            write_float_le(&TxData[4], hfoc->encoder_speed_mechanical); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_BUS_VOLTAGE_CURRENT_REPLY:
            write_float_le(&TxData[0], hfoc->adc_values.vbus); //byte 0-3
            write_float_le(&TxData[4], hfoc->ibus); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_TEMPERATURES_REPLY:
            write_float_le(&TxData[0], hfoc->adc_values.mosfet_temp); //byte 0-3
            write_float_le(&TxData[4], hfoc->adc_values.motor_temp); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_TORQUE_REPLY:
            write_float_le(&TxData[0], hfoc->dq_current_setpoint.q / hfoc->flash_data.motor.torque_constant); //byte 0-3
            write_float_le(&TxData[4], hfoc->dq_current.q / hfoc->flash_data.motor.torque_constant); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_CURRENT_REPLY:
            write_float_le(&TxData[0], hfoc->dq_current_setpoint.q); //byte 0-3
            write_float_le(&TxData[4], hfoc->dq_current.q); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_SPEED_REPLY:
            write_float_le(&TxData[0], hfoc->speed_setpoint); //byte 0-3
            write_float_le(&TxData[4], hfoc->encoder_speed_mechanical); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_POSITION_REPLY:
            write_float_le(&TxData[0], hfoc->angle_setpoint); //byte 0-3
            write_float_le(&TxData[4], hfoc->encoder_angle_mechanical_unwrapped); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        case CAN_ERRORS_REPLY:
            write_u32_le(&TxData[0], FOC_GetActiveErrors(hfoc)); //byte 0-3
            write_u32_le(&TxData[4], FOC_GetLatchedErrors(hfoc)); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        default:
            return;
    }

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfoc->phfdcan) == 0){
        missed_can_messages++;
        return;
    }
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfoc->phfdcan, &TxHeader, TxData) != HAL_OK){
        Error_Handler();
    }
}

/**
 * @brief Checks whether a command is accepted at broadcast node zero.
 * @param command Received CAN command.
 * @return 1 for ESTOP, GET_ADDRESS, or SET_ADDRESS; otherwise 0.
 * @note This controls command acceptance, not whether a response is guaranteed.
 */
static uint8_t reply_in_broadcast_mode(CAN_CommandTypeDef command){
    switch (command){
        case CAN_ESTOP:
        case CAN_GET_ADDRESS:
        case CAN_SET_ADDRESS:
            return 1;
        default:
            return 0;
    }
}

/**
 * @brief Processes at most one pending standard-ID frame from receive FIFO zero.
 * @param hfoc FOC handle used to apply commands and generate replies.
 * @note Called by the regular CAN service loop. Payload checks and responses vary
 *       by command; unimplemented operations are listed in the protocol reference.
 * @see docs/can-protocol.md
 */
void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc){
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    if(HAL_FDCAN_GetRxFifoFillLevel(hfoc->phfdcan, FDCAN_RX_FIFO0) > 0){
        // HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin); //toggle debug led to indicate a message was received
        if(HAL_FDCAN_GetRxMessage(hfoc->phfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
            Error_Handler();
        }
        if(RxHeader.IdType != FDCAN_STANDARD_ID) return; // Only process standard ID messages

        uint8_t command = GET_COMMAND_FROM_CAN_ID(RxHeader.Identifier);
        uint8_t node_id = GET_ID_FROM_CAN_ID(RxHeader.Identifier);

        if(node_id == CAN_BROADCAST_NODE_ID && !reply_in_broadcast_mode((CAN_CommandTypeDef)command)) return;

        switch (command) { //PC -> FOC commands
            case CAN_ESTOP:
                FOC_SetError(hfoc, FOC_ERROR_ESTOP);
                break;
            case CAN_GET_VERSION:
                FOC_TransmitCANMessage(hfoc, CAN_VERSION_REPLY);
                break;
            case CAN_REBOOT:
                //unimplemented
                break;
            case CAN_ENTER_BOOTLOADER:
                //unimplemented
                break;
            case CAN_GET_ADDRESS:
                FOC_QueueCANMessage(hfoc, CAN_ADDRESS_REPLY, hfoc->random_number % 800 + 1); // Queue the address reply message to be sent after a delay of 1-800 ticks
                break;
            case CAN_SET_ADDRESS:
                if((memcmp(&RxData[1], &hfoc->uid, 6) == 0) || memcmp(&RxData[1], "\0\0\0\0\0\0", 6) == 0){ // If the UID matches or is 0, set the node ID to the received value
                    FOC_SetNodeId(hfoc, RxData[0]);
                } else if(RxData[0] == hfoc->flash_data.node.node_id){
                    FOC_SetNodeId(hfoc, 0);
                }
                break;
            case CAN_GET_STATE:
                FOC_TransmitCANMessage(hfoc, CAN_STATE_REPLY);
                break;
            case CAN_SET_CONTROL_MODE:
                FOC_SetControlMode(hfoc, (ControlModeTypeDef)RxData[0]);
                break;
            case CAN_GET_CONTROL_MODE:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_CONTROL_MODE_REPLY);
                break;
            case CAN_GET_HEARTBEAT:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_HEARTBEAT_REPLY);
                break;
            case CAN_GET_ENCODER_ESTIMATES:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_ENCODER_ESTIMATES_REPLY);
                break;
            case CAN_GET_BUS_VOLTGE_CURRENT:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_BUS_VOLTAGE_CURRENT_REPLY);
                break;
            case CAN_GET_TEMPERATURES:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_TEMPERATURES_REPLY);
                break;
            case CAN_GET_TORQUE:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_TORQUE_REPLY);
                break;
            case CAN_GET_CURRENT:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_CURRENT_REPLY);
                break;
            case CAN_GET_SPEED:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_SPEED_REPLY);
                break;
            case CAN_GET_POSITION:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_POSITION_REPLY);
                break;
            case CAN_GET_ERRORS:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_0) return;
                FOC_TransmitCANMessage(hfoc, CAN_ERRORS_REPLY);
                break;
            case CAN_CLEAR_LATCHED_ERRORS:
                FOC_ClearLatchedErrors(hfoc);
                break;

            case CAN_SET_STATE:
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_1) return;
                if(FOC_SetState(hfoc, (FOC_StateTypeDef)RxData[0], FOC_STATE_NONE) != FOC_STATETRANSITION_OK){
                    FOC_TransmitCANMessage(hfoc, CAN_ERROR);
                }
                FOC_TransmitCANMessage(hfoc, CAN_ACK);
                break;
            case CAN_SET_TORQUE: //0-3byte float, torque setpoint
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                float torque_setpoint = read_float_le(RxData);
                hfoc->dq_current_setpoint.q = torque_setpoint * hfoc->flash_data.motor.torque_constant;
                break;
            case CAN_SET_CURRENT: //0-3byte float, current setpoint
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->dq_current_setpoint.q = read_float_le(RxData);
                break;
            case CAN_SET_SPEED: //0-3byte float, speed setpoint
                if(RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->speed_setpoint = read_float_le(RxData);
                break;
            case CAN_SET_POSITION: //0-3byte float, position setpoint
            if(RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->angle_setpoint = read_float_le(RxData);
                break;
            default:
                return;
        }

    }
}

/**
 * @brief Submits cyclic replies whose configured FOC-tick periods have elapsed.
 * @param hfoc FOC handle containing periods, last-send ticks, and the peripheral.
 * @note Disabled periods and node zero produce no cyclic traffic. Attempt timestamps
 *       advance even if a full transmit FIFO causes the reply to be dropped.
 */
void FOC_TransmitCyclicCANMessage(FOC_HandleTypeDef *hfoc){
    if(hfoc == NULL || hfoc->phfdcan == NULL || hfoc->flash_data.node.node_id == CAN_BROADCAST_NODE_ID){ 
        return;
    }

    const uint32_t current_tick = FOC_GetTick(hfoc);

    for(uint32_t i = 0; i < (uint32_t)CAN_CYCLIC_COUNT; i++) {
        const uint32_t period = hfoc->flash_data.node.can_msg_period_ticks[i];

        if(period == 0 || (uint32_t)(current_tick - hfoc->can_msg_last_tick[i]) < period) {
            continue;
        }

        CAN_CommandTypeDef command = CAN_ERROR;
        uint8_t command_valid = 1;

        switch((CAN_CyclicIndexTypeDef)i) {
            case CAN_CYCLIC_HEARTBEAT:
                command = CAN_HEARTBEAT_REPLY;
                break;
            case CAN_CYCLIC_ENCODER_ESTIMATES:
                command = CAN_ENCODER_ESTIMATES_REPLY;
                break;
            case CAN_CYCLIC_BUS_VOLTAGE_CURRENT:
                command = CAN_BUS_VOLTAGE_CURRENT_REPLY;
                break;
            case CAN_CYCLIC_TEMPERATURES:
                command = CAN_TEMPERATURES_REPLY;
                break;
            case CAN_CYCLIC_TORQUE:
                command = CAN_TORQUE_REPLY;
                break;
            case CAN_CYCLIC_CURRENT:
                command = CAN_CURRENT_REPLY;
                break;
            case CAN_CYCLIC_SPEED:
                command = CAN_SPEED_REPLY;
                break;
            case CAN_CYCLIC_POSITION:
                command = CAN_POSITION_REPLY;
                break;
            case CAN_CYCLIC_ERRORS:
                command = CAN_ERRORS_REPLY;
                break;
            default:
                command_valid = 0;
                break;
        }

        if(command_valid == 0) {
            continue;
        }

        hfoc->can_msg_last_tick[i] = current_tick;
        FOC_TransmitCANMessage(hfoc, command);
    }
}

/**
 * @brief Schedules a delayed reply in the single pending-message slot.
 * @param hfoc FOC handle containing the delayed-message fields.
 * @param command Nonzero reply command to transmit later.
 * @param delay_ticks Delay measured in FOC ticks.
 * @note A later call replaces any previously scheduled reply.
 */
void FOC_QueueCANMessage(FOC_HandleTypeDef *hfoc, CAN_CommandTypeDef command, uint32_t delay_ticks){
    if(hfoc == NULL || hfoc->phfdcan == NULL){ 
        return;
    }

    const uint32_t current_tick = FOC_GetTick(hfoc);
    const uint32_t target_tick = current_tick + delay_ticks;

    hfoc->queued_can_message_command = command;
    hfoc->queued_can_message_target_tick = target_tick;
}

/**
 * @brief Submits the pending reply when its target FOC tick has been reached.
 * @param hfoc FOC handle containing the queued command and target tick.
 * @note Uses wrap-safe tick comparison and clears the slot after a transmit attempt.
 */
void FOC_TransmitQueuedCANMessage(FOC_HandleTypeDef *hfoc){
    if(hfoc == NULL || hfoc->phfdcan == NULL){ 
        return;
    }

    const uint32_t current_tick = FOC_GetTick(hfoc);
    if(hfoc->queued_can_message_command != 0 && (uint32_t)(current_tick - hfoc->queued_can_message_target_tick) < 0x80000000u){
        FOC_TransmitCANMessage(hfoc, (CAN_CommandTypeDef)hfoc->queued_can_message_command);
        hfoc->queued_can_message_command = 0;
        hfoc->queued_can_message_target_tick = 0;
    }
}

/**
 * @brief Services cyclic traffic, one received command, and any delayed reply.
 * @param hfoc FOC handle containing the configured CAN peripheral.
 * @note Called regularly by the FOC loop; receive processing does not run in the ISR.
 */
void FOC_CAN_Loop(FOC_HandleTypeDef *hfoc){
    FOC_TransmitCyclicCANMessage(hfoc);
    FOC_ProcessCANMessage(hfoc);
    FOC_TransmitQueuedCANMessage(hfoc);
}

/**
 * @brief Receives FIFO-zero interrupt notifications without draining frames.
 * @param hfdcan HAL peripheral handle; currently unused.
 * @param RxFifo0ITs Receive interrupt flags supplied by HAL.
 * @note Currently performs no action; FOC_ProcessCANMessage polls FIFO zero.
 */
void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    (void)hfdcan; //unused
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){

    }
}
