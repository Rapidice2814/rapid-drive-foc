#include "FOC_Flash.h"
#include <string.h>

#define STORAGE_FLASH_PAGE 56
#define NUMBER_OF_FLASH_PAGES 8
#define STORAGE_FLASH_BASE (0x08000000 + FLASH_PAGE_SIZE * STORAGE_FLASH_PAGE) // 0x0801C000 for page 56, 


// this variable is used to force the linker to include the .permanent section, which is where the flash data is stored. 
// The actual data is read and written directly from flash using the FOC_FLASH_ReadData and FOC_FLASH_WriteData functions, so this variable is not used directly in the code.
__attribute((section(".permanent"))) FLASH_DataTypeDef flash_data_storage; 


// static const FLASH_DataTypeDef flash_data_default_values = {0};
static const FLASH_DataTypeDef flash_data_default_values = {
    .contains_data = 0,

    .motor = {
        .torque_constant = MOTOR_TORQUE_CONSTANT,
        .torque_constant_valid = 1,
        .pole_pairs = MOTOR_POLE_PAIRS,
        .pole_pairs_valid = 1
    },
    
    .controller = {
        .PID_gains_speed = {
            .Kp = 0.1f,
            .Ki = 10.0f,
            .Kd = 0.0f
        },
        .PID_gains_position = {
            .Kp = 5.0f,
            .Ki = 10.0f,
            .Kd = 0.0f
        },
        .current_control_bandwidth = 3000.0f,
        .current_PID_FF_enabled = 0,
    },

    .limits = {
        .vbus_overvoltage_trip_level = 27.0f,
        .vbus_undervoltage_trip_level = 20.0f,
        .max_bus_current = 0.0f,
        .max_voltage = VOLTAGE_LIMIT,
        .max_dq_voltage = VOLTAGE_LIMIT * M_1_SQRT3F,
        .max_dq_current = VOLTAGE_LIMIT * M_1_SQRT3F
    },

    .node = {
        .node_id = 0, // Set a default node ID to unassigned
        .heartbeat_msg_rate_ms = 100
    },
    
    .struct_terminator = 0
};


    
static FLASH_EraseInitTypeDef EraseInitStruct;

/**
 * @brief  Writes the FOC configuration data to flash memory.
 * @param  pdata: Pointer to the data to be written to flash.
 * @retval FLASH_StatusTypeDef: Status of the flash write operation. Returns FLASH_OK if successful, FLASH_ERROR if there was an error, and FLASH_EMPTY if the input data is empty.
 */
FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata){
    if(pdata == NULL){
        return FLASH_ERROR;
    }

    if(sizeof(FLASH_DataTypeDef) > FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES){
        return FLASH_ERROR;
    }

    pdata->contains_data = 1;
    pdata->struct_terminator = FLASH_DATA_STRUCT_TERMINATOR;

    FLASH_DataTypeDef current_data;
    FOC_FLASH_ReadData(&current_data);
    if(memcmp(&current_data, pdata, sizeof(FLASH_DataTypeDef)) == 0){ // if data is the same, no need to write
        return FLASH_OK;
    }

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = STORAGE_FLASH_PAGE;
    EraseInitStruct.NbPages = NUMBER_OF_FLASH_PAGES;

    uint32_t PAGEError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
        return FLASH_ERROR;
    }

    for(uint32_t i = 0; i < sizeof(FLASH_DataTypeDef); i += sizeof(uint64_t)){
        uint64_t data_to_write = 0;
        memcpy(&data_to_write, ((uint8_t*)pdata) + i, sizeof(uint64_t));

        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, STORAGE_FLASH_BASE + i, data_to_write) != HAL_OK){
            return FLASH_ERROR;
        } 
    }
    HAL_FLASH_Lock();
    return FLASH_OK;
}

/**
 * @brief  Reads the FOC configuration data from flash memory.
 * @param  pdata: Pointer to the data structure to be filled with the read data.
 * @retval FLASH_StatusTypeDef: Status of the flash read operation. Returns FLASH_OK if successful, FLASH_ERROR if there was an error, and FLASH_EMPTY if no valid data is found.
 */
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata){
    if(pdata == NULL){
        return FLASH_ERROR;
    }

    if(sizeof(FLASH_DataTypeDef) > FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES){
        return FLASH_ERROR;
    }

    FLASH_DataTypeDef temp_flash_data;
    
    for(uint32_t i = 0; i < sizeof(FLASH_DataTypeDef); i += sizeof(uint64_t)){
        uint64_t data64 = *(uint64_t*)(STORAGE_FLASH_BASE + i);
        memcpy((uint8_t*)(&temp_flash_data) + i, &data64, sizeof(uint64_t));
    }

    if(temp_flash_data.contains_data == 1 && temp_flash_data.struct_terminator == FLASH_DATA_STRUCT_TERMINATOR){
        memcpy(pdata, &temp_flash_data, sizeof(FLASH_DataTypeDef));
    } else {
        return FLASH_EMPTY;
    }
    
    return FLASH_OK;
}

/**
 * @brief  Sets the FOC configuration data to default values.
 * @param  pdata: Pointer to the data structure to be filled with default values.
 * @retval FLASH_StatusTypeDef: Status of the operation. Returns FLASH_OK if successful and FLASH_ERROR if there was an error (e.g., null pointer).
 */
FLASH_StatusTypeDef FOC_FLASH_SetDefault(FLASH_DataTypeDef *pdata){
    if(pdata == NULL){
        return FLASH_ERROR;
    }

    *pdata = flash_data_default_values;
    
    return FLASH_OK;

}