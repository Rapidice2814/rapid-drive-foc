#include <string.h>
#include <assert.h>

#include "FOC_Flash.h"

#define STORAGE_FLASH_PAGE 56
#define NUMBER_OF_FLASH_PAGES 8
#define STORAGE_FLASH_BASE (0x08000000 + FLASH_PAGE_SIZE * STORAGE_FLASH_PAGE) // 0x0801C000 for page 56, 


// this variable is used to force the linker to include the .permanent section, which is where the flash data is stored. 
// The actual data is read and written directly from flash using the FOC_FLASH_ReadData and FOC_FLASH_WriteData functions, so this variable is not used directly in the code.
__attribute((section(".permanent"))) FLASH_DataTypeDef flash_data_storage; 

static_assert(sizeof(FLASH_DataTypeDef) % 8 == 0, "struct not aligned to 8 bytes");
static_assert(sizeof(FLASH_DataTypeDef) <= FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES, "struct size exceeds flash storage size");


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

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata){
    if(pdata == NULL){
        return FLASH_ERROR;
    }

    pdata->contains_data = 1;
    pdata->struct_terminator = FLASH_DATA_STRUCT_TERMINATOR;

    if(FOC_FLASH_CompareData(pdata) == FLASH_SAME){
        return FLASH_OK;
    }

    __disable_irq();
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
    __enable_irq();

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
    
    for(uint32_t i = 0; i < sizeof(FLASH_DataTypeDef) / sizeof(uint64_t); i++){
        uint64_t data64 = *(uint64_t*)(STORAGE_FLASH_BASE + i * sizeof(uint64_t));
        ((uint64_t*)pdata)[i] = data64;
    }

    if(pdata->contains_data == 1 && pdata->struct_terminator == FLASH_DATA_STRUCT_TERMINATOR){
        return FLASH_OK;
    } else {
        return FLASH_EMPTY;
    }
}

/**
 * @brief  Compares the FOC configuration data in flash memory with the provided data structure.
 * @param  pdata: Pointer to the data structure to be compared with the flash data.
 * @retval FLASH_StatusTypeDef: Status of the comparison. Returns FLASH_SAME if the data is the same, FLASH_DIFFERENT if the data is different, and FLASH_ERROR if there was an error (e.g., null pointer).
 */
FLASH_StatusTypeDef FOC_FLASH_CompareData(const FLASH_DataTypeDef *pdata){
    if(pdata == NULL){
        return FLASH_ERROR;
    }

    for(uint32_t i = 0; i < sizeof(FLASH_DataTypeDef) / sizeof(uint64_t); i++){
        uint64_t data64 = *(uint64_t*)(STORAGE_FLASH_BASE + i * sizeof(uint64_t));
        uint64_t ram_data = ((const uint64_t*)pdata)[i];

        if(data64 != ram_data){
            return FLASH_DIFFERENT;
        }
    }

    return FLASH_SAME;
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