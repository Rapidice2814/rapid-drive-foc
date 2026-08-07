#include <string.h>
#include <assert.h>

#include "FOC_Flash.h"

#define STORAGE_FLASH_PAGE 56
#define NUMBER_OF_FLASH_PAGES 8
#define STORAGE_FLASH_BASE (0x08000000 + FLASH_PAGE_SIZE * STORAGE_FLASH_PAGE) // 0x0801C000 for page 56, 


// this variable is used to force the linker to include the .permanent section, which is where the flash data is stored. 
// The actual data is read and written directly from flash using the FOC_FLASH_ReadData and FOC_FLASH_WriteData functions, so this variable is not used directly in the code.
__attribute((section(".permanent"))) FLASH_DataTypeDef flash_data_storage; 

static_assert(sizeof(FLASH_DataTypeDef) % 8 == 0, "struct not aligned to 8 bytes"); // Ensure that the struct size is a multiple of 8 bytes, as required by the flash programming algorithm.
static_assert(sizeof(FLASH_DataTypeDef) <= FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES, "struct size exceeds flash storage size");


// static const FLASH_DataTypeDef flash_data_default_values = {0};
static const FLASH_DataTypeDef flash_data_default_values = {
    .motor = {
        .torque_constant = MOTOR_TORQUE_CONSTANT,
        .pole_pairs = MOTOR_POLE_PAIRS,
    },
    
    .controller = {
        .current_control_bandwidth = 3000.0f,
    },

    .limits = {
        .vbus_overvoltage_trip_level = 27.0f,
        .vbus_undervoltage_trip_level = 20.0f,
        .ibus_overcurrent_trip_level = 0.0f,
        .mosfet_temp_trip_level = MOSFET_MAX_TEMP,
        .motor_temp_trip_level = MOTOR_MAX_TEMP,
        .max_dq_voltage = VOLTAGE_LIMIT * M_1_SQRT3F,
        .max_dq_current = MAX_DQ_CURRENT
    },

    .node = {
        .node_id = 0, // Set a default node ID to unassigned
        .heartbeat_msg_rate_ms = 100
    },
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

    if(READ_BIT(FLASH->SR, FLASH_SR_BSY) !=0U){ // Wait for any ongoing flash operation to complete
        return FLASH_ERROR;
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


/**
 * @brief  Sets the option bytes to default values. This only needs to be run once for every new device.
 * @retval FLASH_StatusTypeDef: Status of the operation. Returns FLASH_OK if successful and FLASH_ERROR if there was an error.
 * @note Explained in RM0440, section 3.4.2 "Option bytes programming sequence".
 */
FLASH_StatusTypeDef FOC_FLASH_SetSystemMemoryBoot(void)
{
    FLASH_OBProgramInitTypeDef ob = {0};

    const uint32_t boot_mask = FLASH_OPTR_nBOOT0_Msk   | FLASH_OPTR_nBOOT1_Msk   | FLASH_OPTR_nSWBOOT0_Msk;

    /*
     * nBOOT0   = 0
     * nBOOT1   = 1
     * nSWBOOT0 = 0
     */
    const uint32_t desired_boot_config = FLASH_OPTR_nBOOT1_Msk;

    HAL_FLASHEx_OBGetConfig(&ob);

    if ((ob.USERConfig & boot_mask) == desired_boot_config) { // Check if the current boot configuration is already set to the desired configuration
        return FLASH_OK;
    }

    if (READ_BIT(FLASH->SR, FLASH_SR_BSY) != 0U) {
        return FLASH_ERROR;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return FLASH_ERROR;
    }

    if (HAL_FLASH_OB_Unlock() != HAL_OK) {
        HAL_FLASH_Lock();
        return FLASH_ERROR;
    }

    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_nBOOT0 |
                    OB_USER_nBOOT1 |
                    OB_USER_nSWBOOT0;

    ob.USERConfig &= ~boot_mask;
    ob.USERConfig |= desired_boot_config;

    if (HAL_FLASHEx_OBProgram(&ob) != HAL_OK) {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return FLASH_ERROR;
    }

    if (HAL_FLASH_OB_Launch() != HAL_OK) {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return FLASH_ERROR;
    }

    //Usually unreachable because HAL_FLASH_OB_Launch() resets the MCU.
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    return FLASH_OK;
}

