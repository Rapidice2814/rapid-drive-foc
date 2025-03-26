#include "FOC_Flash.h"
#include <string.h>

static FLASH_StatusTypeDef FLASH_Unlock(){
    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }

    if((FLASH->CR & FLASH_CR_LOCK) != 0){
        return FLASH_ERROR;
    }
    return FLASH_OK;
}

static FLASH_StatusTypeDef FLASH_Lock(){
    FLASH->CR |= FLASH_CR_LOCK;

    if((FLASH->CR & FLASH_CR_LOCK) != 0){
        return FLASH_OK;
    }
    return FLASH_ERROR;
}

static FLASH_StatusTypeDef FLASH_ErasePage(uint32_t page){
    while (FLASH->SR & FLASH_SR_BSY); // wait for busy flag to be cleared
    if(FLASH->SR & FLASH_SR_PGSERR){
        FLASH->SR |= FLASH_SR_PGSERR; // clear error flag
    }

    FLASH->CR |= FLASH_CR_PER; // set page erase bit
    FLASH->CR |= FLASH_CR_PNB & page; // set page number to 60

    FLASH->CR |= FLASH_CR_STRT; // start the erase operation
    while (FLASH->SR & FLASH_SR_BSY); // wait for busy flag to be cleared

    if(FLASH->SR & FLASH_SR_PGSERR){
        return FLASH_ERROR;
    }
    return FLASH_OK;
}

#define DATA_FLASH_PAGE 60

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *data){

    if(sizeof(FLASH_DataTypeDef) > FLASH_PAGE_SIZE){
        return FLASH_ERROR;
    }

    FLASH_DataTypeDef current_data;
    FOC_FLASH_ReadData(&current_data);
    if(memcmp(&current_data, data, sizeof(FLASH_DataTypeDef)) == 0){ // if data is the same, no need to write
        return FLASH_OK;
    }

    FLASH_Unlock();
    FLASH_ErasePage(DATA_FLASH_PAGE);
    
    while (FLASH->SR & FLASH_SR_BSY); // wait for busy flag to be cleared

    uint32_t flash_address = 0x08000000 + FLASH_PAGE_SIZE * DATA_FLASH_PAGE; 
    memcpy((void*)flash_address, data, sizeof(FLASH_DataTypeDef));
    while (FLASH->SR & FLASH_SR_BSY);

    FLASH_Lock();
    return FLASH_OK;
}



FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *data){
    uint32_t flash_address = 0x08000000 + FLASH_PAGE_SIZE * DATA_FLASH_PAGE;  
    
    memcpy(data, (void*)flash_address, sizeof(FLASH_DataTypeDef));
    return FLASH_OK;
}