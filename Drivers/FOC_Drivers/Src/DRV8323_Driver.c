/*
 * DRV8323_Driver.c
 *
 *  Created on: Nov 1, 2024
 *      Author: Csongor
 */

#include "DRV8323_Driver.h"
#include "DRV8323_Driver_Settigns.h"

static uint16_t DRV8323_TransmitCommand(DRV8323_HandleTypeDef *hdrv8323, uint8_t rwBit, uint8_t address4Bits, uint16_t data11Bits);


DRV8323_StatusTypeDef DRV8323_SetPins(DRV8323_HandleTypeDef *hdrv8323, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin, GPIO_TypeDef *enable_port, uint16_t enable_pin, GPIO_TypeDef *nfault_port, uint16_t nfault_pin){
	hdrv8323->hspi = hspi;
	hdrv8323->slave_select_port = slave_select_port;
	hdrv8323->slave_select_pin = slave_select_pin;
	hdrv8323->enable_port = enable_port;
	hdrv8323->enable_pin = enable_pin;
	hdrv8323->nfault_port = nfault_port;
	hdrv8323->nfault_pin = nfault_pin;

	if(!(hdrv8323->hspi && hdrv8323->slave_select_port && hdrv8323->enable_port && hdrv8323->nfault_port)) return DRV8323_ERROR; //if any of the pointers are NULL, return error

	hdrv8323->pins_set = 1;
	return DRV8323_OK;
}

//Initializes the DRV8323 by configuring the registers
DRV8323_StatusTypeDef DRV8323_Init(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	DRV8323_Enable(hdrv8323); //enable the driver
	HAL_Delay(1); //wait for the driver to power up

	DRV8323_ClearFaults(hdrv8323); //clear faults before configuring the registers

	uint16_t fault_reg1, fault_reg2;
	if(DRV8323_ReadFaultStatusRegister1(hdrv8323, &fault_reg1) != DRV8323_OK) return DRV8323_ERROR;
	if(DRV8323_ReadFaultStatusRegister2(hdrv8323, &fault_reg2) != DRV8323_OK) return DRV8323_ERROR;

	if(fault_reg1 == 0xFFFF || fault_reg2 == 0xFFFF) return DRV8323_NORESPONSE; //if there is no response, return error
	if(fault_reg1 || fault_reg2) return DRV8323_ERROR; //if there are faults, return error

	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x02, REGISTER_2_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x03, REGISTER_3_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x04, REGISTER_4_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x05, REGISTER_5_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x06, REGISTER_6_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;

	hdrv8323->setup_complete = 1;
	return DRV8323_OK;
}

//returns 1 if there is a fault, 0 if there is no fault
uint8_t DRV8323_CheckFault(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return 1; //if the pins are not set, return fault

	if(HAL_GPIO_ReadPin(hdrv8323->nfault_port, hdrv8323->nfault_pin) == 0) return 1;

	return 0;
}

//Reads the content of the Fault Status Register 1
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister1(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	*data = DRV8323_TransmitCommand(hdrv8323, 1, 0x00, 0x00);

	return DRV8323_OK;
}

//Reads the content of the Fault Status Register 2
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister2(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	*data = DRV8323_TransmitCommand(hdrv8323, 1, 0x01, 0x00);

	return DRV8323_OK;
}

//Clears the faults by writing 1 to the corresponding bits in the register
DRV8323_StatusTypeDef DRV8323_ClearFaults(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error
	
	DRV8323_TransmitCommand(hdrv8323, 0, 0x02, REGISTER_2_SETTINGS | 0x01);
	return DRV8323_OK;
}

//Short inputs to current sense amplifier for offset calibration
DRV8323_StatusTypeDef DRV8323_CSACALStart(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	//Sets the CSA_CAL_X bits to 1
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x06, (REGISTER_6_SETTINGS) | 0x1C) == 0xFFFF) return DRV8323_NORESPONSE;
	return DRV8323_OK;
}

//Normal current sense operation
DRV8323_StatusTypeDef DRV8323_CSACALStop(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	//Sets the CSA_CAL_X bits to 0
	if(DRV8323_TransmitCommand(hdrv8323, 0, 0x06, REGISTER_6_SETTINGS) == 0xFFFF) return DRV8323_NORESPONSE;
	return DRV8323_OK;
}

DRV8323_StatusTypeDef DRV8323_Enable(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	HAL_GPIO_WritePin(hdrv8323->enable_port, hdrv8323->enable_pin, 1);
	hdrv8323->driver_enabled = 1;

	return DRV8323_OK;
}

DRV8323_StatusTypeDef DRV8323_Disable(DRV8323_HandleTypeDef *hdrv8323){
	if(hdrv8323->pins_set == 0) return DRV8323_ERROR; //if the pins are not set, return error

	HAL_GPIO_WritePin(hdrv8323->enable_port, hdrv8323->enable_pin, 1);
	hdrv8323->driver_enabled = 0;
	hdrv8323->setup_complete = 0; //if the driver is disabled, the settings are reset to default

	return DRV8323_OK;
}




/**
  * @brief  Transmit and Receive commands to the DRV8323
  * @param  hdrv8323 pointer to the DRV8323 handle
  * @param  rwBit, 0 is write, 1 is read
  * @param  address4Bits, address of the register
  * @param  data11Bits, 11-bit command
  * @retval Returns the content of the specified register. Should be the same as data11Bits.
  */
static uint16_t DRV8323_TransmitCommand(DRV8323_HandleTypeDef *hdrv8323, uint8_t rwBit, uint8_t address4Bits, uint16_t data11Bits){ //format: 1 R/W, 4 address, 11 data
	uint16_t command = 0;
	uint16_t received = 0;

	command = ((uint16_t)(rwBit & 0x01)<<15) | ((uint16_t)(address4Bits & 0x0F) << 11) | (data11Bits & 0x7FF);

	HAL_GPIO_WritePin(hdrv8323->slave_select_port, hdrv8323->slave_select_pin, 0);
	HAL_SPI_TransmitReceive(hdrv8323->hspi, (uint8_t*)&command, (uint8_t*)&received, 1, 100);
	HAL_GPIO_WritePin(hdrv8323->slave_select_port, hdrv8323->slave_select_pin, 1);

	return received;
}
