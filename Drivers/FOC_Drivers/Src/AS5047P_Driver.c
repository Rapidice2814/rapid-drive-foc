#include"AS5047P_Driver.h"

/* Volatile Registers */
#define NOP 0x0000 // No operation
#define ERRFL 0x0001 // Error register
//#define PROG 0x0003 // Programming register
#define DIAAGC 0x3FFC // Diagnostic and AGC
#define MAG 0x3FFD // CORDIC magnitude
#define ANGLEUNC 0x3FFE // Measured angle without dynamic angle error compensation
#define ANGLECOM 0x3FFF // Measured angle with dynamic angle error compensation

/* Non-Volatile Registers */
#define ZPOSM 0x0016 // Zero position MSB
#define ZPOSL 0x0017 // Zero position LSB / MAG diagnostic
#define SETTINGS1 0x0018 // Custom setting register 1
#define SETTINGS2 0x0019 // Custom setting register 2


/* SETTINGS1 values*/
#define NOISESET    0x0 //Noise setting
#define DIR         0x0 // Rotation direction
#define UVW_ABI     0x0 // Defines the PWM Output
#define DAECDIS     0x0 // Disable Dynamic Angle Error Compensation
#define ABIBIN      0x1 // ABI decimal or binary selection, 1-> binary, 0->decimal
#define DATASELECT  0x1 // This bit defines which data can be read form address 16383dec (3FFFhex). 0->DAECANG, 1->CORDICANG(Raw)
#define PWMON       0x0 // Enables PWM

/* SETTINGS2 values*/
#define UVWPP       0x0 // UVW number of pole pairs
#define HYS         0x0 // ABI Hysteresis setting
#define ABIRES     0x0 // ABI resolution setting


#define SETTINGS1_SETTINGS \
    ((PWMON & 0x1) << 7 |            /* PWMON in bit 7 */ \
    (DATASELECT & 0x1) << 6 |        /* DATASELECT in bit 6 */ \
    (ABIBIN & 0x1) << 5 |            /* ABIBIN in bit 5 */ \
    (DAECDIS & 0x1) << 4 |           /* DAECDIS in bit 4 */ \
    (UVW_ABI & 0x1) << 3 |           /* UVW_ABI in bit 3 */ \
    (DIR & 0x1) << 2 |               /* DIR in bit 2 */ \
    (NOISESET & 0x1) << 1 |          /* NOISESET in bit 1 */ \
    0x1)                             /* bit 0 is always 1 */

#define SETTINGS2_SETTINGS \
    ((ABIRES & 0x7) << 5 |            /* ABIRES in bits 5-7 */ \
     (HYS & 0x3) << 3 |               /* HYS in bits 3-4 */ \
     (UVWPP & 0x7))                   /* UVWPP in bits 0-2 */


static uint8_t is_even_parity(uint16_t data);
static uint16_t AS5047P_TransmitCommand(AS5047P_HandleTypeDef *has5047p, uint16_t address14Bits, uint8_t rwBit);
static uint16_t AS5047P_TransmitData(AS5047P_HandleTypeDef *has5047p, uint16_t address14Bits);


AS5047P_StatusTypeDef AS5047P_SetPins(AS5047P_HandleTypeDef *has5047p, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin){
	has5047p->hspi = hspi;
    has5047p->slave_select_port = slave_select_port;
    has5047p->slave_select_pin = slave_select_pin;

    if(!(has5047p->hspi && has5047p->slave_select_port)) return AS5047P_ERROR; //if any of the pointers are NULL, return error

    has5047p->pins_set = 1;
    return AS5047P_OK;
}

AS5047P_StatusTypeDef AS5047P_Init(AS5047P_HandleTypeDef *has5047p){
    if(has5047p->pins_set == 0) return AS5047P_ERROR; //if the pins are not set, return error

    uint16_t error_reg;
    AS5047P_TransmitCommand(has5047p, ERRFL, 1); //read error register
    error_reg = AS5047P_TransmitCommand(has5047p, NOP, 1); //get data from the error register

    if(error_reg == 0xFFFF) return AS5047P_NORESPONSE; //if there is no response, return error

    uint16_t settings1_ret, settings2_ret;
    AS5047P_TransmitCommand(has5047p, SETTINGS1, 0); 
    AS5047P_TransmitData(has5047p, SETTINGS1_SETTINGS); 

    settings1_ret = AS5047P_TransmitCommand(has5047p, SETTINGS2, 0); 
    AS5047P_TransmitData(has5047p, SETTINGS2_SETTINGS); 
    settings2_ret = AS5047P_TransmitCommand(has5047p, NOP, 1); 

    if((settings1_ret & 0x3FFF) != SETTINGS1_SETTINGS) return AS5047P_ERROR; //if the settings are not set correctly, return error
    if((settings2_ret & 0x3FFF) != SETTINGS2_SETTINGS) return AS5047P_ERROR; //if the settings are not set correctly, return error


    has5047p->setup_complete = 1;
    return AS5047P_OK;
}

AS5047P_StatusTypeDef AS5047P_GetAngle(AS5047P_HandleTypeDef *has5047p, float *angle){
    if(has5047p->pins_set == 0) return AS5047P_ERROR; //if the pins are not set, return error

    AS5047P_TransmitCommand(has5047p, ANGLECOM, 1);
    uint16_t ret = AS5047P_TransmitCommand(has5047p, NOP, 1);

    *angle = (ret & 0x3FFF) * (2.0f * 3.14159265359f / 16384.0f);

    return AS5047P_OK;
}

AS5047P_StatusTypeDef AS5047P_GetAngle_Raw(AS5047P_HandleTypeDef *has5047p, float *angle){
    if(has5047p->pins_set == 0) return AS5047P_ERROR; //if the pins are not set, return error

    AS5047P_TransmitCommand(has5047p, ANGLEUNC, 1);
    uint16_t ret = AS5047P_TransmitCommand(has5047p, NOP, 1);

    *angle = (ret & 0x3FFF) * (2.0f * 3.14159265359f / 16384.0f);

    return AS5047P_OK;
}

AS5047P_StatusTypeDef AS5047P_GetCMAG(AS5047P_HandleTypeDef *has5047p, uint16_t *cmag){
    if(has5047p->pins_set == 0) return AS5047P_ERROR; //if the pins are not set, return error

    AS5047P_TransmitCommand(has5047p, MAG, 1);
    uint16_t ret = AS5047P_TransmitCommand(has5047p, NOP, 1);

    *cmag = (ret & 0x3FFF);

    return AS5047P_OK;
}

AS5047P_StatusTypeDef AS5047P_GetDIAAGC(AS5047P_HandleTypeDef *has5047p, uint16_t *value){
    if(has5047p->pins_set == 0) return AS5047P_ERROR; //if the pins are not set, return error

    AS5047P_TransmitCommand(has5047p, DIAAGC, 1);
    uint16_t ret = AS5047P_TransmitCommand(has5047p, NOP, 1);

    *value = (ret & 0x0FFF);

    return AS5047P_OK;
}


/* 14 bit address, 0:Write, 1:Read*/
static uint16_t AS5047P_TransmitCommand(AS5047P_HandleTypeDef *has5047p, uint16_t address14Bits, uint8_t rwBit){
	uint16_t command = 0;
	uint16_t received = 0;

    command = address14Bits & 0x3FFF; // Mask to 14 bits
    command |= (rwBit & 0x01) << 14; // Set the read/write bit

    if(!is_even_parity(command)){
        command |= 0x8000; // Set the parity bit
    }


	HAL_GPIO_WritePin(has5047p->slave_select_port, has5047p->slave_select_pin, 0);
	HAL_SPI_TransmitReceive(has5047p->hspi, (uint8_t*)&command, (uint8_t*)&received, 1, 100);
	HAL_GPIO_WritePin(has5047p->slave_select_port, has5047p->slave_select_pin, 1);

	return received;
}

static uint16_t AS5047P_TransmitData(AS5047P_HandleTypeDef *has5047p, uint16_t address14Bits){
	uint16_t command = 0;
	uint16_t received = 0;

    command = address14Bits & 0x3FFF; // Mask to 14 bits

    if(!is_even_parity(command)){
        command |= 0x8000; // Set the parity bit
    }

	HAL_GPIO_WritePin(has5047p->slave_select_port, has5047p->slave_select_pin, 0);
	HAL_SPI_TransmitReceive(has5047p->hspi, (uint8_t*)&command, (uint8_t*)&received, 1, 100);
	HAL_GPIO_WritePin(has5047p->slave_select_port, has5047p->slave_select_pin, 1);

	return received;
}



static uint8_t is_even_parity(uint16_t data)
{
    uint8_t shift = 1;
    while (shift < (sizeof(data) * 8))
    {
    data ^= (data >> shift);
    shift <<= 1;
    }
    return !(data & 0x1);
}