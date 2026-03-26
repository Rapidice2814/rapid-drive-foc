/*
 * DRV8323_Driver.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Csongor
 */

#ifndef INC_DRV8323_DRIVER_H_
#define INC_DRV8323_DRIVER_H_

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

/*Control register 0x02 */
#define DIS_CPUV 	0x0 //0b = Charge pump UVLO fault is enabled 				| 1b = Charge pump UVLO fault isdisabled
#define DIS_GDF 	0x0 //0b = Gate drive fault is enabled 						| 1b = Gate drive fault is disabled
#define OTW_REP 	0x0 //0b = OTW is not reported on nFAULT or the FAULT bit 	| 1b = OTWis reported on nFAULT and the FAULT bit
#define PWM_MODE 	0x1 //0x0 = 6x PWM Mode 	| 0x1 = 3x PWM mode 			| 0x2 = 1x PWM mode 	| 0x3 = Independent PWM mode
#define PWM_COM 	0x0 //0b = 1x PWM mode uses synchronous rectification 		| 1b = 1x PWM mode uses asynchronous rectification (diode freewheeling)
#define PWM_DIR 	0x0 //In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define COAST 		0x0 //Write a 1 to this bit to put all MOSFETs in the Hi-Z state
#define BRAKE 		0x0 //Write a 1 to this bit to turn on all three low-side MOSFETs in 1xPWM mode. This bit is ORed with the INLC (BRAKE) input.
#define CLR_FLT 	0x0 //Write a 1 to this bit to clear latched fault bits. This bit automatically resetsafter being written.

/*Gate driver HS register  0x03 */
#define LOCK		0x3 //Write 110b to lock the settings, write 011b to unlock the settings.
#define IDRIVEP_HS	0xF //High-side gate pull-up current
#define IDRIVEN_HS	0xF //High-side gate pull-down current

/*Gate driver LS register 0x04 */
#define CBC 		0x1 //Cycle-by cycle operation. In retry OCP_MODE, for both VDS_OCP and SEN_OCP, the fault is automatically cleared when a PWM input is given
#define TDRIVE 		0x3 //0x0 = 500-ns peak gate-current drive time | 0x1 = 1000-ns peak gate-current drive time | 0x2 = 2000-ns peak gate-current drive time | 0x3 = 4000-ns peak gate-current drive time
#define IDRIVEP_LS 	0xF //Low-side gate pull-up current
#define IDRIVEN_LS 	0xF //Low-side gate pull-down current

/*Over Current Protection control register 0x05 */
#define TRETRY 		0x0 //0b = VDS_OCP and SEN_OCP retry time is 4 ms | 1b = VDS_OCP and SEN_OCP retry time is 50 µs
#define DEAD_TIME 	0x1 //0x0 = 50-ns dead time | 0x1 = 100-ns dead time | 0x2 = 200-ns dead time | 0x3 = 400-ns dead time
#define OCP_MODE 	0x1 //0x0 = Overcurrent causes a latched fault | 0x1 = Overcurrent causes an automatic retrying fault | 0x2 = Overcurrent is report only but no action is taken | 0x3 = Overcurrent is not reported and no action is taken
#define OCP_DEG 	0x1 //0x0 = Overcurrent deglitch time of 2 µs | 0x1 = Overcurrent deglitch time of 4 µs | 0x2 = Overcurrent deglitch time of 6 µs | 0x3 = Overcurrent deglitch time of 8 µs
#define VDS_LVL 	0x9 //VDS OCP threshold

/*Current Sense Amplifier control register 0x06 */

#define CSA_FET 	0x0 //0b = Current sense amplifier positive input is SPx | 1b = Current sense amplifier positive input is SHx (also automatically sets the LS_REF bit to 1)
#define VREF_DIV 	0x1 //0b = Current sense amplifier reference voltage is VREF (unidirectional mode) | 1b = Current sense amplifier reference voltage is VREF divided by 2
#define LS_REF 		0x0 //0b = VDS_OCP for the low-side MOSFET is measured across SHx to SPx | 1b = VDS_OCP for the low-side MOSFET is measured across SHx to SNx
#define CSA_GAIN 	0x1 //0b = Current sense amplifier gain is 5V/V | 1b = Current sense amplifier gain is 10V/V | 0x2 = Current sense amplifier gain is 20V/V | 0x3 = Current sense amplifier gain is 40V/V
#define DIS_SEN 	0x0 //0b = Sense overcurrent fault is enabled | 1b = Sense overcurrent fault is disabled
#define CSA_CAL_A 	0x0 //0b = Normal current sense amplifier A operation | 1b = Short inputs to current sense amplifier A for offset calibration
#define CSA_CAL_B 	0x0 //0b = Normal current sense amplifier B operation | 1b = Short inputs to current sense amplifier B for offset calibration
#define CSA_CAL_C 	0x0 //0b = Normal current sense amplifier C operation | 1b = Short inputs to current sense amplifier C for offset calibration
#define SEN_LVL 	0x3 //0x0 = Sense OCP 0.25 V | 0x1 = Sense OCP 0.5 V | 0x2 = Sense OCP 0.75 V | 0x3 = Sense OCP 1 V



/*To use in calculations*/
#if CSA_GAIN == 0x00
  #define CSA_GAIN_VALUE 5.0f //[V/V]
#elif CSA_GAIN == 0x01
  #define CSA_GAIN_VALUE 10.0f //[V/V]
#elif CSA_GAIN == 0x02
  #define CSA_GAIN_VALUE 20.0f //[V/V]
#elif CSA_GAIN == 0x03
  #define CSA_GAIN_VALUE 40.0f //[V/V]
#else
  #error "Invalid CSA_GAIN"
#endif

/* Combine the values into 11 bit numbers */
#define REGISTER_2_SETTINGS \
((DIS_CPUV & 0x1) << 9 |           /* DIS_CPUV in bit 9 */ \
(DIS_GDF & 0x1) << 8 |            /* DIS_GDF in bit 8 */ \
(OTW_REP & 0x1) << 7 |            /* OTW_REP in bit 7 */ \
(PWM_MODE & 0x3) << 5 |          /* PWM_MODE in bits 5-6 */ \
(PWM_COM & 0x1) << 4 |            /* PWM_COM in bit 4 */ \
(PWM_DIR & 0x1) << 3 |            /* PWM_DIR in bit 3 */ \
(COAST & 0x1) << 2 |              /* COAST in bit 2 */ \
(BRAKE & 0x1) << 1 |              /* BRAKE in bit 1 */ \
(CLR_FLT & 0x1))                  /* CLR_FLT in bit 0 */

#define REGISTER_3_SETTINGS \
((LOCK & 0x7) << 8 |        /* LOCK in bits 8-10 */ \
(IDRIVEP_HS & 0xF) << 4 | /* IDRIVEP_HS in bits 4-7 */ \
(IDRIVEN_HS & 0xF))       /* IDRIVEN_HS in bits 0-3 */

#define REGISTER_4_SETTINGS \
((CBC & 0x1) << 10 |            /* CBC in bit 10 */ \
(TDRIVE & 0x3) << 8 |         /* TDRIVE in bits 8-9 */ \
(IDRIVEP_LS & 0xF) << 4 |   /* IDRIVEP_LS in bits 4-7 */ \
(IDRIVEN_LS & 0xF))         /* IDRIVEN_LS in bits 0-3 */

#define REGISTER_5_SETTINGS \
((TRETRY & 0x1) << 10 |         /* TRETRY in bit 10 */ \
(DEAD_TIME & 0x3) << 8 |      /* DEAD_TIME in bits 8-9 */ \
(OCP_MODE & 0x3) << 6 |       /* OCP_MODE in bits 6-7 */ \
(OCP_DEG & 0x3) << 4 |        /* OCP_DEG in bits 4-5 */ \
(VDS_LVL & 0xF))            /* VDS_LVL in bits 0-3 */

#define REGISTER_6_SETTINGS \
((CSA_FET & 0x1) << 10 |           /* CSA_FET in bit 10 */ \
(VREF_DIV & 0x1) << 9 |           /* VREF_DIV in bit 9 */ \
(LS_REF & 0x1) << 8 |             /* LS_REF in bit 8 */ \
(CSA_GAIN & 0x3) << 6 |          /* CSA_GAIN in bits 6-7 */ \
(DIS_SEN & 0x1) << 5 |            /* DIS_SEN in bit 5 */ \
(CSA_CAL_A & 0x1) << 4 |          /* CSA_CAL_A in bit 4 */ \
(CSA_CAL_B & 0x1) << 3 |          /* CSA_CAL_B in bit 3 */ \
(CSA_CAL_C & 0x1) << 2 |          /* CSA_CAL_C in bit 2 */ \
(SEN_LVL & 0x3))                 /* SEN_LVL in bits 0-1 */

typedef enum{
	DRV8323_OK,
	DRV8323_NORESPONSE,
	DRV8323_ERROR
}DRV8323_StatusTypeDef;

typedef struct {
	/* SPI */
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *slave_select_port;
	uint16_t slave_select_pin;

	/* GPIO outputs */
	GPIO_TypeDef *enable_port;
	uint16_t enable_pin;

	/* GPIO input */
	GPIO_TypeDef *nfault_port;
	uint16_t nfault_pin;

	/* Flags */
	uint8_t pins_set;
	uint8_t driver_enabled;
	uint8_t setup_complete;

}DRV8323_HandleTypeDef;

DRV8323_StatusTypeDef DRV8323_SetPins(DRV8323_HandleTypeDef *hdrv8323, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin, GPIO_TypeDef *enable_port, uint16_t enable_pin, GPIO_TypeDef *nfault_port, uint16_t nfault_pin);
DRV8323_StatusTypeDef DRV8323_Init(DRV8323_HandleTypeDef *hdrv8323);

uint8_t DRV8323_CheckFault(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister1(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data);
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister2(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data);

DRV8323_StatusTypeDef DRV8323_ClearFaults(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_CSACALStart(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_CSACALStop(DRV8323_HandleTypeDef *hdrv8323);

DRV8323_StatusTypeDef DRV8323_Enable(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_Disable(DRV8323_HandleTypeDef *hdrv8323);
#endif /* INC_DRV8323_DRIVER_H_ */
