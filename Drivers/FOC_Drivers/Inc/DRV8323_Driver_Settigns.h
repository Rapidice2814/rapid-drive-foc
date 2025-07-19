/*
 * DRV8323_Driver_Settigns.h
 *
 *  Created on: Nov 2, 2024
 *      Author: Csongor
 */

#ifndef INC_DRV8323_DRIVER_SETTIGNS_H_
#define INC_DRV8323_DRIVER_SETTIGNS_H_

/*Control register 0x02 */
#define DIS_CPUV 	0x0
#define DIS_GDF 	0x0
#define OTW_REP 	0x0
#define PWM_MODE 	0x1
#define PWM_COM 	0x0
#define PWM_DIR 	0x0
#define COAST 		0x0
#define BRAKE 		0x0
#define CLR_FLT 	0x0

/*Gate driver HS register  0x03 */
#define LOCK		0x3
#define IDRIVEP_HS	0xF
#define IDRIVEN_HS	0xF

/*Gate driver LS register 0x04 */
#define CBC 		0x1
#define TDRIVE 		0x3
#define IDRIVEP_LS 	0xF
#define IDRIVEN_LS 	0xF

/*OCP control register 0x05 */
#define TRETRY 		0x0
#define DEAD_TIME 	0x1
#define OCP_MODE 	0x1
#define OCP_DEG 	0x1
#define VDS_LVL 	0x9

/*CSA control register 0x06 */

#define CSA_FET 	0x0
#define VREF_DIV 	0x1
#define LS_REF 		0x0
#define CSA_GAIN 	0x1 //10V/V
#define DIS_SEN 	0x0
#define CSA_CAL_A 	0x0
#define CSA_CAL_B 	0x0
#define CSA_CAL_C 	0x0
#define SEN_LVL 	0x3


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

#endif /* INC_DRV8323_DRIVER_SETTIGNS_H_ */
