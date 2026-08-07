#include "Bootloader.h"
#include "main.h"


#define BOOT_ADDR	0x1FFF0000	// my MCU boot code base address
#define	MCU_IRQS	71u	// no. of NVIC IRQ inputs

struct boot_vectable_ {
 uint32_t Initial_SP;
 void (*Reset_Handler)(void);
};

#define BOOTVTAB	((struct boot_vectable_ *)BOOT_ADDR)

void JumpToBootloader(void){
	__disable_irq();

	SysTick->CTRL = 0;

	HAL_RCC_DeInit();

	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++)
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}

	__enable_irq();

	// Set the MSP
	__set_MSP(BOOTVTAB->Initial_SP);

	BOOTVTAB->Reset_Handler();
}