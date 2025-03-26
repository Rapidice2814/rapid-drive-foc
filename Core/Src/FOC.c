#include <math.h>
#include <stdio.h>

#include "FOC.h"
#include "FOC_Driver.h"
#include "DRV8323_Driver.h"
#include "FOC_Utils.h"
#include "PID.h"
#include "FOC_Flash.h"


FLASH_DataTypeDef flash_data = {0};


void FOC_Setup(){
    flash_data.banana = 0x12345678;
    flash_data.apple = 3.14159;
    flash_data.orange = 0x42;
}

void FOC_Loop(){

}
