#include "memory_map.h"

AHB_RCC_OFFSET (0x14UL)
APB2_RCC_OFFSET  (0x18UL)
APB1_RCC_OFFSET (0x1CUL)

RCC_AHB_ENABLE_REG  RCC_BASE  + AHB_RCC_OFFSET
RCC_ABB2_ENABLE_REG  RCC_BASE + APB2_RCC_OFFSET
RCC_APB1_ENABLE_REG  RCC_BASE + APB1_RCC_OFFSET

void vDoEnableRcc(eAhbBus bus, unsigned long peripheral)
{

    if(bus == eApb2Bus)
    {
        




    }




}