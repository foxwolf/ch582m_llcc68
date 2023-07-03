/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CH58x_common.h"

int main()
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();

    PRINT("Hello world!\n");

    while (1)
        ;
}