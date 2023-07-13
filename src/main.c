/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CH58x_common.h"
#include "project_config.h"
// #include "time_units.h"
#include "llcc68_example_recive.h"
#include "llcc68_example_send.h"

/*
  wiring setting
    spi bus:
        LoRa Modules    CHM582M
        NSS_PIN         PA12
        SCLK_PIN        PA13
        MOSI_PIN        PA14
        MISO_PIN        PA15
        RESET_PIN       PA6
        DIO1_PIN        PA5
        DIO4_BUSY_PIN   PA4
 */

void DebugInit() {
  GPIOA_SetBits(bTXD1);
  GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
  UART1_DefInit();
}

#if 0
int main1() {
   SetSysClock(CLK_SOURCE_PLL_60MHz);
   DebugInit();
   PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

   // GPIOA_ModeCfg(RADIO_DIO4_BUSY_PIN, GPIO_ModeIN_PU);
   GPIOA_ModeCfg(RADIO_DIO4_BUSY_PIN, GPIO_ModeOut_PP_5mA);

   GPIOA_ModeCfg(RADIO_DIO1_PIN, GPIO_ModeIN_PD);
   GPIOA_ITModeCfg(RADIO_DIO1_PIN, GPIO_ITMode_RiseEdge);
   PFIC_EnableIRQ(GPIO_A_IRQn);
   uint8_t ret;
      // GPIOA_SetBits(RADIO_DIO4_BUSY_PIN);
      // mDelaymS(1);
      GPIOA_ResetBits(RADIO_DIO4_BUSY_PIN);
   while (1) {
      // ret = GPIOA_ReadPortPin(RADIO_DIO4_BUSY_PIN);
      // printf("ret=%d\n", ret);
      // if (0 == ret) printf("low~\n");
      mDelaymS(1000);
   }
}
#else
int main() {
  SetSysClock(CLK_SOURCE_PLL_60MHz);
  DebugInit();
  PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
  GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);  // led

  // 测试demo，一个程序只能打开一条测试demo，进入测试demo后将进入死循环，不会返回了
#if 0
  ExampleLLCC68ReciveDemo();  // 循环接收demo
#else
  ExampleLLCC68SendDemo();  // 定时发送demo
#endif

  // 开启测试demo后代码就执行不到这里了
  while (1) {
    GPIOB_SetBits(GPIO_Pin_4);
    mDelaymS(500);
    GPIOB_ResetBits(GPIO_Pin_4);
    mDelaymS(500);
  }
}
#endif
