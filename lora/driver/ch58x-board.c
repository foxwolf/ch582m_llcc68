/**
 * @file ch58x-board.h
 * @brief
 *
 * @author  <support@bjsdwlkj.cn>
 * @version 1.0.0
 * @date 2023-07-06
 */
/* Copyright (C) 2018-2023, SDWL Inc.
 * All right reserved
 */

#include "ch58x-board.h"
#include "project_config.h"

#include "radio.h"
#include "llcc68.h"
#include "llcc68-board.h"

typedef struct __softTimer_s {
  uint8_t isRun;     // 0停止，1运行
  uint32_t delayMs;  // 定时器需要多长时间后唤醒
  uint32_t startMs;  // 定时器开始时间
} SoftTimer_t;

static DioIrqHandler* dio1IrqCallback = NULL;  // 记录DIO1的回调函数句柄
static uint32_t timer_count = 0;
static SoftTimer_t txTimerHandle, rxTimerHandle;

/*
 * 初始化LLCC68需要用到的GPIO初始化，将 BUSY 引脚设置为输入模式
 */
void LLCC68IoInit(void) {
  GPIOA_ModeCfg(RADIO_DIO4_BUSY_PIN, GPIO_ModeIN_PU);

  /* SPI 0 */
  GPIOA_SetBits(RADIO_NSS_PIN);
  GPIOA_ModeCfg(RADIO_NSS_PIN | RADIO_SCK_PIN | RADIO_MOSI_PIN, GPIO_ModeOut_PP_5mA);  // 12:cs,13:clk,14:mosi,15:miso
  SPI0_MasterDefInit();
}

/* 初始化 DIO1
 * 将DIO1设置为外部中断(上升沿触发)，并且回调函数为 dioIrq 函数原型 void RadioOnDioIrq( void* context )
 */
void LLCC68IoIrqInit(DioIrqHandler dioIrq) {
  dio1IrqCallback = dioIrq;
  GPIOA_ModeCfg(RADIO_DIO1_PIN, GPIO_ModeIN_PU);
  GPIOA_ITModeCfg(RADIO_DIO1_PIN, GPIO_ITMode_RiseEdge);
  PFIC_EnableIRQ(GPIO_A_IRQn);
}

// 在中断回调函数中回调 void LLCC68IoIrqInit( DioIrqHandler dioIrq ) 中注册的 dioIrq 回调
__INTERRUPT
__HIGH_CODE
// __attribute__((interrupt("WCH-Interrupt-fast")))
// __attribute__((section(".highcode"))) void
void GPIOA_IRQHandler(void) {
  if (GPIOA_ReadITFlagBit(RADIO_DIO1_PIN)) {
    if (NULL != dio1IrqCallback) {
      dio1IrqCallback(NULL);
    }
    GPIOA_ClearITFlagBit(RADIO_DIO1_PIN);
  }
}

void LLCC68IoDeInit(void) {
  // GPIO去初始化代码
}

// 复位按键功能
void LLCC68Reset(void) {
  LLCC68DelayMs(10);

  // 将RST输出低电平
  GPIOA_ModeCfg(RADIO_nRESET_PIN, GPIO_ModeOut_PP_5mA);
  GPIOA_ResetBits(RADIO_nRESET_PIN);
  LLCC68DelayMs(20);

  // 将RST设置为上拉输入模式
  GPIOA_ModeCfg(RADIO_nRESET_PIN, GPIO_ModeIN_PU);
  LLCC68DelayMs(10);
}

// 读取Busy引脚电平状态
void LLCC68WaitOnBusy(void) {
  uint32_t u32_count = 0;

  while (GPIOA_ReadPortPin(RADIO_DIO4_BUSY_PIN) > 0) {
    if (u32_count++ > 1000) {
      printf("wait busy pin timeout\r\n");
      u32_count = 0;
    }
    LLCC68DelayMs(1);
  }
}

// 设置SPI的NSS引脚电平，0低电平，非零高电平
void LLCC68SetNss(uint8_t lev) {
  if (lev) {
    GPIOA_SetBits(RADIO_NSS_PIN);  // 输出高电平
  } else {
    GPIOA_ResetBits(RADIO_NSS_PIN);  // 输出低电平
  }
}

uint8_t HALSpi0InOut(uint8_t data) {
  R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
  R8_SPI0_BUFFER = data;
  while (!(R8_SPI0_INT_FLAG & RB_SPI_FREE))
    ;
  return (R8_SPI0_BUFFER);
}

// spi传输一个字节的数据
uint8_t LLCC68SpiInOut(uint8_t data) {
  return HALSpi0InOut(data);
}

// 检查频率是否符合要求，如果不需要判断则可以直接返回true
bool LLCC68CheckRfFrequency(uint32_t frequency) {
  // Implement check. Currently all frequencies are supported
  return true;
}

// 毫秒延时
void LLCC68DelayMs(uint32_t ms) {
  mDelaymS(ms);
}

// tx/rx定时器操作

// 定时器3中断服务程序
// tx定时结束需要回调 RadioOnTxTimeoutIrq
// rx定时结束需要回调 RadioOnRxTimeoutIrq

/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   TMR0中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void)  // TMR0 定时中断
{
  uint32_t diffMs = 0;
  if (TMR0_GetITFlag(TMR0_3_IT_CYC_END)) {
    TMR0_ClearITFlag(TMR0_3_IT_CYC_END);  // 清除中断标志

    // uint32_t sysTimeMs, curTimeMs;
    // sysTimeMs = get_current_time();
    // curTimeMs = sysTimeMs - (sysTimeMs / 1000) * 1000;
    // PRINT("[%ld]\n", sysTimeMs);

    timer_count++;
    // printf("timer_count=%d\r\n",timer_count);

    // 处理tx定时器
    if (txTimerHandle.isRun) {
      if (timer_count >= txTimerHandle.startMs) {
        diffMs = timer_count - txTimerHandle.startMs;
      } else {  // 溢出了
        diffMs = 0xffffffff - txTimerHandle.startMs + timer_count;
      }
      if (diffMs > txTimerHandle.delayMs) {
        // 计时结束
        // printf("----------------- tx timer irq ---------------\r\n");
        LLCC68TxTimerStop();
        txTimerHandle.startMs = timer_count;
        RadioOnTxTimeoutIrq(NULL);  // tx定时结束需要回调 RadioOnTxTimeoutIrq
      }
    }

    // 处理rx定时器
    if (rxTimerHandle.isRun) {
      if (timer_count >= rxTimerHandle.startMs) {
        diffMs = timer_count - rxTimerHandle.startMs;
      } else {  // 溢出了
        diffMs = 0xffffffff - rxTimerHandle.startMs + timer_count;
      }
      if (diffMs > rxTimerHandle.delayMs) {
        // 计时结束
        // printf("----------------- rx timer irq ---------------\r\n");
        LLCC68RxTimerStop();
        rxTimerHandle.startMs = timer_count;
        RadioOnRxTimeoutIrq(NULL);  // rx定时结束需要回调 RadioOnRxTimeoutIrq
      }
    }
  }
}

// 初始化定时器(这里用TMR0 1ms定时器)
// tx定时结束需要回调 RadioOnTxTimeoutIrq
// rx定时结束需要回调 RadioOnRxTimeoutIrq
void LLCC68TimerInit(void) {
  TMR0_TimerInit(FREQ_SYS / 1000);        // 设置定时时间 1ms
  TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);  // 开启中断
  PFIC_EnableIRQ(TMR0_IRQn);

  // 关闭软件定时器
  txTimerHandle.isRun = 0;
  rxTimerHandle.isRun = 0;
}

void LLCC68SetTxTimerValue(uint32_t nMs) {
  printf("[%s()-%d]set timer out %ld ms\r\n", __func__, __LINE__, nMs);
  txTimerHandle.delayMs = nMs;
}

void LLCC68TxTimerStart(void) {
  printf("[%s()-%d]start timer\r\n", __func__, __LINE__);
  txTimerHandle.startMs = timer_count;
  txTimerHandle.isRun = 1;
}

void LLCC68TxTimerStop(void) {
  printf("[%s()-%d]stop timer\r\n", __func__, __LINE__);
  txTimerHandle.isRun = 0;
}

void LLCC68SetRxTimerValue(uint32_t nMs) {
  printf("[%s()-%d]set timer out %ld ms\r\n", __func__, __LINE__, nMs);
  rxTimerHandle.delayMs = nMs;
}

void LLCC68RxTimerStart(void) {
  printf("[%s()-%d]start timer\r\n", __func__, __LINE__);
  rxTimerHandle.startMs = timer_count;
  rxTimerHandle.isRun = 1;
}

void LLCC68RxTimerStop(void) {
  printf("[%s()-%d]stop timer\r\n", __func__, __LINE__);
  rxTimerHandle.isRun = 0;
}
