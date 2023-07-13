#include "LLCC68_example_send.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
// #include "stm32f10x_it.h"
// #include "delay.h"
#include "string.h"

/*!
 * Radio events function pointer
 * 这个是传参进入其他函数中了，所以用全局变量(局部变量使用完了内存释放可能导致异常)
 */
static RadioEvents_t LLCC68RadioEvents;

const char sendData[20] = "aithinker";
// const char sendData[20] = "PONG";

static void LLCC68OnTxDone( void );
static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void LLCC68OnTxTimeout( void );
static void LLCC68OnRxTimeout( void );
static void LLCC68OnRxError( void );

//开启一个定时发送任务，每隔1S发送一条数据
void ExampleLLCC68SendDemo(void){
	uint32_t u32_count=0;
	uint8_t OCP_Value = 0;
	
	printf("start %s() example\r\n",__func__);
	
	LLCC68RadioEvents.TxDone = LLCC68OnTxDone;
	LLCC68RadioEvents.RxDone = LLCC68OnRxDone;
	LLCC68RadioEvents.TxTimeout = LLCC68OnTxTimeout;
	LLCC68RadioEvents.RxTimeout = LLCC68OnRxTimeout;
	LLCC68RadioEvents.RxError = LLCC68OnRxError;

	Radio.Init( &LLCC68RadioEvents );
	Radio.SetChannel(LORA_FRE);
	Radio.SetTxConfig( MODEM_LORA, LORA_TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );//参数：lora模式,发射功率,fsk用的lora设置为0就可以，带宽，纠错编码率，前导码长度，固定长度数据包(一般是不固定的所以选false)，crc校验，0表示关闭跳频，跳频之间的符号数(关闭跳频这个参数没有意义)，这个应该是表示是否要翻转中断电平的，超时时间

	OCP_Value = Radio.Read(REG_OCP);
	printf("[%s()-%d]read OCP register value:0x%04X\r\n",__func__,__LINE__,OCP_Value);
	
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_LLCC68_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );
	
	printf("all setting\r\n");
	printf("freq: %d\r\n Tx power: %d\r\n band width: %d\r\n FS: %d\r\n CODINGRATE: %d\r\n PREAMBLE_LENGTH: %d\r\n",LORA_FRE,LORA_TX_OUTPUT_POWER,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_PREAMBLE_LENGTH);
	while(1){
		Radio.IrqProcess( ); // Process Radio IRQ
		
		if(0==u32_count%1000){
			// printf("systick=%d ,send u32 data:%d\r\n", Get_SysTick(),u32_count);
			if(0==u32_count%2000){
				Radio.Send((uint8_t *)&u32_count,4);
			}else
				Radio.Send((uint8_t *)sendData,(strlen(sendData)+1));

		}
		u32_count++;

		// delay_ms(1);
		// mDelaymS(1);
		DelayMs(2);
	}
}

static void LLCC68OnTxDone( void )
{
	printf("TxDone\r\n");
	Radio.Standby();
	
	//发送完成闪烁一下led提示
	// GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	// delay_ms(100);
	// GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIOB_SetBits(GPIO_Pin_4);
	mDelaymS(100);
	GPIOB_ResetBits(GPIO_Pin_4);
}

static void LLCC68OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	uint32_t reciveNumber=0;
	Radio.Standby();
	printf("RxDone\r\nsize:%d\r\nrssi:%d\r\nsnr:%d\r\n",size,rssi,snr);
	Radio.Rx( LORA_RX_TIMEOUT_VALUE );
	if(size!=4){
		printf("recive size !=4 is error\r\n");
	}else{
		memcpy(&reciveNumber,payload,4);
		printf("recive u32 data=%ld\r\n",reciveNumber);
		//接收成功闪烁一下led提示
		// GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		// delay_ms(100);
		// GPIO_SetBits(GPIOB,GPIO_Pin_12);
		GPIOB_SetBits(GPIO_Pin_4);
		mDelaymS(100);
		GPIOB_ResetBits(GPIO_Pin_4);
	}
}

static void LLCC68OnTxTimeout( void )
{
	printf("TxTimeout\r\n");
}

static void LLCC68OnRxTimeout( void )
{
	Radio.Standby();
	printf("RxTimeout retry recive\r\n");
	Radio.Rx( LORA_RX_TIMEOUT_VALUE ); 
}

static void LLCC68OnRxError( void )
{
	Radio.Standby();
	printf("RxError retry recive\r\n");
	Radio.Rx(LORA_RX_TIMEOUT_VALUE); 
}
