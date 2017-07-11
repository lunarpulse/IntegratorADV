/**
 * Keil project example for MPU9250
 *
 * Before you start, select your target, on the right of the "Load" button
 *
 * @author    Tilen Majerle
 * @email     tilen@majerle.eu
 * @website   http://stm32f4-discovery.net
 * @ide       Keil uVision 5
 * @conf      PLL parameters are set in "Options for Target" -> "C/C++" -> "Defines"
 * @packs     STM32F4xx/STM32F7xx Keil packs are requred with HAL driver support
 * @stdperiph STM32F4xx/STM32F7xx HAL drivers required
 */
/* Include core modules */
#include "stm32fxxx_hal.h"
/* Include my libraries here */
//#include "defines.h"

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>

#include "hal_i2c.h"
#include "hal_usart.h"
#include "queue.h"
#include "hal_timer.h"

#include "stdio.h"


//TM_MPU9250_t MPU9250;

//TM_AHRSIMU_t IMU;
//USART_TypeDef* uart1;
//USART_TypeDef* uart2;
//USART_TypeDef* uart3;
//USART_TypeDef* uart6;

int main(void) {

	/* Initialize system */
	SystemInit();

	PWM_TIM_t PWM_Data;

	PWM_InitTimer(TIM2, &PWM_Data, 10000);
	PWM_InitChannel(TIM2, PWM_Channel_1, PWM_PinsPack_2);
	/* Set 70% duty cycle */
	PWM_SetChannelPercent(TIM2, &PWM_Data, PWM_Channel_1, 70);
	// use all the usarts
	uint16_t flag = 1;
	//uart1 = USART1;
	//uart2 = USART2;
	//uart3 = USART3;
	//uart6 = USART6;


    
	//timer_init(TIM2, 100000, 10000);
	//int pw = 90;
	//TIM_SetCompare2 (TIM2 , pw);
    //HAL_Init();
    uint8_t flowcontrol =0 ;
    int result = 1 ;
	result &= uart_init(USART1, 38400,flag, flowcontrol);
	result &= uart_init(USART2, 38400,flag, flowcontrol);
	result &= uart_init(USART3, 38400,flag, flowcontrol);
	result &= uart_init(USART6, 38400,flag, flowcontrol);

	if (!result)
		return 0;

	I2C_LowLevel_Init(I2C1, 100000, 0xD0); //mpu9250 address
	/* Delay init */
	//TM_DELAY_Init();
    
    /* Init LEDs */
    //TM_DISCO_LedInit();
    
    /* Init USART, TX = PA2, RX = PA3 */
    //TM_USART_Init(USART2, TM_USART_PinsPack_1, 921600);
    //I2C_Init();
    /* Init MPU9250 */
    //if (TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0) != TM_MPU9250_Result_Ok) {
    //    printf("Device error!\r\n");
        while (1);
    //}
    
    //printf("Device connected!\r\n");
    
    //TM_EXTI_Attach(GPIOA, GPIO_PIN_0, TM_EXTI_Trigger_Rising);
    
    //TM_AHRSIMU_Init(&IMU, 1000, 0.5, 0);
    
    //while (1) {
    //    if (TM_MPU9250_DataReady(&MPU9250) == TM_MPU9250_Result_Ok) {
     //       TM_MPU9250_ReadAcce(&MPU9250);
     //       TM_MPU9250_ReadGyro(&MPU9250);
     //       TM_MPU9250_ReadMag(&MPU9250);
     //
     //       TM_AHRSIMU_UpdateIMU(&IMU, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Ax, MPU9250.Ay, MPU9250.Az);
            
     //       printf("R: %f, P: %f, Y: %f\n", IMU.Roll, IMU.Pitch, IMU.Yaw);
            
//            printf("Ax: %f, Ay: %f, Az: %f, Gx: %f, Gy: %f, Gz: %f\n", 
//                MPU9250.Ax, MPU9250.Ay, MPU9250.Az,
//                MPU9250.Gx, MPU9250.Gy, MPU9250.Gz
//            );
//            printf("Mx: %d, My: %d, Mz: %d\r\n",
//                MPU9250.Mx, MPU9250.My, MPU9250.Mz
//            );
    //    }
	//}
}

/* Printf handler */
int fputc(int ch, FILE* fil) {
    //TM_USART_Putc(USART2, ch);
    
    return ch;
}

/* EXTI handler */
void TM_EXTI_Handler(uint16_t GPIO_Pin) {
    //TM_DISCO_LedToggle(LED_ALL);
}
void USART1_IRQHandler (void)
{
	if( USART_GetITStatus (USART1 , USART_IT_RXNE ) != RESET)
	{
		uint8_t data;
		if(USART1->CR3 == USART_HardwareFlowControl_CTS)
		{
			// clear the interrupt
			USART_ClearITPendingBit (USART1 , USART_IT_RXNE );
		}

		// buffer the data (or toss it if there 's no room
		// Flow control will prevent this
		data = USART_ReceiveData ( USART1 ) & 0xff;
		if (! Enqueue (& UART1_RXq , data))
			RxOverflow = 1;
		if(USART1->CR3 == USART_HardwareFlowControl_CTS)
		{
			if ( QueueAvail (& UART1_RXq ) > HIGH_WATER )
				GPIO_WriteBit (GPIOA , GPIO_Pin_12 , 1);
		}
	}

	if( USART_GetITStatus (USART1 , USART_IT_TXE ) != RESET)
	{
		uint8_t data;
		/* Write one byte to the transmit data register */
		if ( Dequeue (& UART1_TXq , &data)){
			USART_SendData (USART1 , data);
		} else {
		// if we have nothing to send , disable the interrupt
		// and wait for a kick
			USART_ITConfig (USART1 , USART_IT_TXE , DISABLE );
			TxPrimed = 0;
		}
	}
}

volatile uint32_t ticker =0;

void SysTick_Handler(void)
{
  ticker++;
}
