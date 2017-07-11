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
#include "uart_debug.h"
#include "mpu9250.h"
#include "hal_millis.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h> //free

//Safer asprintf macro
#define Sasprintf(write_to, ...) { \
		char *tmp_string_for_extend = (write_to); \
		asprintf(&(write_to), __VA_ARGS__); \
		free(tmp_string_for_extend); \
}
//TM_MPU9250_t MPU9250;

//TM_AHRSIMU_t IMU;
//USART_TypeDef* uart1;
//USART_TypeDef* uart2;
//USART_TypeDef* uart3;
//USART_TypeDef* uart6;

void imu_init(I2C_TypeDef * I2Cx, ImuState_t *imu_state)
{

	imu_state->GyroMeasError = (float)PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	imu_state->GyroMeasDrift = (float)PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	// There is a tradeoff in the beta parameter between accuracy and response speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
	// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
	// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

	imu_state->beta = sqrt(3.0f / 4.0f) * imu_state->GyroMeasError;   // compute beta
	imu_state->zeta = sqrt(3.0f / 4.0f) * imu_state->GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	//import math library to calculate

	imu_state->gscale =  GFS_250DPS;
	imu_state->ascale = AFS_2G;
	imu_state->mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	imu_state->mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	for(int i = 0; i < 3; i++){
		imu_state->magCalibration[i]= 0;
		imu_state->magBias[i]= 0;
		imu_state->magScale[i]= 0;
		imu_state->accelBias[i]= 0;
		imu_state->gyroBias[i]= 0;
		imu_state->q[i+1] =0.0f;
		imu_state->eInt[i] =0.0f;
	}
	imu_state->delt_t = 0; // used to control display output rate
	imu_state->prevMeasure = 0, imu_state->sumCount = 0; // used to control display output rate
	imu_state->deltat = 0.0f, imu_state->sum = 0.0f; // integration interval for both filter schemes
	imu_state->lastUpdate = 0, imu_state->firstUpdate = 0; // used to calculate integration interval
	imu_state->Now = 0;
	/*
   IMU calculation parameters
	*/
	imu_state->q[0] =1.0f;

	/* IMU identification, check and calibration */

	uint8_t c[4];
	//initAK8963(handle, imu_state, imu_state->magCalibration);



	initMPU9250(I2Cx, imu_state);
	delay(5);
	calibrateMPU9250(I2Cx, imu_state->gyroBias, imu_state->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	delay(5);
	delay(5);
	MPU9250SelfTest(I2Cx, imu_state->SelfTest);
  delay(5);
  readBytes(I2Cx, MPU9250_ADDRESS, WHO_AM_I_MPU9250<<0x01, 4, c);  // Read WHO_AM_I register for MPU-9250
  if (SerialDebug) {
		char *temp = NULL;
		Sasprintf(temp, "MPU9250\nI AM %x  I should be %x", *c, 0x71);
		uart_printf(temp);
		free(temp);
  }
  delay(20);
  if (*c == 0x71) // WHO_AM_I should always be 0x71
  {
    if (SerialDebug) {
      uart_printf("MPU9250 is online, now self-testing\n");
    }
    MPU9250SelfTest(I2Cx, imu_state->SelfTest); // Start by performing self test and reporting values
    if (SerialDebug) {
			char *temp = NULL;
			for(int i = 0; i< 6; i++)
			{
				Sasprintf(temp, "x-axis self test: acceleration trim within : %3.1f%% of factory value\n", imu_state->SelfTest[i]);
			}
			uart_printf(temp);
			free(temp);
    }
    delay(10);
    getAres(imu_state);
    getGres(imu_state);
    getMres(imu_state);
    calibrateMPU9250(I2Cx, imu_state->gyroBias, imu_state->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    if (SerialDebug) {
			char *temp = NULL;
			Sasprintf(temp, " MPU9250 calibrated and its bias\n x   y   z  \n %i %i %i mg\n%3.3f %3.3f %3.3fo/s\n", (int)(1000 * imu_state->accelBias[0]),(int)(1000 * imu_state->accelBias[1]),(int)(1000 * imu_state->accelBias[2]),imu_state->gyroBias[0],imu_state->gyroBias[1],imu_state->gyroBias[2]);
			uart_printf(temp);
			free(temp);
    }
    delay(10);
    initMPU9250(I2Cx, imu_state);
    if (SerialDebug) {
      uart_printf("MPU9250 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    uint8_t d = readByte(I2Cx, AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    if (SerialDebug) {
			char *temp = NULL;
			Sasprintf(temp, "AK8963 \nI AM %x  I should be %x", d, 0x48);
			uart_printf(temp);
			free(temp);
    }
    delay(10);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(I2Cx, imu_state, imu_state->magCalibration);
		if (SerialDebug) {
     uart_printf("AK8963 initialized for active data mode....\n"); // Initialize device for active mode read of magnetometer
    }
    {
      //magcalMPU9250(magBias, magScale);
      float magbias[3] = {0, 0, 0};
      magbias[0] = 54.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
      magbias[1] = 280.;  // User environmental x-axis correction in milliGauss
      magbias[2] = -448.;  // User environmental x-axis correction in milliGauss

      imu_state->magBias[0] = (float) magbias[0] * imu_state->mRes * imu_state->magCalibration[0]; // save mag biases in G for main program
      imu_state->magBias[1] = (float) magbias[1] * imu_state->mRes * imu_state->magCalibration[1];
      imu_state->magBias[2] = (float) magbias[2] * imu_state->mRes * imu_state->magCalibration[2];

      // Get soft iron correction estimate hardcoded now but it can be monitored and corrected when new soft iron is introduced.
      imu_state->magScale[0] = 0.92;
      imu_state->magScale[1] = 1.03;
      imu_state->magScale[2] = 1.05;
    }
    if (SerialDebug) {
			uart_printf("Calibration values: \n");
			char *temp = NULL;
			Sasprintf(temp, "X-Axis sensitivity adjustment value %3.2f\nY-Axis sensitivity adjustment value %3.2f\nZ-Axis sensitivity adjustment value %3.2f\n", imu_state->magCalibration[0], imu_state->magCalibration[1], imu_state->magCalibration[2]);
			uart_printf(temp);
			free(temp);
    }
    delay(5);
  }
  else
  {
    if (SerialDebug) {
			char *temp = NULL;
			Sasprintf(temp, "Could not connect to MPU9250: 0x%x", *c);
			uart_printf(temp);
			free(temp);
    }
    while (1) ; // Loop forever if communication doesn't happen
  }
	/* IMU identification, check and calibration */

}
int main(void) {

	/* Initialize system */
	SystemInit();

	PWM_TIM_t PWM_Data;

	PWM_InitTimer(TIM2, &PWM_Data, 10000);
	PWM_InitChannel(TIM2, PWM_Channel_1, PWM_PinsPack_2);
	/* Set 70% duty cycle */
	//TODO: XXX findout the PWM range for the brushless motor and change the speed with this function
	PWM_SetChannelPercent(TIM2, &PWM_Data, PWM_Channel_1, 70);
	//timer_init(TIM2, 100000, 10000);
	//int pw = 90;
	//TIM_SetCompare2 (TIM2 , pw);

	// use all the usarts
	uint16_t flag = 1;

	//HAL_Init();
	hal_debug_uart_init(DEBUG_USART_BAUD_115200);

    uint8_t flowcontrol =0 ;
    int result = 1 ;
	result &= uart_init(USART1, 38400,flag, flowcontrol);
	result &= uart_init(USART2, 38400,flag, flowcontrol);
	result &= uart_init(USART3, 38400,flag, flowcontrol);
	result &= uart_init(USART6, 38400,flag, flowcontrol);

	if (!result)
		while(1);
		//return 0;

	I2C_LowLevel_Init(I2C1, 400000, 0x34<<1); //mpu9250 address

	ImuState_t mpu9250;
	SysTick_Init();
	imu_init(I2C1, &mpu9250);
	//initAK8963(I2C1, &mpu9250);
	uart_printf("set up finished\n");

	while(1)
	{
		sampleIMU(I2C1, &mpu9250);
	}

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
        //while (1);
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

/* EXTI handler */

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
void USART2_IRQHandler (void)
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
void USART3_IRQHandler (void)
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
void USART6_IRQHandler (void)
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
