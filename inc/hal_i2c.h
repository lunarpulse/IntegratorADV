#ifndef HAL_I2C
#define HAL_I2C

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
typedef enum{
	Error,
	Success
} Status;

void I2C_LowLevel_Init ( I2C_TypeDef * I2Cx , int ClockSpeed , int OwnAddress );

Status I2C_Write ( I2C_TypeDef * I2Cx , const uint8_t * buf , uint32_t nbyte , uint8_t SlaveAddress );

Status I2C_Read ( I2C_TypeDef * I2Cx , uint8_t *buf , uint32_t nbyte , uint8_t SlaveAddress );

#ifdef __cplusplus
}
#endif

#endif
