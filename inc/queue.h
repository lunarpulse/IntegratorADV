#ifndef HAL_QUEUE
#define HAL_QUEUE

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>

#define QUEUE_SIZE 80
struct Queue {
	uint16_t pRD , pWR;
	uint8_t q[ QUEUE_SIZE ];
};

int QueueFull ( struct Queue *q);
int QueueEmpty ( struct Queue *q);
int QueueAvail ( struct Queue *q);
int Enqueue ( struct Queue *q, uint8_t data);
int Dequeue ( struct Queue *q, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
