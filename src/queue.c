#include <stm32f4xx.h>
#include "queue.h"

#define QUEUE_SIZE 80

int QueueFull ( struct Queue *q)
{
	return (((q->pWR + 1) % QUEUE_SIZE ) == q->pRD);
}

int QueueEmpty ( struct Queue *q)
{
	return (q->pWR == q->pRD);
}

int QueueAvail ( struct Queue *q )
{
	return (q->pWR - q->pRD);
}

int Enqueue ( struct Queue *q, uint8_t data)
{
	if ( QueueFull (q))
		return 0;

	else {
		q->q[q->pWR] = data;
		q->pWR = ((q->pWR + 1) == QUEUE_SIZE ) ? 0 : q->pWR + 1;
	}
	return 1;

}
int Dequeue ( struct Queue *q, uint8_t *data)
{
	if ( QueueEmpty (q))
		return 0;
	else {
		*data = q->q[q->pRD ];
		q->pRD = ((q->pRD + 1) == QUEUE_SIZE ) ? 0 : q->pRD + 1;
	}
	return 1;
}
