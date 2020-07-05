/***********************************************************************/
/*                                                                     */
/*  FILE        :queue.c                                               */
/*  DATE        :Wed, Jun 14, 2009                                     */
/*  DESCRIPTION :Queue Program                                         */
/*  CPU TYPE    :SH7047                                                */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/

#include "queue.hpp"

#ifndef _QUEUE_CPP__
#define _QUEUE_CPP__


/***********************************************************************/
/*                     静的な領域を利用するFIFO                        */
/***********************************************************************/

uint16_t queueNext(uint16_t n, uint16_t size) {
  return((n + 1) % (size));
}

uint8_t isQueueEmpty(queueType *que) {
  return(que->front == que->rear);
}

uint8_t isQueueFull(queueType *que) {
  return(queueNext(que->rear, que->size) == que->front);
}

uint8_t enqueue(queueType *que, queue_t x) {
  if(isQueueFull(que)) {
    return(0);
  } else {
    que->data[que->rear] = x;
    que->rear = queueNext(que->rear, que->size);
    return(1);
  }
}

queue_t dequeue(queueType *que) {
  queue_t x;

  if(isQueueEmpty(que)) {
    return(0);
  } else {
    x = que->data[que->front];
    que->front = queueNext(que->front, que->size);
    return(x);
  }
}

int8_t *getQueueName(queueType *que) {
  return(que->name);
}

#endif

