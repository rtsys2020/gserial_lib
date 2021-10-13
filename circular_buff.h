/*
 * circular_buff.h
 *
 *  Created on: Jan 31, 2021
 *      Author: jupiter
 */

#ifndef CIRCULAR_BUFF_H_
#define CIRCULAR_BUFF_H_

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#define CB_SUCCESS 0        /* CB operation was successful */
#define CB_MEMORY_ERROR 1   /* Failed to allocate memory */
#define CB_OVERFLOW_ERROR 2 /* CB is full. Cannot push more items. */
#define CB_EMPTY_ERROR 3    /* CB is empty. Cannot pop more items. */

typedef struct circular_buffer {
  void *buffer;
  void *buffer_end;
  size_t sz;
  void *head;
  void *tail;
} circular_buffer;

int cb_init(circular_buffer *cb, size_t capacity, size_t sz);
int cb_free(circular_buffer *cb) ;
int cb_push_back(circular_buffer *cb, const void *item);
int cb_pop_front(circular_buffer *cb, void *item) ;
int cb_availData(circular_buffer *cb);
int cb_availFree(circular_buffer *cb);
int cb_size(circular_buffer *cb);
void cb_flush(circular_buffer *cb);
#endif /* CIRCULAR_BUFF_H_ */
