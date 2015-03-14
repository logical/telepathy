#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "circular_buffer.h"


void cb_init(circular_buffer *cb)
{
    cb->buffer_end = &cb->buffer[BUFFERSIZE-1];
    cb->count = 0;
    cb->head = cb->buffer;
    cb->tail = cb->buffer;
}

void cb_free(circular_buffer *cb)
{
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

void cb_push_back(circular_buffer *cb, unsigned short *item)
{
    if(cb->count == BUFFERSIZE){
    		fprintf(stderr,"full buffer\n");
        return;// handle error
    }
    memcpy(cb->head, item, sizeof(unsigned short));
    cb->head = cb->head++;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
}

void cb_pop_front(circular_buffer *cb,unsigned short *item)
{
    if(cb->count == 0){
    		fprintf(stderr,"empty buffer\n");
        return;// handle error
    }
    memcpy(item, cb->tail, sizeof(unsigned short));
    cb->tail = cb->tail++;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;
}

