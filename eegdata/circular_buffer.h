#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#define BUFFERSIZE 1024
typedef struct circular_buffer
{
    unsigned short buffer[BUFFERSIZE];     // data buffer
    void *buffer_end; // end of data buffer
    size_t count;     // number of items in the buffer
    void *head;       // pointer to head
    void *tail;       // pointer to tail
} circular_buffer;

void cb_init(circular_buffer *cb);
void cb_free(circular_buffer *cb);
void cb_push_back(circular_buffer *cb,unsigned short *item);
void cb_pop_front(circular_buffer *cb,unsigned short *item);
#endif
