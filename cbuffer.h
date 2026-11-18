#ifndef CBUFFER_H
#define CBUFFER_H

/* Single-producer/single-consumer circular buffer. head is written only by the
 * producer, tail only by the consumer, so no critical section is needed. Each
 * instance points to caller-provided storage and records its size, allowing
 * different buffer sizes from one implementation. One slot is kept free to
 * distinguish full from empty, so usable capacity is (size - 1). */

typedef struct {
    char *data;             // caller-provided storage
    int size;               // capacity of that storage, in bytes
    volatile int head;
    volatile int tail;
} circular_buffer;

// Initialize with caller-provided storage
void cb_init(circular_buffer cb[], char *buffer, int size);
int cb_push(circular_buffer cb[], char c);
int cb_count(circular_buffer cb[]);
char cb_pop(circular_buffer cb[]);

#endif
