#include "cbuffer.h"

// Initialize the circular buffer
void cb_init(circular_buffer cb[], char *buffer, int size)
{
    cb[0].data = buffer;
    cb[0].size = size;
    cb[0].head = 0;
    cb[0].tail = 0;
}

// Add one byte. Returns 0 (without writing) if the buffer is full, else 1.
int cb_push(circular_buffer cb[], char c)
{
    int next = cb[0].head + 1;
    if (next >= cb[0].size)
    {
        next = 0;            // wrap to the start
    }

    if (next == cb[0].tail)
    {
        return 0;            // full: one slot is kept free
    }

    cb[0].data[cb[0].head] = c;
    cb[0].head = next;
    return 1;
}

// Remove and return the oldest byte
char cb_pop(circular_buffer cb[])
{
    char c = cb[0].data[cb[0].tail];

    int next = cb[0].tail + 1;
    if (next >= cb[0].size)
    {
        next = 0;            // wrap to the start
    }
    cb[0].tail = next;

    return c;
}

// Number of bytes currently stored (handles the wrap-around).
int cb_count(circular_buffer cb[])
{
    return (cb[0].head - cb[0].tail + cb[0].size) % cb[0].size;
}