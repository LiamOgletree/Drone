
#include "ringbuffer.h"

/*----------------------------------------------------------------------------*/
RB_STATUS RingBuffer_ctor(RingBuffer * const rb, uint32_t const capacity, RingBuffer_t * const buffer){
  // If capacity is 0 or buffer is a nullptr, return failure.
  if(!capacity || !buffer)
  {
    return RB_FAILURE;
  }
  // Else, initialize values and return success.
	rb->capacity = capacity;
	rb->buffer   = buffer;
	rb->size     = (uint32_t)0;
	rb->head     = (uint32_t)0;
	rb->tail     = (uint32_t)0;
	return RB_SUCCESS;
}

/*----------------------------------------------------------------------------*/
RB_STATUS RingBuffer_enqueue(RingBuffer * const rb, RingBuffer_t const item){
	// Add item to head end of RingBuffer. Overwrite old data if necessary.
	rb->buffer[rb->head] = item;
	// If head is at the end of memory, wrap around to 0.
	if(++rb->head == rb->capacity)
	{
		rb->head = 0;
	}
	// Increment size if not already at capacity.
	if(rb->size < rb->capacity)
	{
		rb->size++;
	}
	// Otherwise, increment tail so that we're still reading oldest data.
	else
	{
		if(++rb->tail == rb->capacity)
		{
			rb->tail = 0;
		}
	}
	// Return success to calling function.
	return RB_SUCCESS;
}

/*----------------------------------------------------------------------------*/
RB_STATUS RingBuffer_dequeue(RingBuffer * const rb, RingBuffer_t * const item){
	// If RingBuffer is empty, return false.
	if(!rb->size)
	{
		return RB_FAILURE;
	}
	// Otherwise, take item from tail end of RingBuffer.
	else
	{
		*item = rb->buffer[rb->tail];
		// If tail is at the end of memory, wrap around to 0.
		if(++rb->tail == rb->capacity)
		{
		  rb->tail = 0;
		}
		// Decrement the size of the buffer.
		rb->size--;
	}
	// Return success to calling function.
	return RB_SUCCESS;
}
