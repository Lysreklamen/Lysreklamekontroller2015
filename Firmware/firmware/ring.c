#include <stdint.h>
#include "ring.h"

uint8_t ring_full(ring_t *ring);

inline void ring_init(ring_t *ring, uint8_t *data, uint16_t size)
{
	ring->data = data;
	ring->in = 0;
	ring->out = 0;
	ring->size = size;
}

// |   0in     |     1    |    2out    |

uint8_t ring_push(ring_t *ring, uint8_t ch)
{
	if (ring_full(ring)) return 1;
	ring->data[ring->in] = ch;
	ring->in++;
	if (ring->in >= ring->size) ring->in = 0;
	return 0;
}

uint8_t ring_pop(ring_t *ring, uint8_t *ch)
{
	if (ring_empty(ring)) return 1;
	*ch = ring->data[ring->out];
	ring->out++;
	if (ring->out >= ring->size) ring->out = 0;
	return 0;
}

uint8_t ring_full(ring_t *ring)
{
	uint16_t next_in = ((ring->in+1)==ring->size)?0:ring->in+1;
	return (next_in == ring->out);
}

uint8_t ring_empty(ring_t *ring)
{
	return (ring->in == ring->out); 
}

uint16_t ring_free(ring_t *ring)
{
	int16_t retval = ring->in - ring->out;
	if (retval < 0) retval = -retval - 1;
	else retval = ring->size - retval - 1;
	return retval;
}

inline void ring_clear(ring_t *ring)
{
	ring->in = 0;
	ring->out = 0;
}
