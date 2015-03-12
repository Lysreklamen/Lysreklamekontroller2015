#ifndef RING_H
#define RING_H

#include <stdint.h>

typedef struct _ring_t {
	uint8_t *data;
	uint16_t in;
	uint16_t out;
	uint16_t size;
}ring_t;

inline void ring_init(ring_t *ring, uint8_t *data, uint16_t size);
uint8_t ring_push(ring_t *ring, uint8_t ch);
uint8_t ring_pop(ring_t *ring, uint8_t *ch);
uint8_t ring_empty(ring_t *ring);
inline void ring_clear(ring_t *ring);
uint16_t ring_free(ring_t *ring);

#endif
