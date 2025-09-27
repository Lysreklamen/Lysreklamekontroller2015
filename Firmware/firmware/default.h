#ifndef DEFAULT_H
#define DEFAULT_H

#include <avr/io.h>
#include <stddef.h>

uint8_t boks_nr ( void );
uint8_t* boks_default( void );
uint16_t første_addresse( void );
uint16_t siste_addresse( void );


void default_init( void );
void skriv_default(uint8_t frame[], size_t size);
void endre_id( uint8_t id );

#endif