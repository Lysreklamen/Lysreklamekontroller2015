#ifndef DEFAULT_H
#define DEFAULT_H

#include <avr/io.h>

uint8_t* get_default( void );
uint16_t get_start_address( void );
uint16_t get_end_address( void );

void load_default(uint16_t start_address);
void save_default(uint8_t frame[], size_t size);

void load_id( void );
void write_id( uint8_t id );

#endif