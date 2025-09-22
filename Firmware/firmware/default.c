#include <avr/io.h>
#include <avr/eeprom.h>

#include "cardconf/card06.h"
#include "test/defaultframe.h"


uint8_t EEMEM eeprom_boks_id = 0;
uint8_t EEMEM eeprom_defaultFrame[] = DEFAULT_FRAME;

uint8_t defaultFrame[18] = DMX_DEFAULT_FRAME;
uint8_t boks_id = 0;

void load_default(uint16_t start_address)
{
    eeprom_read_block(defaultFrame, &eeprom_defaultFrame[start_address], 18);
}

void save_default(uint8_t frame[], size_t size) 
{
    eeprom_write_block(frame, eeprom_defaultFrame, size);
}

void load_id( void )
{
    boks_id = eeprom_read_byte(&eeprom_boks_id);
}

void write_id( uint8_t id )
{
    eeprom_write_byte(&eeprom_boks_id, id);
}

uint8_t* get_default( void )
{
    return &defaultFrame[0] ;
}

uint16_t get_start_address( void )
{
    return boks_id * 18;
}

uint16_t get_end_address( void )
{
    return boks_id*18 + 17;
}
