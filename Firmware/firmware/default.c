#include <avr/io.h>
#include <avr/eeprom.h>

#include "default.h"

// #include "cardconf/card06.h"
#include "test/defaultframe.h"


uint8_t EEMEM eeprom_boks_id = 0;
uint8_t EEMEM eeprom_defaultFrame[] = DEFAULT_FRAME;

uint8_t defaultFrame[18];
uint8_t boks_id = 0;

void load_default( void ) {
  eeprom_read_block(defaultFrame, &eeprom_defaultFrame[get_start_address()], 18);
}

void save_default( uint8_t frame[], size_t size ) {
  for (uint16_t i=0; i< size; i++){
    if (frame[i] != eeprom_read_byte(&eeprom_defaultFrame[i])) {
      eeprom_write_byte(&eeprom_defaultFrame[i],frame[i]);
    }
  }
}

void load_id( void ) {
  boks_id = eeprom_read_byte(&eeprom_boks_id);
}

void write_id( uint8_t id ) {
  if (id != boks_id){
    eeprom_write_byte(&eeprom_boks_id, id);
  }    
}

uint8_t* get_default( void ) {
  return &defaultFrame[0] ;
}

uint16_t get_start_address( void ) {
  return boks_id * 18;
}

uint16_t get_end_address( void ) {
  return boks_id*18 + 17;
}
