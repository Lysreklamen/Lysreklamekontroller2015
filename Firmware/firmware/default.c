#include "config.h"

#include <avr/io.h>
#include <avr/eeprom.h>

#include "default.h"

#include "test/defaultframe.h"


uint8_t EEMEM eeprom_boks_id = 9;
uint8_t EEMEM eeprom_frame[DEFAULT_SIZE] = DEFAULT_FRAME;

uint8_t defaultFrame[18];
uint8_t boks_id = 1;

static void les_id( void ) {
  boks_id = eeprom_read_byte(&eeprom_boks_id);
}

static void les_default( void ) {
  eeprom_read_block(defaultFrame, &eeprom_frame[første_addresse()], 18);
}

void default_init( void ){
  les_id();
  les_default();
}

uint8_t* boks_default( void ) {
  return &defaultFrame[0] ;
}

uint16_t første_addresse( void ) {
  return 1 + (boks_id - 1) * 18;
}

uint16_t siste_addresse( void ) {
  return 1 + (boks_id - 1)*18 + 17;
}

void skriv_id( uint8_t id ) {
  if (id != boks_id){
    eeprom_write_byte(&eeprom_boks_id, id);
  }    
  les_id();
  les_default();
}

void skriv_default( uint8_t frame[], size_t size ) {
  for (uint16_t i=0; i< size; i++) {
    // if (frame[i] != defaultFrame[i]) {
      eeprom_write_byte(&eeprom_frame[i],frame[i]);
    // }
  }
  les_default();
}

