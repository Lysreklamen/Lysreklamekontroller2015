#include <stdbool.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define F_CPU 32000000ul
#include <util/delay.h>

#include "dmx.h"
#include "pwm.h"
#include "xmega_usart.h"
#include "xmega_timer.h"
#include "debug_led.h"
#include "default.h"

#define DMX_USART USARTD1
#define DMX_RXC_vect USARTD1_RXC_vect

#define DMX_TIMER TCD1
#define DMX_OVF_vect TCD1_OVF_vect
#define DMX_BREAK_TIME 150 // us

// en dmx-frame er på maks 512 "fields"
// pluss en startbyte
// og så legger vi på en ekstra bolle i posen for å være sikker:
#define DMX_BUFFER_SIZE 514


typedef enum {
  tilstand_start = 0,
  tilstand_break,
  tilstand_motta,
  tilstand_ferdig,
  tilstand_error,
} tilstand_t ;


typedef enum {
  modus_dmx = 0,
  modus_boksnr,
  modus_default,
} modus_t ;


volatile tilstand_t dmx_tilstand = tilstand_start;
volatile modus_t dmx_modus = modus_dmx;
volatile bool timeout = false;
volatile uint16_t dmx_byte_teller;
volatile uint8_t dmx_frame[DMX_BUFFER_SIZE];

volatile uint8_t dmx_boksid_teller = 0;
volatile uint8_t dmx_boksid_ny_verdi = 0;

volatile uint8_t dmx_default_teller = 0;

void applyFrame(volatile uint8_t frame[]);


static void dmx_error( void ) {
  dmx_tilstand = tilstand_error;
  debug_led1_clr();
}


void dmx_init(void) {
  load_id();
  load_default();
  // PD5 er TX-pinnen
  // PD6 er RX-pinnen
  // PD7 velger retning
  PORTD.DIRCLR = PIN6_bm;
  // Sett på pullup på rx-pinnen for å redusere kreft
  PORTD.PIN6CTRL |= PORT_OPC_PULLUP_gc;

  xmega_usart_mode(&DMX_USART, USART_CMODE_ASYNCHRONOUS_gc);
  xmega_usart_frame(&DMX_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, 1);
  xmega_usart_baud(&DMX_USART, 250000);
  xmega_usart_rx_enable(&DMX_USART);
  xmega_usart_rxc_intlevel(&DMX_USART, USART_RXCINTLVL_MED_gc);

  // Sett opp en timer for å telle hvor lenge det er siden sist vi mottok noe
  // Den går på F_CPU / 64 = 500kHz, eller 2μs per tikk
  xmega_timer_prescale(&DMX_TIMER, TC_CLKSEL_DIV64_gc);
  xmega_timer_wgm(&DMX_TIMER, TC_WGMODE_NORMAL_gc);
  // xmega_timer_enable_a(&DMX_TIMER);
  xmega_timer_ovf_interrupt(&DMX_TIMER, TC_OVFINTLVL_LO_gc);

  dmx_byte_teller = 0;
}


void blink_farge( uint8_t n, uint8_t r, uint8_t g, uint8_t b ) {
  for (uint8_t i=0; i<n; i++){
    LED012345_SET(r,g,b);
    _delay_ms(500);
    LED012345_SET(0,0,0);
    _delay_ms(500);
  }
}


void dmx_handle( void ) {
  if (timeout) {
    timeout = false;
    // Vi antar at noe har gått galt™
    dmx_error();
    // og at det er på tide å sette på default.  
    applyFrame(get_default());
    // pwm_set_frame(get_default()); # TODO
    debug_led2_set();
  }

  if (dmx_modus == modus_dmx  && dmx_tilstand == tilstand_motta && dmx_byte_teller >= get_end_address()) {
    dmx_tilstand = tilstand_ferdig;
    applyFrame(&dmx_frame[get_start_address()]);
    // pwm_set_frame(&dmx_frame[get_start_address()]); # TODO
    debug_led2_clr();
  }
}

void applyFrame( volatile uint8_t frame[] ) {
// Apply the new DMX- frame:
  LED0_SET(frame[ 0], frame[ 1], frame[ 2]);
  LED1_SET(frame[ 3], frame[ 4], frame[ 5]);
  LED2_SET(frame[ 6], frame[ 7], frame[ 8]);
  LED3_SET(frame[ 9], frame[10], frame[11]);
  LED4_SET(frame[12], frame[13], frame[14]);
  LED5_SET(frame[15], frame[16], frame[17]);
}

bool frame_error( uint8_t s ) {
  return s & (USART_FERR_bm);
}

void dmx_break( uint8_t c ) {
  dmx_byte_teller = 0;
  switch (c) {
    case 0: 
      dmx_tilstand = tilstand_motta;
      dmx_modus = modus_dmx;
    break;
    case 1:
      if (dmx_modus == modus_boksnr) {
        dmx_boksid_teller++;
      } else {
        dmx_modus = modus_boksnr;
        dmx_boksid_teller = 0;
      }
    break;
    default:
      dmx_error();
  }
}

void dmx_parse( uint8_t c ) {
  if ((dmx_tilstand == tilstand_motta || dmx_tilstand == tilstand_ferdig) && (dmx_byte_teller < DMX_BUFFER_SIZE)) {
    dmx_frame[dmx_byte_teller++] = c;
    // Reset timeout timer:
    DMX_TIMER.CNT = 0;
  } else {
    dmx_error();
  }
}



void default_parse( uint8_t c ) {
  if (dmx_default_teller == 0) {
    dmx_frame[dmx_byte_teller++] = c;
  } else {
    if (dmx_frame[dmx_byte_teller++] != c){
      dmx_error();
    }
  }
  if (dmx_byte_teller==504){
    dmx_default_teller++;
    if (dmx_default_teller == 20){
      save_default((uint8_t*)dmx_frame, 504);
      blink_farge(3,0,0,255);
    }     
  }
  if (dmx_byte_teller >= 512) {
    dmx_error();
  }
}

void boksid_parse( uint8_t c ) {
  if (dmx_byte_teller == 0){ 
    if (dmx_boksid_teller == 0) {
      dmx_boksid_ny_verdi = c;
    } else if ((dmx_boksid_ny_verdi == c) && (dmx_boksid_teller == 20)) {
      write_id(dmx_boksid_ny_verdi);
      blink_farge(3,255,0,0);
    } else if (dmx_boksid_ny_verdi == c) {
      dmx_boksid_teller++;
    } else {
      dmx_error();
    }
    dmx_byte_teller++;
  } else {
    if (dmx_byte_teller++ > 512){
      dmx_error();
    }
  }
}

ISR(DMX_OVF_vect) {
  // Hvis det har gått mer enn 128ms siden sist pakke antar vi at noe har gått galt™
  timeout = true;
}

// Denne tar i mot 1x byte og putter den i bufferet
ISR(DMX_RXC_vect) {
  uint8_t s = DMX_USART.STATUS;
  uint8_t c = xmega_usart_getc(&DMX_USART);
  if (frame_error(s) && c == 0){
    // en dmx 'break' er når man holder linjen lavt i mer enn en frame
    // så hvis vi får en frame error og det bare er 0, er det et godt tegn
    dmx_tilstand = tilstand_break;
    debug_led1_set();
  } else if (frame_error(s)) {
      dmx_error();
  } else {
    if  (dmx_tilstand == tilstand_break){
      dmx_break(c);
    } else {
      switch (dmx_modus) {
        case modus_dmx:
          dmx_parse(c);
        break;
        case modus_boksnr:
          boksid_parse(c);
        break;
        case modus_default:
          default_parse(c);
        break;
        default:
          dmx_error();
      }      
    }
  }
}
