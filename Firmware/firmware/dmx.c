#include "config.h"

#include <stdbool.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

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

typedef enum { ERROR = 0, BREAK, MOTTA, FERDIG } tilstand_t ;
typedef enum { DMX = 0, BOKSNR, DF_CMD, DF_SET } modus_t ;

volatile tilstand_t tilstand = ERROR;
volatile modus_t modus = DMX;

volatile uint8_t  dmx_frame[DMX_BUFFER_SIZE];
volatile uint16_t dmx_ch;             // Hvilken kanal har vi kommet til
volatile uint16_t dmx_timeout_ms = 0; // hvor mange ms er det igjen til vi må sette default

volatile uint8_t  dmx_boksid_teller = 0;
volatile uint8_t  dmx_boksid_ny_verdi = 0;

volatile uint8_t  dmx_default_teller = 0;
volatile uint32_t dmx_default_timeout_ms = 0;

static void dmx_error( void ) {
  tilstand = ERROR;
}

static void blink_farge( uint8_t n, uint8_t r, uint8_t g, uint8_t b ) {
  for (uint8_t i=0; i<n; i++){
    LED012345_SET(r,g,b); _delay_ms(500);
    LED012345_SET(0,0,0); _delay_ms(500);
  }
}

static void applyFrame( volatile uint8_t frame[] ) {
// Apply the new DMX- frame:
  LED0_SET(frame[ 0], frame[ 1], frame[ 2]);
  LED1_SET(frame[ 3], frame[ 4], frame[ 5]);
  LED2_SET(frame[ 6], frame[ 7], frame[ 8]);
  LED3_SET(frame[ 9], frame[10], frame[11]);
  LED4_SET(frame[12], frame[13], frame[14]);
  LED5_SET(frame[15], frame[16], frame[17]);
}

static void oppdater_leds( void ){
  tilstand = FERDIG;
  // pwm_set_frame(&dmx_frame[første_addresse()]); # TODO    
  applyFrame(&dmx_frame[første_addresse()]); 
}

static void sett_ny_boksid( uint8_t boksid ){
  modus = DMX;
  skriv_id(boksid);
  blink_farge(3, 255, 0, 0);
  blink_farge(boksid, 0, 255, 0);
}

static void sett_ny_default( void ) {
  modus = DMX; 
  cli();
  skriv_default((uint8_t*)dmx_frame, DEFAULT_SIZE);
  sei();
  blink_farge(3,0,0,255); 
  applyFrame(boks_default());
}

void dmx_init(void) {
  // PD5 er TX-pinnen
  // PD6 er RX-pinnen
  // PD7 velger retning
  PORTD.DIRCLR = PIN6_bm;
  
  // Sett på pullup på rx-pinnen for å redusere kreft
  PORTD.PIN6CTRL |= PORT_OPC_PULLUP_gc;

  // Sett opp seriemodulen som tar i mot DMX:
  xmega_usart_mode(&DMX_USART, USART_CMODE_ASYNCHRONOUS_gc);
  xmega_usart_frame(&DMX_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, 1);
  xmega_usart_baud(&DMX_USART, 250000);
  xmega_usart_rx_enable(&DMX_USART);
  xmega_usart_rxc_intlevel(&DMX_USART, USART_RXCINTLVL_MED_gc);
  
  // Sett opp en timer for å telle hvor lenge det er siden sist vi mottok noe
  // Den går på F_CPU / 64 = 500kHz, eller 2μs per tikk
  xmega_timer_prescale(&DMX_TIMER, TC_CLKSEL_DIV64_gc);
  xmega_timer_wgm(&DMX_TIMER, TC_WGMODE_NORMAL_gc);
  // og teller til 500, så en runde per ms
  DMX_TIMER.PER = 500;
  // og har interrupts vi kan telle med
  xmega_timer_ovf_interrupt(&DMX_TIMER, TC_OVFINTLVL_LO_gc);
}

// Denne kjører asynkront og kan ta seg god tid med ting som trenger å gjøres noe med.
void dmx_handle( void ) {
  // Hvis timeout er 0 har noe gått galt™  og at det er på tide å sette på default.  
  if (dmx_timeout_ms == 0) { applyFrame(boks_default()); }

  switch (modus) {
    case DMX: if (tilstand == MOTTA && dmx_ch >= siste_addresse()) {oppdater_leds();} break;
    case BOKSNR: if (dmx_boksid_teller > 20) { sett_ny_boksid(dmx_boksid_ny_verdi); } break;
    case DF_SET:
      if (!dmx_default_timeout_ms){ modus = DMX; }
      if (dmx_default_teller > 20){ sett_ny_default(); }
      break;
    default: break;
  }
}

// ------------------------- < Avbrudd-vennlige funksjoner > -------------------------
// alle kategoriene nedover her er forventet å gjøre minst mulig fordi de kjøres i et avbrudd


// ---------- < default > ----------
// i default-set-modus ser vi på pakkene som kommer inn
// og hvis mange nok på rad er like setter man en ny default og går tilbake til normalen

// Denne håndterer første byte-delen og holder orden på antallet
void default_break( void ) {
  static uint8_t cmd_i = 0;
  switch (modus) {
    case DF_CMD: if (cmd_i++ == 20) { modus = DF_SET; dmx_default_teller = 0; dmx_default_timeout_ms = 60000ul; } break;
    case DF_SET: { dmx_default_teller = 0; dmx_default_timeout_ms = 60000ul; } break; 
    default: modus = DF_CMD; cmd_i = 0;
  }
}

// denne funksjonen sjekker at kanalen som kom inn nå var lik som sist, 
// og når vi når slutten av det vi bryr oss om, øker vi tallet 
void default_parse( uint8_t c ) {
  debug_led3_set();
  if (dmx_ch >= 512)  { dmx_error(); }
  if (dmx_default_teller == 0)       { dmx_frame[dmx_ch++] = c; } 
  else if (dmx_frame[dmx_ch++] != c) { dmx_default_teller = 0; }
  if (dmx_ch == DEFAULT_SIZE ) { dmx_default_teller++; } 
}


// ---------- < boks-nummer > ----------

// Hvis vi får mange nok magiske pakker på rad med 0x01 som første byte
// og samme ch1-verdi setter vi ch1-verdien som nytt boksnummer.

// Denne håndterer første byte-delen og holder orden på antallet
void boksnr_break( void ) {
  if ( modus == BOKSNR ) { dmx_boksid_teller++; }
  else                   { modus = BOKSNR; dmx_boksid_teller = 0; } 
}
// denne sjekker ch1-verdien hvis det er der vi er
void boksid_parse( uint8_t c ) {
  debug_led2_set(); 
  if (dmx_ch >= DMX_BUFFER_SIZE) { dmx_error(); }
  else if (dmx_ch++ == 0) { 
    if      (dmx_boksid_teller++ == 0)  { dmx_boksid_ny_verdi = c; } 
    else if (dmx_boksid_ny_verdi != c)  { dmx_error(); }
  }
}

// ---------- < Dmx-mottak > ----------

// Denne tar i mot en vanlig kanal og legger den i bufferet
void dmx_parse( uint8_t c ) {
  if (tilstand == MOTTA) { 
    dmx_frame[dmx_ch++] = c;
    // reset timeout så ikke vi får default
    dmx_timeout_ms = 128;
  }
}

// Denne ser på den første byten og håndterer valg av modus basert på hva den er
void parse_break( uint8_t c ) {
  dmx_ch = 0;
  tilstand = MOTTA;
  switch (c) {
    case 0: if (modus != DF_SET) { modus = DMX;} break;
    case 1: boksnr_break();  break;
    case 2: default_break(); break;
    default: dmx_error();
  }
}

// ------------------------- < Avbrudd: > -------------------------

// Dette kjøres når timeren restarter ca nøyaktig 1 gang i millisekundet.
ISR(DMX_OVF_vect) {
  if ( dmx_timeout_ms )         { dmx_timeout_ms--; }
  if ( dmx_default_timeout_ms ) { dmx_default_timeout_ms--; }
}

// Dette kjøres når det kommer en ny byte på dmx-porten:
ISR(DMX_RXC_vect) {
  uint8_t status =    DMX_USART.STATUS;
  bool frame_error =  status & USART_FERR_bm;
  uint8_t c =         xmega_usart_getc(&DMX_USART);

  // en dmx 'break' er når man holder linjen lavt i mer enn en frame
  // så hvis vi får en frame error og data er 0, er det et godt tegn
  if (frame_error) {
    debug_led1_clr(); debug_led2_clr(); debug_led3_clr();  
    if( c == 0 ) { tilstand = BREAK; } 
    else         { dmx_error(); }    
  } 
  else if (tilstand == BREAK)        { parse_break(c);} 
  else if (dmx_ch > DMX_BUFFER_SIZE) { dmx_error(); } 
  else if (tilstand != ERROR) {
    debug_led1_set();
    switch (modus) {
      case DMX:    dmx_parse(c);      break;
      case BOKSNR: boksid_parse(c);   break;
      case DF_CMD: { /*Ingenting*/ }  break;
      case DF_SET: default_parse(c);  break;
      default:     dmx_error();
    }
  }
}
