#include "dmx.h"
#include "pwm.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <ring.h>
#include <string.h>
#include <xmega_usart.h>

#include "cardconf/card06.h"

#define DMX_BREAK_TIME 150 // us

volatile uint16_t dmx_frame_counter;
volatile uint16_t dmx_byte_counter;
volatile uint32_t dmx_break_timer;

uint8_t dmx_frame[613];

uint16_t dmx_start_address = DMX_START_ADDRESS;
uint16_t EEMEM eeprom_dmx_start_address = 0;
uint8_t defaultFrame[18] = DMX_DEFAULT_FRAME;
uint8_t EEMEM eeprom_defaultFrame[36];

void applyFrame(uint8_t frame[], uint16_t offset);
void saveFrame(uint8_t frame[]);

void dmx_init(void) {
  /* Setup communication: */
  PORTD.DIRCLR = PIN6_bm;
  // PORTD.PIN6CTRL ^= PORT_INVEN_bm;

  xmega_usart_mode(&USARTD1, USART_CMODE_ASYNCHRONOUS_gc);
  xmega_usart_frame(&USARTD1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, 1);
  xmega_usart_baud(&USARTD1, 250000);
  xmega_usart_rx_enable(&USARTD1);
  xmega_usart_rxc_intlevel(&USARTD1, USART_RXCINTLVL_MED_gc);

  /* Setup timer for scheduling: (C0-A) */
  xmega_timer_prescale(&TCD1, TC_CLKSEL_DIV64_gc);
  xmega_timer_wgm(&TCD1, TC_WGMODE_NORMAL_gc);
  xmega_timer_enable_a(&TCD1);

  dmx_frame_counter = 0;
  dmx_byte_counter = 0;
  dmx_break_timer = 0;
}

void dmx_handle(uint32_t dt) {

  if (TCD1.CNT * 2 >= 64000) {
    applyFrame(&defaultFrame[0], 0);
  }

  // No data received?
  if (dmx_byte_counter < (1 + dmx_start_address + 18))
    return;

  // Break?
  if (TCD1.CNT * 2 >= DMX_BREAK_TIME) {

    TCD1.CNT = 0;
    // Reset frame byte counter;
    dmx_byte_counter = 0;

    // Not a data frame?
    if (dmx_frame[0] == 0x00) {
      // Apply the new DMX- frame:
      applyFrame(&dmx_frame[2], dmx_start_address);
    } else if (dmx_frame[0] == 0x01) {
      //			saveFrame(&dmx_frame[2]);
    }

    // Increment frame counter:
    dmx_frame_counter++;
  }
}

void applyFrame(uint8_t frame[], uint16_t offset) {
// Apply the new DMX- frame:
#define FRAME(channel)                                                         \
  (frame[(offset + (channel))] * frame[(offset + (channel))])
  LED0_SET(FRAME(0), FRAME(1), FRAME(2));
  LED1_SET(FRAME(3), FRAME(4), FRAME(5));
  LED2_SET(FRAME(6), FRAME(7), FRAME(8));
  LED3_SET(FRAME(9), FRAME(10), FRAME(11));
  LED4_SET(FRAME(12), FRAME(13), FRAME(14));
  LED5_SET(FRAME(15), FRAME(16), FRAME(17));
}

void saveFrame(uint8_t frame[]) {
  int i;
  for (i = 0; i < 18; i++) {
    defaultFrame[i] = frame[i];
  }

  eeprom_write_block(defaultFrame, eeprom_defaultFrame, sizeof(defaultFrame));
}

void dmx_parse(uint8_t c) {

  // Save byte to frame:
  dmx_frame[dmx_byte_counter] = c;
  dmx_byte_counter++;
  if (dmx_byte_counter > 600) {
    // error: Too many bytes in the frame!
    dmx_byte_counter = 0;
  }
}

ISR(TCD1_OVF_vect) {}

/* Incomming dmx communication: */
ISR(USARTD1_RXC_vect) {
  /* Parse incomming byte: */
  uint8_t c = xmega_usart_getc(&USARTD1);

  /* Send to active parser: */
  dmx_parse(c);

  // Reset break timer:
  TCD1.CNT = 0;
}
