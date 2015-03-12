#ifndef XMEGA_USART_H
#define XMEGA_USART_H

#include <avr/io.h>

#define xmega_usart_txempty(usart) (((usart)->STATUS&USART_DREIF_bm) != 0)
#define xmega_usart_putc(usart, data) ((usart)->DATA = data)
#define xmega_usart_getc(usart) ((usart)->DATA)

#define xmega_usart_mode(usart, mode) \
	((usart)->CTRLC = ((usart)->CTRLC&~USART_CMODE_gm)|mode)

#define xmega_usart_frame(usart, size, parity, sb) \
	(usart)->CTRLC = (uint8_t)(size) | (parity) | ((sb)?USART_SBMODE_bm:0)

#define xmega_usart_2sb(usart, enable) 	\
	(usart)->CTRLC = (enable)?(usart)->CTRLC|USART_SBMODE_bm:(usart)->CTRLC&~USART_SBMODE_bm

#define xmega_usart_bsel(usart, bsel) \
	((usart)->BAUDCTRLA = (uint8_t)bsel); \
	((usart)->BAUDCTRLB = ((usart)->BAUDCTRLB&USART_BSCALE_gm) | (((bsel) >> 8) & 0x0F))

#define xmega_usart_bscale(usart, bscale) \
	((usart)->BAUDCTRLB = ((usart->BAUDCTRLB&(~USART_BSCALE_gm)) | (bscale << USART_BSCALE0_bp))) 

#define xmega_usart_tx_enable(usart) ((usart)->CTRLB |= USART_TXEN_bm)
#define xmega_usart_tx_disable(usart) ((usart)->CTRLB &= ~USART_TXEN_bm)
#define xmega_usart_rx_enable(usart) ((usart)->CTRLB |= USART_RXEN_bm)
#define xmega_usart_rx_disable(usart) ((usart)->CTRLB &= ~USART_RXEN_bm)

#define xmega_usart_txc_intlevel(usart, level) \
	((usart)->CTRLA = ((usart)->CTRLA&~USART_TXCINTLVL_gm)|(level))

#define xmega_usart_rxc_intlevel(usart, level) \
	((usart)->CTRLA = ((usart)->CTRLA&~USART_RXCINTLVL_gm)|(level))

#define xmega_usart_dre_intlevel(usart, level) \
	((usart)->CTRLA = ((usart)->CTRLA&~USART_DREINTLVL_gm)|(level))

void xmega_usart_init_default(USART_t *usart);
void xmega_usart_baud(USART_t *usart, uint32_t baud);

#endif
