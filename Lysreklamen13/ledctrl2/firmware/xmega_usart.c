#include <avr/io.h>
#include "xmega_usart.h"
#include <config.h>

void xmega_usart_init_default(USART_t *usart) 
{
	/* Set mode: */
	xmega_usart_mode(usart, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set frame format: */
	xmega_usart_frame(usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, 0);

	/* Set baudrate: */
	xmega_usart_baud(usart, 19200);
	
	/* Enable RX & TX: */
	xmega_usart_rx_enable(usart);
	xmega_usart_tx_enable(usart);

}

void xmega_usart_baud(USART_t *usart, uint32_t baud)
{
	/* Calculate values based on baud: */
	uint32_t bsel = ((uint32_t)128 * F_PER) / 16 / baud - 128;
	int8_t bscale = -7;

	while (bsel&0xF000) {
		bsel = bsel >> 1;
		bscale++;
	}

	/* Setup baudrate registers: */
	xmega_usart_bsel(usart, bsel);
	xmega_usart_bscale(usart, bscale);
}
