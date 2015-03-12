#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <string.h>
#include <xmega_usart.h>
#include <ring.h>
#include "com.h"
#include "pwm.h"

#define UART_TX_BUFFER_SIZE	512
ring_t uart_tx_ring;
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];

extern volatile uint16_t dmx_frame_counter;
extern volatile uint16_t dmx_byte_counter;
extern volatile uint8_t dmx_frame[513];

extern volatile uint16_t dmx_start_address;
extern uint16_t eeprom_dmx_start_address;

void com_init(void)
{
	/* Setup communication: */
	ring_init(&uart_tx_ring, uart_tx_buffer, UART_TX_BUFFER_SIZE);
	PORTE.DIRSET = PIN7_bm;	
	PORTE.DIRCLR = PIN6_bm;
	xmega_usart_mode(&USARTE1, USART_CMODE_ASYNCHRONOUS_gc);
	xmega_usart_frame(&USARTE1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, 0);
	xmega_usart_baud(&USARTE1, 9600);
	xmega_usart_rx_enable(&USARTE1);
	xmega_usart_tx_enable(&USARTE1);
	xmega_usart_rxc_intlevel(&USARTE1, USART_RXCINTLVL_MED_gc);
	xmega_usart_dre_intlevel(&USARTE1, USART_DREINTLVL_OFF_gc);

}

void com_handle(uint32_t dt)
{
}

void com_putc(uint8_t c)
{
	ring_push(&uart_tx_ring, c);
	xmega_usart_dre_intlevel(&USARTE1, USART_DREINTLVL_LO_gc);
}

void com_write(char *data)
{
	while (*data!=0)
		com_putc(*data++);
}

void com_send(char *data, unsigned int size)
{
	int i;
	for (i=0;i<size;i++) {
		ring_push(&uart_tx_ring, data[i]);
		xmega_usart_dre_intlevel(&USARTE1, USART_DREINTLVL_LO_gc);
	}	
}

void com_write_hex8(uint8_t n)
{
	uint8_t dig[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	com_putc(dig[(n&0xF0)>>4]);
	com_putc(dig[(n&0x0F)]);
}

void com_write_hex16(uint16_t n)
{
	com_write_hex8(n>>8);
	com_write_hex8(n&0x00FF);
}

void com_terminal_parse(uint8_t c)
{
	#define INPUT_SIZE	64
	static uint8_t input[INPUT_SIZE];
	static uint8_t num = 0;

	//Newline?
	if ( (c == '\r') || (c == '\n') ) {
		input[num] = 0;
		com_putc('\n');
		if (strcmp((char*)input, "help") == 0) {
			com_write("help\t\tDisplay this help text.\n");
			com_write("on\t\tTurn all leds on.\n");
			com_write("off\t\tTurn all leds off.\n");
			com_write("dmx\t\tShow dmx status.\n");
			com_write("addr <xxx>\t\tShow/set dmx start address to xxx\n");
		}else if (strcmp((char*)input, "setup") == 0) {
			LED0_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED1_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED2_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED3_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED4_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED5_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			com_write("\n");
		}else if (strcmp((char*)input, "on") == 0) {
			LED0_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED1_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED2_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED3_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED4_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			LED5_SET(PWM_PERIOD, PWM_PERIOD, PWM_PERIOD);
			com_write("\n");
		}else if (strcmp((char*)input, "off") == 0) {
			LED0_SET(0, 0, 0);
			LED1_SET(0, 0, 0);
			LED2_SET(0, 0, 0);
			LED3_SET(0, 0, 0);
			LED4_SET(0, 0, 0);
			LED5_SET(0, 0, 0);
			com_write("\n");
		}else if (strncmp((char*)input, "addr", 4) == 0) {
	
			if (num == 4) {
				com_write("DMX address: ");
				com_write_hex16(dmx_start_address);
				com_write("\n");
			}else if ((num > 5) && (input[4] == ' ')) {
				uint16_t addr = atoi(&input[5]);
				if (addr >= 512) {
					com_write("error: address >= 512\n");
				}else{
					dmx_start_address = addr;
					eeprom_write_word(&eeprom_dmx_start_address, dmx_start_address);
					com_write("DMX address: ");
					com_write_hex16(dmx_start_address);
					com_write("\n");			
				}
			}else{
				com_write("usage addr <xxx>\t\tShow/set dmx start address to xxx\n");
			}

		}else if (strcmp((char*)input, "dmx") == 0) {
			com_write("DMX status:\n");
			com_write("------------------------------\n");
			com_write("Received frames: 0x");
			com_write_hex16(dmx_frame_counter);
			com_write("\n");
			com_write("Byte counter: 0x");
			com_write_hex16(dmx_byte_counter);
			com_write("\n");
			com_write("Last command byte: ");
			com_write_hex8(dmx_frame[0]);
			com_write("\n");
			com_write("First bytes in frame:\n");
			int i;
			for (i=1;i<7;i++) {
				com_write_hex8(dmx_frame[i]);
				com_write(" - ");
			}
			com_write("\n");
		}

		//Send promt:
		com_putc('>');
		num = 0;
	}else if (c == 0x08) {
		if (num) {
			num--;
			com_putc(0x08);
			com_putc(' ');
			com_putc(0x08);
		}
	}else {
		//Save and echo:
		if (num<INPUT_SIZE) {
			input[num++] = c;
			com_putc(c);
		}
	}

}


/* Incomming uart communication: */
ISR(USARTE1_RXC_vect)
{
	/* Parse incomming byte: */
	uint8_t c = xmega_usart_getc(&USARTE1);

	/* Send to active parser: */
	com_terminal_parse(c);
}

/* Outgoing uart communication: */
ISR(USARTE1_DRE_vect)
{
	uint8_t c;
	ring_pop(&uart_tx_ring, &c);
	xmega_usart_putc(&USARTE1, c);

	if (ring_empty(&uart_tx_ring))
	{
		xmega_usart_dre_intlevel(&USARTE1, USART_DREINTLVL_OFF_gc);
	}
}
