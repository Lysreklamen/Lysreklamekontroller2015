#ifndef COM_H
#define COM_H

#include <inttypes.h>

void com_init(void);
void com_putc(uint8_t c);
void com_write(char *data);
void com_send(char *data, unsigned int size);
void com_handle(uint32_t dt);

#endif
