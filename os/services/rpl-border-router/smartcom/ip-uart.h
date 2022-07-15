/**
 * \file
 *         IP fallback interface UART driver header file
 * \author
 *         Raphael Löffel <loeffel@rte-ag.ch>
 */
/*---------------------------------------------------------------------------*/
#ifndef IP_UART_H_
#define IP_UART_H_

#include "contiki.h"

bool ip_uart_init(void);
void ip_uart_set_input_callback(void (*c)(void));
void ip_uart_start(void);
void ip_uart_send(void);
int_fast32_t ip_uart_write(const void *buf, size_t buf_size);

#endif /* IP_UART_H_ */
