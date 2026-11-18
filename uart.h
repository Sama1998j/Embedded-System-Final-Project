#ifndef UART_H
#define UART_H

void uart1_init(void);
void uart1_send_char(char c);
void uart1_send_string(const char *s);
int  uart1_available(void);
char uart1_read(void);
int  uart1_tx_count(void);
int  uart1_rx_count(void);

#endif
