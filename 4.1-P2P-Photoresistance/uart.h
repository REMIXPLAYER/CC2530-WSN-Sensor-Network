#ifndef _UART_H
#define _UART_H

#include <ioCC2530.h>

void Uart_Send_char(char ch);
void Uart_Send_String(char *Data);
int Uart_Recv_char(void);

#endif