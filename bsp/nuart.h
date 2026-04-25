#ifndef __NUART_H
#define __NUART_H

#include "ti_msp_dl_config.h"

void usart_irq_config(void);
void UART_SendBytes(UART_Regs *port, uint8_t *ubuf, uint32_t len);
void UART_SendByte(UART_Regs *port, uint8_t data);

// 步进电机专用发送函数
void Stepper1_Send(uint8_t *cmd, uint32_t len);
void Stepper2_Send(uint8_t *cmd, uint32_t len);

#endif
