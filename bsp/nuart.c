#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "datatype.h"
#include "system.h"
#include "nuart.h"
#include "bluetooth_app.h"
// 步进电机命令发送缓冲区（可选）
// uint8_t stepper1_cmd_buf[20];
// uint8_t stepper2_cmd_buf[20];

// 串口中断配置
void usart_irq_config(void)
{
    // 清除中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    // UART3 不用（超声波已删除）
    NVIC_DisableIRQ(UART_3_INST_INT_IRQN);
    DL_UART_clearInterruptStatus(UART_3_INST, DL_UART_INTERRUPT_RX);

    // 使能需要用到的串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);   // 步进电机1
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);   // MaixCam 视觉
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);   // 步进电机2

    // 清除接收中断标志
    DL_UART_clearInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(UART_1_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(UART_2_INST, DL_UART_INTERRUPT_RX);

    // 设置系统滴答优先级
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// ========== UART0 中断：步进电机1 反馈接收（可选）==========
void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        //uint8_t ch = DL_UART_receiveData(UART_0_INST);
        // 如果需要处理步进电机1的返回数据，在这里添加
        // 例如：stepper1_rx_buf[stepper1_index++] = ch;
        DL_UART_clearInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_RX);
    }
}

// ========== UART1 中断：MaixCam 视觉数据接收 ==========
void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(UART_1_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        //uint8_t ch = DL_UART_receiveData(UART_1_INST);
        // 调用你自己的 MaixCam 数据解析函数
        // MaixCam_Data_Receive(ch);
        DL_UART_clearInterruptStatus(UART_1_INST, DL_UART_INTERRUPT_RX);
    }
}

// ========== UART2 中断：步进电机2 反馈接收（可选）==========
void UART_2_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(UART_2_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        uint8_t ch = DL_UART_receiveData(UART_2_INST);
        bluetooth_app_prase(ch);   // 调用蓝牙协议解析
        DL_UART_clearInterruptStatus(UART_2_INST, DL_UART_INTERRUPT_RX);

    }
}

// ========== 通用串口发送函数 ==========
void UART_SendBytes(UART_Regs *port, uint8_t *ubuf, uint32_t len)
{
    while (len--)
    {
        DL_UART_Main_transmitDataBlocking(port, *ubuf);
        ubuf++;
    }
}

void UART_SendByte(UART_Regs *port, uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(port, data);
}

// 针对两个步进电机的便捷发送函数（假设使用 UART0 和 UART2）
void Stepper1_Send(uint8_t *cmd, uint32_t len)
{
    UART_SendBytes(UART_0_INST, cmd, len);
}

void Stepper2_Send(uint8_t *cmd, uint32_t len)
{
    UART_SendBytes(UART_2_INST, cmd, len);
}

// printf 重定向到 UART0（调试用）
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
    return ch;
}




