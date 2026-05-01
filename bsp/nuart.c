#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "datatype.h"
#include "system.h"
#include "nuart.h"

// 步进电机命令发送缓冲区（可选）
// uint8_t stepper1_cmd_buf[20];
// uint8_t stepper2_cmd_buf[20];

// 串口中断配置
void usart_irq_config(void)
{
    // 清除中断标志
    NVIC_ClearPendingIRQ(stepper_motor1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(stepper_motor2_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(MaixCam_INST_INT_IRQN);
    // UART3 不用（超声波已删除）
    NVIC_DisableIRQ(UART_3_INST_INT_IRQN);
    DL_UART_clearInterruptStatus(UART_3_INST, DL_UART_INTERRUPT_RX);

    // 使能需要用到的串口中断
    NVIC_EnableIRQ(stepper_motor1_INST_INT_IRQN);   // 步进电机1
    NVIC_EnableIRQ(stepper_motor2_INST_INT_IRQN);   // 步进电机2
    NVIC_EnableIRQ(MaixCam_INST_INT_IRQN);   // MaixCam 视觉

    // 清除接收中断标志
    DL_UART_clearInterruptStatus(stepper_motor1_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(stepper_motor2_INST, DL_UART_INTERRUPT_RX);
    DL_UART_clearInterruptStatus(MaixCam_INST, DL_UART_INTERRUPT_RX);

    // 设置系统滴答优先级
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// ========== stepper_motor1 中断：步进电机1 反馈接收（可选）==========
void stepper_motor1_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(stepper_motor1_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        uint8_t ch = DL_UART_receiveData(stepper_motor1_INST);
        // 如果需要处理步进电机1的返回数据，在这里添加
        // 例如：stepper1_rx_buf[stepper1_index++] = ch;
        DL_UART_clearInterruptStatus(stepper_motor1_INST, DL_UART_INTERRUPT_RX);
    }
}

// ========== MaixCam 中断：MaixCam 视觉数据接收 ==========
void MaixCam_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(MaixCam_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        uint8_t ch = DL_UART_receiveData(MaixCam_INST);
        // 调用你自己的 MaixCam 数据解析函数
        // MaixCam_Data_Receive(ch);
        DL_UART_clearInterruptStatus(MaixCam_INST, DL_UART_INTERRUPT_RX);
    }
}

// ========== stepper_motor2 中断：步进电机2 反馈接收（可选）==========
void stepper_motor2_INST_IRQHandler(void)
{
    if (DL_UART_getEnabledInterruptStatus(stepper_motor2_INST, DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
    {
        uint8_t ch = DL_UART_receiveData(stepper_motor2_INST);
        // 如果需要处理步进电机2的返回数据，在这里添加
        // 例如：stepper2_rx_buf[stepper2_index++] = ch;
        DL_UART_clearInterruptStatus(stepper_motor2_INST, DL_UART_INTERRUPT_RX);
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

// 针对两个步进电机的便捷发送函数
void Stepper1_Send(uint8_t *cmd, uint32_t len)
{
    UART_SendBytes(stepper_motor1_INST, cmd, len);
}

void Stepper2_Send(uint8_t *cmd, uint32_t len)
{
    UART_SendBytes(stepper_motor2_INST, cmd, len);
}

// printf 重定向到 stepper_motor1（调试用）
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitDataBlocking(stepper_motor1_INST, ch);
    return ch;
}





