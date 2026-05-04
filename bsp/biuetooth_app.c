#include "bluetooth_app.h"
#include "nuart.h"       // 你已有的发送函数
#include <string.h>      // for memset 等
#include <stdbool.h>

// 联合体用于大小端转换
static union {
    unsigned char floatByte[4];
    float floatValue;
} FloatUnion;

#define PACK_HEAD  0xA5
#define PACK_TAIL  0x5A

static uint8_t bt_app_output_buf[50];
static uint8_t bt_app_input_buf[50];

// 全局数据
float bt_user_data[8] = {0};
uint16_t bluetooth_ppm[8] = {0};
uint8_t bt_update_flag = 0;

// float 与字节转换（与之前相同）
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
{
    FloatUnion.floatValue = 2.0f;
    if(FloatUnion.floatByte[0] == 0) // 小端
    {
        FloatUnion.floatValue = *FloatValue;
        Byte[Subscript]   = FloatUnion.floatByte[0];
        Byte[Subscript+1] = FloatUnion.floatByte[1];
        Byte[Subscript+2] = FloatUnion.floatByte[2];
        Byte[Subscript+3] = FloatUnion.floatByte[3];
    }
    else
    {
        FloatUnion.floatValue = *FloatValue;
        Byte[Subscript]   = FloatUnion.floatByte[3];
        Byte[Subscript+1] = FloatUnion.floatByte[2];
        Byte[Subscript+2] = FloatUnion.floatByte[1];
        Byte[Subscript+3] = FloatUnion.floatByte[0];
    }
}

void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
{
    FloatUnion.floatByte[0] = Byte[Subscript];
    FloatUnion.floatByte[1] = Byte[Subscript+1];
    FloatUnion.floatByte[2] = Byte[Subscript+2];
    FloatUnion.floatByte[3] = Byte[Subscript+3];
    *FloatValue = FloatUnion.floatValue;
}

// 发送函数（现在直接使用你的 UART_SendBytes）
void bluetooth_app_send(float user1, float user2, float user3, float user4,
                        float user5, float user6, float user7, float user8)
{
    uint8_t _cnt = 0, sum = 0;
    bt_app_output_buf[_cnt++] = PACK_HEAD;
    bt_app_output_buf[_cnt++] = 32;
    Float2Byte(&user1, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user2, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user3, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user4, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user5, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user6, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user7, bt_app_output_buf, _cnt); _cnt += 4;
    Float2Byte(&user8, bt_app_output_buf, _cnt); _cnt += 4;

    for(uint16_t i = 1; i < _cnt; i++)
        sum += bt_app_output_buf[i];
    bt_app_output_buf[_cnt++] = sum;
    bt_app_output_buf[_cnt++] = PACK_TAIL;

    // 使用你的发送函数，发送到 UART2
    UART_SendBytes(UART_2_INST, bt_app_output_buf, _cnt);
}

// 解析函数（完全不变）
void bluetooth_app_prase(uint8_t byte)
{
    uint8_t sum = 0;
    static uint8_t state = 0, cnt = 0, data_len = 0;

    if(state == 0 && byte == PACK_HEAD)
    {
        state = 1;
        bt_app_input_buf[0] = byte;
    }
    else if(state == 1 && byte < 50)
    {
        state = 2;
        data_len = byte;
        bt_app_input_buf[1] = byte;
        cnt = 0;
    }
    else if(state == 2 && data_len > 0)
    {
        data_len--;
        bt_app_input_buf[2 + cnt++] = byte;
        if(data_len == 0) state = 3;
    }
    else if(state == 3)   // 接收校验和
    {
        state = 4;
        bt_app_input_buf[2 + cnt++] = byte;
    }
    else if(state == 4 && byte == PACK_TAIL)
    {
        bt_app_input_buf[2 + cnt++] = byte;
        state = 0;
        sum = 0;
        for(uint16_t i = 1; i < cnt; i++)
            sum += bt_app_input_buf[i];
        if(sum == bt_app_input_buf[cnt])   // 校验通过
        {
            Byte2Float(bt_app_input_buf, 2,  &bt_user_data[0]);
            Byte2Float(bt_app_input_buf, 6,  &bt_user_data[1]);
            Byte2Float(bt_app_input_buf, 10, &bt_user_data[2]);
            Byte2Float(bt_app_input_buf, 14, &bt_user_data[3]);
            Byte2Float(bt_app_input_buf, 18, &bt_user_data[4]);
            Byte2Float(bt_app_input_buf, 22, &bt_user_data[5]);
            Byte2Float(bt_app_input_buf, 26, &bt_user_data[6]);
            Byte2Float(bt_app_input_buf, 30, &bt_user_data[7]);

            for(uint16_t j = 0; j < 8; j++)
            {
                bluetooth_ppm[j] = (int)bt_user_data[j];
                bt_update_flag = 1;
            }
        }
    }
    else
        state = 0;
}