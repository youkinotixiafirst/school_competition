#ifndef __BLUETOOTH_APP_H__
#define __BLUETOOTH_APP_H__

#include <stdint.h>

// 发送用户数据到手机 APP
void bluetooth_app_send(float user1, float user2, float user3, float user4,
                        float user5, float user6, float user7, float user8);

// 接收字节解析（需要在 UART 中断中调用）
void bluetooth_app_prase(uint8_t byte);

// 工具函数：发送 N 个字节

// 工具函数：float 与字节数组互转
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript);
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue);

// 全局数据（供外部读取接收到的数据）
extern float bt_user_data[8];
extern uint16_t bluetooth_ppm[8];
extern uint8_t bt_update_flag;

#endif