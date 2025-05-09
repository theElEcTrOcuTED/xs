//
// Created by benea on 25-4-29.
//

#ifndef ESP_H
#define ESP_H




#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

// 配置参数
//#define ESP01_UART       huart2        // 使用的UART句柄
#define WIFI_SSID       "suibian"    // WiFi名称
#define WIFI_PASSWORD   "suibian123"    // WiFi密码
#define TCP_SERVER_PORT 8080           // TCP服务器端口
#define AT_TIMEOUT_MS   4000           // AT指令默认超时时间
#define MAX_RETRY       3              // 指令重试次数

// 回调函数类型定义
typedef void (*DataReceivedCallback)(uint8_t conn_id, uint8_t* data, uint16_t len);

typedef enum {
    ESP01_OK = 0,
    ESP01_TIMEOUT,
    ESP01_ERROR,
    ESP01_BUSY
  } ESP01_Status;

void ESP01_Init(UART_HandleTypeDef* huart,int isMaster);
ESP01_Status ESP01_SendCommand(const char* cmd, const char* expect, uint32_t timeout);
void ESP01_ProcessReceivedData(void);
void ESP01_SetDataCallback(DataReceivedCallback callback);
void ESP01_SendTCPData(uint8_t conn_id, uint8_t* data, uint16_t len);




#endif //ESP_H
