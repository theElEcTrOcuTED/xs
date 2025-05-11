//
// Created by benea on 25-4-29.
//

/* esp01_driver.c */
#include "esp.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <usart.h>

#include "delay.h"

// 环形缓冲区定义
#define RING_BUFFER_SIZE 512
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

static UART_HandleTypeDef* esp_huart;
static RingBuffer rx_buffer = {0};
static DataReceivedCallback data_callback = NULL;
int IsMaster;
// 私有函数声明
void UART_RxCpltCallback(UART_HandleTypeDef *huart);
static ESP01_Status WaitForResponse(const char* expect, uint32_t timeout);
static void ParseIPDPacket(uint8_t* data, uint16_t len);

// 初始化模块
void ESP01_Init(UART_HandleTypeDef* huart, int isMaster) {
    esp_huart = huart;
    IsMaster = isMaster;
    // 配置UART接收中断
    //  注意STM32的所有串口公用一个串口回调函数，所以如果需要非阻塞地同时使用多个串口，这里的回调函数需要重写
    //HAL_UART_RegisterCallback(esp_huart, HAL_UART_RX_COMPLETE_CB_ID, UART_RxCpltCallback);
    HAL_UART_Receive_IT(esp_huart, &rx_buffer.buffer[rx_buffer.head], 1);
    /*
    if(isMaster) {
        // 发送AT指令初始化序列
        ESP01_SendCommand("AT", "OK", AT_TIMEOUT_MS);  // 测试AT指令
        ESP01_SendCommand("AT+CWMODE=2", "OK", AT_TIMEOUT_MS); // AP模式
        ESP01_SendCommand("AT+RST", "OK", AT_TIMEOUT_MS);//重启生效
        delay_ms(300);//等待重启
        char cmd[128];

        //主机IP 192.168.4.1
        snprintf(cmd,sizeof(cmd),"AT+CIPAP=\"%s\"","192.168.4.1");
        ESP01_SendCommand(cmd, "OK", AT_TIMEOUT_MS);

        // 设置AP参数，启用WIFI
        snprintf(cmd, sizeof(cmd), "AT+CWSAP=\"%s\",\"%s\",%d,%d", WIFI_SSID, WIFI_PASSWORD,2,4);//通过snprinf格式化字符串构造AT指令.SSID和PW为宏定义，信道为2，加密方式为WPA_WPA2_PSK.
        ESP01_SendCommand(cmd, "OK", 1000);//发送格式化的AT指令字符串。因为这次指令要启用WIFI，超时长些
        ESP01_SendCommand("AT+CIPMUX=1", "OK", AT_TIMEOUT_MS); // 多连接模式
        // 启动TCP服务器
        snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d", TCP_SERVER_PORT);
        ESP01_SendCommand(cmd, "OK", AT_TIMEOUT_MS);
    }
    else {*/
        // 发送AT指令初始化序列
        //DEBUG
        HAL_UART_Transmit(&huart3,"AT-pre",sizeof("AT-pre"),100);



        ESP01_SendCommand("AT", "OK", AT_TIMEOUT_MS);  // 测试AT指令
        //DEBUG
        HAL_UART_Transmit(&huart3,"AT-after",sizeof("AT-after"),100);



        //HAL_UART_Transmit(&huart3,"AT+UART=230400,8,1,0,0",sizeof("AT+UART=230400,8,1,0,0"),100);
        if(ESP01_OK!=ESP01_SendCommand("AT+UART=230400,8,1,0,0", "OK", AT_TIMEOUT_MS)) {
            HAL_UART_Transmit(&huart3,"UARTSET Command Timeout",sizeof("UARTSET Command Timeout"),100);
        }


        if(ESP01_OK!=ESP01_SendCommand("AT+CWMODE=1", "OK", AT_TIMEOUT_MS)) {
            HAL_UART_Transmit(&huart3,"CWMODE Command Timeout",sizeof("CWMODE Command Timeout"),100);
        }// STA模式
     //   if(ESP01_TIMEOUT==ESP01_SendCommand("AT+RST", "OK", AT_TIMEOUT_MS))//重启生效
    //    {
     //       HAL_UART_Transmit(&huart3,"RST Command Timeout",sizeof("RST Command Timeout"),100);
    //    }
     //   delay_ms(1000);//等待重启
        if(ESP01_OK!=ESP01_SendCommand("AT+CIPMUX=0", "OK", AT_TIMEOUT_MS)) // 单连接模式
        {
            HAL_UART_Transmit(&huart3,"CIPMUX Command Timeout",sizeof("CIPMUX Command Timeout"),100);
        }
        if(ESP01_OK!=ESP01_SendCommand("AT+CIPMODE=0", "OK", AT_TIMEOUT_MS)) {
            HAL_UART_Transmit(&huart3,"CIPMODE Command Error",sizeof("CIPMODE Command Error"),100);
        }
        // 连接WiFi
        char cmd[128];
        //主机IP 192.168.4.2 - 192.168.4.4
        snprintf(cmd,sizeof(cmd),"AT+CIPSTA=\"%s\"","192.168.4.2");
        if(ESP01_OK!=ESP01_SendCommand(cmd, "OK", AT_TIMEOUT_MS)) {
            HAL_UART_Transmit(&huart3,"CIPSTA Command Timeout",sizeof("CIPSTA Command Timeout"),100);
        }
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
        if(ESP01_OK!=ESP01_SendCommand(cmd, "OK", 10000))//连接指定WIFI。因为这次指令要启用WIFI，所以超时长些
        {
            HAL_UART_Transmit(&huart3,"CWJAP Command Timeout",sizeof("CWJAP Command Timeout"),100);
        }
        // 连接到指定TCP服务器,IP 192.168.4.1，端口为宏定义
        snprintf(cmd,sizeof(cmd),"AT+CIPSTART=\"TCP\",\"192.168.4.1\",%d",TCP_SERVER_PORT);
        if(ESP01_OK!=ESP01_SendCommand(cmd, "OK", 10000)) {
            HAL_UART_Transmit(&huart3,"CIPSTART Command Timeout",sizeof("CIPSTART Command Timeout"),100);
        }
            //HAL_UART_Transmit(esp_huart,"AT+CIPSEND\r\n",sizeof("AT+CIPSEND\r\n"),100);
    /*}*/

}

// 发送AT指令
ESP01_Status ESP01_SendCommand(const char* cmd, const char* expect, uint32_t timeout) {
    uint8_t retry = MAX_RETRY;
    ESP01_Status status = ESP01_ERROR;
    while(retry--) {
        HAL_UART_Transmit(esp_huart, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
        //DEBUG
      //  HAL_UART_Transmit(&huart3,(uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
        HAL_UART_Transmit(esp_huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // 添加回车换行
        //DEBUG
       // HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY); // 添加回车换行
        //[DEBUG]
        //HAL_UART_Transmit(&huart3,"beforewaitforresponse",sizeof("beforewaitforresponse"),100);
        status = WaitForResponse(expect, timeout);
        //[DEBUG]
        //HAL_UART_Transmit(&huart3,"afterwaitforresponse",sizeof("afterwaitforresponse"),100);
        if(status == ESP01_OK) break;
        delay_ms(500); // 重试前延迟
    }
    return status;
}

// 等待响应
/*
static ESP01_Status WaitForResponse(const char* expect, uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    uint16_t bytes_available = 0;
    uint8_t response[256] = {0};
    uint16_t index = 0;

    while((HAL_GetTick() - start) < timeout) {//！！！这里有问题
        bytes_available = (rx_buffer.head - rx_buffer.tail) % RING_BUFFER_SIZE;


        //[DEBUG]
        char cmd[16];
        //主机IP 192.168.4.2 - 192.168.4.4
        snprintf(cmd,sizeof(cmd),"%d",HAL_GetTick());
        HAL_UART_Transmit(&huart3,cmd,sizeof(cmd),100);

        while(bytes_available-- > 0) {
            response[index++] = rx_buffer.buffer[rx_buffer.tail];
            rx_buffer.tail = (rx_buffer.tail + 1) % RING_BUFFER_SIZE;

            if(strstr((char*)response, expect) != NULL) {
                return ESP01_OK;
            }
            if(strstr((char*)response, "ERROR") != NULL) {
                return ESP01_ERROR;
            }
        }
        //delay_ms(5);
    }
    return ESP01_TIMEOUT;
}*/

// 等待响应（不依赖HAL_GetTick的版本）
static ESP01_Status WaitForResponse(const char* expect, uint32_t timeout_ms) {
    /*
    uint32_t max_retry = timeout_ms / 10;  // 按每次循环约10ms估算
    uint16_t bytes_available = 0;
    uint8_t response[256] = {0};
    uint16_t index = 0;

    for (uint32_t retry = 0; retry < max_retry; retry++) {

        // 检查环形缓冲区数据
        bytes_available = (rx_buffer.head - rx_buffer.tail) % RING_BUFFER_SIZE;

        // 处理所有可用字节
        while (bytes_available-- > 0) {
            if(rx_buffer.buffer[rx_buffer.tail] != '\0') {
                response[index++] = rx_buffer.buffer[rx_buffer.tail];
                rx_buffer.tail = (rx_buffer.tail + 1) % RING_BUFFER_SIZE;
                //HAL_UART_Transmit(&huart3,"\n[WARNING] ending detected",sizeof("\n[WARNING] ending detected"),100);
            }
            //[DEBUG]
            //HAL_UART_Transmit(&huart3,"waitingforresponse", sizeof("waitingforresponse"), 100);
            // 检测目标响应
            if (strstr((char*)response, expect) != NULL) {
                return ESP01_OK;
            }
            if (strstr((char*)response, "ERROR") != NULL) {
                return ESP01_ERROR;
            }
        }
       // HAL_UART_Transmit(&huart3,"\nresponse:", sizeof("\nresponse:"), 100);
       // HAL_UART_Transmit(&huart3,response, 256, 100);

        // 简单延时（根据CPU频率调整循环次数）

        volatile uint32_t delay = 5000;  // 实测校准该值以接近1ms延时
        while (delay--);
        delay_ms(10);

        // [可选] 调试输出（确认循环频率）
        // HAL_UART_Transmit(&huart3, ".", 1, 100);
    }
    return ESP01_TIMEOUT;*/
        uint32_t max_retry = timeout_ms / 5;
        uint16_t bytes_available = 0;
        uint8_t response[256] = {0};
        uint16_t index = 0;

        for (uint32_t retry = 0; retry < max_retry; retry++) {
            bytes_available = (rx_buffer.head - rx_buffer.tail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

            while (bytes_available-- > 0) {
                // 检查索引是否越界
                if (index < sizeof(response) - 1) {
                    response[index++] = rx_buffer.buffer[rx_buffer.tail];
                    response[index] = '\0'; // 添加终止符
                }
                // 无论是否存储，均移动tail指针以消费数据
                rx_buffer.tail = (rx_buffer.tail + 1) % RING_BUFFER_SIZE;

                // 检查匹配
                if (strstr((char*)response, expect) != NULL) {
                    return ESP01_OK;
                }
                if (strstr((char*)response, "ERROR") != NULL) {
                    return ESP01_ERROR;
                }
            }
             //HAL_UART_Transmit(&huart3,"\nresponse:", sizeof("\nresponse:"), 100);
             //HAL_UART_Transmit(&huart3,response, 256, 100);
            delay_ms(5);
        }
        //HAL_UART_Transmit(&huart3,"\nRbuffer:", sizeof("\nRbuffer:"), 100);
        //HAL_UART_Transmit(&huart3,rx_buffer.buffer, 512, 500);
        return ESP01_TIMEOUT;
}

// 接收中断回调的重写
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    //[DEBUG]
    //HAL_UART_Transmit(&huart3,"Entered USART2 Interrupt",sizeof("Entered USART2 Interrupt"),100);
    if(huart == esp_huart) {
        rx_buffer.head = (rx_buffer.head + 1) % RING_BUFFER_SIZE;
        HAL_UART_Receive_IT(huart, &rx_buffer.buffer[rx_buffer.head], 1);
    }
}

// 数据处理主循环
void ESP01_ProcessReceivedData(void) {
    uint16_t bytes_available = (rx_buffer.head - rx_buffer.tail) % RING_BUFFER_SIZE;
    static uint8_t packet_buffer[256];
    static uint16_t packet_index = 0;

    while(bytes_available--) {
        uint8_t data = rx_buffer.buffer[rx_buffer.tail];
        rx_buffer.tail = (rx_buffer.tail + 1) % RING_BUFFER_SIZE;

        // 检测IPD数据包
        if(data == '+') {
            if(packet_index >= 5 &&
                memcmp(&packet_buffer[packet_index-4], "+IPD", 4) == 0) {
                ParseIPDPacket(packet_buffer, packet_index);
                packet_index = 0;
            }
        }

        packet_buffer[packet_index++] = data;
        if(packet_index >= sizeof(packet_buffer)) {
            packet_index = 0; // 防止溢出
        }
        HAL_UART_Transmit(&huart3,packet_buffer,256,150);
    }
}

// 解析IPD数据包
static void ParseIPDPacket(uint8_t* data, uint16_t len) {
    // 格式: +IPD,<conn_id>,<len>:<data>
    HAL_UART_Transmit(&huart3,"+IPD Packet detected",sizeof("+IPD Packet detected"),150);

    char* ptr = strstr((char*)data, "+IPD");
    if(ptr == NULL) return;
    if(IsMaster) {
        uint8_t conn_id = atoi(ptr + 5); // 跳过"+IPD,"
        char* len_start = strchr(ptr, ',') + 1;
        uint16_t data_len = atoi(len_start);
        char* data_start = strchr(ptr, ':') + 1;

        if(data_callback != NULL && data_len > 0) {
            data_callback(conn_id, (uint8_t*)data_start, data_len);
        }
    }
    else {//从机的单连接模式下+IPD数据包格式有所不同
        uint16_t data_len = atoi(ptr + 5); // 跳过"+IPD,"
        char* data_start = strchr(ptr, ':') + 1;

        if(data_callback != NULL && data_len > 0) {
            data_callback(0, (uint8_t*)data_start, data_len);
        }
    }
}

// 设置数据接收回调
void ESP01_SetDataCallback(DataReceivedCallback callback) {
    data_callback = callback;
}

// 发送TCP数据
void ESP01_SendTCPData(uint8_t conn_id, uint8_t* data, uint16_t len) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", len);
    ESP01_Status st = ESP01_SendCommand(cmd, ">", 1000);
    if(st == ESP01_OK) {
        HAL_UART_Transmit_DMA(esp_huart, data, len);
    }
    else if(st==ESP01_ERROR) {
        HAL_UART_Transmit(&huart3,"CIPSEND Command Error",sizeof("CIPSEND Command Error"),100);
    }
    else if(st==ESP01_TIMEOUT) {
        HAL_UART_Transmit(&huart3,"CIPSEND Command Timeout",sizeof("CIPSEND Command Error"),100);
    }
   // HAL_UART_Transmit(esp_huart, data, len, 1000);
}