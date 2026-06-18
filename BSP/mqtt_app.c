#include "mqtt_app.h"
#include "cJSON.h"
#include "usart.h"
#include "RS485.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static bool is_sending = false;
void RS485_4G_Send(uint8_t *data, uint16_t len) {
    if(is_sending) return;
    is_sending = true;
    
    HAL_UART_AbortReceive(&huart2); // 停止接收，准备发送
    HAL_UART_Transmit(&huart2, data, len, 1000); // 直接发送
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET); // 等待发送完成
    
    // 发送完立即重新开启 DMA 接收
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    HAL_UART_Receive_DMA(&huart2, rx_buffer2, RX_BUFFER_SIZE_2);
    
    is_sending = false;
}
static void add_fmt(cJSON *params, const char *key, float val, const char *fmt) {
    char buf[32];
    snprintf(buf, sizeof(buf), fmt, val);
    cJSON_AddItemToObject(params, key, cJSON_CreateRaw(buf));
}

void Report_Sensor_Data(void) {
    // 1. 定义局部拷贝变量
    float cl, ox, nh, orp, ph, temp;

    // 2. 只在读取全局变量时保护，将中断关闭时间降到最低 (纳秒级)
    __disable_irq(); 
    cl   =g_SensorData.cl_val;
    ox   =g_SensorData.do_val;
    nh   =g_SensorData.nhn_val;
    orp  =g_SensorData.orp;
    ph   =g_SensorData.ph;
    temp =g_SensorData.temp;
    __enable_irq();

      cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "id", "123");
    cJSON_AddStringToObject(root, "version", "1.0");
    
    cJSON *params = cJSON_CreateObject();
    if (!params) { cJSON_Delete(root); return; }
    
    add_fmt(params, "chlorine", cl,   "{\"value\":%.1f}");
    add_fmt(params, "oxygen",   ox,   "{\"value\":%.1f}");
    add_fmt(params, "ammonia",  nh,   "{\"value\":%.1f}");
    add_fmt(params, "orp",      orp,  "{\"value\":%.0f}");
    add_fmt(params, "ph",       ph,   "{\"value\":%.1f}");
    add_fmt(params, "temp",     temp, "{\"value\":%.2f}");
    
    cJSON_AddItemToObject(root, "params", params);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
			// HAL_UART_Transmit(&huart2, (uint8_t*)json_str, strlen(json_str), 100);
       RS485_4G_Send((uint8_t*)json_str, (uint16_t)strlen(json_str));
        free(json_str);
    }
    cJSON_Delete(root);
}


