#ifndef __RS485_H
#define __RS485_H
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define RS485_RX_BUFFER_SIZE 64
#define  RX_BUFFER_SIZE_2 64
extern uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];


// 严格根据山东仁科传感器说明书定义的 Modbus 站号
#define ADDR_DO          0x01   // 溶解氧传感器站号
#define ADDR_CL          0x02   // 一体式余氯传感器站号
#define ADDR_NHN         0x04   // 氨氮传感器485型站号
#define ADDR_ORP         0x03   // ORP传感器站号
#define ADDR_PH          0x05   // pH传感器站号
#define ADDR_TEMP        0x06   // 温度传感器站号（如果独立）
/* --- 动作控制宏封装 --- */
#define BEEP_ON()           HAL_GPIO_WritePin(ALARM_BEEP_GPIO_Port, ALARM_BEEP_Pin, GPIO_PIN_SET)
#define BEEP_OFF()          HAL_GPIO_WritePin(ALARM_BEEP_GPIO_Port, ALARM_BEEP_Pin, GPIO_PIN_RESET)

#define LED_RED_ON()        HAL_GPIO_WritePin(LED_ALARM_RED_GPIO_Port, LED_ALARM_RED_Pin, GPIO_PIN_SET)
#define LED_RED_OFF()       HAL_GPIO_WritePin(LED_ALARM_RED_GPIO_Port, LED_ALARM_RED_Pin, GPIO_PIN_RESET)

#define LED_YELLOW_ON()     HAL_GPIO_WritePin(LED_ALARM_YELLOW_GPIO_Port, LED_ALARM_YELLOW_Pin, GPIO_PIN_SET)
#define LED_YELLOW_OFF()    HAL_GPIO_WritePin(LED_ALARM_YELLOW_GPIO_Port, LED_ALARM_YELLOW_Pin, GPIO_PIN_RESET)

#define LED_GREEN_ON()      HAL_GPIO_WritePin(LED_ALARM_GREEN_GPIO_Port, LED_ALARM_GREEN_Pin, GPIO_PIN_SET)
#define LED_GREEN_OFF()     HAL_GPIO_WritePin(LED_ALARM_GREEN_GPIO_Port, LED_ALARM_GREEN_Pin, GPIO_PIN_RESET)

#define OXYGEN_PUMP_ON()    HAL_GPIO_WritePin(RELAY_OXYGEN_PUMP_GPIO_Port, RELAY_OXYGEN_PUMP_Pin, GPIO_PIN_SET)
#define OXYGEN_PUMP_OFF()   HAL_GPIO_WritePin(RELAY_OXYGEN_PUMP_GPIO_Port, RELAY_OXYGEN_PUMP_Pin, GPIO_PIN_RESET)

#define DOSING_PUMP_ON()    HAL_GPIO_WritePin(RELAY_DOSING_PUMP_GPIO_Port, RELAY_DOSING_PUMP_Pin, GPIO_PIN_SET)
#define DOSING_PUMP_OFF()   HAL_GPIO_WritePin(RELAY_DOSING_PUMP_GPIO_Port, RELAY_DOSING_PUMP_Pin, GPIO_PIN_RESET)

#define RS485_TX()          HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_SET)
#define RS485_RX()          HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET)

bool Send_to_Slave(uint8_t addr, uint8_t reg_addr,uint16_t reg_cnt);
uint16_t Calculate_CRC16(uint8_t *data, uint16_t length);

static bool Modbus_CheckResponse(uint8_t expected_addr, uint8_t expected_data_len);
void Error_LED_On(void);
void Normal_LED_On(void);
void Error_LED_Off(void);
void Normal_LED_Off(void);
bool Three_Take_Two_Judgment(void);
void RS485_Poll_Slaves(void); // 轮询从机函数
void RS485_LED_Control_Task(void); // LED控制任务（处理自动熄灭）
void RS485_TxDone(void); // DMA 发送完成处理

// 根据南美白对虾养殖规范与系统设计设定的安全红线阈值
#define THRESHOLD_DO_MIN     4.0f    // 溶解氧安全红线下限 (mg/L)，低于此值触发强制增氧
#define THRESHOLD_DO_TARGET  6.0f    // 溶解氧目标恢复上限 (mg/L)，高于此值停止增氧以节能
#define THRESHOLD_CL_MIN     0.02f   // 维持消杀所需的有效余氯下限 (mg/L)，低于此值启动蠕动泵
#define THRESHOLD_CL_MAX     0.2f    // 余氯安全红线上限值 (mg/L)，高于此值属于强毒性危险，强制停泵
#define THRESHOLD_NHN_MAX    0.6f    // 氨氮安全红线上限 (mg/L)，超过则触发危险告警
#define THRESHOLD_ORP_MAX    620.0f  // ORP过高 (mV)，超过则停蠕动泵
#define THRESHOLD_PH_MIN     5.5f    // pH最低值（微酸性，次氯酸最优）
#define THRESHOLD_PH_MAX     8.5f    // pH最高值


// 485 异步非阻塞轮询状态机枚举
typedef enum {
    STATE_SEND_DO = 0,
    STATE_WAIT_DO,
    STATE_SEND_CL,
    STATE_WAIT_CL,
    STATE_SEND_NHN,
    STATE_WAIT_NHN,
    STATE_SEND_ORP,
    STATE_WAIT_ORP,
    STATE_SEND_PH,
    STATE_WAIT_PH,
    STATE_SEND_TEMP,
    STATE_WAIT_TEMP,
    STATE_POLL_DELAY
} PollState_t;

// 存储三个传感器的物理世界实际测量值
typedef struct {
    float do_val;    // 溶解氧 (mg/L)
    float cl_val;    // 余氯 (mg/L)
    float nhn_val;   // 氨氮 (mg/L)
  	float orp;			//氧化还原电位
		float ph;				//PH值
		float temp; //温度
} SensorData_t;

// 物联网系统控制与传感器运行状态结构体（直接用于发送给 Mini Program）
typedef struct {
    uint8_t oxygen_pump_status; // 增氧泵当前运行状态: 0-关闭, 1-开启
    uint8_t dosing_pump_status; // 蠕动泵当前运行状态: 0-关闭, 1-开启
    uint8_t alarm_mask;         // 告警掩码: bit0-溶解氧异常, bit1-余氯超标, bit2-氨氮超标
} SystemControl_t;
extern SensorData_t g_SensorData ;
extern uint8_t RS485_RX_BUF[RS485_RX_BUFFER_SIZE];
extern volatile uint16_t RS485_RX_CNT;
extern volatile uint8_t RS485_RX_FLAG;
extern uint8_t RX_BUFFER_2[RX_BUFFER_SIZE_2];
#endif /* __RS485_H */

