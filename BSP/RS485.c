#include "RS485.h"
// 分配全局共享变量的内存空间
SensorData_t g_SensorData = {2.0f, 0.0f, 0.0f, 0.0f, 7.0f, 25.0f};
SystemControl_t g_SystemControl = {0, 0, 0};
static uint8_t tx_buffer[9];           // DMA 发送缓冲，必须为静态或全局
uint8_t RS485_RX_BUF[RS485_RX_BUFFER_SIZE] = {0};
uint8_t RX_BUFFER_2[RX_BUFFER_SIZE_2] = {0};
volatile uint16_t RS485_RX_CNT = 0;
volatile uint8_t RS485_RX_FLAG = 0;
static PollState_t poll_state = STATE_SEND_DO;
static uint32_t state_timer_ms = 0; 
static volatile bool rs485_tx_busy = false; // 发送忙标志


#define TIMEOUT_MS       300   // 单个传感器最大响应超时时间 300ms
#define INTERVAL_MS      1000  // 每一轮查完后，总线静默 1000ms（降低水体扰动与功耗）
void WaterQuality_Control_Logic(void);

uint16_t Calculate_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i]; // 将数据与CRC寄存器异或
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { // 如果最低位为1
                crc >>= 1; // 右移一位
                crc ^= 0xA001; // 与多项式异或
            } else {
                crc >>= 1; // 直接右移一位
            }
        }
    }
    return crc;
}

bool Send_to_Slave(uint8_t addr, uint8_t reg_addr,uint16_t reg_cnt)
{
    if (rs485_tx_busy) {
        return false; // 上一次发送尚未完成，避免重叠发送
    }

       tx_buffer[0] = addr;
    tx_buffer[1] = 0x03; // 读保持寄存器功能码
    tx_buffer[2] = (uint8_t)(reg_addr >> 8);
    tx_buffer[3] = (uint8_t)(reg_addr & 0xFF);
    tx_buffer[4] = (uint8_t)(reg_cnt >> 8);
    tx_buffer[5] = (uint8_t)(reg_cnt & 0xFF);
    
    uint16_t crc = Calculate_CRC16(tx_buffer, 6);
    tx_buffer[6] = (uint8_t)(crc & 0xFF);         // CRC 低位在前
    tx_buffer[7] = (uint8_t)(crc >> 8);           // CRC 高位在后
    

   
    HAL_UART_AbortReceive(&huart1); // 停止当前 DMA 接收，避免与发送冲突

    if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, 8) != HAL_OK) {
        HAL_UART_Receive_DMA(&huart1, rx_buffer, RS485_RX_BUFFER_SIZE);
        return false;
    }

    rs485_tx_busy = true;
    return true;
}




static bool Modbus_CheckResponse(uint8_t expected_addr, uint8_t expected_data_len) {
    uint16_t min_len = 5 + expected_data_len; // 最小物理帧长 = 站号+功能码+长度码+数据+CRC(2字节)
    if (RS485_RX_CNT < min_len) return false;
    
    if (RS485_RX_BUF[0] != expected_addr) return false;
    if (RS485_RX_BUF[1] != 0x03) return false;
    if (RS485_RX_BUF[2] != expected_data_len) return false;
    
    uint16_t calc_crc = Calculate_CRC16 (RS485_RX_BUF, RS485_RX_CNT - 2);
    uint16_t rx_crc = (RS485_RX_BUF[RS485_RX_CNT - 1] << 8) | RS485_RX_BUF[RS485_RX_CNT - 2];
    
    return (calc_crc == rx_crc);
}


void RS485_TxDone(void)
{
    rs485_tx_busy = false;
}

static float Parse_Float_BigEndian(uint8_t *buf) {
    union {
        float f_val;
        uint8_t b[4];
    } data;
    // 严格对应工业级 Modbus 大端字节排列规则
    data.b[3] = buf[0];
    data.b[2] = buf[1];
    data.b[1] = buf[2];
    data.b[0] = buf[3];
    return data.f_val;
}

/**
 * @brief 核心多传感器异步轮询状态机（由 TIM1 1ms 定时器中断推着走）
 */
void RS485_Poll_Slaves(void) {
    uint16_t raw_temp;
    
    switch (poll_state) {
        /* ------------------ 1. 轮询溶解氧变送器 ------------------ */
        case STATE_SEND_DO:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            // 查阅《荧光法溶解氧变送器手册》：溶解氧数值在 0x0002 寄存器，占 2 个寄存器 (4字节浮点数)
            Send_to_Slave(ADDR_DO, 0x0002, 2);
            state_timer_ms = 0;
            poll_state = STATE_WAIT_DO;
            break;
            
        case STATE_WAIT_DO:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_DO, 4)) {
                    g_SensorData.do_val = Parse_Float_BigEndian(&RS485_RX_BUF[3]);
                }
                state_timer_ms = 0;
                poll_state = STATE_SEND_CL; // 切换至余氯查询
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                poll_state = STATE_SEND_CL; // 超时容错
            }
            break;
            
        /* ------------------ 2. 轮询一体式余氯变送器 ------------------ */
        case STATE_SEND_CL:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            // 查阅《一体式余氯变送器手册》：余氯数值在 0x0002 寄存器，占 2 个寄存器 (4字节浮点数)
            Send_to_Slave(ADDR_CL, 0x0002, 2);
            state_timer_ms = 0;
            poll_state = STATE_WAIT_CL;
            break;
            
        case STATE_WAIT_CL:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_CL, 4)) {
                    g_SensorData.cl_val = Parse_Float_BigEndian(&RS485_RX_BUF[3]);
                }
                state_timer_ms = 0;
                poll_state = STATE_SEND_NHN; // 切换至氨氮查询
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                poll_state = STATE_SEND_NHN; 
            }
            break;
            
        /* ------------------ 3. 轮询氨氮变送器(485型) ------------------ */
        case STATE_SEND_NHN:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            // 查阅《氨氮变送器手册》：数据在 0x0002 读 1 个寄存器(2字节十六进制)，实际值需除以 100
            Send_to_Slave(ADDR_NHN, 0x0002, 1); 
            state_timer_ms = 0;
            poll_state = STATE_WAIT_NHN;
            break;
            
        case STATE_WAIT_NHN:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_NHN, 2)) {
                    raw_temp = (RS485_RX_BUF[3] << 8) | RS485_RX_BUF[4];
                    g_SensorData.nhn_val = (float)raw_temp / 100.0f; 
                }
                state_timer_ms = 0;
                poll_state = STATE_POLL_DELAY; // 本轮查完，进入周期等待
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                state_timer_ms = 0;
                poll_state = STATE_POLL_DELAY; 
            }
            break;
            
        /* ------------------ 4. 轮询ORP传感器 ------------------ */
        case STATE_SEND_ORP:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            Send_to_Slave(ADDR_ORP, 0x0002, 2);
            state_timer_ms = 0;
            poll_state = STATE_WAIT_ORP;
            break;

        case STATE_WAIT_ORP:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_ORP, 4)) {
                    g_SensorData.orp = Parse_Float_BigEndian(&RS485_RX_BUF[3]);
                }
                state_timer_ms = 0;
                poll_state = STATE_SEND_PH;
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                poll_state = STATE_SEND_PH;
            }
            break;

        /* ------------------ 5. 轮询pH传感器 ------------------ */
        case STATE_SEND_PH:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            Send_to_Slave(ADDR_PH, 0x0002, 2);
            state_timer_ms = 0;
            poll_state = STATE_WAIT_PH;
            break;

        case STATE_WAIT_PH:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_PH, 4)) {
                    g_SensorData.ph = Parse_Float_BigEndian(&RS485_RX_BUF[3]);
                }
                state_timer_ms = 0;
                poll_state = STATE_SEND_TEMP;
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                poll_state = STATE_SEND_TEMP;
            }
            break;

        /* ------------------ 6. 轮询温度传感器 ------------------ */
        case STATE_SEND_TEMP:
            RS485_RX_FLAG = 0;
            RS485_RX_CNT = 0;
            Send_to_Slave(ADDR_TEMP, 0x0002, 2);
            state_timer_ms = 0;
            poll_state = STATE_WAIT_TEMP;
            break;

        case STATE_WAIT_TEMP:
            state_timer_ms++;
            if (RS485_RX_FLAG == 1) {
                if (Modbus_CheckResponse(ADDR_TEMP, 4)) {
                    g_SensorData.temp = Parse_Float_BigEndian(&RS485_RX_BUF[3]);
                }
                state_timer_ms = 0;
                poll_state = STATE_POLL_DELAY;
            }
            else if (state_timer_ms > TIMEOUT_MS) {
                poll_state = STATE_POLL_DELAY;
            }
            break;

        /* ------------------ 7. 周期停顿等待与数据逻辑闭环 ------------------ */
        case STATE_POLL_DELAY:
            state_timer_ms++;
            if (state_timer_ms >= INTERVAL_MS) {
                // 【核心联动】一整轮新鲜数据全部冲刷入内存后，立即就地进行水质控制判别
                WaterQuality_Control_Logic();

                state_timer_ms = 0;
                poll_state = STATE_SEND_DO; // 开启下一轮健康循环
            }
            break;
    }
}
void WaterQuality_Control_Logic(void) {
    /* 1. 增氧泵联动逻辑：DO < 4mg/L 启动增氧泵 [cite: 15, 26, 53] */
    if (g_SensorData.do_val < THRESHOLD_DO_MIN) {
        OXYGEN_PUMP_ON();
        g_SystemControl.oxygen_pump_status = 1;
    } else if (g_SensorData.do_val >= THRESHOLD_DO_TARGET) {
        OXYGEN_PUMP_OFF();
        g_SystemControl.oxygen_pump_status = 0;
    }

    /* 2. 次氯酸精准投加逻辑：满足余氯低且 ORP/氨氮正常时加药 [cite: 14, 25, 27, 53] */
    if (g_SensorData.cl_val < THRESHOLD_CL_MIN && 
        g_SensorData.orp <= THRESHOLD_ORP_MAX && 
        g_SensorData.nhn_val <= THRESHOLD_NHN_MAX) {
        DOSING_PUMP_ON();
        g_SystemControl.dosing_pump_status = 1;
    } else {
        DOSING_PUMP_OFF(); // ORP过高或氨氮超标时禁投 [cite: 25, 27, 53]
        g_SystemControl.dosing_pump_status = 0;
    }

    /* 3. 多参数告警与灯光分级  */
    // 计算告警掩码
    g_SystemControl.alarm_mask = 0;
    if (g_SensorData.do_val < THRESHOLD_DO_MIN)  g_SystemControl.alarm_mask |= (1 << 0);
    if (g_SensorData.cl_val > THRESHOLD_CL_MAX)  g_SystemControl.alarm_mask |= (1 << 1);
    if (g_SensorData.nhn_val > THRESHOLD_NHN_MAX) g_SystemControl.alarm_mask |= (1 << 2);
    if (g_SensorData.orp > THRESHOLD_ORP_MAX)    g_SystemControl.alarm_mask |= (1 << 3);

    /* 4. 硬件告警联动执行 [cite: 27, 53] */
    if (g_SystemControl.alarm_mask != 0) {
        BEEP_ON();
        LED_RED_ON();      // 红色警告 
        LED_YELLOW_OFF();
        LED_GREEN_OFF();
    } else if (g_SensorData.nhn_val > 0.2) {
        BEEP_OFF();
        LED_RED_OFF();
        LED_YELLOW_ON();   // 黄灯提醒 
        LED_GREEN_OFF();
    } else {
        BEEP_OFF();
        LED_RED_OFF();
        LED_YELLOW_OFF();
        LED_GREEN_ON();    // 正常绿灯 
    }
}

