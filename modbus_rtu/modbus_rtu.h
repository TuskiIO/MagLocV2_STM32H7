#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32h7xx_hal.h"

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256
#define TRX_TIMEOUT 100
#define MB_Broadcast_ID 0x00
#define REPORT_UID_DELAY_FACTOR 100
#define MAX_SENSOR_NUM 256

extern uint8_t rx_buf[];
extern uint8_t tx_buf[];

extern UART_HandleTypeDef hlpuart1;  // LPUART1句柄
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern CRC_HandleTypeDef hcrc;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 通用函数：发送请求帧并通过DMA接收响应帧
 */
HAL_StatusTypeDef Modbus_Master_SendReceive(uint8_t *txFrame, uint16_t txLen, uint8_t *rxFrame);

/**
 * @brief  CMD80 (0x50)：读取从机特定地址处的字节数据
 * @param  slaveId: 从机地址
 * @param  start_reg: 起始地址（1字节）
 * @param  data_length: 读取字节数
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x50 | start_reg | data_length | CRC低 | CRC高 |
 * 响应帧格式: | slaveId | 0x50 | data_length | data1 ... dataN | CRC低 | CRC高 |
 */
HAL_StatusTypeDef Modbus_CMD50_ReadBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length);

/**
 * @brief  CMD81 (0x51)：写从机特定地址处的字节数据（写操作后回显）
 * @param  slaveId: 从机地址
 * @param  start_reg: 起始地址（1字节）
 * @param  data_length: 写入数据字节数
 * @param  pData: 待写入的数据缓冲区
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x51 | start_reg | data_length | data1 ... dataN | CRC低 | CRC高 | 
 * 响应帧格式: 同请求帧（回显）
 */
HAL_StatusTypeDef Modbus_CMD51_WriteBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData);

/**
 * @brief  CMD96 (0x60)：触发一次测量
 * @param  slaveId: 从机地址
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x60 | CRC低 | CRC高 |
 * 响应帧格式: 与请求帧一致
 */
HAL_StatusTypeDef Modbus_CMD60_TriggerMeasurement(uint8_t slaveId);

/**
 * @brief  CMD97 (0x61)：请求从机回报UID
 * @param  UIDCRC8_lower: UIDCRC8低字节
 * @param  UIDCRC8_higher: UIDCRC8高字节
 * @param  delay_lower: 延时低字节
 * @param  delay_higher: 延时高字节
 * @param  return_UID_length: 要求返回的UID长度（单位字节）
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x61 | UIDCRC8_lower | UIDCRC8_higher | delay_lower | delay_higher | return_UID_length | CRC低 | CRC高 |
 * 响应帧格式: | slaveId | 0x61 | UID数据（长度=return_UID_length） | CRC低 | CRC高 |
 */
HAL_StatusTypeDef Modbus_CMD61_BroadcastReportUID(uint8_t UIDCRC8_lower, uint8_t UIDCRC8_higher, uint8_t delay_lower, uint8_t delay_higher, uint8_t return_UID_length);

/**
 * @brief  CMD98 (0x62)：根据UID设置从机地址
 * @param  UID_length: UID的字节数（1、2、4或12）
 * @param  pUID: UID数据缓冲区（按照从低到高存放）
 * @param  new_slave_id: 新的从机地址（合法范围1~247）
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x62 | UID_length | UID数据（高字节先） | new_slave_id | CRC低 | CRC高 |
 * 响应帧格式: 同请求帧（回显）
 */
HAL_StatusTypeDef Modbus_CMD62_BroadcastSetSlaveID(uint8_t UID_length, uint8_t *pUID, uint8_t new_slave_id);

#endif /* MODBUS_RTU_H */