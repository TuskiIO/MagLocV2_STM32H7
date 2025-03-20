#include "modbus_rtu.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal_crc.h"
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

__attribute__((aligned(32))) uint8_t rx_buf[RX_BUF_SIZE] __attribute__((section(".RxLpuart")));
__attribute__((aligned(32))) uint8_t tx_buf[TX_BUF_SIZE] __attribute__((section(".TxLpuart")));
uint16_t rx_size = 0;
uint16_t sensor_0xF7_cnt = 0;
uint8_t *sensor_UID[MAX_SENSOR_NUM];
uint8_t txFrame[256] = {0};
uint8_t rxFrame[256] = {0};
volatile uint32_t RS485_RX_flag;
extern TIM_HandleTypeDef htim2;

#if RS485_RX_USE_RTOS_SEMAPHORE
static osSemaphoreId_t modbusSemaphoreHandle = NULL;
const osSemaphoreAttr_t modbusSemaphore_attr = {
    .name = "ModbusSem"
};
#endif

//微秒级延时，最大1s
void delay_us(uint16_t us) {
    uint16_t start = TIM4->CNT;
    uint16_t ticks = us; // 240MHz时钟
    
    while ((TIM4->CNT - start) < ticks) {
        __NOP();
    }
}

/**
 * @brief 堵塞等待RS485_RX_flag置1，替代osSemaphoreAcquire功能
 * @param timeout_ms        堵塞等待时间，超时输出timeout_ms
 * @retval osOK             接收到RS485_RX_flag == 1
 * @retval osErrorTimeout   timeout_ms时间内未收到RS485_RX_flag == 1
 */
static osStatus_t RS485_RX_flag_Acquire(uint32_t timeout_ms){
    RS485_RX_flag = 0;
    uint32_t start = HAL_GetTick();
    while (RS485_RX_flag == 0) {
        if((uint32_t)(HAL_GetTick() - start) > timeout_ms){
            return osErrorTimeout;
        }
        __NOP();
    }
    return osOK;
}

/**
 * @brief UART接收完成回调函数（中断方式）
 *        当接收完成后，中断中调用此回调函数释放信号量，
 *        通知等待的任务数据已接收完成。
 * @param huart UART句柄
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) 
{
    if(huart->Instance == hlpuart1.Instance)
    {   
        rx_size = Size;
        //刷新缓存，避免优化导致rx_buf不更新
        SCB_InvalidateDCache_by_Addr((uint32_t *)rx_buf, RX_BUF_SIZE);
        // HAL_UART_Transmit(&hlpuart1, rx_buf, Size, 100);  
        CDC_Transmit_HS(rx_buf, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_lpuart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        #if RS485_RX_USE_RTOS_SEMAPHORE
        osSemaphoreRelease(modbusSemaphoreHandle);
        #else
        RS485_RX_flag = 1;
        #endif
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance){
        // memset(rx_buf, 0, RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_buf, RX_BUF_SIZE); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(&hdma_lpuart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        #if RS485_RX_USE_RTOS_SEMAPHORE
        osSemaphoreRelease(modbusSemaphoreHandle);
        #else
        RS485_RX_flag = 1;
        #endif
    }
}

/**
 * @brief 通用函数：发送请求帧并通过DMA接收响应帧
 */
HAL_StatusTypeDef Modbus_Master_SendReceive(uint8_t *tx_frame, uint16_t txLen, uint8_t *rx_frame)
{
    #if RX485_TX_USE_DMA
    while(hlpuart1.gState != HAL_UART_STATE_READY);
    memcpy(tx_buf, tx_frame, txLen);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tx_buf, txLen);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&hlpuart1, tx_buf, txLen);
    #else
    HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, tx_frame, txLen, 100);
    #endif
    if (status != HAL_OK){
        return status;
    }

    /*boardcast frame no need reply*/
    if(tx_frame[0] == MB_Broadcast_ID){
        memcpy(rx_frame, tx_frame, txLen);
        return HAL_OK;
    }

    #if RS485_RX_USE_RTOS_SEMAPHORE
    if (modbusSemaphoreHandle == NULL){
        modbusSemaphoreHandle = osSemaphoreNew(1, 0, &modbusSemaphore_attr);
        if (modbusSemaphoreHandle == NULL){
            return HAL_ERROR;
        }
    }
    
    /* 阻塞等待DMA接收完成（超时10ms，可根据需要调整） */
    if (osSemaphoreAcquire(modbusSemaphoreHandle, RX_TIMEOUT_MS) != osOK){
        return HAL_TIMEOUT;
    }
    #else
    if (RS485_RX_flag_Acquire(RX_TIMEOUT_MS) != osOK){
        return HAL_TIMEOUT;
    }
    #endif

    if(rx_size>RX_BUF_SIZE){
        return HAL_ERROR;
    }
    memcpy(rx_frame,rx_buf,rx_size);
    return HAL_OK;
}


/* CMD0x50：读字节 */
HAL_StatusTypeDef Modbus_CMD50_ReadBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData)
{
    uint16_t txLen = 6;
    /* 构造请求帧: [slaveId, 0x50, start_reg, data_length, CRC低, CRC高] */
    txFrame[0] = slaveId;
    txFrame[1] = 0x50;
    txFrame[2] = start_reg;
    txFrame[3] = data_length;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 4);
    txFrame[4] = (uint8_t)(crc & 0xFF);
    txFrame[5] = (uint8_t)(crc >> 8);

    /* 计算预期响应帧长度: slaveId + func + data_length字节计数 + 数据(data_length) + CRC(2) */
    //uint16_t rxLen = 1 + 1 + 1 + data_length + 2;

    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }

    /* 校验响应：检查slaveId、功能码和数据字节计数 */
    if (rxFrame[0] != slaveId || rxFrame[1] != 0x50 || rxFrame[2] != data_length){
        return HAL_ERROR;
    }

    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rx_size - 2);
    uint16_t recvCRC = rxFrame[rx_size - 2] | (rxFrame[rx_size - 1] << 8);
    if (crc != recvCRC){
        return HAL_ERROR;
    }

    memcpy(pData, &rxFrame[3], data_length);
    return HAL_OK;
}

/* CMD0x51：写字节（写操作回显） */
HAL_StatusTypeDef Modbus_CMD51_WriteBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData)
{
    /* 构造请求帧: [slaveId, 0x51, start_reg, data_length, data..., CRC低, CRC高] */
    uint16_t txLen = 4 + data_length;

    txFrame[0] = slaveId;
    txFrame[1] = 0x51;
    txFrame[2] = start_reg;
    txFrame[3] = data_length;
    memcpy(&txFrame[4], pData, data_length);
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
    txFrame[txLen++] = (uint8_t)(crc & 0xFF);
    txFrame[txLen++] = (uint8_t)(crc >> 8);

    /* 预期响应帧与请求帧一致 */
    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }

    /* 简单对比响应与请求是否一致 */
    if (memcmp(txFrame, rxFrame, txLen) != 0)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* CMD0x60：触发测量 */
HAL_StatusTypeDef Modbus_CMD60_TriggerMeasurement(uint8_t slaveId)
{
    /* 构造请求帧: [slaveId, 0x60, CRC低, CRC高] */
    uint16_t txLen = 4;

    txFrame[0] = slaveId;
    txFrame[1] = 0x60;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 2);
    txFrame[2] = (uint8_t)(crc & 0xFF);
    txFrame[3] = (uint8_t)(crc >> 8);
        
    /* 预期响应帧与请求帧一致 */
    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }
    // delay_us(50);

    if (memcmp(txFrame, rxFrame, txLen) != 0){
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* CMD0x61：请求回报UID */
HAL_StatusTypeDef Modbus_CMD61_BroadcastReportUID(uint8_t UID8_lower, uint8_t UID8_upper, uint8_t delay_max, uint8_t UID_length)
{
    /* 构造请求帧: [slaveId, 0x61, 8bit_UID_Lower, 8bit_UID_Upper, delay_min, delay_max, return_UID_length, CRC低, CRC高] */
    uint16_t txLen = 11;

    txFrame[0] = MB_Temp_ID;
    txFrame[1] = 0x61;
    txFrame[2] = UID8_lower;
    txFrame[3] = UID8_upper;
    txFrame[4] = 0x00;  //MBID_lower;
    txFrame[5] = 0xFF;  //MBID_upper;
    txFrame[6] = 0x00;  //delay_min;
    txFrame[7] = delay_max;
    txFrame[8] = UID_length;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 9);
    txFrame[9] = (uint8_t)(crc & 0xFF);
    txFrame[10] = (uint8_t)(crc >> 8);

    //清空UID记录
    sensor_0xF7_cnt = 0;
    for (uint8_t i = 0; i < MAX_SENSOR_NUM; i++) {
        if (sensor_UID[i] != NULL) {
            free(sensor_UID[i]);
            sensor_UID[i] = NULL;
        }
    }
    #if RX485_TX_USE_DMA
    while(hlpuart1.gState != HAL_UART_STATE_READY);
    memcpy(tx_buf, txFrame, txLen);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)tx_buf, txLen);
    HAL_UART_Transmit_DMA(&hlpuart1, tx_buf, txLen);
    #else
    HAL_UART_Transmit(&hlpuart1, txFrame, txLen, 100);
    #endif
    
    /*等待delay_max*FACTOR(100ms)时间接收帧，每接收一个刷新时间*/
    #if RS485_RX_USE_RTOS_SEMAPHORE
    while(osSemaphoreAcquire(modbusSemaphoreHandle, delay_max*REPORT_UID_DELAY_FACTOR) == osOK){
    #else
    while(RS485_RX_flag_Acquire(delay_max*REPORT_UID_DELAY_FACTOR) == osOK){   
    #endif 
        memcpy(rxFrame, rx_buf, rx_size);
        /* 预期响应帧长度: slaveId + func + Return_UID_Length + UID数据(UID_length) + CRC(2) */
        if (rx_size != 1+1+1+UID_length+2){
            continue;
        }
        crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rx_size - 2);
        uint16_t recvCRC = rxFrame[rx_size - 2] | (rxFrame[rx_size - 1] << 8);
        if (crc != recvCRC){
            continue;
        }
        
        sensor_UID[sensor_0xF7_cnt] = (uint8_t *)malloc(UID_length);
        memcpy(sensor_UID[sensor_0xF7_cnt],&rxFrame[3],UID_length);
        sensor_0xF7_cnt++;
    }
    
    if(sensor_0xF7_cnt != 0){
        return HAL_OK;
    }
    return HAL_TIMEOUT;
}

/* CMD0x62：根据UID设置从机地址 */
HAL_StatusTypeDef Modbus_CMD62_BroadcastSetSlaveID(uint8_t UID_length, uint8_t *pUID, uint8_t new_slave_id)
{
    /* 构造请求帧: [BoardcastId, 0x62, UID_length, UID数据（高字节先），new_slave_id, CRC低, CRC高] */
    uint16_t txLen = 3 + UID_length + 1;

    txFrame[0] = MB_Temp_ID;
    txFrame[1] = 0x62;
    txFrame[2] = UID_length;
    memcpy(txFrame+3, pUID, UID_length);
    txFrame[txLen-1] = new_slave_id;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
    txFrame[txLen++] = (uint8_t)(crc & 0xFF);
    txFrame[txLen++] = (uint8_t)(crc >> 8);

    /* 预期响应帧为回显请求帧 */
    uint16_t rxLen = txLen;
    Modbus_Master_SendReceive(txFrame, txLen, rxFrame);
    if (memcmp(txFrame, rxFrame, rxLen) != 0){
        return HAL_ERROR;
    }

    return HAL_OK;
}



