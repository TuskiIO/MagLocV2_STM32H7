#include "modbus_rtu.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal_crc.h"
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

uint8_t rx_buf[RX_BUF_SIZE] __attribute__((section(".RxLpuart")));
uint8_t tx_buf[TX_BUF_SIZE] __attribute__((section(".TxLpuart")));
uint16_t rx_size = 0;
uint16_t sensor_num = 0;

static osSemaphoreId_t modbusSemaphoreHandle = NULL;
const osSemaphoreAttr_t modbusSemaphore_attr = {
    .name = "ModbusSem"
};


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
        //HAL_UART_Transmit(&hlpuart1, rx_buf, Size, 100);        
        CDC_Transmit_HS(rx_buf, Size);
    
        // memset(rx_buf, 0, RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_lpuart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        osSemaphoreRelease(modbusSemaphoreHandle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance){
        // memset(rx_buf, 0, RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_buf, RX_BUF_SIZE); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(&hdma_lpuart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        osSemaphoreRelease(modbusSemaphoreHandle);
    }
}

// static HAL_StatusTypeDef Modbus_Transmit_wCRC(uint8_t *txFrame, uint16_t txLen){
//     uint8_t txFrame_wCRC[256];
//     memcpy(txFrame_wCRC,txFrame,txLen);

//     uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
//     txFrame_wCRC[txLen++] = (uint8_t)(crc & 0xFF);
//     txFrame_wCRC[txLen++] = (uint8_t)(crc >> 8);

//     /* 发送请求帧（阻塞方式） */
//     HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, txFrame, txLen+2, TRX_TIMEOUT);
//     if (status != HAL_OK){
//         return status;
//     }
// }

/**
 * @brief 通用函数：发送请求帧并通过DMA接收响应帧
 */
HAL_StatusTypeDef Modbus_Master_SendReceive(uint8_t *txFrame, uint16_t txLen, uint8_t *rxFrame)
{

    //Modbus_Transmit_wCRC(txFrame,txLen);
    HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, txFrame, txLen, TRX_TIMEOUT);
    if (status != HAL_OK){
        return status;
    }

    if (modbusSemaphoreHandle == NULL){
        modbusSemaphoreHandle = osSemaphoreNew(1, 0, &modbusSemaphore_attr);
        if (modbusSemaphoreHandle == NULL){
            return HAL_ERROR;
        }
    }

    /* 阻塞等待DMA接收完成（超时100ms，可根据需要调整） */
    if (osSemaphoreAcquire(modbusSemaphoreHandle, TRX_TIMEOUT) != osOK){
        return HAL_TIMEOUT;
    }

    memcpy(rxFrame,rx_buf,rx_size);
    return HAL_OK;
}

/* CMD0x50：读字节 */
HAL_StatusTypeDef Modbus_CMD50_ReadBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length)
{
    uint8_t txFrame[6];
    uint8_t rxFrame[256] = {0};

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


    Modbus_Master_SendReceive(txFrame, sizeof(txFrame), rxFrame);

    /* 校验响应：检查slaveId、功能码和数据字节计数 */
    if (rxFrame[0] != slaveId || rxFrame[1] != 0x50 || rxFrame[2] != data_length){
        return HAL_ERROR;
    }

    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rx_size - 2);
    uint16_t recvCRC = rxFrame[rx_size - 2] | (rxFrame[rx_size - 1] << 8);
    if (crc != recvCRC){
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* CMD0x51：写字节（写操作回显） */
HAL_StatusTypeDef Modbus_CMD51_WriteBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData)
{
    uint8_t txFrame[256];
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
    uint8_t rxFrame[256] = {0};
    Modbus_Master_SendReceive(txFrame, txLen, rxFrame);

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
    uint8_t txFrame[4];
    /* 构造请求帧: [slaveId, 0x60, CRC低, CRC高] */
    txFrame[0] = slaveId;
    txFrame[1] = 0x60;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 2);
    txFrame[2] = (uint8_t)(crc & 0xFF);
    txFrame[3] = (uint8_t)(crc >> 8);
        
    /* 预期响应帧与请求帧一致 */
    uint8_t rxFrame[4] = {0};
    Modbus_Master_SendReceive(txFrame, sizeof(txFrame), rxFrame);

    /* 广播包无回复 */
    if(slaveId == 0x00){
        return HAL_OK;
    }

    if (memcmp(txFrame, rxFrame, sizeof(txFrame)) != 0){
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* CMD0x61：请求回报UID */
HAL_StatusTypeDef Modbus_CMD61_BroadcastReportUID(uint8_t UIDCRC8_lower, uint8_t UIDCRC8_higher, uint8_t delay_lower, uint8_t delay_higher, uint8_t return_UID_length)
{
    if (modbusSemaphoreHandle == NULL){
        modbusSemaphoreHandle = osSemaphoreNew(1, 0, &modbusSemaphore_attr);
        if (modbusSemaphoreHandle == NULL){
            return HAL_ERROR;
        }
    }

    uint8_t txFrame[9];
    /* 构造请求帧: [slaveId, 0x61, UIDCRC8_lower, UIDCRC8_higher, delay_lower, delay_higher, return_UID_length, CRC低, CRC高] */
    txFrame[0] = MB_Broadcast_ID;
    txFrame[1] = 0x61;
    txFrame[2] = UIDCRC8_lower;
    txFrame[3] = UIDCRC8_higher;
    txFrame[4] = delay_lower;
    txFrame[5] = delay_higher;
    txFrame[6] = return_UID_length;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 7);
    txFrame[7] = (uint8_t)(crc & 0xFF);
    txFrame[8] = (uint8_t)(crc >> 8);

    HAL_UART_Transmit(&hlpuart1, txFrame, sizeof(txFrame), TRX_TIMEOUT);
    sensor_num = 0;

    uint8_t rxFrame,rxSize;
    /* 至少一个Sensor */
    if (osSemaphoreAcquire(modbusSemaphoreHandle, delay_higher*REPORT_UID_DELAY_FACTOR) != osOK){
        return HAL_TIMEOUT;
    }
    rxSize = rx_size;
    memcpy(rxFrame,rx_buf, rxSize);

    /* 预期响应帧长度: slaveId + func + UID数据(return_UID_length) + CRC(2) */
    if (rxSize != 1+1+return_UID_length+2){
        return HAL_ERROR;
    }

    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rxSize - 2);
    uint16_t recvCRC = rxFrame[rxSize - 2] | (rxFrame[rxSize - 1] << 8);
    if (crc != recvCRC){
        return HAL_ERROR;
    }

    sensor_num++;

    
    // //uint16_t rxLen = 1 + 1 + return_UID_length + 2;

    // uint8_t rxFrame[256] = {0};
    // //Modbus_Master_SendReceive(txFrame, sizeof(txFrame), rxFrame);
    // HAL_UART_Transmit(&hlpuart1, txFrame, sizeof(txFrame), TRX_TIMEOUT);

    // if (rxFrame[0] != slaveId || rxFrame[1] != 0x61)
    // {
    //     return HAL_ERROR;
    // }
    // crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rxLen - 2);
    // uint16_t recvCRC = rxFrame[rxLen - 2] | (rxFrame[rxLen - 1] << 8);
    // if (crc != recvCRC)
    // {
    //     return HAL_ERROR;
    // }

    
    return HAL_OK;
}

/* CMD0x62：根据UID设置从机地址 */
HAL_StatusTypeDef Modbus_CMD62_BroadcastSetSlaveID(uint8_t UID_length, uint8_t *pUID, uint8_t new_slave_id)
{
    uint8_t txFrame[256];
    /* 构造请求帧: [BoardcastId, 0x62, UID_length, UID数据（高字节先），new_slave_id, CRC低, CRC高] */
    uint16_t txLen = 3 + UID_length + 1;

    txFrame[0] = MB_Broadcast_ID;
    txFrame[1] = 0x62;
    txFrame[2] = UID_length;
    memcpy(txFrame+3, pUID, UID_length);
    txFrame[txLen-1] = new_slave_id;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
    txFrame[txLen++] = (uint8_t)(crc & 0xFF);
    txFrame[txLen++] = (uint8_t)(crc >> 8);

    /* 预期响应帧为回显请求帧 */
    uint16_t rxLen = txLen;
    uint8_t rxFrame[256] = {0};
    Modbus_Master_SendReceive(txFrame, txLen, rxFrame);
    if (memcmp(txFrame, rxFrame, rxLen) != 0){
        return HAL_ERROR;
    }

    return HAL_OK;
}



