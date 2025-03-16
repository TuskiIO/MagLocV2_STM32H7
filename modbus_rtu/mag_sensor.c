#include "mag_sensor.h"
#include "modbus_rtu.h"
#include "usbd_cdc_if.h"

volatile MAG_SENSOR_module_t mag_sensor[MAX_SENSOR_NUM];
volatile float timestamp;
volatile uint32_t TIM2_time_s;
uint16_t sensor_num = 0;
uint8_t slaveID_tba;       //slaveID to be allocated
uint32_t slaveID_map[8] = {0};     //bit map of used slaveID
extern uint16_t sensor_0xF7_cnt;

static void Init_SlaveID_Map(void) {
    memset((void*)slaveID_map, 0, sizeof(slaveID_map));
    //保留MB_Broadcast_ID与MB_MAX_ID~0xFF的ID
    SET_SLAVEID_MAP(MB_Broadcast_ID);
    slaveID_map[7] = 0xFFFFFFFF << (MB_MAX_ID % 32);
    slaveID_tba = 0x01;
}

static uint8_t Find_Free_SlaveID(void) {
    for(uint8_t i=0; i<8; i++){
        if(slaveID_map[i] != 0xFFFFFFFF){
            for(uint8_t b=0; b<32; b++){
                if(!(slaveID_map[i] & (1UL << b))){
                    return (i<<5) + b;
                }
            }
        }
    }
    return 0xFF;
}

HAL_StatusTypeDef Get_MagSensors_Plugged(void){
    uint8_t UID_length, error_mark=0;
    uint8_t delay_max;
    uint8_t retry_times;

    //确认slaveID_map中的空闲id
    if(slaveID_tba == 0xFF){
        return HAL_ERROR;
    }

    if(sensor_num == 0)
        delay_max = INITIAL_GETUID_DELAY_TIME;
    else
        delay_max = ADD_GETUID_DELAY_TIME;
    //return uid length = 2
    //return uid length = 4    //红色形态
    UID_length = 4;
    if(Modbus_CMD61_BroadcastReportUID(0x00, 0xFF, delay_max, UID_length) == HAL_OK){
        for(uint8_t i=0; i<sensor_0xF7_cnt; i++){
            //分配ID
            slaveID_tba = Find_Free_SlaveID();
            if(slaveID_tba == 0xFF){
                return HAL_ERROR;
            }
            //设置对应UID未收到正确回复->可能有重复冲突或者干扰
            //重试SET_ID_RETRY_TIMES次
            for(retry_times = SET_ID_RETRY_TIMES; retry_times>0; retry_times--){
                HAL_StatusTypeDef state = Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba);
                if(state == HAL_OK){
                    break;
                }
            }
            free(sensor_UID[i]);
            sensor_UID[i] = NULL;
            //分配失败
            if(retry_times == 0){
                error_mark++;
                continue;
            }
            //分配成功
            mag_sensor[sensor_num].cfg.mag_sensor_cfg.mb_slave_id = slaveID_tba;
            #if USE_USB_PRINTF
            usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, slaveID_tba);
            #endif
            sensor_num++;   //分配成功的sensor才会被加入sensor_num里

            //处理slaveID_map
            SET_SLAVEID_MAP(slaveID_tba);
        }
    }
    else{
        //没有来自未分配地址的回复
        return HAL_TIMEOUT;
    }
   
    //return uid length = 12    //究极红色形态

    //error_mark表明有无法分配ID的传感器
    if(error_mark == 0)
        return HAL_OK;
    else{
        #if USE_USB_PRINTF
        usb_printf("Set SlaveID Error, error_num=%d\n", error_mark);
        #endif
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef Get_MagSensor_Config(MAG_SENSOR_module_t *sensor){
    // uint8_t rxFrame[256] = {0};
    //get config
    if(Modbus_CMD50_ReadBytes(sensor->cfg.mag_sensor_cfg.mb_slave_id, MAG_SENSOR_CONFIG_OFFSET, MAG_SENSOR_CONFIG_LENGTH, (uint8_t*)&sensor->cfg) != HAL_OK){
        //Handle error
        return HAL_ERROR;
    }
    // memcpy(&mag_sensor[sensor_idx].cfg, rxFrame, MAG_SENSOR_CONFIG_LENGTH);
    return HAL_OK;
}


HAL_StatusTypeDef Set_MagSensor_Config(MAG_SENSOR_module_t *sensor){
    if(Modbus_CMD51_WriteBytes(sensor->cfg.mag_sensor_cfg.mb_slave_id, MAG_SENSOR_CONFIG_OFFSET, MAG_SENSOR_CONFIG_LENGTH,(uint8_t*)&sensor->cfg) != HAL_OK){
        //Handle error
        return HAL_ERROR;
    }
    return HAL_OK;
}


HAL_StatusTypeDef Get_MagSensors_Data(void){
    // osDelay(5);

    // //等待mag_sensor_DRDY，多个传感器任意一个DRDY后即开始读数据
    uint8_t mag_sensor_rdy = 0;
    do{
        osDelay(1);
        for(uint8_t i=0; i<sensor_num; i++){
            Modbus_CMD50_ReadBytes(mag_sensor[i].cfg.mag_sensor_cfg.mb_slave_id, offsetof(MAG_SENSOR_module_t, mag_sensor_DRDY), 0x01, &mag_sensor_rdy);
            if(mag_sensor_rdy == 0x01){
                break;
            }
        }
    }while(mag_sensor_rdy != 0x01);

    for(uint8_t i=0; i<sensor_num; i++){
        //get data
        if(Modbus_CMD50_ReadBytes(mag_sensor[i].cfg.mag_sensor_cfg.mb_slave_id, MAG_SENSOR_DATA_OFFSET, MAG_SENSOR_DATA_LENGTH, (uint8_t*)&mag_sensor[i]+MAG_SENSOR_DATA_OFFSET) != HAL_OK){
            //Handle error
            continue;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef Check_MagSensors_SlaveID(void){
    uint8_t temp_slaveID = 0;

    //初始化sensor_num与slaveID_map
    sensor_num = 0;
    Init_SlaveID_Map();

    //轮询确认已有的slaveID
    for(uint8_t i=1; i<MB_MAX_ID; i++){
        HAL_StatusTypeDef state=Modbus_CMD50_ReadBytes(i, 0x00, 0x01, &temp_slaveID);
        if(state == HAL_OK){
            //分配slaveID_map
            SET_SLAVEID_MAP(temp_slaveID);
            //记录slaveID
            mag_sensor[sensor_num].cfg.mag_sensor_cfg.mb_slave_id = temp_slaveID;
            sensor_num++;
            #if USE_USB_PRINTF
            usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, temp_slaveID);
            #endif
        }
        else if(state == HAL_ERROR){
            //出现冲突，地址配置为0xF7
            RESET_SLAVEID_MAP(temp_slaveID);
            temp_slaveID = MB_Temp_ID;
            Modbus_CMD51_WriteBytes(i, 0x00, 0x01, &temp_slaveID);
            #if USE_USB_PRINTF
            usb_printf("SlaveID Conflict: %x\n", temp_slaveID);
            #endif
        }
    }

    if(sensor_num == 0)
        return HAL_TIMEOUT;
    return HAL_OK;
}

float Update_TimeStamp_ms(void) {
    timestamp = (float)TIM2_time_s + (float)(TIM2->CNT)/1000000.0f;
    return timestamp;
}


uint8_t PC_Trans_Buff[583] = {0};
uint16_t PC_TRANS_Assemble(void)
{
    volatile uint32_t temp;
    uint16_t mag_idx = 0;
    uint16_t ptr = 0;
    PC_Trans_Buff[ptr++] = 0x55;
    PC_Trans_Buff[ptr++] = 0xaa;
    PC_Trans_Buff[ptr++] = 0xff;
    PC_Trans_Buff[ptr++] = (sensor_num) & 0xff;
    PC_Trans_Buff[ptr++] = (sensor_num >> 8) & 0xff;

    memcpy((uint8_t *)&temp, (uint8_t *)&timestamp, 4);
    PC_Trans_Buff[ptr++] = (temp) & 0xff;
    PC_Trans_Buff[ptr++] = (temp >> 8) & 0xff;
    PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
    PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;

    for (mag_idx = 0; mag_idx < sensor_num; mag_idx++){
        for(uint8_t i = 0; i<3; i++){
            //assemble float magVal[3]
            memcpy((uint8_t *)&temp, (uint8_t *)&mag_sensor[mag_idx].magVal[i], 4);
            PC_Trans_Buff[ptr++] = (temp) & 0xff;
            PC_Trans_Buff[ptr++] = (temp >> 8) & 0xff;
            PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
            PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;
        }
    }
    return ptr;
}