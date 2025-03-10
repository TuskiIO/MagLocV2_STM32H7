#include "mag_sensor.h"
#include "modbus_rtu.h"
#include "usbd_cdc_if.h"

MAG_SENSOR_module_t mag_sensor[MAX_SENSOR_NUM];
uint16_t sensor_num = 0;
uint8_t slaveID_tba = 0x01;       //slaveID to be allocated
volatile uint32_t slaveID_map[8] = {0};     //bit map of used slaveID
extern uint16_t sensor_0xF7_cnt;
extern uint8_t *sensor_UID[MAX_SENSOR_NUM];


HAL_StatusTypeDef Get_MagSensors_Plugged(void){
    uint8_t UID_length, error_mark=0;
    uint8_t delay_max;

    if(sensor_num == 0)
        delay_max = INITIAL_GETUID_DELAY_TIME;
    else
        delay_max = ADD_GETUID_DELAY_TIME;

    //return uid length = 2

    //return uid length = 4
    //红色形态
    UID_length = 4;
    if(Modbus_CMD61_BroadcastReportUID(0x00, 0xFF, delay_max, UID_length) == HAL_OK){
        if(slaveID_tba == MB_MAX_ID){
            return HAL_ERROR;
        }

        sensor_0xF7_cnt += sensor_num;      //分配成功的sensor才会被加入sensor_num里
        for(uint8_t i=sensor_num; i<sensor_0xF7_cnt; i++){

            //分配ID
            if(Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba) != HAL_OK){
                //设置对应UID未收到正确回复->可能有重复冲突或者干扰
                //重试一次
                if(Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba) != HAL_OK){
                    //重试仍失败
                    error_mark++;
                    continue;
                }
            }
            
            //分配成功
            mag_sensor[sensor_num].cfg.mag_sensor_cfg.mb_slave_id = slaveID_tba;
            sensor_num++;

            //待分配的slaveID++
            //slaveID_map[slaveID_tba>>5] |= (1UL << (slaveID_tba & 0x1F));       //slaveID_map对应位置置1  //暂时没用上
            //slaveID_map[slaveID_tba>>5] &= ~(1UL << (slaveID_tba & 0x1F));    //置0操作
            slaveID_tba++;
        }
    }
    else{
        //没有来自未分配地址的回复
        return HAL_TIMEOUT;
    }

    //error_mark表明有未分配好ID的传感器

    //return uid length = 12
    //究极红色形态

    if(error_mark == 0)
        return HAL_OK;
    else{
        usb_printf("Set SlaveID Error, error_mark=%d\n", error_mark);
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef Get_MagSensors_Config(uint8_t sensor_idx){
    uint8_t rxFrame[256] = {0};

    //get config
    if(Modbus_CMD50_ReadBytes(mag_sensor[sensor_idx].cfg.mag_sensor_cfg.mb_slave_id, 0x00, 0x46, rxFrame) != HAL_OK){
        //Handle error
        return HAL_ERROR;
    }
    memcpy(&mag_sensor[sensor_idx].cfg, rxFrame, sizeof(FULL_CFG_t));

    return HAL_OK;
}

HAL_StatusTypeDef Get_MagSensors_Data(void){
    uint8_t rxFrame[256] = {0};

    //trigger measure and mark the time
    Modbus_CMD60_TriggerMeasurement(MB_Broadcast_ID);
    for(uint8_t i=0; i<sensor_num; i++){
        //get data
        if(Modbus_CMD50_ReadBytes(mag_sensor[i].cfg.mag_sensor_cfg.mb_slave_id, 0x46, 0x28, rxFrame) != HAL_OK){
            //Handle error
            continue;
        }
        memcpy((uint8_t*)&mag_sensor[i] + sizeof(FULL_CFG_t), rxFrame, sizeof(MAG_SENSOR_module_t)-sizeof(FULL_CFG_t));
    }

    return HAL_OK;
}