#include "mag_sensor.h"
#include "modbus_rtu.h"
#include "usbd_cdc_if.h"

MAG_SENSOR_module_t mag_sensor[MAX_SENSOR_NUM];
uint16_t sensor_num = 0;
uint8_t sensor_UID_duplicated[MAX_SENSOR_NUM];
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
        for(uint8_t i=0; i<sensor_0xF7_cnt; i++){

            //分配ID
            mag_sensor[sensor_num+i].cfg.mag_sensor_cfg.mb_slave_id = slaveID_tba;
            if(Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba) != HAL_OK){
                //设置对应UID未收到正确回复->可能有重复冲突或者干扰
                //重试一次
                if(Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba) != HAL_OK){
                    //重试仍失败
                    error_mark++;
                    continue;
                }
            }
            //待分配的slaveID++
            slaveID_map[slaveID_tba>>5] |= (1UL << (slaveID_tba & 0x1F)); //slaveID_map对应位置置1
            //slaveID_map[slaveID_tba>>5] &= ~(1UL << (slaveID_tba & 0x1F));   //置0操作
            slaveID_tba++;
        }
        sensor_num += sensor_0xF7_cnt;
    }
    else{
        //没有来自未分配地址的回复
        return HAL_TIMEOUT;
    }

    //return uid length = 12
    //究极红色形态

    if(error_mark == 0)
        return HAL_OK;
    else{
        usb_printf("Set SlaveID Error, error_mark=%d\n", error_mark);
        return HAL_ERROR;
    }
}


HAL_StatusTypeDef Get_MagSensors_Data(void){
    return HAL_OK;
}