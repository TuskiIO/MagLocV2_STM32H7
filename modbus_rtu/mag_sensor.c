#include "mag_sensor.h"
#include "modbus_rtu.h"
#include "usbd_cdc_if.h"

MAG_SENSOR_module_t mag_sensor[MAX_SENSOR_NUM];
uint16_t sensor_num = 0;
uint8_t slaveID_tba = 0x01;       //slaveID to be allocated
volatile uint32_t slaveID_map[8] = {0};     //bit map of used slaveID
extern uint16_t sensor_0xF7_cnt;


HAL_StatusTypeDef Get_MagSensors_Plugged(void){
    uint8_t UID_length, error_mark=0;
    uint8_t delay_max;
    uint8_t retry_times;

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
            //设置对应UID未收到正确回复->可能有重复冲突或者干扰
            //重试SET_ID_RETRY_TIMES次
            for(retry_times = SET_ID_RETRY_TIMES; retry_times>0; retry_times--){
                HAL_StatusTypeDef state = Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba);
                if(state == HAL_OK){
                    break;
                }
            }

            if(retry_times == 0){
                //分配失败
                error_mark++;
                continue;
            }

            //分配成功
            mag_sensor[sensor_num].cfg.mag_sensor_cfg.mb_slave_id = slaveID_tba;
            #if USE_USB_PRINTF
            usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, slaveID_tba);
            #endif
            sensor_num++;

            //待分配的slaveID++
            //slaveID_map[slaveID_tba>>5] |= (1UL << (slaveID_tba & 0x1F));         //slaveID_map对应位置置1  //暂时没用上
            //slaveID_map[slaveID_tba>>5] &= ~(1UL << (slaveID_tba & 0x1F));        //置0操作
            slaveID_tba++;
        }
    }
    else{
        //没有来自未分配地址的回复
        return HAL_TIMEOUT;
    }
   

    //return uid length = 12
    //究极红色形态

    //error_mark表明有未分配好ID的传感器
    if(error_mark == 0)
        return HAL_OK;
    else{
        usb_printf("Set SlaveID Error, error_mark=%d\n", error_mark);
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
    // uint8_t rxFrame[256] = {0};

    for(uint8_t i=0; i<sensor_num; i++){
        //get data
        if(Modbus_CMD50_ReadBytes(mag_sensor[i].cfg.mag_sensor_cfg.mb_slave_id, MAG_SENSOR_DATA_OFFSET, MAG_SENSOR_DATA_LENGTH, (uint8_t*)&mag_sensor[i]+MAG_SENSOR_DATA_OFFSET) != HAL_OK){
            //Handle error
            continue;
        }
        // memcpy((uint8_t*)&mag_sensor[i] + MAG_SENSOR_DATA_OFFSET, rxFrame, MAG_SENSOR_DATA_LENGTH);
    }

    return HAL_OK;
}

HAL_StatusTypeDef Check_MagSensors_SlaveID(void){
    uint8_t temp_slaveID = 0;

    //初始化sensor_num与slaveID_map
    sensor_num = 0;

    //轮询确认已有的slaveID
    for(uint8_t i=1; i<MB_MAX_ID; i++){
        HAL_StatusTypeDef state=Modbus_CMD50_ReadBytes(i, 0x00, 0x01, &temp_slaveID);
        if(state == HAL_OK){
          //分配slaveID_map
    
          //记录slaveID
          mag_sensor[sensor_num].cfg.mag_sensor_cfg.mb_slave_id = temp_slaveID;
          sensor_num++;
          #if USE_USB_PRINTF
          usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, temp_slaveID);
          #endif
        }
        else if(state == HAL_ERROR){
          //出现冲突，地址配置为0xF7
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