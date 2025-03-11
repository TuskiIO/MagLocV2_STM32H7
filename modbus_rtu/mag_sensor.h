#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "stm32h7xx_hal.h"



#define AUTO_GET_NEWLY_PLUGGED_SENSOR   0
#define INITIAL_GETUID_DELAY_TIME  50   //factor=100ms
#define ADD_GETUID_DELAY_TIME      1   //factor=100ms

#define MAG_SENSOR_CONFIG_OFFSET    0x00
#define MAG_SENSOR_CONFIG_LENGTH    0x46
#define MAG_SENSOR_DATA_OFFSET      0X46
#define MAG_SENSOR_DATA_LENGTH      0X2C

#define MAG_SENSOR_MAGADC_OFFSET    0x54
#define MAG_SENSOR_MAGVAL_OFFSET    0x61

typedef enum{
    FALSE = 0,
    TRUE = 1
}bool;

#pragma pack(1) //align memory allocation with 1 Byte

// magnetometer continuous measurement rate(approximate, to set the TMRC register value at 0x0B)
// update rate is restricted by cycle counts
// for 3-axis measurement:
// cycle count = 300 >>>>>> max update rate = 100Hz, Gain = 113 LSB/uT
// cycle count = 200 >>>>>> max update rate = 150Hz, Gain =  75 LSB/uT
// cycle count = 150 >>>>>> max update rate = 200Hz, Gain =  57 LSB/uT
// cycle count = 100 >>>>>> max update rate = 300Hz, Gain =  38 LSB/uT
typedef enum{
    RM3100_CMM_RATE_600 = 0x92,
    RM3100_CMM_RATE_300 = 0x93,
    RM3100_CMM_RATE_150 = 0x94,
    RM3100_CMM_RATE_75  = 0x95,
    RM3100_CMM_RATE_37  = 0x96, // default
    RM3100_CMM_RATE_18  = 0x97,
    RM3100_CMM_RATE_9   = 0x98,
}CMM_Rate_t;

typedef enum{
    CONTINUOUS=0,
    ON_TRIG,
}Measure_Mode_t;

typedef struct {
    uint8_t             mb_slave_id;
    uint32_t            mb_baudrate;
    uint16_t            update_rate;        // update rate in continous mode
    Measure_Mode_t      measure_mode;       // continuous or on-trig
    uint8_t             filter_enable;      // enable magVal filtering
    float               filter_factor;      // output = filter_factor * last_output + (1-filter_factor) * new_data
} MAG_SENSOR_Config_t; // size: 13 Bytes

typedef struct {
    CMM_Rate_t          CMM_rate;
    uint16_t            cycle_count;
} RM3100_Config_t; // size: 3 Bytes

typedef struct {
    int32_t             V[3];               // Offset vector V, in LSB counts
    float               M[3][3];            // calibration matrix M, to left-multiply with (magADC-V)
    float               gain;               // uT/LSB  magVal = M*(magADC-V)*gain
} EllipMagCal_t; // size: 52 Bytes

typedef struct {
    MAG_SENSOR_Config_t mag_sensor_cfg;     // reg offset =  0(0x00), len = 13 Bytes
    RM3100_Config_t     rm3100_cfg;         // reg offset = 13(0x0D), len =  3 Bytes
    EllipMagCal_t       mag_cal;            // reg offset = 16(0x10), len = 52 Bytes
    uint16_t            crc16;              // reg offset = 68(0x44), len =  2 Bytes
}FULL_CFG_t; // size: 70 Bytes

// main data struct & Reg offset defines
typedef struct{
    // read/write registers
    FULL_CFG_t          cfg;                // reg offset =  0(0x00), len = 70 Bytes
    float               timestamp_ref;      // reg offset = 70(0x46), len =  4 Bytes      // write to this value will sync slave time with ref (with communication delays)
    // read-only registers
    uint32_t            UID32;              // reg offset = 74(0x4A), len =  4 Bytes
    bool                mag_sensor_DRDY;    // reg offset = 78(0x4E), len =  1 Bytes
    bool                rm3100_DRDY;        // reg offset = 79(0x4F), len =  1 Bytes
    float               timestamp;          // reg offset = 80(0x50), len =  4 Bytes      // timestamp in ms when RM3100 finishes measurement
    int32_t             magADC[3];          // reg offset = 84(0x54), len = 12 Bytes      // Raw magnetometer readings, in LSB counts
    uint8_t             magADC_CRC;         // CRC-8 for magADC data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magVal[3];          // reg offset = 97(0x61), len = 12 Bytes      // Calibrated magnet-field intensity, in uT
    uint8_t             magVal_CRC;         // CRC-8 for magVal data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magVal_t;           // reg offset =110(0x6E), len = 4 Bytes       // Calibrated magnet-field total intensity, in uT
    float               magStd;             // reg offset =114(0x72), len = 4 Bytes       // standard deviation of the magnetic field total intensity
} MAG_SENSOR_module_t; // size: 118 Bytes

#pragma pack() //align memory allocation with default strategy


extern uint16_t sensor_num;
extern MAG_SENSOR_module_t mag_sensor[];
extern uint8_t slaveID_tba;
extern volatile uint32_t slaveID_map[];

/**
 * @brief  将所有默认地址的传感器分配slaveID，并修改全局变量sensor_num，将slaveID保存在MAG_SENSOR_Config_t中，初次检测会等待更长时间
 * @retval HAL_OK       检测到新传感器，并完成分配
 * @retval HAL_TIMEOUT  未检测到传感器
 * @retval HAL_ERROR    其他错误
 */
 HAL_StatusTypeDef Get_MagSensors_Plugged(void);

/**
 * @brief  读单个传感器配置FULL_CFG_t
 * @param  sensor: 传感器结构体指针
 * @retval HAL_OK
 * @retval HAL_ERROR
 */
HAL_StatusTypeDef Get_MagSensor_Config(MAG_SENSOR_module_t *sensor);

 /**
 * @brief  写单个传感器配置FULL_CFG_t
 * @param  sensor: 传感器结构体指针
 * @retval HAL_OK
 * @retval HAL_ERROR
 */
HAL_StatusTypeDef Set_MagSensors_Config(MAG_SENSOR_module_t *sensor);
 
/**
 * @brief  TriggerMeasure后将所有传感器数据更新到MAG_SENSOR_module_t中
 * @retval HAL_OK       
 */
 HAL_StatusTypeDef Get_MagSensors_Data(void);


 /**
 * @brief  初始化sensor_num，扫描已连接并分配了ID的传感器，将冲突传感器slaveID分配为0xF7
 * @retval HAL_OK       检测到传感器，更新到sensor_numS
 * @retval HAL_TIMEOUT  未检测到传感器
 */
 HAL_StatusTypeDef Check_MagSensors_SlaveID(void);

#endif