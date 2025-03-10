#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "stm32h7xx_hal.h"

extern uint16_t sensor_num;
#define INITIAL_GETUID_DELAY_TIME  50   //factor=100ms
#define ADD_GETUID_DELAY_TIME      10   //factor=100ms

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
    FULL_CFG_t          cfg;                // reg offset =  0(0x00), len = 70 Bytes

    bool                mag_sensor_DRDY;    // reg offset = 70(0x46), len =  1 Byte
    bool                rm3100_DRDY;        // reg offset = 71(0x47), len =  1 Byte
    float               finish_meas_t_ms;   // reg offset = 72(0x48), len =  4 Byte       // timestamp in ms when RM3100 finishes measurement
    int32_t             magADC[3];          // reg offset = 76(0x4C), len = 12 Byte       // Raw magnetometer readings, in LSB counts
    uint8_t             magADC_CRC;         // CRC-8 for magADC data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magVal[3];          // reg offset = 89(0x59), len = 12 Byte       // Calibrated magnet-field intensity, in uT
    uint8_t             magVal_CRC;         // CRC-8 for magVal data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magVal_t;           // reg offset =102(0x66), len = 4 Byte        // Calibrated magnet-field total intensity, in uT
    float               magStd;             // reg offset =106(0x6A), len = 4 Byte        // standard deviation of the magnetic field total intensity
} MAG_SENSOR_module_t; // size: 110 Bytes

#pragma pack() //align memory allocation with default strategy

/**
 * @brief  将所有默认地址的传感器分配slaveID，并修改全局变量sensor_num，将slaveID保存在MAG_SENSOR_Config_t中，初次检测会等待更长时间
 * @retval HAL_OK       检测到新传感器，并完成分配
 * @retval HAL_TIMEOUT  未检测到传感器
 * @retval HAL_ERROR    其他错误
 */
 HAL_StatusTypeDef Get_MagSensors_Plugged(void);

/**
 * @brief  将单个传感器配置FULL_CFG_t更新
 * @param  sensor_idx: 传感器索引
 * @retval HAL_OK
 * @retval HAL_ERROR
 */
 HAL_StatusTypeDef Get_MagSensors_Config(uint8_t sensor_idx);
 
/**
 * @brief  TriggerMeasure后将所有传感器数据更新到MAG_SENSOR_module_t中
 * @retval HAL_OK       
 */
 HAL_StatusTypeDef Get_MagSensors_Data(void);

#endif