#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "stm32h7xx_hal.h"

#define GET_MAGSENSOR_DATA                  1
#define GET_SET_CONFIG_OF_SENSORS           1   //UDP接收SENSOR的config struct,并广播发送
#define INITIAL_GET_NEWLY_PLUGGED_SENSOR    1
#define AUTO_GET_NEWLY_PLUGGED_SENSOR       0
#define RESET_ALL_SLAVE_ID_WHEN_RESET       0

#define PRESS_KEY_B2_SET_SLAVEID  0
#define PRESS_KEY_B2_SET_SLAVEID_ADDRESS 0x02

#define INITIAL_GETUID_DELAY_TIME  100   //factor=100ms
#define ADD_GETUID_DELAY_TIME      1   //factor=100ms

#define USE_MAG_SENSOR_DRDY         0
#define ONLY_GET_SENSOR_MAGVAL      1

#define MAG_SENSOR_CONFIG_OFFSET    0
#define MAG_SENSOR_CONFIG_LENGTH    148
#define MAG_SENSOR_DATA_OFFSET      148
#define MAG_SENSOR_DATA_LENGTH      107  //255-148

#define MAG_SENSOR_MAGADC_OFFSET    92
#define MAG_SENSOR_MAGVAL_OFFSET    105

#define GET_SLAVEID_MAP(idx)    ((slaveID_map[(idx) >> 5] >> ((idx) & 0x1F)) & 0x01)
#define SET_SLAVEID_MAP(idx)    (slaveID_map[(idx) >> 5] |= (1U << ((idx) & 0x1F)))
#define RESET_SLAVEID_MAP(idx)  (slaveID_map[(idx) >> 5] &= ~(1U << ((idx) & 0x1F)))


#pragma pack(1) //align memory allocation with 1 Byte

//! Sensor Data Structures
typedef enum{
    CONTINUOUS=0,
    ON_TRIG,
} Measure_Mode_t;

typedef struct {
    uint8_t             mb_slave_id;
    uint32_t            mb_baudrate;
    Measure_Mode_t      measure_mode;       // CONTINUOUS or ON_TRIG
    uint16_t            update_rate;        // (CONTINUOUS mode) update rate in continous mode
    uint8_t             latch_mode_enable;  // (ON_TRIG mode) enable latch mode, sensor reading will update upon the next trigger signal 
    uint8_t             filter_enable;      // enable magVal filtering
    float               filter_factor;      // output = filter_factor * last_output + (1-filter_factor) * new_data
    uint8_t             calc_std_enable;    // enable calculate magVal std
    uint64_t            timestamp_ref;      // in [us]. Write to this value will sync slave time with ref (with communication delays)
    uint16_t            reserved;
}  SENSOR_Config_t; // size: 25 Bytes

typedef struct {
    uint32_t            UID32;              // Unique ID for mcu
    uint8_t             whoami;             // who-am-i register for the sensor. fixed to 0x31 (49 in decimal)
    uint8_t             firmware_ver;       // sensor firmware version.
    uint8_t             sensor_DRDY;        // True, when measurement finishes. False, when data read operation
    uint64_t            timestamp;          // timestamp in us when RM3100 finishes measurement
    uint16_t            reserved;
} SENSOR_Data_t; // size: 17 Bytes

//! RM3100 Magnetometer Data Structures

// magnetometer continuous measurement rate(approximate, to set the TMRC register value at 0x0B)
// update rate is restricted by cycle counts
// for 3-axis measurement:
// cycle count = 300 >>>>>> measure time = 9.2ms, max update rate = 100Hz, Gain = 113 LSB/uT
// cycle count = 200 >>>>>> measure time = 6.2ms, max update rate = 150Hz, Gain =  75 LSB/uT
// cycle count = 150 >>>>>> measure time = 4.6ms, max update rate = 200Hz, Gain =  57 LSB/uT
// cycle count = 100 >>>>>> measure time = 3.2ms, max update rate = 300Hz, Gain =  38 LSB/uT
typedef enum{
    RM3100_CMM_RATE_600 = 0x92,
    RM3100_CMM_RATE_300 = 0x93,
    RM3100_CMM_RATE_150 = 0x94,
    RM3100_CMM_RATE_75  = 0x95,
    RM3100_CMM_RATE_37  = 0x96, // default
    RM3100_CMM_RATE_18  = 0x97,
    RM3100_CMM_RATE_9   = 0x98,
}CMM_Rate_t;

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
    RM3100_Config_t     rm3100_cfg;  
    EllipMagCal_t       mag_cal;     
}MAG_Config_t; // size: 55 Bytes

typedef struct {
    int32_t             magADC[3];          // Raw magnetometer readings, in LSB counts
    float               magVal[3];          // Calibrated magnetic-field intensity, in uT
    float               magStd[3];          // Standard deviation of the magnetic field intensity, in uT
    float               magVal_t;           // Calibrated magnetic-field total intensity, in uT
}MAG_Data_t; // size: 40 Bytes

//! IMU Data Structures
typedef struct
{
    uint8_t accel_mode; // Accelerometer mode: `0`: OFF `1`: STANDBY `2`: LOW_POWER `3`: LOW_NOISE
    uint8_t gyro_mode;  // Gyroscope mode:     `0`: OFF `1`: STANDBY `2`: LOW_POWER `3`: LOW_NOISE

    uint8_t gyro_fs;  // Gyroscope full scale: ±16000dps, ±8000dps, ±4000dps, ±2000dps
    uint8_t gyro_odr; // Gyroscope output data rate: 500Hz to 32kHz

    uint8_t accel_fs;  // Accelerometer full scale: ±16g, ±8g, ±4g, ±2g  
    uint8_t accel_odr; // Accelerometer output data rate: 500Hz to 32kHz

    uint8_t accel_filt_bw; // Accelerometer filter bandwidth: See ICM42688P datasheet for detail 
    uint8_t gyro_filt_bw; // Gyroscope filter bandwidth: See ICM42688P datasheet for detail
} ICM42688_Config_t; // size: 8 bytes

typedef struct {
    ICM42688_Config_t   icm_cfg;            // config for ICM42688P measurement
    int16_t             accel_offset[3];    // zeros offset of accelerometer (in LSB)
    int16_t             gyro_offset[3];     // zeros offset of gyroscope (in LSB)
    float               accel_scale[3];     // accelerometer scale from LSB to m/s^2
    float               gyro_scale[3];      // gyroscope scale from LSB to dps
    float               accel_trans_scale;  // accel_trans = (int16_t)(accel_calib * accel_trans_scale)
    float               gyro_trans_scale;   // gyro_trans = (int16_t)(gyro_calib * gyro_trans_scale)
} IMU_Config_t; // size: 52 Bytes

typedef struct {
    int16_t             temp;               // temperature (in 1e-2*degrees) real_temperature = temp * 0.01
    int16_t             accel_raw[3];       // raw accelerometer counts (in LSB)
    int16_t             gyro_raw[3];        // raw gyroscope counts (in LSB)
    float               accel_calib[3];     // calibrated accelerometer reading (in m/s^2) accel_calib = (accel_raw + accel_offset) * accel_scale
    float               gyro_calib[3];      // calibrated gyroscope reading (in dps) gyro_calib = (gyro_raw + gyro_offset) * gyro_scale
    int16_t             accel_trans[3];     // accel_trans = (int16_t)(accel_calib * accel_trans_scale), for smaller communication bandwidth
    int16_t             gyro_trans[3];      // gyro_trans = (int16_t)(gyro_calib * gyro_trans_scale), for smaller communication bandwidth
} IMU_Data_t; // size: 50 Bytes

//! LED Data Structures
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t brightness;
} LED_Color_t; // size: 4 Bytes

typedef struct {
    uint8_t             led_mode;           // 0: disable led, 1: mag_t, 2: acc_t, 3: gyro_t, 4: manual set
    LED_Color_t         color;              // LED color to be set
    float               colormap_min;       // min value to normalize color_source to color_level
    float               colormap_max;       // max value to normalize color_source to color_level
    uint8_t             color_level;        // color_level of colormap
} LED_Config_t; // size: 14 Bytes

//! Config Data Structures
typedef struct {
    SENSOR_Config_t     sensor_cfg;         // reg offset =   0(0x00), len = 25 Bytes
    MAG_Config_t        mag_cfg;            // reg offset =  25(0x19), len = 55 Bytes
    IMU_Config_t        imu_cfg;            // reg offset =  80(0x50), len = 52 Bytes
    LED_Config_t        led_cfg;            // reg offset = 132(0x84), len = 14 Bytes
    uint16_t            cfg_crc16;          // reg offset = 146(0x92), len =  2 Bytes
} FULL_CFG_t; // size: 148 Bytes

//! Full Sensor Data Structure
typedef struct{
    // read/write registers
    SENSOR_Config_t     sensor_cfg;         // reg offset =   0(0x00), len = 25 Bytes
    MAG_Config_t        mag_cfg;            // reg offset =  25(0x19), len = 55 Bytes
    IMU_Config_t        imu_cfg;            // reg offset =  80(0x50), len = 52 Bytes
    LED_Config_t        led_cfg;            // reg offset = 132(0x84), len = 14 Bytes
    uint16_t            cfg_crc16;          // reg offset = 146(0x92), len =  2 Bytes
    // read-only registers
    SENSOR_Data_t       sensor_data;        // reg offset = 148(0x94), len = 17 Bytes
    MAG_Data_t          mag_data;           // reg offset = 165(0xA5), len = 40 Bytes
    IMU_Data_t          imu_data;           // reg offset = 205(0xCD), len = 50 Bytes
} MY_SENSOR_module_t; // size: 255 Bytes

#pragma pack() //align memory allocation with default strategy

extern volatile double mcu_timestamp;
extern volatile uint32_t TIM2_time_s;
extern uint8_t sensor_num;
extern MY_SENSOR_module_t mag_sensor[];
extern uint8_t slaveID_tba;
extern uint32_t slaveID_map[];
extern uint8_t PC_Trans_Buff[1024];
extern uint32_t sensor_pkg_cnt;
extern uint32_t sensor_err_pkg_cnt;

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
HAL_StatusTypeDef Get_MagSensor_Config(MY_SENSOR_module_t *sensor);

 /**
 * @brief  写单个传感器配置FULL_CFG_t
 * @param  slaveID: 传感器mb_slaveID
 * @param  FULL_CFG_t: 待修改的内容，包含mb_slaveID
 * @retval HAL_OK
 * @retval HAL_ERROR
 */
HAL_StatusTypeDef Set_MagSensor_Config(uint8_t slaveID, FULL_CFG_t *new_full_cfg);
 
/**
 * @brief  TriggerMeasure后将所有传感器数据更新到MAG_SENSOR_module_t中
 * @retval HAL_OK       
 */
 HAL_StatusTypeDef Get_MagSensors_Data(void);


 /**
 * @brief  初始化sensor_num，扫描已连接并分配了ID的传感器，将冲突传感器slaveID分配为0xF7
 * @retval HAL_OK       检测到传感器，更新到sensor_num
 * @retval HAL_TIMEOUT  未检测到传感器
 */
 HAL_StatusTypeDef Check_MagSensors_SlaveID(void);

/**
 * @brief  将sensor_num个传感器数据打包
 * @retval PC_Trans_Buff包长       
 */
uint16_t PC_TRANS_Assemble(void);

/**
 * @brief  更新timestamp    
 * @retval 当前时间值(s)
 */
 double Update_TimeStamp(void);

 void Init_SlaveID_Map(void);
 uint8_t Find_Free_SlaveID(void);

#endif