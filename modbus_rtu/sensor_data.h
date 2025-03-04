#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "stm32h7xx_hal.h"

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
    uint8_t             i2c_slave_addr;
    uint16_t            update_rate;
    Measure_Mode_t      measure_mode;
} MAG_SENSOR_Config_t; // size: 4 Bytes

typedef struct {
    CMM_Rate_t          CMM_rate;
    uint16_t            cycle_count;
} RM3100_Config_t; // size: 3 Bytes

typedef struct {
    int32_t             V[3];             // Offset vector V, in LSB counts
    float               M[3][3];          // calibration matrix M, to left-multiply with (magADC-V)
    float               gain;             // uT/LSB  magVal = M*(magADC-V)*gain
} EllipMagCal_t; // size: 52 Bytes

typedef struct {
    MAG_SENSOR_Config_t mag_sensor_cfg;
    RM3100_Config_t     rm3100_cfg;
    EllipMagCal_t       mag_cal;
}FULL_CFG_t; // size: 59 Bytes


// main data struct & I2C reg offset defines
typedef struct{
    MAG_SENSOR_Config_t mag_sensor_cfg; // reg offset =  0(0x00), len =  4 Bytes
    RM3100_Config_t     rm3100_cfg;     // reg offset =  4(0x04), len =  3 Bytes
    EllipMagCal_t       mag_cal;        // reg offset =  7(0x07), len = 52 Bytes
    
    uint8_t                mag_sensor_DRDY;// reg offset = 59(0x3B), len =  1 Byte
    uint8_t                rm3100_DRDY;    // reg offset = 60(0x3C), len =  1 Byte
    int32_t             magADC[3];      // reg offset = 61(0x3D), len = 12 Byte       // Raw magnetometer readings, in LSB counts
    uint8_t             magADC_CRC;     // CRC-8 for magADC data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magVal[3];      // reg offset = 74(0x4A), len = 12 Byte       // Calibrated magnet-field intensity, in uT
    uint8_t             magVal_CRC;     // CRC-8 for magVal data, Motorola, polinomial x^8+x^5+x^4+x^0 (0x31 0b100110001)
    float               magStd;         // reg offset = 87(0x57), len = 4 Byte        // standard deviation of the magnetic field total intensity
} MAG_SENSOR_module_t; // size: 91 Bytes


#endif