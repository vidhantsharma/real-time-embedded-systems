
#ifndef __LSM303AGR_H
#define __LSM303AGR_H


/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "lib.h"

void delay_ms(uint32_t ms);
 
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
/* Exported constant IO ------------------------------------------------------*/
#define ACC_I2C_ADDRESS                      0x32
#define MAG_I2C_ADDRESS                      0x3C

/* Temperature sensor Registers (New vs lsm303dlhc.h) */
#define LSM303AGR_STATUS_REG_AUX_A          0x07  /* status register */
#define LSM303AGR_OUT_TEMP_L_A              0x0C  /* output temperature register */
#define LSM303AGR_OUT_TEMP_H_A              0x0D  /* output temperature register */
#define LSM303AGR_IN_COUNTER_REG_A          0x0E  /* register */
/* Acceleration Registers */
#define LSM303AGR_WHO_AM_I_ADDR             0x0F  /* device identification register (0x33) */
#define LSM303AGR_WHO_AM_I_A                LSM303AGR_WHO_AM_I_ADDR
/* Temperature sensor Registers (New vs lsm303dlhc.h) */
#define LSM303AGR_TEMP_CFG_REG_A            0x1F  /* temperature configuration register */

/* Acceleration Registers */
#define LSM303AGR_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303AGR_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303AGR_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303AGR_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303AGR_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303AGR_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303AGR_REFERENCE_A               0x26  /* Reference register acceleration */
#define LSM303AGR_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303AGR_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303AGR_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303AGR_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303AGR_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303AGR_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303AGR_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */ 
#define LSM303AGR_FIFO_CTRL_REG_A           0x2E  /* Fifo control Register acceleration */
#define LSM303AGR_FIFO_SRC_REG_A            0x2F  /* Fifo src Register acceleration */

#define LSM303AGR_INT1_CFG_A                0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303AGR_INT1_SOURCE_A             0x31  /* Interrupt 1 source Register acceleration */
#define LSM303AGR_INT1_THS_A                0x32  /* Interrupt 1 Threshold register acceleration */
#define LSM303AGR_INT1_DURATION_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303AGR_INT2_CFG_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303AGR_INT2_SOURCE_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303AGR_INT2_THS_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303AGR_INT2_DURATION_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303AGR_CLICK_CFG_A               0x38  /* Click configuration Register acceleration */
#define LSM303AGR_CLICK_SOURCE_A            0x39  /* Click 2 source Register acceleration */
#define LSM303AGR_CLICK_THS_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303AGR_TIME_LIMIT_A              0x3B  /* Time Limit Register acceleration */
#define LSM303AGR_TIME_LATENCY_A            0x3C  /* Time Latency Register acceleration */
#define LSM303AGR_TIME_WINDOW_A             0x3D  /* Time window register acceleration */

/* System Registers(New vs lsm303dlhc.h) */
#define LSM303AGR_Act_THS_A                 0x3E  /* return to sleep activation threshold register */
#define LSM303AGR_Act_DUR_A                 0x3F  /* return to sleep duration register */

/* Magnetometer */
#define LSM303AGR_X_REG_L_M                 0x45  /* Hard-iron X magnetic field */
#define LSM303AGR_X_REG_H_M                 0x46  /* Hard-iron X magnetic field */
#define LSM303AGR_Y_REG_L_M                 0x47  /* Hard-iron Y magnetic field */
#define LSM303AGR_Y_REG_H_M                 0x48  /* Hard-iron Y magnetic field */
#define LSM303AGR_Z_REG_L_M                 0x49  /* Hard-iron Z magnetic field */
#define LSM303AGR_Z_REH_H_M                 0x4A  /* Hard-iron Z magnetic field */
#define LSM303AGR_WHO_AM_I_M                0x4F  /* Who am i register magnetic field (0x40) */
#define LSM303AGR_CFG_REG_A_M               0x60  /* Configuration register A magnetic field */
#define LSM303AGR_CFG_REG_B_M               0x61  /* Configuration register B magnetic field */
#define LSM303AGR_CFG_REG_C_M               0x62  /* Configuration register C magnetic field */
#define LSM303AGR_INT_CTRL_REG_M            0x63  /* interrupt control register magnetic field */
#define LSM303AGR_INT_SOURCE_REG_M          0x64  /* interrupt source register magnetic field */
#define LSM303AGR_INT_THS_L_REG_M           0x65  /* interrupt threshold register magnetic field */
#define LSM303AGR_INT_THS_H_REG_M           0x66  /* interrupt threshold register magnetic field*/
#define LSM303AGR_STATUS_REG_M              0x67  /* Status Register magnetic field */
#define LSM303AGR_OUTX_L_REG_M              0x68  /* Output Register X magnetic field */
#define LSM303AGR_OUTX_H_REG_M              0x69  /* Output Register X magnetic field */
#define LSM303AGR_OUTY_L_REG_M              0x6A  /* Output Register X magnetic field */
#define LSM303AGR_OUTY_H_REG_M              0x6B  /* Output Register X magnetic field */
#define LSM303AGR_OUTZ_L_REG_M              0x6C  /* Output Register X magnetic field */
#define LSM303AGR_OUTZ_H_REG_M              0x6D  /* Output Register X magnetic field */

// i2c pins
#define SCL_PIN 8 
#define SDA_PIN 16
/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/
#define LSM303AGR_BLE_LSB                 ((uint8_t)0x00) /*!< Little Endian: data LSB @ lower address */
#define LSM303AGR_BLE_MSB                 ((uint8_t)0x40) /*!< Big Endian: data MSB @ lower address */
#define LSM303AGR_BLE_MSB_MAG             ((uint8_t)0x08)

/* Power Mode selection */
typedef enum mode {
  LSM303_ACCEL_MODE_NORMAL,          ///< Normal measurement mode; 10-bit
  LSM303_ACCEL_MODE_HIGH_RESOLUTION, ///< High resolution mode; 12-bit
  LSM303_ACCEL_MODE_LOW_POWER,       ///< Low power mode; 8-bit
} lsm303_accel_mode;

/* Accelerometer Data Rate*/
typedef enum dataRate{
  LSM303_ACCEL_DATARATE_0HZ,
  LSM303_ACCEL_DATARATE_1HZ,
  LSM303_ACCEL_DATARATE_10HZ,
  LSM303_ACCEL_DATARATE_25HZ,
  LSM303_ACCEL_DATARATE_50HZ,
  LSM303_ACCEL_DATARATE_100HZ,
  LSM303_ACCEL_DATARATE_200HZ,
  LSM303_ACCEL_DATARATE_400HZ,
  LSM303_ACCEL_DATARATE_1600HZ,
  LSM303_ACCEL_DATARATE_1300HZ
} lsm303_accel_data_rate;

/* Range values of accelerometer*/
typedef enum range{
LSM303_ACCEL_RANGE_2G,
LSM303_ACCEL_RANGE_4G,
LSM303_ACCEL_RANGE_8G,
LSM303_ACCEL_RANGE_16G
} lsm303_accel_range;


/* Data rate of magnetometer */
typedef enum magDataRate{
  LSM303_MAG_DATARATE_10HZ,
  LSM303_MAG_DATARATE_20HZ,
  LSM303_MAG_DATARATE_50HZ,
  LSM303_MAG_DATARATE_100HZ
} lsm303_mag_data_rate;

/* Magnetometer system modes*/
typedef enum magSysModes{
  LSM303_MAG_SYSMODE_CONTINUOUS,
  LSM303_MAG_SYSMODE_SINGLE,
  LSM303_MAG_SYSMODE_IDLE_1,
  LSM303_MAG_SYSMODE_IDLE_2
} lsm303_mag_sys_modes;

typedef enum magPowerModes{
  LSM303_MAG_HIGH_RESOLUTION_MODE,
  LSM303_MAG_LOW_POWER_MODE
} lsm303_mag_power_modes;


#define LSM303AGR_ACCEL_ENABLE 0x7 // all axis
#define LSM303AGR_ACCEL_DISABLE 0x0 // all axis

#define EARTH_GRAVITY 9.80665f

/* IMU functions defined below */
void IMUinit(int accel_mode, int accel_range, int accel_data_rate, 
             int mag_power_mode, int mag_sys_mode, int mag_dataRate);

void accInit(int mode,int range, int data_rate);
void magInit(int mag_power_mode, int mag_sys_mode, int mag_dataRate);

void accReadXYZ(float* data);

struct LSM303AGR_acc_data
{
  uint8_t range;
  uint8_t mode;
  uint8_t rate;
};

struct LSM303AGR_mag_data{
  uint8_t dataRate;
  uint8_t power_mode;
  uint8_t sys_mode;
};
float getACCLSB();
uint8_t getACCShift();

void magReadXYZ(float* data);

void motionsensor(float* data);

void    COMPASSACCELERO_IO_ITConfig(void);
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr);

void accSelfTest();

#endif /* __LSM303AGR_H */