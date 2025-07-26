/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi088.h
  * @brief          : bmi088 interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "bmi088_reg.h"

/* Exported defines -----------------------------------------------------------*/
/* 使用SPI通信接口 */
#define BMI088_USE_SPI

/* 温度转换因子：每单位原始数据的温度变化量为0.125摄氏度 */
#define BMI088_TEMP_FACTOR 0.125f
/* 温度偏移量：温度数据转换为摄氏度时的基准偏移量为23.0摄氏度 */
#define BMI088_TEMP_OFFSET 23.0f

/* 加速度计寄存器写入数量：需要配置的加速度计寄存器数量为6个 */
#define BMI088_WRITE_ACCEL_REG_NUM 6
/* 陀螺仪寄存器写入数量：需要配置的陀螺仪寄存器数量为6个 */
#define BMI088_WRITE_GYRO_REG_NUM 6

/* 数据就绪标志位：陀螺仪数据就绪标志位为第0位 */
#define BMI088_GYRO_DATA_READY_BIT 0
/* 数据就绪标志位：加速度计数据就绪标志位为第1位 */
#define BMI088_ACCEL_DATA_READY_BIT 1
/* 数据就绪标志位：加速度计温度数据就绪标志位为第2位 */
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

/* 长延时时间：用于软件复位等操作的等待时间，80毫秒 */
#define BMI088_LONG_DELAY_TIME 80

/* 通信等待时间：传感器通信初始化或操作之间的等待时间，150微秒 */
#define BMI088_COM_WAIT_SENSOR_TIME 150

/* 加速度计I2C地址：0x18左移1位，用于I2C通信 */
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
/* 陀螺仪I2C地址：0x68左移1位，用于I2C通信 */
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

/* 加速度计量程选择：设置为±3g量程 */
#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

/* 陀螺仪量程选择：设置为±2000°/s量程 */
#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

/* 加速度计灵敏度：3g量程下，每单位原始数据对应的加速度值（m/s²） = 3/32768*9.8 */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f

/* 加速度计灵敏度：6g量程下，每单位原始数据对应的加速度值（m/s²） */
#define BMI088_ACCEL_6G_SEN 0.00179443359375f

/* 加速度计灵敏度：12g量程下，每单位原始数据对应的加速度值（m/s²） */
#define BMI088_ACCEL_12G_SEN 0.0035888671875f

/* 加速度计灵敏度：24g量程下，每单位原始数据对应的加速度值（m/s²） */
#define BMI088_ACCEL_24G_SEN 0.007177734375f

/* 陀螺仪灵敏度：2000°/s量程下，每单位原始数据对应的角速度值（弧度/s） = 2000/32768/180*π */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f

/* 陀螺仪灵敏度：1000°/s量程下，每单位原始数据对应的角速度值（弧度/s） */
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f

/* 陀螺仪灵敏度：500°/s量程下，每单位原始数据对应的角速度值（弧度/s） */
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f

/* 陀螺仪灵敏度：250°/s量程下，每单位原始数据对应的角速度值（弧度/s） */
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f

/* 陀螺仪灵敏度：125°/s量程下，每单位原始数据对应的角速度值（弧度/s） */
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status for the BMI088.
 */
typedef enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
}BMI088_Status_e;

/**
 * @brief typedef structure that contains the information for the received data.
 */
typedef struct
{
	int16_t accelx;   /*!< received x-axis angular acceleration data */
	int16_t accely;   /*!< received y-axis angular acceleration data */
	int16_t accelz;   /*!< received z-axis angular acceleration data */

	int16_t gyrox;    /*!< received x-axis angular velocity data */
	int16_t gyroy;    /*!< received y-axis angular velocity data */
	int16_t gyroz;    /*!< received z-axis angular velocity data */

	int16_t temperature; /*!< received temperature data */
}MPU_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the BMI088.
 */
typedef struct
{
    bool offsets_init;    /*!< Initializes the offset flag */

    float accel[3];       /*!< converted accelerator data *//*转换后的加速度数据*/
    float gyro[3];        /*!< converted gyro data */
    float temperature;    /*!< converted temperature data */

    MPU_Info_Typedef mpu_info;/*!< bmi088 received data */

    float offsets_gyrox;   /*!< offsets of x-axis angular velocity *//*x轴角速度偏置*/
    float offsets_gyroy;   /*!< offsets of y-axis angular velocity */
    float offsets_gyroz;   /*!< offsets of z-axis angular velocity */
}BMI088_Info_Typedef;

extern BMI088_Info_Typedef BMI088_Info;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the BMI088 according to writing the specified data 
  *        to the internal configuration registers of the sensor.
  */
extern void BMI088_Init(void);
/**
  * @brief Updates the BMI088 Information.
  */
extern void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info);

#endif //DEVICE_BMI088_H
