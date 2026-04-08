#ifndef F407_RM_TMPLATE_HAL_BMI088_REG_H
#define F407_RM_TMPLATE_HAL_BMI088_REG_H

#include "stdint.h"


/*---------------------------------- accelerator register address ----------------------------------*/
/* 加速度计芯片ID寄存器地址：用于读取芯片标识（Who am I），值为0x1E */
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

/* 错误状态寄存器地址：用于检查加速度计错误状态 */
#define BMI088_ACC_ERR_REG 0x02
/* 配置错误标志位：第2位表示配置错误 */
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
/* 致命错误标志位：第0位表示致命错误 */
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)
/* 状态寄存器地址：用于检查加速度计数据就绪状态 */
#define BMI088_ACC_STATUS 0x03
/* 数据就绪标志位：第7位表示加速度计数据就绪 */
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)


/* 加速度计三轴输出寄存器地址：分别存储X、Y、Z轴的低位和高位数据 */
#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

/* 传感器时间寄存器地址：存储传感器时间数据（低、中、高位） */
#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

/* 中断状态寄存器地址：用于检查加速度计中断状态 */
#define BMI088_ACC_INT_STAT_1 0x1D
/* 数据就绪中断标志位：第7位表示数据就绪中断 */
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

/* 温度数据寄存器地址：存储温度数据高位和低位 */
#define BMI088_TEMP_M 0x22
#define BMI088_TEMP_L 0x23

/* 加速度计配置寄存器地址：用于设置采样频率和输出频率 */
#define BMI088_ACC_CONF 0x40
/* 配置寄存器必须置位标志：bit7-bit4用于设置采样频率 */
#define BMI088_ACC_CONF_MUST_Set 0x80
/* 采样频率设置：4倍过采样 */
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
/* 采样频率设置：2倍过采样 */
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
/* 采样频率设置：正常采样 */
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)
/* 输出频率设置：bit3-bit0用于设置输出频率 */
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)  // 12.5Hz
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)    // 25Hz
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)    // 50Hz
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)   // 100Hz
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)   // 200Hz
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)   // 400Hz
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)   // 800Hz
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)  // 1600Hz

/* 加速度计量程寄存器地址：用于设置加速度计量程 */
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_RANGE_SHFITS 0x00
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)  // ±3g
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)  // ±6g
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS) // ±12g
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS) // ±24g

/* INT1引脚配置寄存器地址：配置INT1输入输出引脚 */
#define BMI088_INT1_IO_CTRL 0x53
/* INT1使能标志位：第3位使能INT1作为输出引脚 */
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
/* INT1输出模式：第2位设置推挽输出 */
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
/* INT1输出模式：开漏输出 */
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
/* INT1电平设置：第1位设置低电平 */
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
/* INT1电平设置：高电平 */
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

/* INT2引脚配置寄存器地址：配置INT2输入输出引脚（类似INT1） */
#define BMI088_INT2_IO_CTRL 0x54
/* INT2使能标志位：第3位使能INT2作为输出引脚 */
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
/* INT2输出模式：推挽输出 */
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
/* INT2输出模式：开漏输出 */
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
/* INT2电平设置：低电平 */
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
/* INT2电平设置：高电平 */
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

/* 中断映射寄存器地址：将中断映射到INT1或INT2引脚 */
#define BMI088_INT_MAP_DATA 0x58
/* INT2数据就绪中断映射：第6位将数据就绪中断映射到INT2 */
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
/* INT1数据就绪中断映射：第2位将数据就绪中断映射到INT1 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

/* 自检寄存器地址：用于加速度计自检 */
#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00               /* 关闭自检 */
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D   /* 正信号自检 */
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09   /* 负信号自检 */

/* 电源模式寄存器地址：切换加速度计电源模式 */
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03  /* 暂停模式，停止数据采集 */
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00   /* 激活模式，开始数据采集 */

/* 电源控制寄存器地址：控制加速度计开关 */
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00   /* 关闭加速度计 */
#define BMI088_ACC_ENABLE_ACC_ON 0x04    /* 开启加速度计 */

/* 软件复位寄存器地址：用于软复位加速度计 */
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6  /* 写入0xB6触发软复位 */

/*---------------------------------- gyroscope register address ----------------------------------*/
/* 陀螺仪芯片ID寄存器地址：用于读取芯片标识，值为0x0F */
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

/* 陀螺仪三轴输出寄存器地址：分别存储X、Y、Z轴的低位和高位数据 */
#define BMI088_GYRO_XOUT_L 0x02
#define BMI088_GYRO_XOUT_H 0x03
#define BMI088_GYRO_YOUT_L 0x04
#define BMI088_GYRO_YOUT_H 0x05
#define BMI088_GYRO_ZOUT_L 0x06
#define BMI088_GYRO_ZOUT_H 0x07

/* 中断状态寄存器地址：用于检查陀螺仪数据就绪中断状态 */
#define BMI088_GYRO_INT_STAT_1 0x0A
/* 数据就绪标志位：第7位表示陀螺仪数据就绪 */
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

/* 角速度量程和分辨率寄存器地址：设置陀螺仪量程 */
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  /* ±2000°/s */
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)  /* ±1000°/s */
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)   /* ±500°/s */
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)   /* ±250°/s */
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)   /* ±125°/s */

/* 数据滤波带宽和输出速率寄存器地址：设置数据传输速率和带宽 */
#define BMI088_GYRO_BANDWIDTH 0x10
/* 带宽寄存器必须置位标志 */
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
/* 数据传输速率2kHz，带宽532Hz */
#define BMI088_GYRO_2000_532_HZ 0x00
/* 数据传输速率2kHz，带宽230Hz */
#define BMI088_GYRO_2000_230_HZ 0x01
/* 数据传输速率1kHz，带宽116Hz */
#define BMI088_GYRO_1000_116_HZ 0x02
/* 数据传输速率400Hz，带宽47Hz */
#define BMI088_GYRO_400_47_HZ 0x03
/* 数据传输速率200Hz，带宽23Hz */
#define BMI088_GYRO_200_23_HZ 0x04
/* 数据传输速率100Hz，带宽12Hz */
#define BMI088_GYRO_100_12_HZ 0x05
/* 数据传输速率200Hz，带宽64Hz */
#define BMI088_GYRO_200_64_HZ 0x06
/* 数据传输速率100Hz，带宽32Hz */
#define BMI088_GYRO_100_32_HZ 0x07

/* 电源模式选择寄存器地址：设置陀螺仪工作模式 */
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00         /* 正常模式 */
#define BMI088_GYRO_SUSPEND_MODE 0x80        /* 暂停模式 */
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20   /* 深度暂停模式 */

/* 软件复位寄存器地址：用于软复位陀螺仪 */
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6     /* 写入0xB6触发软复位 */

/* 数据中断触发寄存器地址：控制新数据触发中断 */
#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00    /* 关闭数据就绪中断 */
#define BMI088_DRDY_ON 0x80     /* 开启数据就绪中断 */

/* 中断引脚配置寄存器地址：配置INT3和INT4引脚 */
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
/* INT4输出模式：第3位设置推挽输出 */
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
/* INT4输出模式：开漏输出 */
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
/* INT4电平设置：第2位设置低电平 */
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
/* INT4电平设置：高电平 */
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
/* INT3输出模式：第1位设置推挽输出 */
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
/* INT3输出模式：开漏输出 */
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
/* INT3电平设置：第0位设置低电平 */
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
/* INT3电平设置：高电平 */
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

/* 中断映射寄存器地址：将数据就绪中断映射到INT3或INT4 */
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_GYRO_DRDY_IO_OFF 0x00         /* 不映射中断 */
#define BMI088_GYRO_DRDY_IO_INT3 0x01        /* 映射到INT3 */
#define BMI088_GYRO_DRDY_IO_INT4 0x80        /* 映射到INT4 */
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4) /* 同时映射到INT3和INT4 */

/* 自检寄存器地址：用于陀螺仪自检 */
#define BMI088_GYRO_SELF_TEST 0x3C
/* 自检速率正常标志位：第4位表示速率正常 */
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
/* 自检失败标志位：第2位表示自检失败 */
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
/* 自检就绪标志位：第1位表示自检就绪 */
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
/* 自检触发标志位：第0位触发自检 */
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#endif
