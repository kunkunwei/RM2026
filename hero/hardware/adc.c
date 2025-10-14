#include "adc.h"
#include "delay.h"
#include "stm32f4xx_hal.h"

static uint16_t get_ADC1(uint8_t ch);
static uint16_t get_ADC3(uint8_t ch);
static void temperature_ADC_Reset(void);
static void voltage_ADC_Reset(void);

// HAL库中使用extern声明
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;// 3.3f/4096.0f

void temp_ADC_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // ADC时钟和复位在HAL_Init中已完成，这里无需再重复
    // 配置温度传感器和内部参考电压通道
    HAL_ADCEx_EnableTempSensor();

    // 配置温度传感器通道
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 配置内部参考电压通道
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 启动ADC
    HAL_ADC_Start(&hadc1);
}

//ADC_BAT PF10
void voltage_ADC_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_ChannelConfTypeDef sConfig = {0};

    // 使能GPIOF时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // 配置GPIO为模拟输入
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

    // 配置ADC通道
    sConfig.Channel = ADC_CHANNEL_8; // PF10对应ADC3的通道8
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    // 启动ADC
    HAL_ADC_Start(&hadc3);
}

static void temperature_ADC_Reset(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置温度传感器通道
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void voltage_ADC_Reset(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置电池电压通道
    sConfig.Channel = ADC_CHANNEL_8; // PF10对应ADC3的通道8
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc3, &sConfig);
}

static uint16_t get_ADC1(uint8_t ch)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置ADC通道
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 启动ADC转换
    HAL_ADC_Start(&hadc1);

    // 等待转换完成
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    // 获取转换结果
    uint16_t value = HAL_ADC_GetValue(&hadc1);

    // 停止ADC
    HAL_ADC_Stop(&hadc1);

    return value;
}

static uint16_t get_ADC3(uint8_t ch)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 配置ADC通道
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    // 启动ADC转换
    HAL_ADC_Start(&hadc3);

    // 等待转换完成
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);

    // 获取转换结果
    uint16_t value = HAL_ADC_GetValue(&hadc3);

    // 停止ADC
    HAL_ADC_Stop(&hadc3);

    return value;
}

void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += get_ADC1(ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

// temperate = (adc - 0.76f) * 400.0f + 25.0f
fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate = 0;
    temperature_ADC_Reset();
    adcx = get_ADC1(ADC_CHANNEL_TEMPSENSOR);
    // temperate = (fp32)adcx * (3.3f / 4096.0f);//不使用VREFINT校准
    temperate = (fp32)adcx * voltage_vrefint_proportion;//使用VREFINT校准
    temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
    return temperate;
}

// 分压的电阻值为 200KΩ和 22KΩ，(22K Ω + 200K Ω) / 22K Ω = 10.09，
fp32 get_battery_voltage(void)
{
    uint16_t adcx = 0;
    fp32 voltage = 0;
    voltage_ADC_Reset();
    adcx = get_ADC3(ADC_CHANNEL_8);
    voltage = (fp32)adcx * voltage_vrefint_proportion;
    voltage = voltage * 10.090909090909090909090909090909f;
    return voltage;
}
