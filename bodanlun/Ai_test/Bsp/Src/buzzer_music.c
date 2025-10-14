#include "buzzer_music.h"
#include "bsp_tim.h"
#include "cmsis_os.h"
#include "tim.h" // 引入tim.h以使用htim4
// 声明外部的TIM4句柄
extern TIM_HandleTypeDef htim4;


/**
  * @brief          根据频率设置蜂鸣器音高
  * @note           这是一个本地静态函数，仅在此文件内使用
  * @param[in]      freq: 要播放的音符频率 (Hz)
  * @retval         无
  */
static void buzzer_set_note(uint16_t freq)
{
    // 根据 tim.c 的初始化: TIM4 计数时钟 = 84MHz / (167+1) = 500,000 Hz
    // ARR = (计数时钟 / 目标频率) - 1
    uint32_t arr = (500000 / freq) - 1;
    // 50% 占空比
    uint32_t ccr = arr / 2;

    // 设置自动重装载值 (ARR) 来改变频率
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    // 设置比较值 (CCR) 来控制占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
    // 启动PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
  * @brief          停止蜂鸣器发声
  * @note           这是一个本地静态函数，仅在此文件内使用
  * @retval         无
  */
static void buzzer_stop_note()
{
    // 停止PWM
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}


/**
  * @brief          播放音乐
  * @param[in]      score: 乐谱指针
  * @param[in]      score_size: 乐谱大小
  * @retval         无
  */
void play_music(const music_note_t* score, uint16_t score_size)
{
    for (int i = 0; i < score_size; i++)
    {
        if (score[i].freq > 0)
        {
            // 调用新的本地函数来设置音高
            buzzer_set_note(score[i].freq);
        }
        else
        {
            // 频率为0表示休止符，调用新的本地函数停止声音
            buzzer_stop_note();
        }
        // 延迟以控制音符的持续时间
        osDelay(score[i].duration);
    }
    // 播放完毕后关闭蜂鸣器
    buzzer_stop_note();
}