/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : kalman.c
  * @brief          : Kalman滤波器
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待完善
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "arm_math.h"
#include "config.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief 用户内存分配宏定义，返回指向分配大小内存的指针
 */
#ifndef user_malloc
#ifdef _CMSIS_OS_H
	#define user_malloc pvPortMalloc
#else
	#define user_malloc malloc
#endif
#endif

/**
 * @brief 矩阵计算的宏定义
 */
#define matrix             arm_matrix_instance_f32
#define matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 包含Chi Square检验信息的结构体定义
 */
typedef struct
{
  bool TestFlag;    /*!< 启用/禁用标志 */
  matrix ChiSquare_Matrix;   /*!< chi square检验矩阵 */
  float ChiSquare_Data[1];    /*!< chi square检验矩阵数据 */
  float ChiSquareTestThresholds;    /*!< chi square检验矩阵阈值 */
  uint8_t ChiSquareCnt;   /*!< chi square检验计数 */
  bool result;   /*!< chi square检验结果 */
}ChiSquareTest_Typedef;

/**
 * @brief 包含kalman滤波器信息的结构体定义
 */
typedef struct KF_Info_TypeDef
{
  uint16_t sizeof_float, sizeof_double; /*!< float/double的大小 */

  uint8_t xhatSize;   /*!< 状态向量维度 */
  uint8_t uSize;      /*!< 控制向量维度 */
  uint8_t zSize;      /*!< 测量向量维度 */

  float dt;   /*!< 更新周期 */
  float *MeasuredVector; /*!< 外部测量向量指针 */
  float *ControlVector;  /*!< 外部控制向量指针 */

  ChiSquareTest_Typedef ChiSquareTest;  /*!< Chi Square检验 */

  /**
   * @brief 浮点矩阵结构的实例结构体
   */
  struct 
  {
    matrix xhat;              /*!< 后验估计矩阵 */
    matrix xhatminus;         /*!< 先验估计矩阵 */
    matrix u;                 /*!< 控制向量 */
    matrix z;                 /*!< 测量向量 */
    matrix B;                 /*!< 控制矩阵 */
    matrix A,AT;              /*!< 状态转移矩阵 */
    matrix H,HT;              /*!< 测量矩阵 */
    matrix P;                 /*!< 后验协方差矩阵 */
    matrix Pminus;            /*!< 先验协方差矩阵 */
    matrix Q;                 /*!< 过程噪声协方差矩阵 */
    matrix R;                 /*!< 测量噪声协方差矩阵 */
    matrix K;                 /*!< kalman增益矩阵 */
    matrix K_denominator;     /*!< K_denominator矩阵 (K_denominator = H Pminus HT + R) */
    matrix cache_matrix[2];   /*!< 计算缓存矩阵 */
    matrix cache_vector[2];   /*!< 计算缓存向量 */
  }Mat;

  arm_status MatStatus;   /*!< 错误状态 */

  /**
   * @brief 浮点矩阵数据指针的实例结构体
   */
  struct 
  {
    float *xhat,*xhatminus;   /*!< 后验/先验估计矩阵内存指针 */
    float *u;                 /*!< 控制向量内存指针 */
    float *z;                 /*!< 测量向量内存指针 */
    float *B;                 /*!< 控制矩阵内存指针 */
    float *A,*AT;             /*!< 状态转移矩阵内存指针 */
    float *H,*HT;             /*!< 测量矩阵内存指针 */
    float *P;                 /*!< 后验协方差矩阵内存指针 */
    float *Pminus;            /*!< 先验协方差矩阵内存指针 */
    float *Q;                 /*!< 过程噪声协方差矩阵内存指针 */
    float *R;                 /*!< 测量噪声协方差矩阵内存指针 */
    float *K;                 /*!< kalman增益矩阵内存指针 */
    float *K_denominator;     /*!< K_denominator矩阵内存指针 */
    float *cache_matrix[2];   /*!< 计算缓存矩阵内存指针 */
    float *cache_vector[2];   /*!< 计算缓存向量内存指针 */
  }Data;

  uint8_t SkipStep1 : 1;  /*!< 跳过kalman滤波器更新第一步的标志 */
  uint8_t SkipStep2 : 1;  /*!< 跳过kalman滤波器更新第二步的标志 */
  uint8_t SkipStep3 : 1;  /*!< 跳过kalman滤波器更新第三步的标志 */
  uint8_t SkipStep4 : 1;  /*!< 跳过kalman滤波器更新第四步的标志 */
  uint8_t SkipStep5 : 1;  /*!< 跳过kalman滤波器更新第五步的标志 */
  uint8_t reserve   : 3;

  /**
   * @brief 可以替代kalman滤波器更新任何步骤的用户函数
   */
  void (*User_Function0)(struct KF_Info_TypeDef *kf);
  void (*User_Function1)(struct KF_Info_TypeDef *kf);
  void (*User_Function2)(struct KF_Info_TypeDef *kf);
  void (*User_Function3)(struct KF_Info_TypeDef *kf);
  void (*User_Function4)(struct KF_Info_TypeDef *kf);
  void (*User_Function5)(struct KF_Info_TypeDef *kf);
  void (*User_Function6)(struct KF_Info_TypeDef *kf);

  float *Output;  /*!< kalman滤波器输出 */

}KalmanFilter_Info_TypeDef;

/**
 * @brief 简化版卡尔曼滤波器结构体定义（用于单维度滤波）
 */
typedef struct
{
    float x;     /*!< 状态估计值 */
    float P;     /*!< 误差协方差 */
    float Q;     /*!< 过程噪声协方差 */
    float R;     /*!< 测量噪声协方差 */
    float K;     /*!< 卡尔曼增益 */
    float out;   /*!< 滤波输出 */
} KalmanInfo;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief 根据KalmanFilter_Info_TypeDef中指定的参数初始化kalman滤波器
  */
extern void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize);
/**
  * @brief 根据KalmanFilter_Info_TypeDef中指定的参数更新Kalman滤波器
  */
extern float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *kf);


  void Kalman_Filter_Simple_Init(KalmanInfo *kf);
  float Kalman_Filter_Fun(KalmanInfo *kf, float input);
#endif //KALMAN_FILTER_H
