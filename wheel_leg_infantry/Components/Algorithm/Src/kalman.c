/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : kalman.c
  * @brief          : 卡尔曼滤波器
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待完善
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "kalman.h"

/**
  * @brief 根据 KalmanFilter_Info_TypeDef 中的参数初始化卡尔曼滤波器。
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @param xhatSize: 状态向量维度
  * @param uSize: 控制向量维度
  * @param zSize: 测量向量维度
  * @retval none
  */
void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize)
{
    /* 更新 float/double 的字节大小 */
    kf->sizeof_float = sizeof(float);
    kf->sizeof_double = sizeof(double);

    /* 判断向量长度是否合法 */
    if(xhatSize == 0 || zSize == 0)
    {
        kf->MatStatus = ARM_MATH_LENGTH_ERROR;  
    }
    
    /* 初始化状态向量维度 */
    kf->xhatSize = xhatSize;

     /* 初始化控制向量维度 */
    kf->uSize = uSize;

    /**< 初始化测量向量维度 */
    kf->zSize = zSize;

    /* 初始化卡方检验矩阵 */
    memset(kf->ChiSquareTest.ChiSquare_Data,0,sizeof(kf->ChiSquareTest.ChiSquare_Data));
    Matrix_Init(&kf->ChiSquareTest.ChiSquare_Matrix, 1, 1, (float *)kf->ChiSquareTest.ChiSquare_Data);

    /* 初始化外部测量向量 */
    kf->MeasuredVector = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, kf->sizeof_float * zSize);

    /* 初始化 xhat */
    kf->Data.xhat = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Data.xhat, 0, kf->sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat.xhat, kf->xhatSize, 1, (float *)kf->Data.xhat);

    /* 初始化 xhatminus */
    kf->Data.xhatminus = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Data.xhatminus, 0, kf->sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat.xhatminus, kf->xhatSize, 1, (float *)kf->Data.xhatminus);

    /* 初始化测量向量 */
    kf->Data.z = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->Data.z, 0, kf->sizeof_float * zSize);
    Matrix_Init(&kf->Mat.z, kf->zSize, 1, (float *)kf->Data.z);

    if (kf->uSize != 0)
    {
				/* 初始化外部控制向量 */
				kf->ControlVector = (float *)user_malloc(kf->sizeof_float * uSize);
				memset(kf->ControlVector, 0, kf->sizeof_float * uSize);

        /* 初始化控制向量 */
        kf->Data.u = (float *)user_malloc(kf->sizeof_float * uSize);
        memset(kf->Data.u, 0, kf->sizeof_float * uSize);
        Matrix_Init(&kf->Mat.u, kf->uSize, 1, (float *)kf->Data.u);

        /* 初始化控制矩阵 */
        kf->Data.B = (float *)user_malloc(kf->sizeof_float * xhatSize * uSize);
        memset(kf->Data.B, 0, kf->sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->Mat.B, kf->xhatSize, kf->uSize, (float *)kf->Data.B);
    }

    /* 初始化状态转移矩阵 */
    kf->Data.A = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.A, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.A, kf->xhatSize, kf->xhatSize, (float *)kf->Data.A);

    kf->Data.AT = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.AT, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.AT, kf->xhatSize, kf->xhatSize, (float *)kf->Data.AT);

    /* 初始化测量矩阵 */
    kf->Data.H = (float *)user_malloc(kf->sizeof_float * zSize * xhatSize);
    memset(kf->Data.H, 0, kf->sizeof_float * zSize * xhatSize);
    Matrix_Init(&kf->Mat.H, kf->zSize, kf->xhatSize, (float *)kf->Data.H);

    kf->Data.HT = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
    memset(kf->Data.HT, 0, kf->sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat.HT, kf->xhatSize, kf->zSize, (float *)kf->Data.HT);

    /* 初始化后验协方差矩阵 */
    kf->Data.P = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.P, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.P, kf->xhatSize, kf->xhatSize, (float *)kf->Data.P);

    /* 初始化先验协方差矩阵 */
    kf->Data.Pminus = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.Pminus, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Data.Pminus);

    /* 初始化过程噪声协方差矩阵 */
    kf->Data.Q = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
    memset(kf->Data.Q, 0, kf->sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat.Q, kf->xhatSize, kf->xhatSize, (float *)kf->Data.Q);

    /* 初始化测量噪声协方差矩阵 */
    kf->Data.R = (float *)user_malloc(kf->sizeof_float * zSize * zSize);
    memset(kf->Data.R, 0, kf->sizeof_float * zSize * zSize);
    Matrix_Init(&kf->Mat.R, kf->zSize, kf->zSize, (float *)kf->Data.R);

    /* 初始化卡尔曼增益 */
    kf->Data.K = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
    memset(kf->Data.K, 0, kf->sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat.K, kf->xhatSize, kf->zSize, (float *)kf->Data.K);

    /* 初始化 K_denominator (K_denominator = H Pminus HT + R) */
    // kf->Data.K_denominator = (float *)user_malloc(kf->sizeof_float * kf->zSize * kf->zSize);
    // kf->Data.K_denominator = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    // memset(kf->Data.K_denominator, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    // Matrix_Init(&kf->Mat.K_denominator, kf->xhatSize, kf->xhatSize, (float *)kf->Data.K_denominator);
    kf->Data.K_denominator = (float *)user_malloc(kf->sizeof_float * kf->zSize * kf->zSize);
    memset(kf->Data.K_denominator, 0, kf->sizeof_float * kf->zSize * kf->zSize);
    Matrix_Init(&kf->Mat.K_denominator, kf->zSize, kf->zSize, (float *)kf->Data.K_denominator);

    /* 初始化计算缓存矩阵 */
    kf->Data.cache_matrix[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Data.cache_matrix[0],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_matrix[0], kf->xhatSize, kf->xhatSize, (float *)kf->Data.cache_matrix[0]);

    kf->Data.cache_matrix[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Data.cache_matrix[1],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_matrix[1], kf->xhatSize, kf->xhatSize, (float *)kf->Data.cache_matrix[1]);

    /* 初始化计算缓存向量 */
    kf->Data.cache_vector[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
    memset(kf->Data.cache_vector[0],0,kf->sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_vector[0], kf->xhatSize, 1, (float *)kf->Data.cache_vector[0]);

    kf->Data.cache_vector[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
    memset(kf->Data.cache_vector[1],0,kf->sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat.cache_vector[1], kf->xhatSize, 1, (float *)kf->Data.cache_vector[1]);

    /* 初始化滤波器输出 */
    kf->Output = (float *)user_malloc(kf->sizeof_float * xhatSize);
    memset(kf->Output, 0, kf->sizeof_float * xhatSize);
}
//------------------------------------------------------------------------------


/**
  * @brief 更新测量信息
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @retval none
  */
static void Kalman_Filter_Measurement_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 从外部测量向量更新测量向量 */
    memcpy(kf->Data.z, kf->MeasuredVector, kf->sizeof_float * kf->zSize);

    /* 清空外部测量向量 */
    memset(kf->MeasuredVector, 0, kf->sizeof_float * kf->zSize);

    if(kf->uSize > 0)
    {
      /* 从外部控制向量更新控制向量 */
      memcpy(kf->Data.u, kf->ControlVector, kf->sizeof_float * kf->uSize);
    }
}
//------------------------------------------------------------------------------


/**
  * @brief 更新先验估计
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @note xhatminus = A xhat(k-1) + B u(k-1)
  * @retval none
  */
static void Kalman_Filter_xhatminus_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 跳过此步骤（可由用户函数替代） */
    if(kf->SkipStep1 == 1)
    {
      return;
    }

    if(kf->uSize > 0)
    {
        /* cache_vector[0] = A xhat(k-1) */ 
        kf->Mat.cache_vector[0].numRows = kf->xhatSize;
        kf->Mat.cache_vector[0].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.xhat, &kf->Mat.cache_vector[0]);   

        /* cache_vector[1] = B u(k-1) */ 
      kf->Mat.cache_vector[1].numRows = kf->xhatSize;
      kf->Mat.cache_vector[1].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat.B, &kf->Mat.u, &kf->Mat.cache_vector[1]);    

        /* xhatminus = A xhat(k-1) + B u(k-1) */
        kf->MatStatus = Matrix_Add(&kf->Mat.cache_vector[0], &kf->Mat.cache_vector[1], &kf->Mat.xhatminus);   
    }
    /* 无控制向量时 */
    else
    {
        /* xhatminus = A xhat(k-1) */
        kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.xhat, &kf->Mat.xhatminus);   
    }
}
//------------------------------------------------------------------------------

/**
  * @brief 更新先验协方差矩阵
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @note Pminus = A P(k-1) AT + Q
  * @retval none
  */
static void Kalman_Filter_Pminus_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 跳过此步骤（可由用户函数替代） */
    if(kf->SkipStep2 == 1)
    {
      return;
    }

    /* 计算 AT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat.A, &kf->Mat.AT);

    /* Pminus = A P(k-1) */ 
    kf->MatStatus = Matrix_Multiply(&kf->Mat.A, &kf->Mat.P, &kf->Mat.Pminus); 

    /* cache_matrix[0] = A P(k-1) AT */ 
    kf->Mat.cache_matrix[0].numRows = kf->Mat.Pminus.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.AT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.AT, &kf->Mat.cache_matrix[0]); 

    /* Pminus = A P(k-1) AT + Q */
    kf->MatStatus = Matrix_Add(&kf->Mat.cache_matrix[0], &kf->Mat.Q, &kf->Mat.Pminus);  
}
//------------------------------------------------------------------------------

/**
  * @brief 更新卡尔曼增益
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @note K = H·Pminus / (H·Pminus·HT + R)
  * @retval none
  */
static void Kalman_Filter_K_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 跳过此步骤（可由用户函数替代） */
    if(kf->SkipStep3 == 1)
    {
      return;
    }

    /* 计算 HT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat.H, &kf->Mat.HT);

    /* cache_matrix[0] = H·Pminus */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.H.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.Pminus, &kf->Mat.cache_matrix[0]); 

    /* cache_matrix[1] = H·Pminus·HT */
    kf->Mat.cache_matrix[1].numRows = kf->Mat.cache_matrix[0].numRows;
    kf->Mat.cache_matrix[1].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.HT, &kf->Mat.cache_matrix[1]);  

    /* K_denominator = H·Pminus·HT + R */
    kf->Mat.K_denominator.numRows = kf->Mat.R.numRows;
    kf->Mat.K_denominator.numCols = kf->Mat.R.numCols;
    kf->MatStatus = Matrix_Add(&kf->Mat.cache_matrix[1], &kf->Mat.R, &kf->Mat.K_denominator); 

    /* cache_matrix[1] = inverse(H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Inverse(&kf->Mat.K_denominator, &kf->Mat.cache_matrix[1]);

    /* cache_matrix[0] = Pminus·HT */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.Pminus.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.HT, &kf->Mat.cache_matrix[0]);

    /* K = Pminus·HT * inverse(H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.cache_matrix[1], &kf->Mat.K);
}
//------------------------------------------------------------------------------

/**
  * @brief 更新后验估计
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @note xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  * @retval none
  */
static void Kalman_Filter_xhat_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 跳过此步骤（可由用户函数替代） */
    if(kf->SkipStep4 == 1)
    {
      return;
    }

    /* cache_vector[0] = H xhatminus */
    kf->Mat.cache_vector[0].numRows = kf->Mat.H.numRows;
    kf->Mat.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.xhatminus, &kf->Mat.cache_vector[0]);

    /* cache_vector[1] = z(k) - H·xhatminus */
    kf->Mat.cache_vector[1].numRows = kf->Mat.z.numRows;
    kf->Mat.cache_vector[1].numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->Mat.z, &kf->Mat.cache_vector[0], &kf->Mat.cache_vector[1]); 

    /* cache_vector[0] = K(k)·(z(k) - H·xhatminus) */
    kf->Mat.cache_vector[0].numRows = kf->Mat.K.numRows;
    kf->Mat.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.cache_vector[1], &kf->Mat.cache_vector[0]);

    /* xhat = xhatminus + K(k)·(z(k) - H·xhatminus) */
    kf->MatStatus = Matrix_Add(&kf->Mat.xhatminus, &kf->Mat.cache_vector[0], &kf->Mat.xhat); 
}
//------------------------------------------------------------------------------
/**
  * @brief  更新后验协方差矩阵
  * @param  kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @note   P = (I - K(k)·H)·Pminus
  * @retval none
  */
static void Kalman_Filter_P_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 跳过此步骤（可由用户函数替代） */
    if(kf->SkipStep5 == 1)
    {
      return;
    }

    /* cache_matrix[0] = K(k)·H */
    kf->Mat.cache_matrix[0].numRows = kf->Mat.K.numRows;
    kf->Mat.cache_matrix[0].numCols = kf->Mat.H.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.H, &kf->Mat.cache_matrix[0]);

    /* cache_matrix[1] = K(k)·H·Pminus */
    kf->Mat.cache_matrix[1].numRows = kf->Mat.cache_matrix[0].numRows;
    kf->Mat.cache_matrix[1].numCols = kf->Mat.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.cache_matrix[0], &kf->Mat.Pminus, &kf->Mat.cache_matrix[1]);
		
    /* P = (I - K(k)·H)·Pminus */
    kf->MatStatus = Matrix_Subtract(&kf->Mat.Pminus, &kf->Mat.cache_matrix[1], &kf->Mat.P); 
}
//------------------------------------------------------------------------------

/**
  * @brief 根据 KalmanFilter_Info_TypeDef 中的参数更新卡尔曼滤波器。
  * @param kf: 指向包含卡尔曼滤波器信息的 KalmanFilter_Info_TypeDef 结构体的指针。
  * @retval 返回卡尔曼滤波器的输出指针
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  */
float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *kf)
{
    /* 更新测量信息 */
    Kalman_Filter_Measurement_Update(kf);
    /* 用户函数 0 */
    if(kf->User_Function0 != NULL)
    {
      kf->User_Function0(kf);
    }

    /* 更新先验估计 */
    Kalman_Filter_xhatminus_Update(kf);
    /* 用户函数 1 */
    if(kf->User_Function1 != NULL)
    {
      kf->User_Function1(kf);
    }

    /* 更新先验协方差矩阵 */
    Kalman_Filter_Pminus_Update(kf);
    /* 用户函数 2 */
    if(kf->User_Function2 != NULL)
    {
      kf->User_Function2(kf);
    }
    /* 更新卡尔曼增益 */
    Kalman_Filter_K_Update(kf);
    /* 用户函数 3 */
    if(kf->User_Function3 != NULL)
    {
      kf->User_Function3(kf);
    }

    /* 更新后验估计 */
    Kalman_Filter_xhat_Update(kf);
    /* 用户函数 4 */
    if(kf->User_Function4 != NULL)
    {
      kf->User_Function4(kf);
    }

    /* 更新后验协方差矩阵 */
    Kalman_Filter_P_Update(kf);
    /* 用户函数 5 */
    if(kf->User_Function5 != NULL)
    {
      kf->User_Function5(kf);
    }

    /* 用户函数 6 */
    if(kf->User_Function6 != NULL)
    {
      kf->User_Function6(kf);
    }

    /* 更新卡尔曼滤波器输出 */
    memcpy(kf->Output, kf->Data.xhat, kf->sizeof_float * kf->xhatSize);

    return kf->Output;
}
//------------------------------------------------------------------------------
