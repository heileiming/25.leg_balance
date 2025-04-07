#ifndef __KALAMAN_H_
#define __KALAMAN_H_

#include "sys.h"
#include "arm_math.h"
#include "IMUTASK.H"


#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32


typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;      // 量测与状态的关系 how measurement relates to the state
    float *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float *MatR_DiagonalElements; // 量测方差 variance for each measurement
    float *StateMinVariance;      // 最小方差 避免方差过度收敛               suppress filter excessive convergence
    uint8_t *temp;

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT,temp_F;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);
    
    // 矩阵存储空间指针
    float xhat_data[2], xhatminus_data[2];
    float u_data[4];
    float z_data[2];
    float P_data[4], Pminus_data[4];
    float F_data[4], FT_data[4],temp_F_data[4];
    float B_data[4];
    float H_data[4], HT_data[4];
    float Q_data[4];
    float R_data[4];
    float K_data[4];
    float S_data[4], temp_matrix_data[4], temp_matrix_data1[4], temp_vector_data[4], temp_vector_data1[4];
} KalmanFilter_t;




void Kalaman_feedback(KalmanFilter_t *kf,float dt,float z0,float z1);
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);


extern KalmanFilter_t Kalman0;
extern KalmanFilter_t Kalman1;
extern KalmanFilter_t Kalman2;
#endif

