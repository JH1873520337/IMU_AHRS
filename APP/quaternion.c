//
// Created by ASUS on 26-2-4.
//

#include "quaternion.h"

// 快速开方，结果为根号下x分之一
static float invSqrt(float x)
{
    float half_x = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1); // 魔数（不用纠结，用就行）
    y = *(float*)&i;
    y = y * (1.5f - (half_x * y * y));
    return y;
}

/**
 * 生成共轭四元数
 * @param q 输入一个四元数
 * @return 共轭四元数
 */
Quaternion Quat_Conjugate(const Quaternion *q)
{
    Quaternion result = *q;
    result.q2 = -q->q2;
    result.q3 = -q->q3;
    result.q4 = -q->q4;
    return result;
}

/**
 * 四元数乘法，表示Q1⊗Q2
 * @param Q1 四元数1
 * @param Q2 四元数2
 * @return Q1⊗Q2的结果
 */
Quaternion Quat_Multiply(const Quaternion *Q1, const Quaternion *Q2)
{
    Quaternion result = *Q1;
    result.q1 = Q1->q1*Q2->q1 - Q1->q2*Q2->q2 - Q1->q3*Q2->q3 - Q1->q4*Q2->q4;
    result.q2 = Q1->q1*Q2->q2 + Q1->q2*Q2->q1 + Q1->q3*Q2->q4 - Q1->q4*Q2->q3;
    result.q3 = Q1->q1*Q2->q3 - Q1->q2*Q2->q4 + Q1->q3*Q2->q1 + Q1->q4*Q2->q2;
    result.q4 = Q1->q1*Q2->q4 + Q1->q2*Q2->q3 - Q1->q3*Q2->q2 + Q1->q4*Q2->q1;
    return result;
}

/**
 * 四元数归一化
 * @param Q1 要做归一化的四元数
 * @return  归一化的四元数
 */
Quaternion Quat_Normalize(const Quaternion *Q1)
{
    float M_Q1_2 = (Q1->q1)*(Q1->q1)+(Q1->q2)*(Q1->q2)+(Q1->q3)*(Q1->q3)+(Q1->q4)*(Q1->q4);
    float inv_norm = invSqrt(M_Q1_2);
    Quaternion result = *Q1;
    result.q1 = Q1->q1 * inv_norm ;
    result.q2 = Q1->q2 * inv_norm ;
    result.q3 = Q1->q3 * inv_norm ;
    result.q4 = Q1->q4 * inv_norm ;
    return result;
}

/**
 * 四元数旋转向量（核心：Q⊗V⊗Q*）
 * @param Q  旋转四元数（会先归一化）
 * @param V  待旋转向量封装的纯四元数（格式：q1=0, q2=vx, q3=vy, q4=vz）
 * @return   旋转后的向量对应的纯四元数
 */
Quaternion Quat_RotateVector(const Quaternion *Q, const Quaternion *V)
{
    // 1. 确保旋转四元数Q是单位四元数（关键：旋转四元数必须归一化）
    Quaternion Q_norm = Quat_Normalize(Q);

    // 2. 计算Q的共轭四元数Q*
    Quaternion Q_conj = Quat_Conjugate(&Q_norm);

    // 3. 分步计算：第一步 Q⊗V
    Quaternion QV = Quat_Multiply(&Q_norm, V);

    // 4. 第二步 (Q⊗V)⊗Q*，得到旋转后的纯四元数
    Quaternion V_rot = Quat_Multiply(&QV, &Q_conj);

    return V_rot;
}

/**
 * 从三维向量构造纯四元数（q1=0, q2=vx, q3=vy, q4=vz）
 * @param vx 向量X分量
 * @param vy 向量Y分量
 * @param vz 向量Z分量
 * @return 纯四元数
 */
Quaternion Quat_FromVector(float vx, float vy, float vz)
{
    Quaternion q;
    q.q1 = 0.0f;
    q.q2 = vx;
    q.q3 = vy;
    q.q4 = vz;
    return q;
}
