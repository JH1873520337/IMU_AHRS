//
// Created by ASUS on 26-2-4.
//

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
} Quaternion;

// 常量定义：单位四元数（无旋转）
#define QUAT_IDENTITY  ((Quaternion){1.0f, 0.0f, 0.0f, 0.0f})
// 浮点精度阈值：避免浮点误差导致的判断错误
#define QUAT_EPSILON   1e-6f

Quaternion Quat_Conjugate(const Quaternion *q);
Quaternion Quat_Multiply(const Quaternion *Q1, const Quaternion *Q2);
Quaternion Quat_Normalize(const Quaternion *Q1);
Quaternion Quat_RotateVector(const Quaternion *Q, const Quaternion *V);

Quaternion Quat_FromVector(float vx, float vy, float vz);
#endif //QUATERNION_H
