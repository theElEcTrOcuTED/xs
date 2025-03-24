//
// Created by benea on 25-3-23.
//
//惯性导航系统 ins


#ifndef INS_H
#define INS_H

//用于标记当前INS系统工作的姿态表示方法。
typedef enum Attitude_Mode{
  ATTITUDE_MODE_QUATERNION,//表示当前工作于四元数模式下。此时，欧拉角的输入将被禁用。
  ATTITUDE_MODE_EULER,//表示当前工作于欧拉角模式下。此时，四元数的输入将被禁用。
} Attitude_Mode;

//更新当前位置。执行该函数将会利用INS系统当前的欧拉角或四元数进行一次积分和变换以更新惯性导航系统的数据。在调用该函数之前，请先更新四元数或是欧拉角。
void ins_update_pos(float ax, float ay, float az, float DT);

//更新INS系统目前存储的四元数。
void ins_update_current_quaternion(float q0, float q1, float q2, float q3);
void ins_get_position(float *px, float *py, float *pz);
void ins_get_velocity(float *vx, float *vy, float *vz);
void ins_init(Attitude_Mode mode, float v0X, float v0Y, float v0Z, float gravityAcc);
#endif //INS_H
