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

void ins_update_pos(float ax, float ay, float az, float DT);
#endif //INS_H
