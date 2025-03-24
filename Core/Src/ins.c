//
// Created by benea on 25-3-23.
//

#include "ins.h"

//https://www.cnblogs.com/WangHongxi/p/12357230.html
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct {
    float pitch;
    float yaw;
    float roll;
} EulerAngle;

typedef struct {
    float x;
    float y;
    float z;
} Vector3; //辅助结构体：三维向量

Attitude_Mode attitudeMode; //表示当前工作状态
Quaternion current_quaternion; //当前姿态（四元数形式）
EulerAngle current_eulerangle; //当前姿态（欧拉角模式）

Vector3 v0;//初速度（大地坐标系下）
Vector3 pos;//当前速度（大地坐标系下）
Vector3 vel;//当前位置（大地坐标系下）
float gravity;//重力大小的绝对值


void ins_init(Attitude_Mode mode, float v0X, float v0Y, float v0Z, float gravityAcc) {
    attitudeMode = mode;
    v0.x = v0X;
    v0.y = v0Y;
    v0.z = v0Z;
    pos.x = 0;
    pos.y = 0;
    pos.z = 0;
    gravity = gravityAcc;
}

//辅助函数，用来将指定三维向量从大地坐标系转换到设备坐标系，用于启用四元数时
Vector3 quaternion_cast_to_device(Vector3 v3) {
    Vector3 v;
    v.x =
            v3.x * (1 - 2 * (current_quaternion.q2 * current_quaternion.q2 + current_quaternion.q3 * current_quaternion.
                             q3))
            + v3.y * (2 * (current_quaternion.q1 * current_quaternion.q2 + current_quaternion.q0 * current_quaternion.
                           q3))
            + v3.z * (2 * (current_quaternion.q1 * current_quaternion.q3 - current_quaternion.q0 * current_quaternion.
                           q2));
    v.y =
            v3.x * (2 * (current_quaternion.q1 * current_quaternion.q2 - current_quaternion.q0 * current_quaternion.q3))
            + v3.y * (1 - 2 * (current_quaternion.q1 * current_quaternion.q1 + current_quaternion.q3 *
                               current_quaternion.q3))
            + v3.z * (2 * (current_quaternion.q2 * current_quaternion.q3 + current_quaternion.q0 * current_quaternion.
                           q1));
    v.z =
            v3.x * (2 * (current_quaternion.q1 * current_quaternion.q3 + current_quaternion.q0 * current_quaternion.q2))
            + v3.y * (2 * (current_quaternion.q2 * current_quaternion.q3 - current_quaternion.q0 * current_quaternion.
                           q1))
            + v3.z * (1 - 2 * (current_quaternion.q1 * current_quaternion.q1 + current_quaternion.q2 *
                               current_quaternion.q2));
    return v;
}

Vector3 euler_cast_to_device() {
}

Vector3 quaternion_cast_to_ground(Vector3 v3) {
    Vector3 v;
    v.x =
        v3.x*(1-2*(current_quaternion.q2*current_quaternion.q2+current_quaternion.q3*current_quaternion.q3))
        +v3.y*(2*(current_quaternion.q1*current_quaternion.q2 - current_quaternion.q0 * current_quaternion.q3))
        +v3.z*(2*(current_quaternion.q1*current_quaternion.q3+current_quaternion.q0 * current_quaternion.q2));
    v.y =
        v3.x*(2*(current_quaternion.q1*current_quaternion.q2+current_quaternion.q0* current_quaternion.q3))
        +v3.y*(1-2*(current_quaternion.q1*current_quaternion.q1+current_quaternion.q3*current_quaternion.q3))
        +v3.z*(2*(current_quaternion.q2*current_quaternion.q3-current_quaternion.q0 * current_quaternion.q1));
    v.z =
        v3.x*(2*(current_quaternion.q1*current_quaternion.q3-current_quaternion.q0*current_quaternion.q2))
        +v3.y*(2*(current_quaternion.q2*current_quaternion.q3+current_quaternion.q0 * current_quaternion.q1))
        +v3.z*(1-2*(current_quaternion.q1*current_quaternion.q1+current_quaternion.q2*current_quaternion.q2));
    return v;
}

Vector3 euler_cast_to_ground() {
}



void ins_update_current_quaternion(float q0, float q1, float q2, float q3) {
    current_quaternion.q0 = q0;
    current_quaternion.q1 = q1;
    current_quaternion.q2 = q2;
    current_quaternion.q3 = q3;
}

void ins_update_current_euler(float pitch, float roll, float yaw) {
    current_eulerangle.roll = roll;
    current_eulerangle.pitch = pitch;
    current_eulerangle.yaw = yaw;
}

void ins_update_pos(float ax, float ay, float az, float DT) {
    //先将用姿态角估计的重力分量转换到设备坐标系
    Vector3 gravity_delta = quaternion_cast_to_device({0,0,-1*gravity});
    //在设备坐标系下，去除重力分量
    ax -= gravity_delta.x;
    ay -= gravity_delta.y;
    az -= gravity_delta.z;
    //将上一时刻速度转换到设备坐标系
    Vector3 current_velocity = quaternion_cast_to_device(vel);
    //在设备坐标系下，将加速度积分到速度
    current_velocity.x += ax * DT;
    current_velocity.y += ay * DT;
    current_velocity.z += az * DT;
    //将上一时刻位移转换到设备坐标系
    Vector3 current_position = quaternion_cast_to_device(pos);
    //在设备坐标系下，将速度积分到位移
    current_position.x += current_velocity.x * DT;
    current_position.y += current_velocity.y * DT;
    current_position.z += current_velocity.z * DT;
    //把设备坐标系下的速度和位移数据转换到大地坐标系并更新当前速度和位移
    vel = quaternion_cast_to_ground(current_velocity);
    pos = quaternion_cast_to_ground(current_position);
}
void ins_get_velocity(float *vx, float *vy, float *vz) {
    *vx = vel.x;
    *vy = vel.y;
    *vz = vel.z;
}
void ins_get_position(float *px, float *py, float *pz) {
    *px = pos.x;
    *py = pos.y;
    *pz = pos.z;

}
