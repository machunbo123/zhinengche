#ifndef CODE_IMU_APP_H_
#define CODE_IMU_APP_H_

#include <stdbool.h>

bool imu_init(void);

void imu_proc(void);

extern volatile bool imu_ready;
extern volatile float global_pitch;
extern volatile float global_roll;
extern volatile float global_yaw;
extern volatile float global_gyro_roll;

//将角度归一化，走最短路径
float normalize_angle(float angle);

#endif /* CODE_IMU_APP_H_ */
