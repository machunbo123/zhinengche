#ifndef CODE_SERVE_APP_H_
#define CODE_SERVE_APP_H_

#include "zf_common_headfile.h"

extern int32_t serve_mid;
extern int32_t serve_min;
extern int32_t serve_max;
extern volatile int32_t servo_final_duty;

void set_servo_duty(int16 pid_duty);

#endif /* CODE_SERVE_APP_H_ */
