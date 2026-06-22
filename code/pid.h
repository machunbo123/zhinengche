#ifndef __PID_H
#define __PID_H

/* pid结构体 */
typedef struct
{
    float kp;                   /* 比例 */
    float ki;                   /* 积分 */
    float kd;                   /* 微分 */
    float target;               /* 目标值 */
    float current;              /* 当前值 */
    float out;                  /* 执行量 */
    float limit;                /* PID(out)输出限幅值 */
    float integral_limit;       /* 积分项限幅值 */
    float d_filter_alpha;       /* D项低通滤波系数 0~1，1=不滤波 */

    float error;                /* 当前误差 */
    float last_error;           /* 上一次误差 */
    float last2_error;          /* 上上次误差 */
    float last_out;             /* 上一次执行量 */
    float integral;             /* 积分（累加） */
    float p_out,i_out,d_out;    /* 比例、积分、微分值 */
}PID_T;

/*
    提供给用户调用的API
*/
/* PID初始化 */
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit);

/* 设置PID目标值 */
void pid_set_target(PID_T * _tpPID, float _target);

/* 设置PID参数 */
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd);

/* 设置PID输出限幅 */
void pid_set_limit(PID_T * _tpPID, float _limit);

/* 设置积分限幅值 */
void pid_set_integral_limit(PID_T * _tpPID, float _integral_limit);

/* 设置D项低通滤波系数 */
void pid_set_d_filter_alpha(PID_T * _tpPID, float _alpha);

/* 重置PID控制器 */
void pid_reset(PID_T * _tpPID);

/* 计算位置式PID */
float pid_calculate_positional(PID_T * _tpPID, float _current);

/* 计算增量式PID */
float pid_calculate_incremental(PID_T * _tpPID, float _current);

/* 根据外部误差计算位置式PID（适合航向角等需要外部处理角度环绕的场景） */
float pid_calculate_by_error(PID_T * _tpPID, float _error);

/* 限幅函数 */
float pid_constrain(float value, float min, float max);

/* 积分限幅函数 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max);

#endif
