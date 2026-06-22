#include "imu_app.h"
#include "zf_common_headfile.h"

#include <math.h>

#define IMU_UPDATE_PERIOD_S             (0.002f)
#define DEG_TO_RAD                      (0.01745329252f)
#define RAD_TO_DEG                      (57.29577951f)

#define MAHONY_KP                       (2.0f)
#define MAHONY_KI                       (0.0f)
#define MAHONY_INTEGRAL_LIMIT           (0.10f)
#define ACC_NORM_MIN_G                  (0.80f)
#define ACC_NORM_MAX_G                  (1.20f)
#define QUATERNION_NORM_MIN_SQ          (1.0e-6f)

#define IMU_INIT_RETRY_COUNT            (10U)
// Startup-only stationary gyro calibration; the loop waits 2 ms per sample,
// so 1000 samples keep the car still for about 2 s before imu_ready is set.
#define GYRO_CALIBRATION_SAMPLE_COUNT   (1000U)
#define GYRO_CALIBRATION_RETRY_COUNT    (3U)
#define GYRO_CALIBRATION_MAX_SPAN_DPS   (3.0f)

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static float ex_int = 0.0f;
static float ey_int = 0.0f;
static float ez_int = 0.0f;

static float gyro_zero_bias_x = 0.0f;
static float gyro_zero_bias_y = 0.0f;
static float gyro_zero_bias_z = 0.0f;

volatile bool imu_ready = false;
volatile float global_pitch = 0.0f;
volatile float global_roll = 0.0f;
volatile float global_yaw = 0.0f;
volatile float global_gyro_roll = 0.0f;

static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az);

static float constrain_float(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool float_is_valid(float value)
{
    return (value == value) && (value <= 3.402823466e+38f) && (value >= -3.402823466e+38f);
}

static void imu_reset_filter(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    ex_int = 0.0f;
    ey_int = 0.0f;
    ez_int = 0.0f;

    global_pitch = 0.0f;
    global_roll = 0.0f;
    global_yaw = 0.0f;
    global_gyro_roll = 0.0f;
}

static void imu_update_euler_angles(void)
{
    float asin_input = 2.0f * (q0 * q2 - q1 * q3);
    float pitch_rad;
    float roll_rad;
    float yaw_rad;

    asin_input = constrain_float(asin_input, -1.0f, 1.0f);

    pitch_rad = asinf(asin_input);
    roll_rad = atan2f(2.0f * (q0 * q1 + q2 * q3),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2));
    yaw_rad = atan2f(2.0f * (q0 * q3 + q1 * q2),
                     1.0f - 2.0f * (q2 * q2 + q3 * q3));

    if(float_is_valid(pitch_rad) && float_is_valid(roll_rad) && float_is_valid(yaw_rad))
    {
        global_pitch = pitch_rad * RAD_TO_DEG;
        global_roll = roll_rad * RAD_TO_DEG;
        global_yaw = yaw_rad * RAD_TO_DEG;
    }
}

static bool imu_calibrate_gyro(void)
{
    uint8 attempt;

    for(attempt = 0U; attempt < GYRO_CALIBRATION_RETRY_COUNT; attempt++)
    {
        int32 sum_x = 0;
        int32 sum_y = 0;
        int32 sum_z = 0;
        int16 min_x = 32767;
        int16 min_y = 32767;
        int16 min_z = 32767;
        int16 max_x = -32768;
        int16 max_y = -32768;
        int16 max_z = -32768;
        uint16 sample;
        float max_span_raw = GYRO_CALIBRATION_MAX_SPAN_DPS * imu660rb_transition_factor[1];

        for(sample = 0U; sample < GYRO_CALIBRATION_SAMPLE_COUNT; sample++)
        {
            imu660rb_get_gyro();

            sum_x += imu660rb_gyro_x;
            sum_y += imu660rb_gyro_y;
            sum_z += imu660rb_gyro_z;

            if(imu660rb_gyro_x < min_x) min_x = imu660rb_gyro_x;
            if(imu660rb_gyro_y < min_y) min_y = imu660rb_gyro_y;
            if(imu660rb_gyro_z < min_z) min_z = imu660rb_gyro_z;
            if(imu660rb_gyro_x > max_x) max_x = imu660rb_gyro_x;
            if(imu660rb_gyro_y > max_y) max_y = imu660rb_gyro_y;
            if(imu660rb_gyro_z > max_z) max_z = imu660rb_gyro_z;

            system_delay_ms(2);
        }

        if(((float)((int32)max_x - (int32)min_x) <= max_span_raw) &&
           ((float)((int32)max_y - (int32)min_y) <= max_span_raw) &&
           ((float)((int32)max_z - (int32)min_z) <= max_span_raw))
        {
            gyro_zero_bias_x = (float)sum_x / (float)GYRO_CALIBRATION_SAMPLE_COUNT;
            gyro_zero_bias_y = (float)sum_y / (float)GYRO_CALIBRATION_SAMPLE_COUNT;
            gyro_zero_bias_z = (float)sum_z / (float)GYRO_CALIBRATION_SAMPLE_COUNT;
            return true;
        }

        gpio_toggle_level(P20_9);
        system_delay_ms(200);
    }

    return false;
}

static bool imu_initialize_attitude_from_acc(void)
{
    float ax;
    float ay;
    float az;
    float norm_sq;
    float norm;
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;

    imu660rb_get_acc();

    ax = -(float)imu660rb_acc_y / imu660rb_transition_factor[0];
    ay = (float)imu660rb_acc_x / imu660rb_transition_factor[0];
    az = (float)imu660rb_acc_z / imu660rb_transition_factor[0];
    norm_sq = ax * ax + ay * ay + az * az;

    if(norm_sq < ACC_NORM_MIN_G * ACC_NORM_MIN_G ||
       norm_sq > ACC_NORM_MAX_G * ACC_NORM_MAX_G)
    {
        return false;
    }

    norm = 1.0f / sqrtf(norm_sq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;

    q0 = cosf(half_roll) * cosf(half_pitch);
    q1 = sinf(half_roll) * cosf(half_pitch);
    q2 = cosf(half_roll) * sinf(half_pitch);
    q3 = -sinf(half_roll) * sinf(half_pitch);

    imu_update_euler_angles();
    return true;
}

float normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

bool imu_init(void)
{
    uint8 retry;
    bool hardware_ready = false;

    imu_ready = false;
    imu_reset_filter();

    for(retry = 0U; retry < IMU_INIT_RETRY_COUNT; retry++)
    {
        if(imu660rb_init() == 0U)
        {
            hardware_ready = true;
            break;
        }

        gpio_toggle_level(P20_9);
        system_delay_ms(200);
    }

    if(!hardware_ready)
    {
        gpio_set_level(P20_9, GPIO_LOW);
        return false;
    }

    gpio_set_level(P20_9, GPIO_LOW);
    system_delay_ms(200);

    if(!imu_calibrate_gyro() || !imu_initialize_attitude_from_acc())
    {
        gpio_set_level(P20_9, GPIO_LOW);
        return false;
    }

    imu_ready = true;
    gpio_set_level(P20_9, GPIO_HIGH);
    return true;
}

void imu_proc(void)
{
    float gyro_x_mapped;
    float gyro_y_mapped;
    float gyro_z_mapped;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;

    if(!imu_ready)
    {
        global_gyro_roll = 0.0f;
        return;
    }

    imu660rb_get_acc();
    imu660rb_get_gyro();

    gyro_x_mapped = -((float)imu660rb_gyro_y - gyro_zero_bias_y);
    gyro_y_mapped = (float)imu660rb_gyro_x - gyro_zero_bias_x;
    gyro_z_mapped = (float)imu660rb_gyro_z - gyro_zero_bias_z;

    ax = -(float)imu660rb_acc_y / imu660rb_transition_factor[0];
    ay = (float)imu660rb_acc_x / imu660rb_transition_factor[0];
    az = (float)imu660rb_acc_z / imu660rb_transition_factor[0];

    global_gyro_roll = gyro_x_mapped / imu660rb_transition_factor[1];

    gx = global_gyro_roll * DEG_TO_RAD;
    gy = (gyro_y_mapped / imu660rb_transition_factor[1]) * DEG_TO_RAD;
    gz = (gyro_z_mapped / imu660rb_transition_factor[1]) * DEG_TO_RAD;

    Mahony_Update(gx, gy, gz, ax, ay, az);
}

static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float acc_norm_sq = ax * ax + ay * ay + az * az;
    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;
    float q_norm_sq;

    if(acc_norm_sq >= ACC_NORM_MIN_G * ACC_NORM_MIN_G &&
       acc_norm_sq <= ACC_NORM_MAX_G * ACC_NORM_MAX_G)
    {
        float norm = 1.0f / sqrtf(acc_norm_sq);
        float vx;
        float vy;
        float vz;
        float ex;
        float ey;
        float ez;

        ax *= norm;
        ay *= norm;
        az *= norm;

        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        if(MAHONY_KI > 0.0f)
        {
            ex_int = constrain_float(ex_int + ex * MAHONY_KI * IMU_UPDATE_PERIOD_S,
                                     -MAHONY_INTEGRAL_LIMIT,
                                     MAHONY_INTEGRAL_LIMIT);
            ey_int = constrain_float(ey_int + ey * MAHONY_KI * IMU_UPDATE_PERIOD_S,
                                     -MAHONY_INTEGRAL_LIMIT,
                                     MAHONY_INTEGRAL_LIMIT);
            ez_int = constrain_float(ez_int + ez * MAHONY_KI * IMU_UPDATE_PERIOD_S,
                                     -MAHONY_INTEGRAL_LIMIT,
                                     MAHONY_INTEGRAL_LIMIT);
        }
        else
        {
            ex_int = 0.0f;
            ey_int = 0.0f;
            ez_int = 0.0f;
        }

        gx += MAHONY_KP * ex + ex_int;
        gy += MAHONY_KP * ey + ey_int;
        gz += MAHONY_KP * ez + ez_int;
    }

    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * IMU_UPDATE_PERIOD_S);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * IMU_UPDATE_PERIOD_S);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * IMU_UPDATE_PERIOD_S);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * IMU_UPDATE_PERIOD_S);

    q_norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if(!float_is_valid(q_norm_sq) || q_norm_sq < QUATERNION_NORM_MIN_SQ)
    {
        q0 = q0_last;
        q1 = q1_last;
        q2 = q2_last;
        q3 = q3_last;
        ex_int = 0.0f;
        ey_int = 0.0f;
        ez_int = 0.0f;
        return;
    }

    {
        float norm = 1.0f / sqrtf(q_norm_sq);
        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
    }

    imu_update_euler_angles();
}
