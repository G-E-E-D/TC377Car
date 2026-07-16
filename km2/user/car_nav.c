#include "car_nav.h"
#include <math.h>

#pragma section all "cpu0_dsram"

#define CAR_EARTH_RADIUS_M             (6378137.0)
#define CAR_EKF_STATE_DIM              (6)
#define CAR_EKF_X                      (0)
#define CAR_EKF_Y                      (1)
#define CAR_EKF_YAW                    (2)
#define CAR_EKF_SPEED                  (3)
#define CAR_EKF_GYRO_BIAS              (4)
#define CAR_EKF_ACC_BIAS               (5)

car_path_point_struct car_path[CAR_PATH_MAX_POINTS];

volatile car_state_enum car_state = CAR_STATE_SELF_TEST;
volatile car_replay_mode_enum car_replay_mode = CAR_REPLAY_NONE;
volatile uint32 car_elapsed_ms = 0;
volatile uint8 car_key1_level = CAR_KEY_RELEASE_LEVEL;
volatile uint8 car_key2_level = CAR_KEY_RELEASE_LEVEL;
volatile uint8 car_mpu6050_ok = 0;
volatile uint8 car_gps_valid = 0;
volatile uint8 car_gps_satellites = 0;
volatile uint8 car_error_code = CAR_ERROR_NONE;

volatile int32 car_steer_encoder_count = 0;
volatile float car_steer_angle_deg = 0.0f;
volatile float car_model_yaw_rate_dps = 0.0f;

volatile float car_gyro_z_dps = 0.0f;
volatile float car_gyro_z_bias = 0.0f;
volatile float car_acc_forward_mps2 = 0.0f;
volatile float car_acc_forward_bias_g = 0.0f;
volatile float car_pose_x_m = 0.0f;
volatile float car_pose_y_m = 0.0f;
volatile float car_yaw_deg = 0.0f;
volatile float car_speed_mps = 0.0f;
volatile float car_gps_x_m = 0.0f;
volatile float car_gps_y_m = 0.0f;
volatile float car_gps_speed_mps = 0.0f;
volatile float car_gps_direction_deg = 0.0f;
volatile uint32 car_gps_fix_count = 0;
volatile float car_ekf_pos_residual_m = 0.0f;
volatile float car_ekf_speed_residual_mps = 0.0f;
volatile float car_ekf_yaw_residual_deg = 0.0f;
volatile uint32 car_ekf_gps_update_count = 0;
volatile float car_record_s_m = 0.0f;
volatile float car_replay_s_m = 0.0f;
volatile uint32 car_path_duration_ms = 0;
volatile uint8 car_path_has_gps = 0;

volatile uint32 car_path_count = 0;
volatile uint32 car_nearest_index = 0;
volatile uint32 car_target_index = 0;
volatile float car_cross_track_error_m = 0.0f;
volatile float car_heading_error_deg = 0.0f;
volatile float car_yaw_rate_error_dps = 0.0f;
volatile float car_lookahead_m = CAR_LOOKAHEAD_MIN_M;
volatile float car_target_speed_mps = 0.0f;
volatile float car_target_steer_count = 0.0f;
volatile float car_pure_pursuit_curvature = 0.0f;
volatile float car_replay_avg_speed_mps = 0.0f;
volatile int16 car_fixed_rear_pwm = 0;
volatile float car_pwm_sim_x_m = 0.0f;
volatile float car_pwm_sim_y_m = 0.0f;
volatile float car_pwm_sim_yaw_deg = 0.0f;
volatile float car_pwm_sim_speed_mps = 0.0f;
volatile float car_pwm_sim_steer_angle_deg = 0.0f;
volatile uint32 car_pwm_sim_nearest_index = 0;
volatile uint8 car_pwm_sim_gps_dir_locked = 0;
volatile int16 car_steer_pwm_output = 0;
volatile int16 car_rear_pwm_output = 0;
volatile int8 car_drive_direction = 0;
volatile uint8 car_track_correction_active = 0;
volatile uint8 car_self_test_step = 0;

static double gps_origin_lat = 0.0;
static double gps_origin_lon = 0.0;
static uint8 gps_origin_ready = 0;
static float gps_filter_x[CAR_GPS_FILTER_WINDOW] = {0.0f};
static float gps_filter_y[CAR_GPS_FILTER_WINDOW] = {0.0f};
static uint8 gps_filter_count = 0;
static uint8 gps_filter_index = 0;
static volatile uint8 gps_valid_fix_streak = 0;
static volatile uint32 gps_last_update_ms = 0;
static volatile uint32 nav_uptime_ms = 0;
static uint32 gps_stop_last_fix = 0;
static uint32 last_record_ms = 0;
static float last_record_x = 0.0f;
static float last_record_y = 0.0f;
static float last_record_yaw_deg = 0.0f;
static float last_record_steer_count = 0.0f;
static int16 rear_pwm_ramped = 0;
static uint16 imu_still_count = 0;
static float ekf_state[CAR_EKF_STATE_DIM] = {0.0f};
static float ekf_p[CAR_EKF_STATE_DIM][CAR_EKF_STATE_DIM] = {{0.0f}};
static volatile uint8 gps_update_pending = 0;
static volatile float gps_pending_x_m = 0.0f;
static volatile float gps_pending_y_m = 0.0f;
static volatile float gps_pending_speed_mps = 0.0f;
static volatile float gps_pending_yaw_deg = 0.0f;
static float guide_start_gps_x_m = 0.0f;
static float guide_start_gps_y_m = 0.0f;
static uint32 self_test_step_start_ms = 0;
static uint8 self_test_settle_count = 0;
static uint16 gps_priority_stop_count = 0;
static volatile uint8 car_path_prepared = 0;
static uint8 finish_endpoint_fix_count = 0;
static uint32 finish_last_fix = 0;

static car_pid_struct steer_pos_pid = {
        CAR_STEER_POS_KP, CAR_STEER_POS_KI, CAR_STEER_POS_KD,
        CAR_STEER_INTEGRAL_LIMIT, CAR_STEER_PWM_LIMIT,
        0.0f, 0.0f, 0.0f
};

static car_pid_struct track_pid = {
        CAR_TRACK_CORRECTION_KP, CAR_TRACK_CORRECTION_KI, CAR_TRACK_CORRECTION_KD,
        CAR_TRACK_INTEGRAL_LIMIT, CAR_TRACK_CORRECTION_LIMIT,
        0.0f, 0.0f, 0.0f
};

static void car_path_calc_fixed_rear_pwm(void);
static void car_gps_filter_reset(void);
static void car_path_prepare(void);

static float car_abs_f(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float car_limit_f(float value, float limit)
{
    if(value > limit)  return limit;
    if(value < -limit) return -limit;
    return value;
}

static int16 car_limit_i16(int32 value, int16 limit)
{
    if(value > limit)  return limit;
    if(value < -limit) return -limit;
    return (int16)value;
}

static float car_wrap_deg(float angle)
{
    while(angle > 180.0f)  angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static float car_angle_to_360(float angle)
{
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f)    angle += 360.0f;
    return angle;
}

static uint8 car_replay_is_return(void)
{
    return (CAR_REPLAY_B_RETURN == car_replay_mode) ? 1 : 0;
}

static float car_path_travel_yaw(uint32 index)
{
    float yaw_deg;

    if(index >= car_path_count)
    {
        return car_yaw_deg;
    }
    yaw_deg = car_path[index].yaw_deg;
    if(car_replay_is_return())
    {
        yaw_deg += 180.0f;
    }
    return car_angle_to_360(yaw_deg);
}

static uint32 car_replay_endpoint_index(void)
{
    if(0 == car_path_count)
    {
        return 0;
    }
    return car_replay_is_return() ? 0 : (car_path_count - 1);
}

static float car_gps_course_to_math_yaw(float gps_course_deg)
{
    return car_angle_to_360(90.0f - gps_course_deg);
}

static void car_gps_filter_reset(void)
{
    uint8 i;

    for(i = 0; i < CAR_GPS_FILTER_WINDOW; i++)
    {
        gps_filter_x[i] = 0.0f;
        gps_filter_y[i] = 0.0f;
    }
    gps_filter_count = 0;
    gps_filter_index = 0;
    car_gps_x_m = 0.0f;
    car_gps_y_m = 0.0f;
}

static float car_gps_median(const float *samples, uint8 count)
{
    uint8 i;
    uint8 j;
    float temp[CAR_GPS_FILTER_WINDOW];
    float value;

    for(i = 0; i < count; i++)
    {
        temp[i] = samples[i];
    }
    for(i = 1; i < count; i++)
    {
        value = temp[i];
        j = i;
        while((j > 0) && (temp[j - 1] > value))
        {
            temp[j] = temp[j - 1];
            j--;
        }
        temp[j] = value;
    }
    return temp[count / 2];
}

static void car_gps_filter_update(float raw_x, float raw_y)
{
    float median_x;
    float median_y;

    gps_filter_x[gps_filter_index] = raw_x;
    gps_filter_y[gps_filter_index] = raw_y;
    gps_filter_index++;
    if(gps_filter_index >= CAR_GPS_FILTER_WINDOW)
    {
        gps_filter_index = 0;
    }
    if(gps_filter_count < CAR_GPS_FILTER_WINDOW)
    {
        gps_filter_count++;
    }

    median_x = car_gps_median(gps_filter_x, gps_filter_count);
    median_y = car_gps_median(gps_filter_y, gps_filter_count);
    if(1 == gps_filter_count)
    {
        car_gps_x_m = median_x;
        car_gps_y_m = median_y;
    }
    else
    {
        car_gps_x_m += (median_x - car_gps_x_m) * CAR_GPS_FILTER_ALPHA;
        car_gps_y_m += (median_y - car_gps_y_m) * CAR_GPS_FILTER_ALPHA;
    }
}

static int8 car_speed_sign(float speed)
{
    if(speed > CAR_SIGNED_SPEED_DEADBAND_MPS)  return 1;
    if(speed < -CAR_SIGNED_SPEED_DEADBAND_MPS) return -1;
    return 0;
}

static float car_steer_count_to_angle_deg(float steer_count)
{
    float angle;

    angle = steer_count * (CAR_STEER_MAX_ANGLE_DEG / CAR_STEER_ENC_MAX_ABS_COUNT) * CAR_STEER_ANGLE_SIGN;
    return car_limit_f(angle, CAR_STEER_MAX_ANGLE_DEG);
}

static float car_steer_model_yaw_rate_dps(float speed_mps, float steer_count)
{
#if CAR_STEER_MODEL_ENABLE
    float steer_angle_rad;
    float yaw_rate_dps;

    if(car_abs_f(speed_mps) < CAR_STEER_MODEL_MIN_SPEED_MPS)
    {
        return 0.0f;
    }

    steer_angle_rad = car_steer_count_to_angle_deg(steer_count) * CAR_DEG_TO_RAD;
    yaw_rate_dps = speed_mps / CAR_WHEELBASE_M * (float)tan(steer_angle_rad) * CAR_RAD_TO_DEG;
    return car_limit_f(yaw_rate_dps, CAR_STEER_MODEL_MAX_YAW_DPS);
#else
    (void)speed_mps;
    (void)steer_count;
    return 0.0f;
#endif
}

static void car_pwm_sim_reset(void)
{
    car_pwm_sim_x_m = 0.0f;
    car_pwm_sim_y_m = 0.0f;
    car_pwm_sim_yaw_deg = 0.0f;
    car_pwm_sim_speed_mps = 0.0f;
    car_pwm_sim_steer_angle_deg = 0.0f;
    car_pwm_sim_nearest_index = 0;
    car_pwm_sim_gps_dir_locked = 0;
}

static void car_pwm_sim_update(void)
{
#if CAR_GUIDE_PWM_SIM_ENABLE
    float duty_abs;
    float speed;
    float yaw_rad;
    float steer_rad;
    float yaw_rate_dps;
    float steer_count;

    duty_abs = car_abs_f((float)car_rear_pwm_output);
    if(duty_abs < (float)CAR_GUIDE_PWM_SIM_MIN_DUTY)
    {
        car_pwm_sim_speed_mps = 0.0f;
        return;
    }

    speed = duty_abs / (float)CAR_REAR_DUTY_LIMIT * CAR_GUIDE_PWM_SIM_SPEED_MPS;
    speed = car_limit_f(speed, CAR_GUIDE_PWM_SIM_MAX_SPEED);
    if(car_rear_pwm_output < 0)
    {
        speed = -speed;
    }
    speed *= CAR_GUIDE_PWM_SIM_DIR_SIGN;

#if CAR_GUIDE_PWM_SIM_USE_TARGET
    steer_count = car_target_steer_count;
#else
    steer_count = (float)car_steer_encoder_count;
#endif
    car_pwm_sim_steer_angle_deg = car_steer_count_to_angle_deg(steer_count);
    steer_rad = car_pwm_sim_steer_angle_deg * CAR_DEG_TO_RAD;
    yaw_rate_dps = speed / CAR_WHEELBASE_M * (float)tan(steer_rad) * CAR_RAD_TO_DEG;
    yaw_rate_dps = car_limit_f(yaw_rate_dps, CAR_STEER_MODEL_MAX_YAW_DPS);
    car_pwm_sim_yaw_deg = car_wrap_deg(car_pwm_sim_yaw_deg + yaw_rate_dps * CAR_CONTROL_PERIOD_S);

    yaw_rad = car_pwm_sim_yaw_deg * CAR_DEG_TO_RAD;
    car_pwm_sim_x_m += speed * (float)cos(yaw_rad) * CAR_CONTROL_PERIOD_S;
    car_pwm_sim_y_m += speed * (float)sin(yaw_rad) * CAR_CONTROL_PERIOD_S;
    car_pwm_sim_speed_mps = speed;
#endif
}

static void car_pwm_sim_try_lock_gps_direction(void)
{
#if CAR_GUIDE_SIM_GPS_DIR_ENABLE
    float dx;
    float dy;
    float dist2;
    float min_dist2;

    if(car_pwm_sim_gps_dir_locked || !car_gps_valid)
    {
        return;
    }

    dx = car_gps_x_m - guide_start_gps_x_m;
    dy = car_gps_y_m - guide_start_gps_y_m;
    dist2 = dx * dx + dy * dy;
    min_dist2 = CAR_GUIDE_SIM_GPS_DIR_MIN_M * CAR_GUIDE_SIM_GPS_DIR_MIN_M;
    if(dist2 >= min_dist2)
    {
        car_pwm_sim_yaw_deg = (float)atan2(dy, dx) * CAR_RAD_TO_DEG;
        car_pwm_sim_gps_dir_locked = 1;
    }
#endif
}

float car_pid_calc(car_pid_struct *pid, float target, float current)
{
    float output;

    pid->error = target - current;
    pid->integral += pid->error;
    pid->integral = car_limit_f(pid->integral, pid->integral_limit);

    output = pid->kp * pid->error
           + pid->ki * pid->integral
           + pid->kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    return car_limit_f(output, pid->output_limit);
}

static void car_pid_clear(car_pid_struct *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}

static void car_self_test_next_step(uint8 step)
{
    car_self_test_step = step;
    self_test_step_start_ms = car_elapsed_ms;
    self_test_settle_count = 0;
    car_pid_clear(&steer_pos_pid);
}

static void car_motor_set(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int16 duty, gpio_level_enum forward_level, gpio_level_enum reverse_level)
{
    uint32 pwm_duty;

    if(duty >= 0)
    {
        gpio_set_level(dir_pin, forward_level);
        pwm_duty = (uint32)duty;
    }
    else
    {
        gpio_set_level(dir_pin, reverse_level);
        pwm_duty = (uint32)(-duty);
    }

    if(pwm_duty > PWM_DUTY_MAX)
    {
        pwm_duty = PWM_DUTY_MAX;
    }
    pwm_set_duty(pwm_ch, pwm_duty);
}

static void car_ekf_sync_outputs(void)
{
    car_pose_x_m = ekf_state[CAR_EKF_X];
    car_pose_y_m = ekf_state[CAR_EKF_Y];
    car_yaw_deg = car_angle_to_360(ekf_state[CAR_EKF_YAW] * CAR_RAD_TO_DEG);
    car_speed_mps = car_limit_f(ekf_state[CAR_EKF_SPEED], CAR_IMU_SPEED_LIMIT_MPS);
}

static void car_ekf_init_state(float x, float y, float yaw_deg, float speed_mps)
{
    uint8 i;
    uint8 j;

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        ekf_state[i] = 0.0f;
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            ekf_p[i][j] = 0.0f;
        }
    }

    ekf_state[CAR_EKF_X] = x;
    ekf_state[CAR_EKF_Y] = y;
    ekf_state[CAR_EKF_YAW] = yaw_deg * CAR_DEG_TO_RAD;
    ekf_state[CAR_EKF_SPEED] = speed_mps;
    ekf_state[CAR_EKF_GYRO_BIAS] = 0.0f;
    ekf_state[CAR_EKF_ACC_BIAS] = 0.0f;

    ekf_p[CAR_EKF_X][CAR_EKF_X] = 0.25f;
    ekf_p[CAR_EKF_Y][CAR_EKF_Y] = 0.25f;
    ekf_p[CAR_EKF_YAW][CAR_EKF_YAW] = 0.030f;
    ekf_p[CAR_EKF_SPEED][CAR_EKF_SPEED] = 0.20f;
    ekf_p[CAR_EKF_GYRO_BIAS][CAR_EKF_GYRO_BIAS] = 0.010f;
    ekf_p[CAR_EKF_ACC_BIAS][CAR_EKF_ACC_BIAS] = 0.25f;

    car_ekf_pos_residual_m = 0.0f;
    car_ekf_speed_residual_mps = 0.0f;
    car_ekf_yaw_residual_deg = 0.0f;
    car_ekf_gps_update_count = 0;
    car_ekf_sync_outputs();
}

static void car_ekf_predict(float gyro_dps, float acc_mps2)
{
#if CAR_EKF_ENABLE
    uint8 i;
    uint8 j;
    uint8 k;
    float f[CAR_EKF_STATE_DIM][CAR_EKF_STATE_DIM];
    float fp[CAR_EKF_STATE_DIM][CAR_EKF_STATE_DIM];
    float p_new[CAR_EKF_STATE_DIM][CAR_EKF_STATE_DIM];
    float q[CAR_EKF_STATE_DIM];
    float yaw;
    float v;
    float acc;
    float v_mid;
    float cos_yaw;
    float sin_yaw;
    float gyro_corrected_dps;
    float model_yaw_rate_dps;
    float yaw_rate_dps;
    float steer_angle_rad;
    float steer_blend = 0.0f;
    float tan_steer = 0.0f;
    float dt = CAR_CONTROL_PERIOD_S;

    yaw = ekf_state[CAR_EKF_YAW];
    v = ekf_state[CAR_EKF_SPEED];
    acc = acc_mps2 - ekf_state[CAR_EKF_ACC_BIAS];
    v_mid = v + 0.5f * acc * dt;
    cos_yaw = (float)cos(yaw);
    sin_yaw = (float)sin(yaw);

    ekf_state[CAR_EKF_X] += v_mid * cos_yaw * dt;
    ekf_state[CAR_EKF_Y] += v_mid * sin_yaw * dt;
    gyro_corrected_dps = gyro_dps - ekf_state[CAR_EKF_GYRO_BIAS];
    model_yaw_rate_dps = car_steer_model_yaw_rate_dps(v_mid, (float)car_steer_encoder_count);
#if CAR_STEER_MODEL_ENABLE
    if(car_abs_f(v_mid) >= CAR_STEER_MODEL_MIN_SPEED_MPS)
    {
        steer_blend = CAR_STEER_MODEL_BLEND;
        steer_angle_rad = car_steer_count_to_angle_deg((float)car_steer_encoder_count) * CAR_DEG_TO_RAD;
        tan_steer = (float)tan(steer_angle_rad);
    }
#endif
    yaw_rate_dps = gyro_corrected_dps * (1.0f - steer_blend) + model_yaw_rate_dps * steer_blend;
    car_model_yaw_rate_dps = model_yaw_rate_dps;
    ekf_state[CAR_EKF_YAW] += yaw_rate_dps * CAR_DEG_TO_RAD * dt;
    ekf_state[CAR_EKF_YAW] = car_wrap_deg(ekf_state[CAR_EKF_YAW] * CAR_RAD_TO_DEG) * CAR_DEG_TO_RAD;
    ekf_state[CAR_EKF_SPEED] += acc * dt;
    ekf_state[CAR_EKF_SPEED] = car_limit_f(ekf_state[CAR_EKF_SPEED], CAR_IMU_SPEED_LIMIT_MPS);

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            f[i][j] = (i == j) ? 1.0f : 0.0f;
            fp[i][j] = 0.0f;
            p_new[i][j] = 0.0f;
        }
    }

    f[CAR_EKF_X][CAR_EKF_YAW] = -v_mid * sin_yaw * dt;
    f[CAR_EKF_X][CAR_EKF_SPEED] = cos_yaw * dt;
    f[CAR_EKF_X][CAR_EKF_ACC_BIAS] = -0.5f * cos_yaw * dt * dt;
    f[CAR_EKF_Y][CAR_EKF_YAW] = v_mid * cos_yaw * dt;
    f[CAR_EKF_Y][CAR_EKF_SPEED] = sin_yaw * dt;
    f[CAR_EKF_Y][CAR_EKF_ACC_BIAS] = -0.5f * sin_yaw * dt * dt;
    f[CAR_EKF_YAW][CAR_EKF_SPEED] = steer_blend * tan_steer / CAR_WHEELBASE_M * dt;
    f[CAR_EKF_YAW][CAR_EKF_GYRO_BIAS] = -(1.0f - steer_blend) * CAR_DEG_TO_RAD * dt;
    f[CAR_EKF_SPEED][CAR_EKF_ACC_BIAS] = -dt;

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            for(k = 0; k < CAR_EKF_STATE_DIM; k++)
            {
                fp[i][j] += f[i][k] * ekf_p[k][j];
            }
        }
    }

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            for(k = 0; k < CAR_EKF_STATE_DIM; k++)
            {
                p_new[i][j] += fp[i][k] * f[j][k];
            }
        }
    }

    q[CAR_EKF_X] = CAR_EKF_Q_POS;
    q[CAR_EKF_Y] = CAR_EKF_Q_POS;
    q[CAR_EKF_YAW] = CAR_EKF_Q_YAW_RAD;
    q[CAR_EKF_SPEED] = CAR_EKF_Q_SPEED;
    q[CAR_EKF_GYRO_BIAS] = CAR_EKF_Q_GYRO_BIAS;
    q[CAR_EKF_ACC_BIAS] = CAR_EKF_Q_ACC_BIAS;

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            ekf_p[i][j] = p_new[i][j];
        }
        ekf_p[i][i] += q[i];
        if(ekf_p[i][i] < CAR_EKF_MIN_COV)
        {
            ekf_p[i][i] = CAR_EKF_MIN_COV;
        }
    }

    car_ekf_sync_outputs();
#else
    (void)gyro_dps;
    (void)acc_mps2;
#endif
}

static void car_ekf_update_scalar(const float h[CAR_EKF_STATE_DIM], float innovation, float r)
{
#if CAR_EKF_ENABLE
    uint8 i;
    uint8 j;
    uint8 k;
    float s = r;
    float ph[CAR_EKF_STATE_DIM] = {0.0f};
    float gain[CAR_EKF_STATE_DIM];
    float p_old[CAR_EKF_STATE_DIM][CAR_EKF_STATE_DIM];
    float hp;

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            p_old[i][j] = ekf_p[i][j];
            ph[i] += ekf_p[i][j] * h[j];
        }
        s += h[i] * ph[i];
    }

    if(s < CAR_EKF_MIN_COV)
    {
        return;
    }

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        gain[i] = ph[i] / s;
        ekf_state[i] += gain[i] * innovation;
    }
    ekf_state[CAR_EKF_YAW] = car_wrap_deg(ekf_state[CAR_EKF_YAW] * CAR_RAD_TO_DEG) * CAR_DEG_TO_RAD;
    ekf_state[CAR_EKF_SPEED] = car_limit_f(ekf_state[CAR_EKF_SPEED], CAR_IMU_SPEED_LIMIT_MPS);

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        for(j = 0; j < CAR_EKF_STATE_DIM; j++)
        {
            hp = 0.0f;
            for(k = 0; k < CAR_EKF_STATE_DIM; k++)
            {
                hp += h[k] * p_old[k][j];
            }
            ekf_p[i][j] = p_old[i][j] - gain[i] * hp;
        }
    }

    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        if(ekf_p[i][i] < CAR_EKF_MIN_COV)
        {
            ekf_p[i][i] = CAR_EKF_MIN_COV;
        }
    }

    car_ekf_sync_outputs();
#else
    (void)h;
    (void)innovation;
    (void)r;
#endif
}

static void car_ekf_update_gps(float gps_x, float gps_y, float gps_speed_mps, float gps_yaw_deg)
{
#if CAR_EKF_ENABLE
    float h[CAR_EKF_STATE_DIM] = {0.0f};
    float dx = gps_x - ekf_state[CAR_EKF_X];
    float dy = gps_y - ekf_state[CAR_EKF_Y];
    float gps_signed_speed;
    float yaw_innovation;
    int8 direction;

    car_ekf_pos_residual_m = (float)sqrt(dx * dx + dy * dy);
    if(car_ekf_pos_residual_m < CAR_EKF_GPS_POS_GATE_M)
    {
        h[CAR_EKF_X] = 1.0f;
        car_ekf_update_scalar(h, dx, CAR_EKF_R_GPS_POS);

        h[CAR_EKF_X] = 0.0f;
        h[CAR_EKF_Y] = 1.0f;
        car_ekf_update_scalar(h, gps_y - ekf_state[CAR_EKF_Y], CAR_EKF_R_GPS_POS);
    }

    direction = car_speed_sign(ekf_state[CAR_EKF_SPEED]);
    if(0 == direction)
    {
        direction = (car_drive_direction < 0) ? -1 : 1;
    }
    gps_signed_speed = gps_speed_mps * (float)direction;
    car_ekf_speed_residual_mps = gps_signed_speed - ekf_state[CAR_EKF_SPEED];
    h[CAR_EKF_Y] = 0.0f;
    h[CAR_EKF_SPEED] = 1.0f;
    car_ekf_update_scalar(h, car_ekf_speed_residual_mps, CAR_EKF_R_GPS_SPEED);

    if((CAR_STATE_RECORDING != car_state)
            && (gps_speed_mps > (CAR_GPS_YAW_MIN_SPEED_KMH / 3.6f)))
    {
        h[CAR_EKF_SPEED] = 0.0f;
        h[CAR_EKF_YAW] = 1.0f;
        yaw_innovation = car_wrap_deg(gps_yaw_deg - ekf_state[CAR_EKF_YAW] * CAR_RAD_TO_DEG) * CAR_DEG_TO_RAD;
        car_ekf_yaw_residual_deg = yaw_innovation * CAR_RAD_TO_DEG;
        car_ekf_update_scalar(h, yaw_innovation, CAR_EKF_R_GPS_YAW_RAD);
    }

    car_ekf_gps_update_count++;
#else
    (void)gps_x;
    (void)gps_y;
    (void)gps_speed_mps;
    (void)gps_yaw_deg;
#endif
}

static void car_ekf_update_still(void)
{
#if CAR_EKF_ENABLE
    float h[CAR_EKF_STATE_DIM] = {0.0f};

    h[CAR_EKF_SPEED] = 1.0f;
    car_ekf_update_scalar(h, -ekf_state[CAR_EKF_SPEED], CAR_EKF_R_STILL_SPEED);
#endif
}

static void car_ekf_apply_pending_gps(void)
{
    float gps_x;
    float gps_y;
    float gps_speed;
    float gps_yaw;

    if(!gps_update_pending)
    {
        return;
    }

    gps_x = gps_pending_x_m;
    gps_y = gps_pending_y_m;
    gps_speed = gps_pending_speed_mps;
    gps_yaw = gps_pending_yaw_deg;
    gps_update_pending = 0;
    car_ekf_update_gps(gps_x, gps_y, gps_speed, gps_yaw);
}

void car_all_motor_stop(void)
{
    car_motor_set(CAR_LEFT_MOTOR_PWM_CH,  CAR_LEFT_MOTOR_DIR_PIN,  0, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_RIGHT_MOTOR_PWM_CH, CAR_RIGHT_MOTOR_DIR_PIN, 0, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOTOR_DIR_PIN, 0, CAR_STEER_FORWARD_LEVEL, CAR_STEER_REVERSE_LEVEL);
    car_rear_pwm_output = 0;
    car_steer_pwm_output = 0;
    car_pwm_sim_speed_mps = 0.0f;
    rear_pwm_ramped = 0;
}

#if CAR_STEER_SELF_TEST_ENABLE
static uint8 car_self_test_target_reached(float target_count)
{
    float error = target_count - (float)car_steer_encoder_count;

    if(car_abs_f(error) <= CAR_STEER_SELF_TEST_TOL_CNT)
    {
        if(self_test_settle_count < CAR_STEER_SELF_TEST_SETTLE_CT)
        {
            self_test_settle_count++;
        }
    }
    else
    {
        self_test_settle_count = 0;
    }

    return (self_test_settle_count >= CAR_STEER_SELF_TEST_SETTLE_CT) ? 1 : 0;
}

static uint8 car_self_test_stage_passed(float target_count)
{
    if(target_count < 0.0f)
    {
        return (car_steer_encoder_count <= -(int32)CAR_STEER_SELF_TEST_PASS_CNT) ? 1 : 0;
    }
    if(target_count > 0.0f)
    {
        return (car_steer_encoder_count >= (int32)CAR_STEER_SELF_TEST_PASS_CNT) ? 1 : 0;
    }
    return car_self_test_target_reached(target_count);
}

static void car_self_test_drive_steer(float target_count)
{
    int16 steer_pwm;
    float error;

    steer_pwm = car_limit_i16((int32)car_pid_calc(&steer_pos_pid, target_count, (float)car_steer_encoder_count),
                              CAR_STEER_PWM_LIMIT);
    error = target_count - (float)car_steer_encoder_count;
    if(car_abs_f(error) > CAR_STEER_SELF_TEST_TOL_CNT)
    {
        if((steer_pwm > 0) && (steer_pwm < CAR_STEER_SELF_TEST_MIN_DUTY))
        {
            steer_pwm = CAR_STEER_SELF_TEST_MIN_DUTY;
        }
        else if((steer_pwm < 0) && (steer_pwm > -CAR_STEER_SELF_TEST_MIN_DUTY))
        {
            steer_pwm = -CAR_STEER_SELF_TEST_MIN_DUTY;
        }
    }
    car_target_steer_count = target_count;
    car_steer_pwm_output = steer_pwm;
    car_rear_pwm_output = 0;

    car_motor_set(CAR_LEFT_MOTOR_PWM_CH,  CAR_LEFT_MOTOR_DIR_PIN,  0, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_RIGHT_MOTOR_PWM_CH, CAR_RIGHT_MOTOR_DIR_PIN, 0, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOTOR_DIR_PIN, steer_pwm, CAR_STEER_FORWARD_LEVEL, CAR_STEER_REVERSE_LEVEL);
}

static void car_self_test_drive_rear(int16 rear_pwm)
{
    car_target_steer_count = 0.0f;
    car_steer_pwm_output = 0;
    car_rear_pwm_output = rear_pwm;

    car_motor_set(CAR_LEFT_MOTOR_PWM_CH,  CAR_LEFT_MOTOR_DIR_PIN,  rear_pwm * CAR_LEFT_MOTOR_DUTY_SIGN,  CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_RIGHT_MOTOR_PWM_CH, CAR_RIGHT_MOTOR_DIR_PIN, rear_pwm * CAR_RIGHT_MOTOR_DUTY_SIGN, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOTOR_DIR_PIN, 0, CAR_STEER_FORWARD_LEVEL, CAR_STEER_REVERSE_LEVEL);
}
#endif

static void car_self_test_update(void)
{
#if CAR_STEER_SELF_TEST_ENABLE
    float target = CAR_STEER_SELF_TEST_CENTER_CNT;
    uint8 running_target = 0;

    if(0 == car_self_test_step)
    {
        car_all_motor_stop();
        if(car_elapsed_ms >= CAR_STEER_SELF_TEST_DELAY_MS)
        {
            car_self_test_next_step(1);
        }
        return;
    }

    if(1 == car_self_test_step)
    {
        car_self_test_drive_rear(CAR_REAR_SELF_TEST_DUTY);
        if((car_elapsed_ms - self_test_step_start_ms) >= CAR_REAR_SELF_TEST_TIME_MS)
        {
            car_self_test_next_step(2);
        }
        return;
    }
    else if(2 == car_self_test_step)
    {
        car_all_motor_stop();
        if((car_elapsed_ms - self_test_step_start_ms) >= CAR_REAR_SELF_TEST_STOP_MS)
        {
            car_self_test_next_step(3);
        }
        return;
    }
    else if(3 == car_self_test_step)
    {
        car_self_test_drive_rear(-CAR_REAR_SELF_TEST_DUTY);
        if((car_elapsed_ms - self_test_step_start_ms) >= CAR_REAR_SELF_TEST_TIME_MS)
        {
            car_self_test_next_step(4);
        }
        return;
    }
    else if(4 == car_self_test_step)
    {
        target = CAR_STEER_SELF_TEST_LEFT_CNT;
        running_target = 1;
    }
    else if(5 == car_self_test_step)
    {
        target = CAR_STEER_SELF_TEST_RIGHT_CNT;
        running_target = 1;
    }
    else if(6 == car_self_test_step)
    {
        target = CAR_STEER_SELF_TEST_CENTER_CNT;
        running_target = 1;
    }

    if(running_target)
    {
        car_self_test_drive_steer(target);
        if(car_self_test_stage_passed(target))
        {
            car_self_test_next_step(car_self_test_step + 1);
        }
        else if((car_elapsed_ms - self_test_step_start_ms) >= CAR_STEER_SELF_TEST_STEP_MS)
        {
            car_error_code = CAR_ERROR_SELF_TEST;
            car_state = CAR_STATE_ERROR;
            car_all_motor_stop();
        }
        return;
    }

    car_all_motor_stop();
    car_target_steer_count = 0.0f;
    car_pid_clear(&steer_pos_pid);
    car_state = CAR_STATE_WAIT_START;
#else
    car_all_motor_stop();
    car_state = CAR_STATE_WAIT_START;
#endif
}

static int16 car_slew_pwm(int16 target_pwm)
{
    int32 delta = (int32)target_pwm - rear_pwm_ramped;

    if(delta > CAR_REAR_PWM_SLEW_STEP)       delta = CAR_REAR_PWM_SLEW_STEP;
    else if(delta < -CAR_REAR_PWM_SLEW_STEP) delta = -CAR_REAR_PWM_SLEW_STEP;

    rear_pwm_ramped = (int16)(rear_pwm_ramped + delta);
    return rear_pwm_ramped;
}

static void car_reset_runtime_pose_at(float x_m, float y_m, float yaw_deg, uint8 reset_steer)
{
    car_elapsed_ms = 0;
    car_ekf_init_state(x_m, y_m, yaw_deg, 0.0f);
    gps_update_pending = 0;
    car_acc_forward_mps2 = 0.0f;
    car_record_s_m = 0.0f;
    car_replay_s_m = 0.0f;
    car_pwm_sim_reset();
    car_steer_angle_deg = 0.0f;
    car_model_yaw_rate_dps = 0.0f;
    imu_still_count = 0;
    car_nearest_index = 0;
    car_target_index = 0;
    car_cross_track_error_m = 0.0f;
    car_heading_error_deg = 0.0f;
    car_yaw_rate_error_dps = 0.0f;
    car_lookahead_m = CAR_LOOKAHEAD_MIN_M;
    car_target_speed_mps = 0.0f;
    car_target_steer_count = 0.0f;
    car_pure_pursuit_curvature = 0.0f;
    car_drive_direction = 0;
    car_track_correction_active = 0;
    car_pid_clear(&steer_pos_pid);
    car_pid_clear(&track_pid);
    rear_pwm_ramped = 0;
    gps_priority_stop_count = 0;
    gps_stop_last_fix = car_gps_fix_count;
    finish_endpoint_fix_count = 0;
    finish_last_fix = car_gps_fix_count;
    if(reset_steer)
    {
        car_steer_encoder_count = 0;
        encoder_clear_count(CAR_STEER_ENCODER);
    }
}

static void car_start_recording(void)
{
    if(!car_gps_valid || (gps_valid_fix_streak < CAR_GPS_READY_FIX_COUNT))
    {
        car_error_code = CAR_ERROR_GPS_LOST;
        car_all_motor_stop();
        return;
    }

    car_error_code = CAR_ERROR_NONE;
    car_replay_mode = CAR_REPLAY_NONE;
    car_reset_runtime_pose_at(car_gps_x_m, car_gps_y_m, car_yaw_deg, 1);
    car_path_count = 0;
    car_path_duration_ms = 0;
    car_path_has_gps = 1;
    car_path_prepared = 0;
    car_replay_avg_speed_mps = CAR_LOW_SPEED_MPS;
    car_fixed_rear_pwm = CAR_LOW_SPEED_REAR_PWM;
    last_record_ms = 0;
    last_record_x = car_pose_x_m;
    last_record_y = car_pose_y_m;
    last_record_yaw_deg = car_yaw_deg;
    last_record_steer_count = 0.0f;
    car_state = CAR_STATE_RECORDING;
    car_all_motor_stop();
}

static void car_finish_recording(void)
{
    car_all_motor_stop();
    if(car_path_count >= CAR_START_MIN_POINTS)
    {
        car_path_duration_ms = car_path[car_path_count - 1].time_ms;
        car_state = CAR_STATE_PROCESSING;
    }
    else
    {
        car_error_code = CAR_ERROR_PATH_SHORT;
        car_state = CAR_STATE_ERROR;
        car_all_motor_stop();
    }
}

static void car_start_guide(car_replay_mode_enum mode)
{
    float start_yaw_deg;
    float start_dx;
    float start_dy;
    float start_distance_m;
    uint32 start_index;

    if(car_path_prepared && (car_path_count >= CAR_START_MIN_POINTS) && car_gps_valid
            && (gps_valid_fix_streak >= CAR_GPS_READY_FIX_COUNT))
    {
        start_index = (CAR_REPLAY_B_RETURN == mode) ? (car_path_count - 1) : 0;
        start_dx = car_gps_x_m - car_path[start_index].x;
        start_dy = car_gps_y_m - car_path[start_index].y;
        start_distance_m = (float)sqrt(start_dx * start_dx + start_dy * start_dy);
        if(start_distance_m > CAR_GUIDE_START_MAX_DIST_M)
        {
            car_error_code = CAR_ERROR_START_TOO_FAR;
            car_state = CAR_STATE_ERROR;
            car_cross_track_error_m = start_distance_m;
            car_all_motor_stop();
            return;
        }

        car_replay_mode = mode;
        start_yaw_deg = car_path_travel_yaw(start_index);
        car_reset_runtime_pose_at(car_gps_x_m, car_gps_y_m, start_yaw_deg, 0);
        car_nearest_index = start_index;
        car_target_index = start_index;
        car_path_calc_fixed_rear_pwm();
        car_pwm_sim_x_m = car_path[start_index].x;
        car_pwm_sim_y_m = car_path[start_index].y;
        car_pwm_sim_yaw_deg = start_yaw_deg;
        car_pwm_sim_nearest_index = start_index;
        guide_start_gps_x_m = car_gps_x_m;
        guide_start_gps_y_m = car_gps_y_m;
        car_pwm_sim_gps_dir_locked = 0;
        car_state = CAR_STATE_GUIDE;
    }
    else
    {
        car_error_code = (!car_path_prepared || (car_path_count < CAR_START_MIN_POINTS))
                ? CAR_ERROR_PATH_SHORT : CAR_ERROR_GPS_LOST;
        car_state = CAR_STATE_ERROR;
        car_all_motor_stop();
    }
}

static double car_signed_lat(double lat, int8 ns)
{
    return (('S' == ns) || ('s' == ns)) ? -lat : lat;
}

static double car_signed_lon(double lon, int8 ew)
{
    return (('W' == ew) || ('w' == ew)) ? -lon : lon;
}

static void car_gps_to_local(double lat, double lon, float *x, float *y)
{
    double lat0_rad = gps_origin_lat * (double)CAR_DEG_TO_RAD;
    double d_lat = (lat - gps_origin_lat) * (double)CAR_DEG_TO_RAD;
    double d_lon = (lon - gps_origin_lon) * (double)CAR_DEG_TO_RAD;

    *x = (float)(d_lon * cos(lat0_rad) * CAR_EARTH_RADIUS_M);
    *y = (float)(d_lat * CAR_EARTH_RADIUS_M);
}

void car_nav_gnss_poll(void)
{
    double lat;
    double lon;
    float raw_x;
    float raw_y;

    if(!gnss_flag)
    {
        if((nav_uptime_ms - gps_last_update_ms) > CAR_GPS_STALE_MS)
        {
            car_gps_valid = 0;
            gps_valid_fix_streak = 0;
        }
        return;
    }

    gnss_flag = 0;
    (void)gnss_data_parse();
    // RMC owns latitude/longitude/speed. GGA-only arrivals must not be counted as duplicate position fixes.
    if(!gnss_rmc_flag)
    {
        return;
    }
    gnss_rmc_flag = 0;

    car_gps_satellites = gnss.satellite_used;
    car_gps_valid = (gnss.state && (gnss.satellite_used >= CAR_GPS_MIN_SATELLITES)) ? 1 : 0;
    car_gps_speed_mps = gnss.speed / 3.6f;
    car_gps_direction_deg = car_gps_course_to_math_yaw(gnss.direction);

    if(!car_gps_valid)
    {
        gps_valid_fix_streak = 0;
        return;
    }
    if(gps_valid_fix_streak < CAR_GPS_READY_FIX_COUNT)
    {
        gps_valid_fix_streak++;
    }

    lat = car_signed_lat(gnss.latitude, gnss.ns);
    lon = car_signed_lon(gnss.longitude, gnss.ew);
    if(!gps_origin_ready)
    {
        gps_origin_lat = lat;
        gps_origin_lon = lon;
        gps_origin_ready = 1;
        car_gps_filter_reset();
    }

    car_gps_to_local(lat, lon, &raw_x, &raw_y);
    car_gps_filter_update(raw_x, raw_y);
    car_gps_fix_count++;
    gps_last_update_ms = nav_uptime_ms;

    gps_pending_x_m = car_gps_x_m;
    gps_pending_y_m = car_gps_y_m;
    gps_pending_speed_mps = car_gps_speed_mps;
    gps_pending_yaw_deg = car_gps_direction_deg;
    gps_update_pending = 1;
}

static void car_encoder_update(void)
{
    int16 steer_delta = encoder_get_count(CAR_STEER_ENCODER);

    encoder_clear_count(CAR_STEER_ENCODER);

#if CAR_STEER_ENCODER_REVERSE
    steer_delta = -steer_delta;
#endif

    car_steer_encoder_count += steer_delta;
    car_steer_angle_deg = car_steer_count_to_angle_deg((float)car_steer_encoder_count);
}

static void car_pose_update(void)
{
    float gyro_z;
    float acc_raw_g;
    float acc_mps2;
    float predict_acc_mps2;
    float guide_model_speed;
    float last_x;
    float last_y;
    float velocity_leak;
    float ds;

    mpu6050_get_gyro();
    mpu6050_get_acc();
    gyro_z = (mpu6050_gyro_transition(mpu6050_gyro_z) - car_gyro_z_bias) * CAR_GYRO_SCALE;
    if(car_abs_f(gyro_z) < CAR_GYRO_DEADBAND_DPS)
    {
        gyro_z = 0.0f;
    }
    car_gyro_z_dps = gyro_z;

    acc_raw_g = mpu6050_acc_transition(CAR_IMU_FORWARD_RAW);
    acc_mps2 = (acc_raw_g - car_acc_forward_bias_g) * CAR_GRAVITY_MPS2 * CAR_IMU_FORWARD_SIGN;
    car_acc_forward_mps2 += (acc_mps2 - car_acc_forward_mps2) * CAR_ACC_LOWPASS_ALPHA;
    if(car_abs_f(car_acc_forward_mps2) < CAR_ACC_DEADBAND_MPS2)
    {
        car_acc_forward_mps2 = 0.0f;
    }

    if((CAR_STATE_RECORDING == car_state) || (CAR_STATE_GUIDE == car_state))
    {
        // Constant low speed has almost no longitudinal acceleration; it is not a stationary condition.
        imu_still_count = 0;
    }
    else if((car_abs_f(acc_mps2) < CAR_ACC_STILL_LIMIT_MPS2)
            && (car_abs_f(gyro_z) < CAR_GYRO_STILL_LIMIT_DPS))
    {
        if(imu_still_count < CAR_STILL_CONFIRM_COUNT)
        {
            imu_still_count++;
        }
    }
    else
    {
        imu_still_count = 0;
    }

    last_x = car_pose_x_m;
    last_y = car_pose_y_m;
    predict_acc_mps2 = car_acc_forward_mps2;
    if(CAR_STATE_RECORDING == car_state)
    {
        guide_model_speed = CAR_LOW_SPEED_MPS;
        predict_acc_mps2 = (guide_model_speed - car_speed_mps) / CAR_GUIDE_SPEED_MODEL_TAU_S;
        predict_acc_mps2 = car_limit_f(predict_acc_mps2, CAR_GUIDE_SPEED_MODEL_ACC_MAX);
    }
    else if(CAR_STATE_GUIDE == car_state)
    {
        guide_model_speed = CAR_LOW_SPEED_MPS;
        predict_acc_mps2 = (guide_model_speed - car_speed_mps) / CAR_GUIDE_SPEED_MODEL_TAU_S;
        predict_acc_mps2 = car_limit_f(predict_acc_mps2, CAR_GUIDE_SPEED_MODEL_ACC_MAX);
    }
    car_ekf_predict(gyro_z, predict_acc_mps2);
    if((CAR_STATE_GUIDE != car_state) && (0.0f == car_acc_forward_mps2))
    {
        velocity_leak = CAR_VELOCITY_LEAK_PER_SEC * CAR_CONTROL_PERIOD_S;
        if(ekf_state[CAR_EKF_SPEED] > velocity_leak)       ekf_state[CAR_EKF_SPEED] -= velocity_leak;
        else if(ekf_state[CAR_EKF_SPEED] < -velocity_leak) ekf_state[CAR_EKF_SPEED] += velocity_leak;
        else                                               ekf_state[CAR_EKF_SPEED] = 0.0f;
        car_ekf_sync_outputs();
    }
    if(imu_still_count >= CAR_STILL_CONFIRM_COUNT)
    {
        car_ekf_update_still();
    }
    ds = (float)sqrt((car_pose_x_m - last_x) * (car_pose_x_m - last_x)
                   + (car_pose_y_m - last_y) * (car_pose_y_m - last_y));
    car_ekf_apply_pending_gps();

    if(CAR_STATE_RECORDING == car_state)
    {
        car_record_s_m += car_abs_f(ds);
    }
    else if(CAR_STATE_GUIDE == car_state)
    {
        car_replay_s_m += car_abs_f(car_target_speed_mps) * CAR_CONTROL_PERIOD_S;
    }
}

static float car_record_speed(void)
{
    return CAR_LOW_SPEED_MPS;
}

static void car_path_calc_fixed_rear_pwm(void)
{
    car_replay_avg_speed_mps = CAR_LOW_SPEED_MPS;
    car_fixed_rear_pwm = CAR_LOW_SPEED_REAR_PWM;
}

static void car_store_path_point(uint32 index)
{
    // The EKF pose is GPS-anchored at 10Hz and propagated between fixes by the front-encoder bicycle model.
    car_path[index].x = car_pose_x_m;
    car_path[index].y = car_pose_y_m;
    car_path[index].s_m = car_record_s_m;
    car_path[index].yaw_deg = car_yaw_deg;
    car_path[index].yaw_rate_dps = car_model_yaw_rate_dps;
    car_path[index].speed_mps = car_record_speed();
    car_path[index].steer_count = (float)car_steer_encoder_count;
    car_path[index].curvature = 0.0f;
    car_path[index].time_ms = car_elapsed_ms;
    car_path[index].direction = 1;
    car_path[index].gps_valid = car_gps_valid;
    if(car_gps_valid)
    {
        car_path_has_gps = 1;
    }
}

static void car_path_record_update(void)
{
    float dx;
    float dy;
    float move_m;
#if CAR_PATH_RECORD_USE_YAW
    float yaw_delta;
#endif
#if CAR_PATH_RECORD_USE_STEER
    float steer_delta;
#endif
    uint8 need_record;

    if(!car_gps_valid)
    {
        return;
    }

    if(0 == car_path_count)
    {
        car_store_path_point(0);
        car_path_count = 1;
        last_record_ms = car_elapsed_ms;
        last_record_x = car_path[0].x;
        last_record_y = car_path[0].y;
        last_record_yaw_deg = car_yaw_deg;
        last_record_steer_count = (float)car_steer_encoder_count;
        return;
    }

    if((car_elapsed_ms - last_record_ms) < CAR_PATH_RECORD_INTERVAL_MS)
    {
        return;
    }

    dx = car_pose_x_m - last_record_x;
    dy = car_pose_y_m - last_record_y;
    move_m = (float)sqrt(dx * dx + dy * dy);
#if CAR_PATH_RECORD_USE_YAW
    yaw_delta = car_abs_f(car_wrap_deg(car_yaw_deg - last_record_yaw_deg));
#endif
#if CAR_PATH_RECORD_USE_STEER
    steer_delta = car_abs_f((float)car_steer_encoder_count - last_record_steer_count);
#endif
    need_record = 0;

    if(move_m >= CAR_PATH_RECORD_MIN_MOVE_M)
    {
        need_record = 1;
    }

#if CAR_PATH_RECORD_USE_YAW
    if(yaw_delta >= CAR_PATH_RECORD_MIN_YAW_DEG)
    {
        need_record = 1;
    }
#endif

#if CAR_PATH_RECORD_USE_STEER
    if(steer_delta >= CAR_PATH_RECORD_MIN_STEER)
    {
        need_record = 1;
    }
#endif

    if(0 == need_record)
    {
        return;
    }

    if(car_path_count < CAR_PATH_MAX_POINTS)
    {
        car_store_path_point(car_path_count);
        car_path_count++;
        last_record_ms = car_elapsed_ms;
        last_record_x = car_pose_x_m;
        last_record_y = car_pose_y_m;
        last_record_yaw_deg = car_yaw_deg;
        last_record_steer_count = (float)car_steer_encoder_count;
    }
    else
    {
        car_state = CAR_STATE_RECORD_FULL;
        car_all_motor_stop();
    }
}

static void car_record_drive_update(void)
{
    int16 rear_pwm;

    if(!car_gps_valid)
    {
        car_error_code = CAR_ERROR_GPS_LOST;
        car_state = CAR_STATE_ERROR;
        car_all_motor_stop();
        return;
    }

    rear_pwm = car_slew_pwm(CAR_LOW_SPEED_REAR_PWM);
    car_target_speed_mps = CAR_LOW_SPEED_MPS;
    car_target_steer_count = (float)car_steer_encoder_count;
    car_drive_direction = 1;
    car_steer_pwm_output = 0;            // Human/remote steering stays mechanically free while teaching.
    car_rear_pwm_output = rear_pwm;

    car_motor_set(CAR_LEFT_MOTOR_PWM_CH, CAR_LEFT_MOTOR_DIR_PIN,
            rear_pwm * CAR_LEFT_MOTOR_DUTY_SIGN, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_RIGHT_MOTOR_PWM_CH, CAR_RIGHT_MOTOR_DIR_PIN,
            rear_pwm * CAR_RIGHT_MOTOR_DUTY_SIGN, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOTOR_DIR_PIN,
            0, CAR_STEER_FORWARD_LEVEL, CAR_STEER_REVERSE_LEVEL);

    car_path_record_update();
}

static void car_path_prepare(void)
{
    uint32 i;
    uint32 pass;
    uint32 old_count;
    uint32 write_index;
    uint32 index_a;
    uint32 index_b;
    float prev_x;
    float prev_y;
    float prev_steer;
    float current_x;
    float current_y;
    float current_steer;
    float next_x;
    float next_y;
    float next_steer;
    float dx;
    float dy;
    float point_distance;
    float yaw_a;
    float yaw_b;
    float ds;

    if(car_path_count < 2)
    {
        return;
    }

    // Convert noisy discrete fixes to one continuous polyline while preserving every bend in the taught route.
    for(pass = 0; pass < CAR_PATH_SMOOTH_PASSES; pass++)
    {
        if(car_path_count < 3)
        {
            break;
        }
        prev_x = car_path[0].x;
        prev_y = car_path[0].y;
        prev_steer = car_path[0].steer_count;
        current_x = car_path[1].x;
        current_y = car_path[1].y;
        current_steer = car_path[1].steer_count;
        for(i = 1; i + 1 < car_path_count; i++)
        {
            next_x = car_path[i + 1].x;
            next_y = car_path[i + 1].y;
            next_steer = car_path[i + 1].steer_count;
            car_path[i].x = (prev_x + 2.0f * current_x + next_x) * 0.25f;
            car_path[i].y = (prev_y + 2.0f * current_y + next_y) * 0.25f;
            car_path[i].steer_count = car_limit_f(
                    (prev_steer + 2.0f * current_steer + next_steer) * 0.25f,
                    CAR_STEER_TARGET_LIMIT);
            prev_x = current_x;
            prev_y = current_y;
            prev_steer = current_steer;
            current_x = next_x;
            current_y = next_y;
            current_steer = next_steer;
        }
    }

    // Compact to near-uniform spatial points so lookahead is measured in metres.
    old_count = car_path_count;
    write_index = 1;
    for(i = 1; i + 1 < old_count; i++)
    {
        dx = car_path[i].x - car_path[write_index - 1].x;
        dy = car_path[i].y - car_path[write_index - 1].y;
        point_distance = (float)sqrt(dx * dx + dy * dy);
        if(point_distance >= CAR_PATH_RESAMPLE_M)
        {
            if(write_index != i)
            {
                car_path[write_index] = car_path[i];
            }
            write_index++;
        }
    }
    dx = car_path[old_count - 1].x - car_path[write_index - 1].x;
    dy = car_path[old_count - 1].y - car_path[write_index - 1].y;
    point_distance = (float)sqrt(dx * dx + dy * dy);
    if(point_distance >= (CAR_PATH_RESAMPLE_M * 0.35f))
    {
        car_path[write_index] = car_path[old_count - 1];
        write_index++;
    }
    else if(write_index > 1)
    {
        car_path[write_index - 1] = car_path[old_count - 1];
    }
    car_path_count = write_index;

    car_path[0].s_m = 0.0f;
    for(i = 1; i < car_path_count; i++)
    {
        dx = car_path[i].x - car_path[i - 1].x;
        dy = car_path[i].y - car_path[i - 1].y;
        car_path[i].s_m = car_path[i - 1].s_m + (float)sqrt(dx * dx + dy * dy);
    }

    // Geometry supplies absolute course; the steering encoder supplies the low-rate curvature/feed-forward shape.
    for(i = 0; i < car_path_count; i++)
    {
        index_a = (i > 0) ? (i - 1) : 0;
        index_b = i + 1;
        if(index_b >= car_path_count)
        {
            index_b = car_path_count - 1;
        }
        dx = car_path[index_b].x - car_path[index_a].x;
        dy = car_path[index_b].y - car_path[index_a].y;
        car_path[i].yaw_deg = car_angle_to_360((float)atan2(dy, dx) * CAR_RAD_TO_DEG);
        car_path[i].direction = 1;
        car_path[i].speed_mps = CAR_LOW_SPEED_MPS;
        car_path[i].time_ms = (uint32)(car_path[i].s_m / CAR_LOW_SPEED_MPS * 1000.0f);
    }

    for(i = 0; i < car_path_count; i++)
    {
        if((i > 0) && (i + 1 < car_path_count))
        {
            yaw_a = car_path[i - 1].yaw_deg;
            yaw_b = car_path[i + 1].yaw_deg;
            ds = car_path[i + 1].s_m - car_path[i - 1].s_m;
            if(ds > 0.001f)
            {
                car_path[i].curvature = car_wrap_deg(yaw_b - yaw_a) * CAR_DEG_TO_RAD / ds;
            }
        }
    }

    if(car_path_count > 2)
    {
        car_path[0].curvature = car_path[1].curvature;
        car_path[car_path_count - 1].curvature = car_path[car_path_count - 2].curvature;
    }
    for(i = 0; i < car_path_count; i++)
    {
        car_path[i].yaw_rate_dps = car_path[i].curvature * CAR_LOW_SPEED_MPS * CAR_RAD_TO_DEG;
    }
    car_path_duration_ms = car_path[car_path_count - 1].time_ms;
    car_path_calc_fixed_rear_pwm();
}

static uint32 car_find_nearest_index(void)
{
    uint32 i;
    uint32 begin;
    uint32 end;
    uint32 previous = car_nearest_index;
    uint32 best;
    float dx;
    float dy;
    float dist2;
    float heading_delta;
    float best_dist2 = 1000000.0f;
    uint8 found = 0;

    if(previous >= car_path_count)
    {
        previous = car_replay_is_return() ? (car_path_count - 1) : 0;
    }
    best = previous;
    if(car_replay_is_return())
    {
        begin = (best > CAR_NEAREST_SEARCH_AHEAD_PT) ? (best - CAR_NEAREST_SEARCH_AHEAD_PT) : 0;
        end = best + CAR_NEAREST_SEARCH_BACK_PT + 1;
    }
    else
    {
        begin = (best > CAR_NEAREST_SEARCH_BACK_PT) ? (best - CAR_NEAREST_SEARCH_BACK_PT) : 0;
        end = best + CAR_NEAREST_SEARCH_AHEAD_PT;
    }
    if(end > car_path_count)
    {
        end = car_path_count;
    }

    for(i = begin; i < end; i++)
    {
        dx = car_path[i].x - car_pose_x_m;
        dy = car_path[i].y - car_pose_y_m;
        dist2 = dx * dx + dy * dy;
        heading_delta = car_abs_f(car_wrap_deg(car_path_travel_yaw(i) - car_yaw_deg));
        if(heading_delta > CAR_NEAREST_HEADING_GATE_DEG)
        {
            continue;
        }
        if(dist2 < best_dist2)
        {
            best_dist2 = dist2;
            best = i;
            found = 1;
        }
    }

    if(!found)
    {
        for(i = begin; i < end; i++)
        {
            dx = car_path[i].x - car_pose_x_m;
            dy = car_path[i].y - car_pose_y_m;
            dist2 = dx * dx + dy * dy;
            if(dist2 < best_dist2)
            {
                best_dist2 = dist2;
                best = i;
            }
        }
    }

    if((!car_replay_is_return() && (best < previous))
            || (car_replay_is_return() && (best > previous)))
    {
        best = previous;
        dx = car_path[best].x - car_pose_x_m;
        dy = car_path[best].y - car_pose_y_m;
        best_dist2 = dx * dx + dy * dy;
    }

    car_cross_track_error_m = sqrt(best_dist2);
    return best;
}

static uint32 car_find_gps_nearest_index(float *distance_m)
{
    uint32 i;
    uint32 best = 0;
    float dx;
    float dy;
    float dist2;
    float best_dist2 = 1000000.0f;

    for(i = 0; i < car_path_count; i++)
    {
        dx = car_path[i].x - car_gps_x_m;
        dy = car_path[i].y - car_gps_y_m;
        dist2 = dx * dx + dy * dy;
        if(dist2 < best_dist2)
        {
            best_dist2 = dist2;
            best = i;
        }
    }

    *distance_m = sqrt(best_dist2);
    return best;
}

static uint8 car_gps_priority_should_stop(void)
{
#if CAR_GPS_PRIORITY_STOP_ENABLE
    float gps_distance_m = 0.0f;

    if(!(car_path_has_gps && car_gps_valid) || (car_path_count < CAR_START_MIN_POINTS))
    {
        gps_priority_stop_count = 0;
        return 0;
    }
    if(gps_stop_last_fix == car_gps_fix_count)
    {
        return 0;
    }
    gps_stop_last_fix = car_gps_fix_count;

    (void)car_find_gps_nearest_index(&gps_distance_m);
    if(gps_distance_m > CAR_GPS_PRIORITY_STOP_M)
    {
        if(gps_priority_stop_count < CAR_GPS_PRIORITY_STOP_COUNT)
        {
            gps_priority_stop_count++;
        }
    }
    else
    {
        gps_priority_stop_count = 0;
    }

    if(gps_priority_stop_count >= CAR_GPS_PRIORITY_STOP_COUNT)
    {
        car_cross_track_error_m = gps_distance_m;
        return 1;
    }
#endif
    return 0;
}

static uint32 car_find_sim_nearest_index(void)
{
    uint32 i;
    uint32 begin;
    uint32 end;
    uint32 best = car_pwm_sim_nearest_index;
    float dx;
    float dy;
    float dist2;
    float best_dist2 = 1000000.0f;

    if(best >= car_path_count)
    {
        best = car_replay_is_return() ? (car_path_count - 1) : 0;
    }
    if(car_replay_is_return())
    {
        begin = (best > 80) ? (best - 80) : 0;
        end = best + 21;
    }
    else
    {
        begin = (best > 20) ? (best - 20) : 0;
        end = best + 80;
    }
    if(end > car_path_count)
    {
        end = car_path_count;
    }

    for(i = begin; i < end; i++)
    {
        dx = car_path[i].x - car_pwm_sim_x_m;
        dy = car_path[i].y - car_pwm_sim_y_m;
        dist2 = dx * dx + dy * dy;
        if(dist2 < best_dist2)
        {
            best_dist2 = dist2;
            best = i;
        }
    }

    return best;
}

static uint8 car_guide_should_finish(void)
{
    uint32 finish_time_ms;
    uint32 end_index;
    float end_dx;
    float end_dy;
    float end_distance_m;
    uint8 endpoint_index_reached;

    end_index = car_replay_endpoint_index();
    end_dx = car_path[end_index].x - car_pose_x_m;
    end_dy = car_path[end_index].y - car_pose_y_m;
    end_distance_m = (float)sqrt(end_dx * end_dx + end_dy * end_dy);

    if(car_elapsed_ms >= CAR_GUIDE_FINISH_MIN_MS)
    {
        if((((!car_replay_is_return())
                        && (car_nearest_index + CAR_PATH_FINISH_INDEX_REMAIN >= car_path_count))
                    || (car_replay_is_return()
                        && (car_nearest_index <= CAR_PATH_FINISH_INDEX_REMAIN)))
                && (end_distance_m <= CAR_PATH_FINISH_DISTANCE_M))
        {
            return 1;
        }

        endpoint_index_reached = (((!car_replay_is_return())
                    && (car_nearest_index + CAR_PATH_FINISH_INDEX_STOP >= car_path_count))
                || (car_replay_is_return()
                    && (car_nearest_index <= CAR_PATH_FINISH_INDEX_STOP))) ? 1 : 0;
        if(finish_last_fix != car_gps_fix_count)
        {
            finish_last_fix = car_gps_fix_count;
            if(endpoint_index_reached && (end_distance_m <= CAR_GPS_PRIORITY_STOP_M))
            {
                if(finish_endpoint_fix_count < CAR_PATH_FINISH_STOP_FIX_COUNT)
                {
                    finish_endpoint_fix_count++;
                }
            }
            else
            {
                finish_endpoint_fix_count = 0;
            }
        }
        if(finish_endpoint_fix_count >= CAR_PATH_FINISH_STOP_FIX_COUNT)
        {
            return 1;
        }

#if CAR_GUIDE_FINISH_USE_SIM
        if(((!car_replay_is_return())
                    && (car_pwm_sim_nearest_index + CAR_PATH_FINISH_INDEX_REMAIN >= car_path_count))
                || (car_replay_is_return()
                    && (car_pwm_sim_nearest_index <= CAR_PATH_FINISH_INDEX_REMAIN)))
        {
            return 1;
        }
#endif
    }

    if(car_path_duration_ms > 0)
    {
        finish_time_ms = (uint32)((float)car_path_duration_ms * CAR_GUIDE_FINISH_TIME_SCALE)
                       + CAR_GUIDE_FINISH_MARGIN_MS;
        if(car_elapsed_ms >= finish_time_ms)
        {
            return 2;
        }
    }

    return 0;
}

static uint32 car_find_time_index(uint32 time_ms)
{
    uint32 i;
    uint32 best = car_nearest_index;

    if(best >= car_path_count)
    {
        best = 0;
    }

    for(i = best; i < car_path_count; i++)
    {
        if(car_path[i].time_ms >= time_ms)
        {
            return i;
        }
    }
    return car_path_count - 1;
}

static uint32 car_find_target_index(uint32 nearest)
{
    uint32 i;
    float target_s_m;

    car_lookahead_m = CAR_LOOKAHEAD_MIN_M + car_abs_f(car_target_speed_mps) * CAR_LOOKAHEAD_GAIN;
    if(car_lookahead_m > CAR_LOOKAHEAD_MAX_M)
    {
        car_lookahead_m = CAR_LOOKAHEAD_MAX_M;
    }

    if(car_replay_is_return())
    {
        target_s_m = car_path[nearest].s_m - car_lookahead_m;
        if(target_s_m < 0.0f)
        {
            target_s_m = 0.0f;
        }
        i = nearest;
        while(1)
        {
            if(car_path[i].s_m <= target_s_m)
            {
                return i;
            }
            if(0 == i)
            {
                break;
            }
            i--;
        }
        return 0;
    }

    target_s_m = car_path[nearest].s_m + car_lookahead_m;
    for(i = nearest; i < car_path_count; i++)
    {
        if(car_path[i].s_m >= target_s_m)
        {
            return i;
        }
    }
    return car_path_count - 1;
}

static float car_signed_cross_track(uint32 index)
{
    float yaw_rad;
    float dx;
    float dy;
    float cte;

    if(index >= car_path_count)
    {
        return 0.0f;
    }

    yaw_rad = car_path_travel_yaw(index) * CAR_DEG_TO_RAD;
    dx = car_pose_x_m - car_path[index].x;
    dy = car_pose_y_m - car_path[index].y;
    cte = -(float)sin(yaw_rad) * dx + (float)cos(yaw_rad) * dy;
    return car_limit_f(cte, CAR_GPS_CTE_LIMIT_M);
}

static int16 car_steer_position_pwm(float target_count)
{
    float error;
    int32 output;

    error = target_count - (float)car_steer_encoder_count;
    if(car_abs_f(error) <= CAR_STEER_POSITION_TOL_CNT)
    {
        car_pid_clear(&steer_pos_pid);
        return 0;
    }

    output = (int32)car_pid_calc(&steer_pos_pid, target_count, (float)car_steer_encoder_count);
    if((output > 0) && (output < CAR_STEER_MOVE_MIN_DUTY))
    {
        output = CAR_STEER_MOVE_MIN_DUTY;
    }
    else if((output < 0) && (output > -CAR_STEER_MOVE_MIN_DUTY))
    {
        output = -CAR_STEER_MOVE_MIN_DUTY;
    }
    return car_limit_i16(output, CAR_STEER_PWM_LIMIT);
}

static void car_select_reference(void)
{
    uint8 use_gps_track = 0;
    uint32 time_target;

#if CAR_GPS_TRACK_ENABLE
    use_gps_track = (car_path_has_gps && car_gps_valid) ? 1 : 0;
#endif

    if(use_gps_track)
    {
        car_nearest_index = car_find_nearest_index();
        car_target_index = car_find_target_index(car_nearest_index);
    }
    else
    {
        car_nearest_index = car_find_time_index(car_elapsed_ms);
        time_target = car_elapsed_ms + CAR_TIME_LOOKAHEAD_MS;
        car_target_index = car_find_time_index(time_target);
        car_cross_track_error_m = 0.0f;
        car_lookahead_m = 0.0f;
    }
}

static float car_track_pid_correction(float signed_cte_m)
{
    float excess_cte_m;
    float correction;

    if(car_abs_f(signed_cte_m) <= CAR_TRACK_TOLERANCE_M)
    {
        car_track_correction_active = 0;
        car_pid_clear(&track_pid);
        return 0.0f;
    }

    car_track_correction_active = 1;
    excess_cte_m = signed_cte_m;
    if(excess_cte_m > 0.0f)
    {
        excess_cte_m -= CAR_TRACK_TOLERANCE_M;
    }
    else
    {
        excess_cte_m += CAR_TRACK_TOLERANCE_M;
    }

    correction = car_pid_calc(&track_pid, excess_cte_m, 0.0f)
               - CAR_TRACK_HEADING_K * car_heading_error_deg;
    return car_limit_f(correction, CAR_TRACK_CORRECTION_LIMIT);
}

static void car_guide_update(void)
{
    float cte = 0.0f;
    float feedforward;
    float correction;
    float steer_target;
    int16 rear_pwm;
    int16 steer_pwm;
    uint8 finish_reason;

    if((car_path_count < CAR_START_MIN_POINTS) || !car_gps_valid)
    {
        car_error_code = (!car_path_prepared || (car_path_count < CAR_START_MIN_POINTS))
                ? CAR_ERROR_PATH_SHORT : CAR_ERROR_GPS_LOST;
        car_state = CAR_STATE_ERROR;
        car_all_motor_stop();
        return;
    }

    car_target_speed_mps = CAR_LOW_SPEED_MPS;
    car_drive_direction = 1;
    car_select_reference();

#if CAR_GPS_PRIORITY_STOP_ENABLE
    if(car_gps_priority_should_stop())
    {
        car_error_code = CAR_ERROR_GPS_CORRIDOR;
        car_state = CAR_STATE_ERROR;
        car_all_motor_stop();
        return;
    }
#endif

    car_heading_error_deg = car_wrap_deg(car_path_travel_yaw(car_target_index) - car_yaw_deg);
    car_yaw_rate_error_dps = car_path[car_nearest_index].yaw_rate_dps
            * (car_replay_is_return() ? -1.0f : 1.0f) - car_gyro_z_dps;

#if CAR_GPS_TRACK_ENABLE
    if(car_path_has_gps && car_gps_valid)
    {
        cte = car_signed_cross_track(car_nearest_index);
        car_cross_track_error_m = car_abs_f(cte);
    }
#endif

    feedforward = car_path[car_nearest_index].steer_count * CAR_STEER_FF_GAIN;
    if(car_replay_is_return())
    {
        feedforward = -feedforward;
    }
    correction = car_track_pid_correction(cte);
    steer_target = feedforward + correction;
    car_pure_pursuit_curvature = car_path[car_nearest_index].curvature
            * (car_replay_is_return() ? -1.0f : 1.0f);
    car_target_steer_count = car_limit_f(steer_target, CAR_STEER_TARGET_LIMIT);

    steer_pwm = car_steer_position_pwm(car_target_steer_count);
    rear_pwm = car_slew_pwm(CAR_LOW_SPEED_REAR_PWM);

    car_steer_pwm_output = steer_pwm;
    car_rear_pwm_output = rear_pwm;
    car_pwm_sim_try_lock_gps_direction();
    car_pwm_sim_update();
    car_pwm_sim_nearest_index = car_find_sim_nearest_index();

    car_motor_set(CAR_LEFT_MOTOR_PWM_CH,  CAR_LEFT_MOTOR_DIR_PIN,  rear_pwm * CAR_LEFT_MOTOR_DUTY_SIGN,  CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_RIGHT_MOTOR_PWM_CH, CAR_RIGHT_MOTOR_DIR_PIN, rear_pwm * CAR_RIGHT_MOTOR_DUTY_SIGN, CAR_MOTOR_FORWARD_LEVEL, CAR_MOTOR_REVERSE_LEVEL);
    car_motor_set(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOTOR_DIR_PIN, steer_pwm, CAR_STEER_FORWARD_LEVEL, CAR_STEER_REVERSE_LEVEL);

    finish_reason = car_guide_should_finish();
    if(finish_reason)
    {
        if(2 == finish_reason)
        {
            car_error_code = CAR_ERROR_GUIDE_TIMEOUT;
            car_state = CAR_STATE_ERROR;
        }
        else
        {
            car_error_code = CAR_ERROR_NONE;
            car_state = CAR_STATE_FINISHED;
        }
        car_all_motor_stop();
    }
}

void car_nav_control_10ms(void)
{
    static uint8 last_key1_level = CAR_KEY_RELEASE_LEVEL;
    static uint8 last_key2_level = CAR_KEY_RELEASE_LEVEL;
    static uint8 key1_cooldown = 0;
    static uint8 key2_cooldown = 0;
    uint8 key_a_cmd = 0;
    uint8 key_b_cmd = 0;

    nav_uptime_ms += CAR_CONTROL_PERIOD_MS;
    car_elapsed_ms += CAR_CONTROL_PERIOD_MS;
    if((nav_uptime_ms - gps_last_update_ms) > CAR_GPS_STALE_MS)
    {
        car_gps_valid = 0;
        gps_valid_fix_streak = 0;
    }
    car_key1_level = gpio_get_level(CAR_KEY_A_PIN);
    car_key2_level = gpio_get_level(CAR_KEY_B_PIN);

    if(key1_cooldown > 0)
    {
        key1_cooldown--;
    }
    if(key2_cooldown > 0)
    {
        key2_cooldown--;
    }

    if((0 == key1_cooldown) && (CAR_KEY_RELEASE_LEVEL == last_key1_level) && (CAR_KEY_ACTIVE_LEVEL == car_key1_level))
    {
        key_a_cmd = 1;
        key1_cooldown = CAR_KEY_COOLDOWN_COUNT;
    }
    if((0 == key2_cooldown) && (CAR_KEY_RELEASE_LEVEL == last_key2_level) && (CAR_KEY_ACTIVE_LEVEL == car_key2_level))
    {
        key_b_cmd = 1;
        key2_cooldown = CAR_KEY_COOLDOWN_COUNT;
    }
    last_key1_level = car_key1_level;
    last_key2_level = car_key2_level;

    car_encoder_update();
    car_pose_update();

    if(CAR_STATE_SELF_TEST == car_state)
    {
        car_self_test_update();
        return;
    }

    if(CAR_STATE_WAIT_START == car_state)
    {
        car_all_motor_stop();
        car_replay_mode = CAR_REPLAY_NONE;
        if(key_a_cmd)
        {
            car_start_recording();
        }
        return;
    }

    if(CAR_STATE_RECORDING == car_state)
    {
        if(key_a_cmd || key_b_cmd)
        {
            car_finish_recording();
        }
        else
        {
            car_record_drive_update();
        }
        return;
    }

    if(CAR_STATE_RECORD_FULL == car_state)
    {
        car_all_motor_stop();
        if(key_a_cmd || key_b_cmd)
        {
            car_finish_recording();
        }
        return;
    }

    if(CAR_STATE_PROCESSING == car_state)
    {
        car_all_motor_stop();
        return;
    }

    if((CAR_STATE_READY == car_state) || (CAR_STATE_FINISHED == car_state))
    {
        car_all_motor_stop();
        if(key_a_cmd && key_b_cmd)
        {
            car_path_count = 0;
            car_path_duration_ms = 0;
            car_path_has_gps = 0;
            car_path_prepared = 0;
            car_replay_mode = CAR_REPLAY_NONE;
            car_error_code = CAR_ERROR_NONE;
            car_state = CAR_STATE_WAIT_START;
        }
        else if(key_a_cmd)
        {
            car_start_guide(CAR_REPLAY_A_FORWARD);
        }
        else if(key_b_cmd)
        {
            car_start_guide(CAR_REPLAY_B_RETURN);
        }
        return;
    }

    if(CAR_STATE_GUIDE == car_state)
    {
        if(key_a_cmd || key_b_cmd)
        {
            car_state = CAR_STATE_READY;
            car_all_motor_stop();
        }
        else
        {
            car_guide_update();
        }
        return;
    }

    if(CAR_STATE_ERROR == car_state)
    {
        car_all_motor_stop();
        if(key_a_cmd || key_b_cmd)
        {
            car_error_code = CAR_ERROR_NONE;
            car_replay_mode = CAR_REPLAY_NONE;
            if(car_path_prepared && (car_path_count >= CAR_START_MIN_POINTS))
            {
                car_state = CAR_STATE_READY;
            }
            else
            {
                car_path_count = 0;
                car_path_duration_ms = 0;
                car_path_has_gps = 0;
                car_path_prepared = 0;
                car_state = CAR_STATE_WAIT_START;
            }
        }
        return;
    }

    car_all_motor_stop();
}

void car_nav_background(void)
{
    if(CAR_STATE_PROCESSING == car_state)
    {
        car_path_prepare();
        if(car_path_count >= CAR_START_MIN_POINTS)
        {
            car_error_code = CAR_ERROR_NONE;
            car_replay_mode = CAR_REPLAY_NONE;
            car_path_prepared = 1;
            car_state = CAR_STATE_READY;
        }
        else
        {
            car_path_prepared = 0;
            car_error_code = CAR_ERROR_PATH_SHORT;
            car_state = CAR_STATE_ERROR;
        }
    }
}

static void car_display_state_text(uint16 x, uint16 y)
{
    if(CAR_STATE_SELF_TEST == car_state)         ips200_show_string(x, y, "SELF  ");
    else if(CAR_STATE_WAIT_START == car_state)   ips200_show_string(x, y, "WAIT  ");
    else if(CAR_STATE_RECORDING == car_state)    ips200_show_string(x, y, "REC   ");
    else if(CAR_STATE_PROCESSING == car_state)   ips200_show_string(x, y, "SMOOTH");
    else if(CAR_STATE_READY == car_state)        ips200_show_string(x, y, "READY ");
    else if((CAR_STATE_GUIDE == car_state) && (CAR_REPLAY_B_RETURN == car_replay_mode))
                                                    ips200_show_string(x, y, "BACK B");
    else if(CAR_STATE_GUIDE == car_state)        ips200_show_string(x, y, "FWD A ");
    else if(CAR_STATE_RECORD_FULL == car_state)  ips200_show_string(x, y, "FULL  ");
    else if(CAR_STATE_FINISHED == car_state)     ips200_show_string(x, y, "FINISH");
    else                                        ips200_show_string(x, y, "ERROR ");
}

static void car_display_header(void)
{
    ips200_show_string(0, 0, CAR_APP_VERSION_TEXT);
    ips200_show_string(0, 16, "STA:");
    car_display_state_text(32, 16);
    ips200_show_string(96, 16, "T:");
    ips200_show_uint(120, 16, car_elapsed_ms / 1000, 3);
    ips200_show_string(168, 16, "K:");
    ips200_show_uint(192, 16, car_key1_level, 1);
    ips200_show_uint(208, 16, car_key2_level, 1);
}

static void car_display_map_point(float x_m, float y_m, uint16 color)
{
    int16 map_top = CAR_DISPLAY_MAP_TOP_Y;
    int16 map_bottom = (int16)ips200_height_max - 1;
    int16 map_height = map_bottom - map_top + 1;
    int16 map_width = (int16)ips200_width_max;
    int16 map_center_x = (int16)ips200_width_max / 2;
    int16 map_center_y = map_top + ((map_bottom - map_top) / 2);
    float scale_x;
    float scale_y;
    float scale;
    float half_field;
    int16 px;
    int16 py;

    scale_x = (float)(map_width - 2) / CAR_DISPLAY_MAP_FIELD_M;
    scale_y = (float)(map_height - 2) / CAR_DISPLAY_MAP_FIELD_M;
    scale = (scale_x < scale_y) ? scale_x : scale_y;
    half_field = CAR_DISPLAY_MAP_FIELD_M * 0.5f;

    x_m = car_limit_f(x_m, half_field);
    y_m = car_limit_f(y_m, half_field);
    px = (int16)(map_center_x + x_m * scale);
    py = (int16)(map_center_y - y_m * scale);

    if((px >= 0) && (px < (int16)ips200_width_max) && (py >= map_top) && (py <= map_bottom))
    {
        ips200_draw_point((uint16)px, (uint16)py, color);
    }
}

static void car_display_map_axes(void)
{
    int16 map_top = CAR_DISPLAY_MAP_TOP_Y;
    int16 map_bottom = (int16)ips200_height_max - 1;
    int16 map_center_x = (int16)ips200_width_max / 2;
    int16 map_center_y = map_top + ((map_bottom - map_top) / 2);

    if(map_top < map_bottom)
    {
        ips200_draw_line(0, (uint16)map_center_y, ips200_width_max - 1, (uint16)map_center_y, RGB565_GRAY);
        ips200_draw_line((uint16)map_center_x, (uint16)map_top, (uint16)map_center_x, (uint16)map_bottom, RGB565_GRAY);
    }
}

static void car_display_record_trace(void)
{
    uint32 i;
    uint32 count;
    uint32 step;

    car_display_map_axes();

    count = car_path_count;
    if(count > CAR_PATH_MAX_POINTS)
    {
        count = CAR_PATH_MAX_POINTS;
    }
    step = count / 220 + 1;

    for(i = 0; i < count; i += step)
    {
        car_display_map_point(car_path[i].x, car_path[i].y, car_path[i].gps_valid ? RGB565_GREEN : RGB565_YELLOW);
    }
    car_display_map_point(car_gps_x_m, car_gps_y_m, RGB565_RED);
}

static void car_display_guide_trace(void)
{
    uint32 i;
    uint32 count;
    uint32 step;

    car_display_map_axes();

    count = car_path_count;
    if(count > CAR_PATH_MAX_POINTS)
    {
        count = CAR_PATH_MAX_POINTS;
    }
    step = count / 180 + 1;

    for(i = 0; i < count; i += step)
    {
        car_display_map_point(car_path[i].x, car_path[i].y, car_path[i].gps_valid ? RGB565_GREEN : RGB565_YELLOW);
    }
    car_display_map_point(car_pose_x_m, car_pose_y_m, RGB565_RED);
    car_display_map_point(car_pwm_sim_x_m, car_pwm_sim_y_m, (uint16)CAR_DISPLAY_SIM_COLOR);
}

static void car_display_record(void)
{
    car_display_header();

    ips200_show_string(0, 32, "GPS:");
    ips200_show_uint(40, 32, car_gps_valid, 1);
    ips200_show_string(64, 32, "SAT:");
    ips200_show_uint(104, 32, car_gps_satellites, 2);
    ips200_show_string(136, 32, "ER:");
    ips200_show_float(168, 32, car_ekf_pos_residual_m, 2, 2);

    ips200_show_string(0, 48, "PATH:");
    ips200_show_uint(48, 48, car_path_count, 4);
    ips200_show_string(104, 48, "EKF:");
    ips200_show_uint(144, 48, car_ekf_gps_update_count, 4);

    ips200_show_string(0, 64, "X:");
    ips200_show_float(24, 64, car_gps_x_m, 4, 2);
    ips200_show_string(96, 64, "Y:");
    ips200_show_float(120, 64, car_gps_y_m, 4, 2);
    ips200_show_string(184, 64, "M:");
    ips200_show_uint(208, 64, (uint32)CAR_DISPLAY_MAP_FIELD_M, 2);

    car_display_record_trace();
}

static void car_display_guide(void)
{
    car_display_header();

    ips200_show_string(0, 32, "GPS:");
    ips200_show_uint(40, 32, car_gps_valid, 1);
    ips200_show_string(64, 32, "SAT:");
    ips200_show_uint(104, 32, car_gps_satellites, 2);
    ips200_show_string(136, 32, "ER:");
    ips200_show_float(168, 32, car_ekf_pos_residual_m, 2, 2);

    ips200_show_string(0, 48, "I:");
    ips200_show_uint(16, 48, car_nearest_index, 4);
    ips200_show_string(72, 48, "C:");
    ips200_show_float(88, 48, car_cross_track_error_m, 3, 2);
    ips200_show_string(168, 48, "M:");
    ips200_show_string(184, 48, (CAR_REPLAY_B_RETURN == car_replay_mode) ? "B" : "A");

    ips200_show_string(0, 64, "HE:");
    ips200_show_float(24, 64, car_heading_error_deg, 4, 1);
    ips200_show_string(112, 64, "ST:");
    ips200_show_int(136, 64, (int32)car_target_steer_count, 4);

    ips200_show_string(0, 80, "EN:");
    ips200_show_int(24, 80, car_steer_encoder_count, 4);
    ips200_show_string(104, 80, "PID:");
    ips200_show_uint(136, 80, car_track_correction_active, 1);
    ips200_show_string(160, 80, "SP:");
    ips200_show_int(184, 80, car_steer_pwm_output, 5);

    car_display_guide_trace();
}

static void car_display_ready(void)
{
    car_display_header();

    ips200_show_string(0, 32, "GPS:");
    ips200_show_uint(40, 32, car_gps_valid, 1);
    ips200_show_string(64, 32, "SAT:");
    ips200_show_uint(104, 32, car_gps_satellites, 2);
    ips200_show_string(136, 32, "EC:");
    ips200_show_uint(160, 32, car_error_code, 1);

    ips200_show_string(0, 48, "PATH:");
    ips200_show_uint(48, 48, car_path_count, 4);
    ips200_show_string(104, 48, "YAW:");
    ips200_show_float(144, 48, car_yaw_deg, 4, 1);

    ips200_show_string(0, 64, "ENC:");
    ips200_show_int(40, 64, car_steer_encoder_count, 6);
    ips200_show_string(112, 64, "AV:");
    ips200_show_float(144, 64, car_replay_avg_speed_mps, 3, 2);

    if(CAR_STATE_WAIT_START == car_state)
    {
        ips200_show_string(0, 80, "KEY A: TEACH (GPS READY)");
    }
    else if((CAR_STATE_READY == car_state) || (CAR_STATE_FINISHED == car_state))
    {
        ips200_show_string(0, 80, "A:FWD  B:RETURN  A+B:CLEAR");
    }
    else if(CAR_STATE_PROCESSING == car_state)
    {
        ips200_show_string(0, 80, "SMOOTHING PATH...");
    }
    else
    {
        ips200_show_string(0, 80, "KEY: ACK/STOP");
    }
}

void car_nav_display(void)
{
    static car_state_enum last_display_state = CAR_STATE_ERROR;

    if(last_display_state != car_state)
    {
        ips200_clear();
        last_display_state = car_state;
    }

    if(CAR_STATE_RECORDING == car_state)
    {
        car_display_record();
    }
    else if(CAR_STATE_GUIDE == car_state)
    {
        car_display_guide();
    }
    else
    {
        car_display_ready();
    }
}

void car_nav_init(void)
{
    uint16 i;
    float gyro_sum = 0.0f;
    float acc_sum = 0.0f;

    gpio_init(CAR_LEFT_MOTOR_DIR_PIN,  GPO, CAR_MOTOR_FORWARD_LEVEL, GPO_PUSH_PULL);
    gpio_init(CAR_RIGHT_MOTOR_DIR_PIN, GPO, CAR_MOTOR_FORWARD_LEVEL, GPO_PUSH_PULL);
    gpio_init(CAR_STEER_MOTOR_DIR_PIN, GPO, CAR_STEER_FORWARD_LEVEL, GPO_PUSH_PULL);

    gpio_init(CAR_KEY_A_PIN, GPI, CAR_KEY_RELEASE_LEVEL, GPI_PULL_UP);
    gpio_init(CAR_KEY_B_PIN, GPI, CAR_KEY_RELEASE_LEVEL, GPI_PULL_UP);

    pwm_init(CAR_LEFT_MOTOR_PWM_CH,  CAR_PWM_FREQ_HZ, 0);
    pwm_init(CAR_RIGHT_MOTOR_PWM_CH, CAR_PWM_FREQ_HZ, 0);
    pwm_init(CAR_STEER_MOTOR_PWM_CH, CAR_PWM_FREQ_HZ, 0);

#if CAR_STEER_ENCODER_DIR_MODE
    encoder_dir_init(CAR_STEER_ENCODER, CAR_STEER_ENCODER_CH1_PIN, CAR_STEER_ENCODER_CH2_PIN);
#else
    encoder_quad_init(CAR_STEER_ENCODER, CAR_STEER_ENCODER_CH1_PIN, CAR_STEER_ENCODER_CH2_PIN);
#endif

    encoder_clear_count(CAR_STEER_ENCODER);

    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(CAR_IPS200_TYPE);
    ips200_clear();
    ips200_show_string(0, 0, "MPU6050 INIT...");

    car_mpu6050_ok = (0 == mpu6050_init()) ? 1 : 0;
    ips200_show_string(0, 16, car_mpu6050_ok ? "MPU6050 OK     " : "MPU6050 ERROR  ");
    ips200_show_string(0, 32, "KEEP STILL BIAS");

    for(i = 0; i < CAR_GYRO_BIAS_SAMPLE_COUNT; i++)
    {
        mpu6050_get_gyro();
        gyro_sum += mpu6050_gyro_transition(mpu6050_gyro_z);
        system_delay_ms(CAR_GYRO_BIAS_SAMPLE_DELAY_MS);
    }
    car_gyro_z_bias = gyro_sum / (float)CAR_GYRO_BIAS_SAMPLE_COUNT;

    for(i = 0; i < CAR_ACC_BIAS_SAMPLE_COUNT; i++)
    {
        mpu6050_get_acc();
        acc_sum += mpu6050_acc_transition(CAR_IMU_FORWARD_RAW);
        system_delay_ms(CAR_GYRO_BIAS_SAMPLE_DELAY_MS);
    }
    car_acc_forward_bias_g = acc_sum / (float)CAR_ACC_BIAS_SAMPLE_COUNT;
    car_ekf_init_state(0.0f, 0.0f, 0.0f, 0.0f);

    ips200_show_string(0, 48, "GNSS INIT...");
    gnss_init(CAR_GNSS_DEVICE);

    car_all_motor_stop();
    car_elapsed_ms = 0;
    car_self_test_next_step(0);
    car_state = CAR_STATE_SELF_TEST;
    ips200_clear();
}

#pragma section all restore
