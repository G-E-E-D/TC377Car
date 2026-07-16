#ifndef _car_nav_h_
#define _car_nav_h_

#include "car_config.h"

typedef enum
{
    CAR_STATE_SELF_TEST = 0,
    CAR_STATE_WAIT_GPS,
    CAR_STATE_RECORDING,
    CAR_STATE_READY,
    CAR_STATE_GUIDE,
    CAR_STATE_FINISHED,
    CAR_STATE_RECORD_FULL,
    CAR_STATE_ERROR,
} car_state_enum;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float error;
    float last_error;
    float integral;
} car_pid_struct;

typedef struct
{
    float x;
    float y;
    float s_m;
    float yaw_deg;
    float yaw_rate_dps;
    float speed_mps;
    float steer_count;
    float curvature;
    uint32 time_ms;
    int8 direction;
    uint8 gps_valid;
} car_path_point_struct;

typedef struct
{
    float x;
    float y;
    float yaw_deg;
    float speed_mps;
    float distance_m;
} car_pose_struct;

extern volatile car_state_enum car_state;
extern volatile uint32 car_elapsed_ms;
extern volatile uint8 car_key1_level;
extern volatile uint8 car_key2_level;
extern volatile uint8 car_mpu6050_ok;
extern volatile uint8 car_gps_valid;
extern volatile uint8 car_gps_satellites;
extern volatile uint8 car_error_code;

extern volatile int32 car_steer_encoder_count;
extern volatile float car_steer_angle_deg;
extern volatile float car_model_yaw_rate_dps;

extern volatile float car_gyro_z_dps;
extern volatile float car_gyro_z_bias;
extern volatile float car_acc_forward_mps2;
extern volatile float car_acc_forward_bias_g;
extern volatile float car_pose_x_m;
extern volatile float car_pose_y_m;
extern volatile float car_yaw_deg;
extern volatile float car_speed_mps;
extern volatile float car_gps_x_m;
extern volatile float car_gps_y_m;
extern volatile float car_gps_speed_mps;
extern volatile float car_gps_direction_deg;
extern volatile uint32 car_gps_fix_count;
extern volatile float car_ekf_pos_residual_m;
extern volatile float car_ekf_speed_residual_mps;
extern volatile float car_ekf_yaw_residual_deg;
extern volatile uint32 car_ekf_gps_update_count;
extern volatile float car_record_s_m;
extern volatile float car_replay_s_m;
extern volatile uint32 car_path_duration_ms;
extern volatile uint8 car_path_has_gps;

extern volatile uint32 car_path_count;
extern volatile uint32 car_nearest_index;
extern volatile uint32 car_target_index;
extern volatile float car_cross_track_error_m;
extern volatile float car_heading_error_deg;
extern volatile float car_yaw_rate_error_dps;
extern volatile float car_lookahead_m;
extern volatile float car_target_speed_mps;
extern volatile float car_target_steer_count;
extern volatile float car_pure_pursuit_curvature;
extern volatile float car_replay_avg_speed_mps;
extern volatile int16 car_fixed_rear_pwm;
extern volatile float car_pwm_sim_x_m;
extern volatile float car_pwm_sim_y_m;
extern volatile float car_pwm_sim_yaw_deg;
extern volatile float car_pwm_sim_speed_mps;
extern volatile float car_pwm_sim_steer_angle_deg;
extern volatile uint32 car_pwm_sim_nearest_index;
extern volatile uint8 car_pwm_sim_gps_dir_locked;
extern volatile int16 car_steer_pwm_output;
extern volatile int16 car_rear_pwm_output;
extern volatile int8 car_drive_direction;

void  car_nav_init             (void);
void  car_nav_control_10ms     (void);
void  car_nav_display          (void);
void  car_nav_gnss_poll        (void);
float car_pid_calc             (car_pid_struct *pid, float target, float current);
void  car_all_motor_stop       (void);

#endif
