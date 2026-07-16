#ifndef _car_config_h_
#define _car_config_h_

#include "zf_common_headfile.h"

//==================================================== hardware mapping ====================================================
#define CAR_USE_BOARD_KEYS             (1)
#define CAR_KEY_A_PIN                  (P20_6)      // Board KEY1: start/stop teach, then start A (start -> finish).
#define CAR_KEY_B_PIN                  (P20_7)      // Board KEY2: stop teach, then start B (finish -> start).
#define CAR_KEY_RELEASE_LEVEL          (GPIO_HIGH)
#define CAR_KEY_ACTIVE_LEVEL           (GPIO_LOW)
#define CAR_KEY_COOLDOWN_COUNT         (30)          // 30 * 10ms = 300ms debounce/cooldown.

#define CAR_ERROR_NONE                 (0)
#define CAR_ERROR_SELF_TEST            (1)
#define CAR_ERROR_PATH_SHORT           (2)
#define CAR_ERROR_GPS_LOST             (3)
#define CAR_ERROR_START_TOO_FAR        (4)
#define CAR_ERROR_GPS_CORRIDOR         (5)
#define CAR_ERROR_GUIDE_TIMEOUT        (6)
#define CAR_ERROR_DATUM_NOT_READY      (7)

#define CAR_LEFT_MOTOR_DIR_PIN         (P21_4)
#define CAR_LEFT_MOTOR_PWM_CH          (ATOM0_CH3_P21_5)
#define CAR_RIGHT_MOTOR_DIR_PIN        (P21_2)
#define CAR_RIGHT_MOTOR_PWM_CH         (ATOM0_CH1_P21_3)
#define CAR_MOTOR_FORWARD_LEVEL        (GPIO_LOW)
#define CAR_MOTOR_REVERSE_LEVEL        (GPIO_HIGH)
#define CAR_LEFT_MOTOR_DUTY_SIGN       (1)          // Positive rear PWM must drive the real car forward.
#define CAR_RIGHT_MOTOR_DUTY_SIGN      (1)

#define CAR_STEER_MOTOR_DIR_PIN        (P02_5)
#define CAR_STEER_MOTOR_PWM_CH         (ATOM0_CH4_P02_4)
#define CAR_STEER_FORWARD_LEVEL        (GPIO_HIGH)
#define CAR_STEER_REVERSE_LEVEL        (GPIO_LOW)

// Steering motor encoder.
#define CAR_STEER_ENCODER              (TIM2_ENCODER)
#define CAR_STEER_ENCODER_CH1_PIN      (TIM2_ENCODER_CH1_P33_7)
#define CAR_STEER_ENCODER_CH2_PIN      (TIM2_ENCODER_CH2_P33_6)
#define CAR_STEER_ENCODER_REVERSE      (0)
#define CAR_STEER_ENCODER_DIR_MODE     (0)          // The PDF specifies dual-Hall A/B quadrature outputs.

#define CAR_IPS200_TYPE                (IPS200_TYPE_SPI)
#define CAR_GNSS_DEVICE                (TAU1201)

//==================================================== timing ====================================================
#define CAR_CONTROL_PERIOD_MS          (10)
#define CAR_CONTROL_PERIOD_S           (0.01f)
#define CAR_UI_REFRESH_MS              (100)
#define CAR_DISPLAY_MAP_TOP_Y          (96)
#define CAR_DISPLAY_MAP_FIELD_M        (30.0f)      // REC trace view covers a 30m x 30m field.
#define CAR_DISPLAY_SIM_COLOR          (0x07FF)     // RGB565 cyan.

//==================================================== vehicle parameters ====================================================
#define CAR_PATH_MAX_POINTS            (2400)
#define CAR_PATH_RECORD_INTERVAL_MS    (50)          // 2400 * 50ms = 120s maximum teaching route.
#define CAR_PATH_RECORD_MIN_MOVE_M     (0.015f)       // 0.35m/s creates about one fused point every 50ms.
#define CAR_PATH_RECORD_MIN_SPEED_MPS  (0.08f)
#define CAR_PATH_RECORD_USE_YAW        (0)
#define CAR_PATH_RECORD_MIN_YAW_DEG    (3.00f)
#define CAR_PATH_RECORD_USE_STEER      (1)           // Front steering encoder is a primary teaching input.
#define CAR_PATH_RECORD_MIN_STEER      (10.0f)
#define CAR_PATH_SMOOTH_PASSES         (5)
#define CAR_PATH_RESAMPLE_M            (0.100f)
#define CAR_PATH_DIRECTION_CONFIRM_PT  (3)
#define CAR_TIME_LOOKAHEAD_MS          (300)
#define CAR_PATH_FINISH_DISTANCE_M     (0.450f)
#define CAR_PATH_FINISH_INDEX_REMAIN   (6)
#define CAR_PATH_FINISH_INDEX_STOP     (1)           // Fail-safe stop at the final spatial sample even if GPS range is biased.
#define CAR_PATH_FINISH_STOP_FIX_COUNT (3)           // Confirm the fail-safe endpoint with consecutive new RMC fixes.
#define CAR_START_MIN_POINTS           (8)
#define CAR_GUIDE_START_MAX_DIST_M     (2.00f)
#define CAR_WHEELBASE_M                (0.260f)      // Measure front axle to rear axle, then update this value.
#define CAR_STEER_ENC_MAX_ABS_COUNT    (100.0f)      // Your measured full-left/full-right encoder count.
#define CAR_STEER_MAX_ANGLE_DEG        (45.0f)       // Front wheel angle at +/-CAR_STEER_ENC_MAX_ABS_COUNT.
#define CAR_STEER_ANGLE_SIGN           (-1.0f)       // ENC=-100 is left; left turn is positive model yaw.
#define CAR_STEER_MODEL_ENABLE         (1)
#define CAR_STEER_MODEL_MIN_SPEED_MPS  (0.12f)
#define CAR_STEER_MODEL_BLEND          (0.80f)       // GPS position + front encoder bicycle model are primary.
#define CAR_STEER_MODEL_MAX_YAW_DPS    (180.0f)
#define CAR_STEER_SELF_TEST_ENABLE     (0)           // Power-on must remain stopped and wait for a key.
#define CAR_STEER_SELF_TEST_DELAY_MS   (1000)
#define CAR_STEER_SELF_TEST_LEFT_CNT   (-105.0f)
#define CAR_STEER_SELF_TEST_RIGHT_CNT  (105.0f)
#define CAR_STEER_SELF_TEST_CENTER_CNT (0.0f)
#define CAR_STEER_SELF_TEST_PASS_CNT   (95.0f)
#define CAR_STEER_SELF_TEST_TOL_CNT    (5.0f)
#define CAR_STEER_SELF_TEST_STEP_MS    (2500)
#define CAR_STEER_SELF_TEST_SETTLE_CT  (8)
#define CAR_STEER_SELF_TEST_MIN_DUTY   (3500)
#define CAR_REAR_SELF_TEST_DUTY        (3000)
#define CAR_REAR_SELF_TEST_TIME_MS     (400)
#define CAR_REAR_SELF_TEST_STOP_MS     (800)
#define CAR_GUIDE_PWM_SIM_ENABLE       (1)
#define CAR_GUIDE_PWM_SIM_USE_TARGET   (1)          // 1: simulate software target steer, 0: simulate actual encoder steer.
#define CAR_GUIDE_PWM_SIM_DIR_SIGN     (1.0f)       // Set to -1.0f if the cyan simulated trace is 180deg reversed.
#define CAR_GUIDE_PWM_SIM_SPEED_MPS    (1.40f)      // Simulated speed at CAR_REAR_DUTY_LIMIT.
#define CAR_GUIDE_PWM_SIM_MIN_DUTY     (300)
#define CAR_GUIDE_PWM_SIM_MAX_SPEED    (1.80f)
#define CAR_GUIDE_SIM_START_SCAN       (80)
#define CAR_GUIDE_SIM_START_MIN_DIST_M (1.00f)
#define CAR_GUIDE_SIM_GPS_DIR_ENABLE   (1)
#define CAR_GUIDE_SIM_GPS_DIR_MIN_M    (0.60f)
#define CAR_GUIDE_FINISH_USE_SIM       (0)          // The screen simulation must never finish the real vehicle.
#define CAR_GUIDE_FINISH_MIN_MS        (1500)
#define CAR_GUIDE_FINISH_TIME_SCALE    (1.60f)
#define CAR_GUIDE_FINISH_MARGIN_MS     (1500)
#define CAR_GUIDE_FORCE_FORWARD        (0)          // Preserve forward/reverse route segments for reverse parking.
#define CAR_GUIDE_DIRECTION_HOLD_MS    (500)
#define CAR_GPS_PRIORITY_STOP_ENABLE   (1)
#define CAR_GPS_PRIORITY_STOP_M        (1.50f)      // GUIDE stops when filtered GPS stays this far from REC path.
#define CAR_GPS_PRIORITY_STOP_COUNT    (4)          // Four consecutive NEW 10Hz fixes outside the corridor.

//==================================================== imu/gps fusion ====================================================
#define CAR_GYRO_BIAS_SAMPLE_COUNT     (500)
#define CAR_GYRO_BIAS_SAMPLE_DELAY_MS  (2)
#define CAR_GYRO_DEADBAND_DPS          (0.20f)
#define CAR_GYRO_SCALE                 (1.00f)
#define CAR_IMU_FORWARD_RAW            (mpu6050_acc_x) // Change to mpu6050_acc_y if the MPU Y axis points forward.
#define CAR_IMU_FORWARD_SIGN           (1.0f)          // Set to -1.0f if forward acceleration is displayed negative.
#define CAR_GRAVITY_MPS2               (9.80665f)
#define CAR_ACC_BIAS_SAMPLE_COUNT      (500)
#define CAR_ACC_LOWPASS_ALPHA          (0.12f)
#define CAR_ACC_DEADBAND_MPS2          (0.10f)
#define CAR_ACC_STILL_LIMIT_MPS2       (0.16f)
#define CAR_GYRO_STILL_LIMIT_DPS       (0.50f)
#define CAR_STILL_CONFIRM_COUNT        (60)             // 60 * 10ms = 0.6s before velocity is forced to zero.
#define CAR_VELOCITY_LEAK_PER_SEC      (0.015f)          // Small drift suppression while acceleration is near zero.
#define CAR_IMU_SPEED_LIMIT_MPS        (1.50f)
#define CAR_DEG_TO_RAD                 (0.01745329252f)
#define CAR_RAD_TO_DEG                 (57.295779513f)
#define CAR_GPS_MIN_SATELLITES         (6)
#define CAR_GPS_READY_FIX_COUNT        (10)          // Require consecutive valid frames before a key can start motion.
#define CAR_ALLOW_RECORD_WITHOUT_GPS   (0)
#define CAR_LOCAL_DATUM_FIXED_ENABLE   (0)           // 0: average fixes at power-on; 1: use the surveyed coordinates below.
#define CAR_LOCAL_DATUM_FIXED_CONFIGURED (0)         // Set to 1 only after entering and checking both fixed coordinates.
#define CAR_LOCAL_DATUM_FIXED_LAT_DEG  (0.0)         // WGS-84 decimal degrees; fill before enabling fixed mode.
#define CAR_LOCAL_DATUM_FIXED_LON_DEG  (0.0)
#define CAR_LOCAL_DATUM_FIXED_MAX_DIST_M (500.0f)    // Refuse a fixed datum implausibly far from the live receiver.
#define CAR_LOCAL_DATUM_SAMPLE_COUNT   (50)          // Keep still for 50 valid 10Hz RMC fixes (about 5 seconds).
#define CAR_LOCAL_DATUM_MAX_SPEED_MPS  (0.20f)       // Restart averaging if the receiver reports motion.
#define CAR_LOCAL_DATUM_MAX_RADIUS_M   (1.50f)       // Every accepted fix must stay near the running mean.
#define CAR_LOCAL_DATUM_RESEED_REJECTS (3)           // A sustained shift restarts the cluster; isolated jumps are discarded.
#define CAR_GPS_FILTER_WINDOW          (3)
#define CAR_GPS_FILTER_ALPHA           (0.55f)
#define CAR_GPS_STALE_MS               (600)
#define CAR_GPS_DATA_MAX_SPEED_MPS     (20.0f)       // Reject corrupt/non-finite RMC speed before fusion.
#define CAR_GPS_POS_BLEND              (0.015f)
#define CAR_GPS_YAW_BLEND              (0.010f)
#define CAR_GPS_YAW_MIN_SPEED_KMH      (3.0f)        // Low-speed course is noisy; use steering encoder yaw instead.
#define CAR_GPS_MAX_CORRECT_M          (3.0f)
#define CAR_GPS_SPEED_BLEND            (0.20f)
#define CAR_GPS_TRACK_ENABLE           (1)          // GPS is a slow spatial corrector, not the main slalom feedback.

//==================================================== EKF estimator ====================================================
#define CAR_EKF_ENABLE                 (1)
#define CAR_EKF_GPS_POS_GATE_M         (5.0f)       // Reject one-shot GPS jumps larger than this.
#define CAR_EKF_Q_POS                  (0.0025f)
#define CAR_EKF_Q_YAW_RAD              (0.000030f)
#define CAR_EKF_Q_SPEED                (0.0180f)
#define CAR_EKF_Q_GYRO_BIAS            (0.0000008f)
#define CAR_EKF_Q_ACC_BIAS             (0.000080f)
#define CAR_EKF_R_GPS_POS              (0.18f)       // Filtered 10Hz TAU1201 fixes strongly anchor the fused path.
#define CAR_EKF_R_GPS_SPEED            (0.040f)
#define CAR_EKF_R_GPS_YAW_RAD          (0.035f)
#define CAR_EKF_R_STILL_SPEED          (0.004f)
#define CAR_EKF_MIN_COV                (0.000001f)

//==================================================== control parameters ====================================================
#define CAR_PWM_FREQ_HZ                (17000)
#define CAR_REAR_DUTY_LIMIT            (5200)
#define CAR_REAR_MIN_MOVE_DUTY         (1500)
#define CAR_REAR_PWM_SLEW_STEP         (140)
#define CAR_LOW_SPEED_MPS              (0.35f)        // Fixed teaching/A/B model speed; calibrate on the real car.
#define CAR_LOW_SPEED_REAR_PWM         (2000)         // Fixed low rear PWM used in teaching and both replay modes.
#define CAR_DEFAULT_TARGET_SPEED_MPS   (CAR_LOW_SPEED_MPS)
#define CAR_MIN_TARGET_SPEED_MPS       (0.20f)
#define CAR_MAX_TARGET_SPEED_MPS       (0.55f)
#define CAR_REPLAY_SPEED_SCALE         (1.00f)
#define CAR_CURVE_SPEED_GAIN           (1.60f)
#define CAR_GUIDE_FIXED_REAR_PWM       (1)          // 1: use one fixed rear PWM during GUIDE, ignoring noisy VEL feedback.
#define CAR_GUIDE_FIXED_SPEED_MIN_MPS  (0.20f)
#define CAR_GUIDE_FIXED_SPEED_MAX_MPS  (0.55f)
#define CAR_GUIDE_FIXED_PWM_PER_MPS    (4200.0f)
#define CAR_GUIDE_FIXED_PWM_MIN        (1700)
#define CAR_GUIDE_FIXED_PWM_MAX        (3200)
#define CAR_GUIDE_SPEED_MODEL_TAU_S    (0.35f)
#define CAR_GUIDE_SPEED_MODEL_ACC_MAX  (1.50f)

#define CAR_LOOKAHEAD_MIN_M            (0.35f)
#define CAR_LOOKAHEAD_GAIN             (0.35f)
#define CAR_LOOKAHEAD_MAX_M            (0.90f)
#define CAR_PURE_PURSUIT_GAIN          (1.00f)
#define CAR_PATH_CURVATURE_FF_GAIN     (0.20f)
#define CAR_NEAREST_SEARCH_BACK_PT     (3)
#define CAR_NEAREST_SEARCH_AHEAD_PT    (20)
#define CAR_NEAREST_HEADING_GATE_DEG   (85.0f)
#define CAR_CROSS_TRACK_STOP_M         (1.50f)
#define CAR_STEER_TARGET_LIMIT         (95.0f)
#define CAR_STEER_PWM_LIMIT            (6000)
#define CAR_STEER_MOVE_MIN_DUTY        (3500)
#define CAR_STEER_POSITION_TOL_CNT     (3.0f)
#define CAR_STEER_FF_GAIN              (1.00f)       // Replay the smoothed front-encoder steering inside the corridor.
#define CAR_STEER_YAW_K                (0.0f)        // Pure Pursuit now creates the steering target directly.
#define CAR_STEER_RATE_K               (0.0f)
#define CAR_STEER_CTE_K                (0.0f)
#define CAR_GPS_CTE_LIMIT_M            (1.20f)
#define CAR_GPS_CTE_MIN_SPEED_MPS      (0.15f)

// Outside this corridor, an outer PID adds steering correction. Inside it, correction is exactly zero.
#define CAR_TRACK_TOLERANCE_M           (0.35f)
#define CAR_TRACK_CORRECTION_KP         (80.0f)
#define CAR_TRACK_CORRECTION_KI         (0.0f)
#define CAR_TRACK_CORRECTION_KD         (12.0f)
#define CAR_TRACK_INTEGRAL_LIMIT        (2.0f)
#define CAR_TRACK_CORRECTION_LIMIT      (55.0f)
#define CAR_TRACK_HEADING_K             (0.35f)

#define CAR_STEER_POS_KP               (18.0f)
#define CAR_STEER_POS_KI               (0.0f)
#define CAR_STEER_POS_KD               (2.0f)
#define CAR_STEER_INTEGRAL_LIMIT       (800.0f)

#define CAR_SPEED_KP                   (2600.0f)
#define CAR_SPEED_KI                   (0.0f)
#define CAR_SPEED_KD                   (120.0f)
#define CAR_SPEED_INTEGRAL_LIMIT       (3.0f)
#define CAR_SPEED_FEEDFORWARD          (2800.0f)
#define CAR_SIGNED_SPEED_DEADBAND_MPS  (0.06f)

#define CAR_APP_VERSION_TEXT           "KM2_AB_20260716F"

#endif
