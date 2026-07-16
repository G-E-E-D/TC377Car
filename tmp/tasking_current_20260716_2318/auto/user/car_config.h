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
#define CAR_KEY_CHORD_WINDOW_COUNT     (30)          // Delay READY single-key action 300ms so A+B cannot launch by mistake.
#define CAR_KEY_CHORD_HOLD_COUNT       (80)          // Hold A+B 800ms to clear the taught path.
#define CAR_GUIDE_ARM_CONFIRM_ENABLE   (1)           // First A/B selects and arms; press the same key again to run.
#define CAR_GUIDE_ARM_MIN_MS           (300)

#define CAR_ERROR_NONE                 (0)
#define CAR_ERROR_SELF_TEST            (1)
#define CAR_ERROR_PATH_SHORT           (2)
#define CAR_ERROR_GPS_LOST             (3)
#define CAR_ERROR_START_TOO_FAR        (4)
#define CAR_ERROR_GPS_CORRIDOR         (5)
#define CAR_ERROR_GUIDE_TIMEOUT        (6)
#define CAR_ERROR_DATUM_NOT_READY      (7)
#define CAR_ERROR_START_HEADING        (8)
#define CAR_ERROR_NUMERIC              (9)
#define CAR_ERROR_EKF_REJECT           (10)
#define CAR_ERROR_ENCODER              (11)
#define CAR_ERROR_MPU                  (12)
#define CAR_ERROR_PATH_INVALID         (13)
#define CAR_ERROR_NO_PROGRESS          (14)
#define CAR_ERROR_STEER_NOT_CENTERED   (15)
#define CAR_ERROR_CONTROL_OVERRUN      (16)
#define CAR_ERROR_CONFIG               (17)

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
#define CAR_CONTROL_OVERRUN_10NS       (900000U)     // 9ms deadline, system_getval() unit is 10ns.
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
#define CAR_PATH_YAW_SPAN_M            (0.500f)      // Spatial chord used for noise-resistant endpoint/interior headings.
#define CAR_PATH_MIN_LENGTH_M          (0.800f)
#define CAR_PATH_MAX_SEGMENT_M         (1.000f)      // Reject an unbridged GNSS jump/dropout in a taught path.
#define CAR_PATH_DIRECTION_CONFIRM_PT  (3)
#define CAR_TIME_LOOKAHEAD_MS          (300)
#define CAR_PATH_FINISH_DISTANCE_M     (0.450f)
#define CAR_PATH_FINISH_FAILSAFE_M     (0.750f)
#define CAR_PATH_FINISH_INDEX_REMAIN   (6)
#define CAR_PATH_FINISH_INDEX_STOP     (1)           // Fail-safe stop at the final spatial sample even if GPS range is biased.
#define CAR_PATH_FINISH_STOP_FIX_COUNT (3)           // Confirm the fail-safe endpoint with consecutive new RMC fixes.
#define CAR_START_MIN_POINTS           (8)
#define CAR_GUIDE_START_MAX_DIST_M     (0.60f)       // Must be well inside the 1.5m emergency corridor before PWM is enabled.
#define CAR_GUIDE_START_HEADING_MAX_DEG (45.0f)      // Reject a kart facing the wrong way; static single-GNSS heading is unavailable.
#define CAR_WHEELBASE_M                (0.260f)      // 占位值：必须实测前后轴中心距后修改。
#define CAR_STEER_ENC_MAX_ABS_COUNT    (100.0f)      // 占位值：必须实测左右极限编码器计数。
#define CAR_STEER_MAX_ANGLE_DEG        (45.0f)       // 占位值：必须实测上述计数对应的真实前轮角。
#define CAR_STEER_ANGLE_SIGN           (-1.0f)       // ENC=-100 is left; left turn is positive model yaw.
#define CAR_STEER_GEOMETRY_CALIBRATED  (0)           // 完成轴距和轮角映射标定后才可改为1。
#define CAR_STEER_MODEL_ENABLE         (1)
#define CAR_STEER_MODEL_MIN_SPEED_MPS  (0.12f)
#if CAR_STEER_GEOMETRY_CALIBRATED
#define CAR_STEER_MODEL_BLEND          (0.65f)
#else
#define CAR_STEER_MODEL_BLEND          (0.15f)       // Safe fallback: encoder still supplies route FF, but cannot dominate yaw uncalibrated.
#endif
#define CAR_STEER_MODEL_MAX_YAW_DPS    (180.0f)
#define CAR_STEER_ENCODER_MAX_DELTA_10MS (100)
#define CAR_STEER_ENCODER_HARD_LIMIT   (140)
#define CAR_RECORD_START_STEER_CENTER_TOL_CNT (5)    // Power on with wheels mechanically centred; no absolute sensor exists.
#define CAR_STEER_WRONG_DIR_COUNT      (5)
#define CAR_STEER_STALL_COUNT          (100)         // 1s commanded motion with no encoder response locks the motors.
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
#define CAR_GUIDE_TIMEOUT_MIN_SPEED_MPS (0.15f)
#define CAR_GUIDE_FINISH_MARGIN_MS     (3000)
#define CAR_GUIDE_NO_PROGRESS_MS       (4000)
#define CAR_GUIDE_PROGRESS_MIN_M       (0.15f)
#define CAR_GUIDE_FORCE_FORWARD        (0)          // Preserve forward/reverse route segments for reverse parking.
#define CAR_GUIDE_DIRECTION_HOLD_MS    (500)
#define CAR_GPS_PRIORITY_STOP_ENABLE   (1)
#define CAR_GPS_PRIORITY_STOP_M        (1.50f)      // GUIDE stops when filtered GPS stays this far from REC path.
#define CAR_GPS_PRIORITY_STOP_COUNT    (4)          // Four consecutive NEW 10Hz fixes outside the corridor.
#define CAR_GPS_SAFETY_WINDOW_BACK_PT  (15)
#define CAR_GPS_SAFETY_WINDOW_AHEAD_PT (30)

//==================================================== imu/gps fusion ====================================================
#define CAR_GYRO_BIAS_SAMPLE_COUNT     (500)
#define CAR_GYRO_BIAS_SAMPLE_DELAY_MS  (2)
#define CAR_GYRO_BIAS_STD_MAX_DPS      (2.0f)
#define CAR_GYRO_DEADBAND_DPS          (0.20f)
#define CAR_GYRO_SCALE                 (1.00f)
#define CAR_IMU_GYRO_MAX_DPS           (500.0f)
#define CAR_IMU_FORWARD_RAW            (mpu6050_acc_x) // Change to mpu6050_acc_y if the MPU Y axis points forward.
#define CAR_IMU_FORWARD_SIGN           (1.0f)          // Set to -1.0f if forward acceleration is displayed negative.
#define CAR_GRAVITY_MPS2               (9.80665f)
#define CAR_IMU_ACC_MAX_G              (4.0f)
#define CAR_ACC_BIAS_SAMPLE_COUNT      (500)
#define CAR_ACC_BIAS_STD_MAX_G         (0.08f)
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
#ifndef CAR_LOCAL_DATUM_FIXED_ENABLE
#define CAR_LOCAL_DATUM_FIXED_ENABLE   (0)           // 0: average fixes at power-on; 1: use the surveyed coordinates below.
#endif
#ifndef CAR_LOCAL_DATUM_FIXED_CONFIGURED
#define CAR_LOCAL_DATUM_FIXED_CONFIGURED (0)         // Set to 1 only after entering and checking both fixed coordinates.
#endif
#ifndef CAR_LOCAL_DATUM_FIXED_LAT_DEG
#define CAR_LOCAL_DATUM_FIXED_LAT_DEG  (0.0)         // WGS-84 decimal degrees; fill before enabling fixed mode.
#endif
#ifndef CAR_LOCAL_DATUM_FIXED_LON_DEG
#define CAR_LOCAL_DATUM_FIXED_LON_DEG  (0.0)
#endif
#define CAR_LOCAL_DATUM_FIXED_MAX_DIST_M (500.0f)    // Refuse a fixed datum implausibly far from the live receiver.
#define CAR_LOCAL_DATUM_SAMPLE_COUNT   (50)          // Keep still for 50 valid 10Hz RMC fixes (about 5 seconds).
#define CAR_LOCAL_DATUM_MAX_SPEED_MPS  (0.20f)       // Restart averaging if the receiver reports motion.
#define CAR_LOCAL_DATUM_MAX_RADIUS_M   (1.50f)       // Every accepted fix must stay near the running mean.
#define CAR_LOCAL_DATUM_RESEED_REJECTS (3)           // A sustained shift restarts the cluster; isolated jumps are discarded.
#define CAR_LOCAL_DATUM_MAX_CENTROID_DRIFT_M (0.60f) // First/last half centroids must agree before the datum can lock.
#define CAR_GPS_FILTER_WINDOW          (3)
#define CAR_GPS_FILTER_ALPHA           (0.55f)
#define CAR_GPS_FILTER_DELAY_S         (0.18f)        // Median3 + IIR delay at a 10Hz RMC rate; used for bounded projection.
#define CAR_GPS_HEADING_BASELINE_M     (0.50f)        // Position-displacement heading remains usable below RMC course threshold.
#define CAR_GPS_STALE_MS               (600)
#define CAR_GPS_GGA_STALE_MS           (1000)
#define CAR_GPS_MAX_HDOP               (4.0f)
#define CAR_GPS_DATA_MAX_SPEED_MPS     (2.0f)        // Physical plausibility limit for this low-speed kart.
#define CAR_GPS_RAW_JUMP_BASE_M        (0.80f)
#define CAR_GPS_RAW_JUMP_MAX_SPEED_MPS (2.50f)
#define CAR_GPS_POS_BLEND              (0.015f)
#define CAR_GPS_YAW_BLEND              (0.010f)
#define CAR_GPS_YAW_MIN_SPEED_KMH      (3.0f)        // Low-speed course is noisy; use steering encoder yaw instead.
#define CAR_GPS_MAX_CORRECT_M          (3.0f)
#define CAR_GPS_SPEED_BLEND            (0.20f)
#define CAR_GPS_TRACK_ENABLE           (1)          // GPS is a slow spatial corrector, not the main slalom feedback.

//==================================================== EKF estimator ====================================================
#define CAR_EKF_ENABLE                 (1)
#define CAR_EKF_GPS_POS_GATE_M         (2.0f)       // Reject a position inconsistent with the low-speed vehicle model.
#define CAR_EKF_POS_REJECT_STOP_COUNT  (3)
#define CAR_EKF_GPS_SPEED_GATE_MPS     (0.80f)
#define CAR_EKF_GPS_YAW_GATE_DEG       (75.0f)
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
#define CAR_LOW_SPEED_MPS              (0.35f)        // 模型目标速度；必须用10m实跑校准。
#define CAR_LOW_SPEED_REAR_PWM         (2000)         // 教学和A/B共用低PWM；固定PWM不能保证真实速度恒定。
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
#define CAR_NEAREST_MAX_ADVANCE_PT     (1)
#define CAR_NEAREST_MAX_REGRESS_PT     (2)
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
#define CAR_TRACK_RECOVER_M             (0.28f)      // Once outside, keep correcting until safely back inside.
#define CAR_TRACK_CORRECTION_KP         (80.0f)
#define CAR_TRACK_CORRECTION_KI         (0.15f)
#define CAR_TRACK_CORRECTION_KD         (12.0f)
#define CAR_TRACK_INTEGRAL_LIMIT        (120.0f)
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

#define CAR_GPS_ANTENNA_FORWARD_M      (0.0f)        // 从后轴参考点量到天线，车头方向为正；需实测。
#define CAR_GPS_ANTENNA_LEFT_M         (0.0f)        // 天线相对参考点向左为正；需实测。

#if (CAR_GPS_FILTER_WINDOW < 1) || (CAR_GPS_FILTER_WINDOW > 255)
#error CAR_GPS_FILTER_WINDOW must be in 1..255
#endif
#if (CAR_PATH_MAX_POINTS < CAR_START_MIN_POINTS)
#error CAR_PATH_MAX_POINTS must be at least CAR_START_MIN_POINTS
#endif
#if (CAR_KEY_CHORD_HOLD_COUNT > 255) || (CAR_KEY_CHORD_WINDOW_COUNT > 255)
#error Key timing counters must fit uint8
#endif
#if (CAR_LOW_SPEED_REAR_PWM > CAR_REAR_DUTY_LIMIT)
#error CAR_LOW_SPEED_REAR_PWM exceeds CAR_REAR_DUTY_LIMIT
#endif

#define CAR_APP_VERSION_TEXT           "KM2_AB_20260716G"

#endif
