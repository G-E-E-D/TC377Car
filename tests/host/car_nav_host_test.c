#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../km2/user/car_nav.c"

gnss_info_struct gnss;
volatile uint8 gnss_flag = 0;
volatile uint8 gnss_rmc_flag = 0;
volatile uint8 gnss_gga_flag = 0;
volatile uint32 gnss_rmc_count = 0;
volatile uint32 gnss_gga_count = 0;
int16 mpu6050_gyro_z = 0;
int16 mpu6050_acc_x = 0;
int16 mpu6050_acc_y = 0;
uint16 ips200_width_max = 240;
uint16 ips200_height_max = 320;

static gpio_level_enum test_key_a = GPIO_HIGH;
static gpio_level_enum test_key_b = GPIO_HIGH;
static int16 test_encoder_delta = 0;
static uint32 test_left_pwm_duty = 0;
static uint32 test_right_pwm_duty = 0;
static uint32 test_steer_pwm_duty = 0;
static int test_failures = 0;
static uint32 test_checks = 0;

#if defined(CAR_NEAREST_MAX_ADVANCE_PT)
#define TEST_NEAREST_MAX_PROGRESS_PT   ((uint32)CAR_NEAREST_MAX_ADVANCE_PT)
#else
#define TEST_NEAREST_MAX_PROGRESS_PT   (5U)
#endif

#define TEST_ASSERT(condition, message) do { \
    test_checks++; \
    if(!(condition)) { \
        fprintf(stderr, "FAIL: %s (line %d)\n", (message), __LINE__); \
        test_failures++; \
    } \
} while(0)

#define TEST_NEAR(actual, expected, tolerance, message) do { \
    float test_actual_ = (float)(actual); \
    float test_expected_ = (float)(expected); \
    test_checks++; \
    if(fabsf(test_actual_ - test_expected_) > (tolerance)) { \
        fprintf(stderr, "FAIL: %s: got %.7f expected %.7f tol %.7f (line %d)\n", \
                (message), test_actual_, test_expected_, (float)(tolerance), __LINE__); \
        test_failures++; \
    } \
} while(0)

static void reset_navigation_globals(void);
static void datum_offset(double latitude_deg, double longitude_deg,
        double east_m, double north_m, double *latitude_out, double *longitude_out);
static void set_gnss_fix(double latitude_deg, double longitude_deg,
        float speed_mps, float course_deg);
static void poll_gga_frame(void);
static void poll_rmc_frame(void);

static void test_gps_quality_jump_and_stale_watchdogs(void)
{
    float accepted_x;
    float accepted_y;
    uint32 accepted_fix_count;
    uint32 accepted_sequence;
    double jump_lat;
    double jump_lon;
    const double lat0 = 31.840000;
    const double lon0 = 117.250000;

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    nav_uptime_ms = 100;
    set_gnss_fix(lat0, lon0, 0.0f, 90.0f);
    poll_rmc_frame();
    TEST_ASSERT(0 == car_gps_valid,
            "RMC is rejected until at least one fresh GGA frame has been observed");
    TEST_ASSERT(0 == gps_update_pending, "RMC without GGA cannot publish a pending snapshot");

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    nav_uptime_ms = 100;
    set_gnss_fix(lat0, lon0, 0.0f, 90.0f);
    gnss.fix_quality = 0;
    poll_gga_frame();
    poll_rmc_frame();
    TEST_ASSERT(0 == car_gps_valid, "GGA fix_quality zero invalidates RMC position");

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    nav_uptime_ms = 100;
    set_gnss_fix(lat0, lon0, 0.0f, 90.0f);
    gnss.hdop = CAR_GPS_MAX_HDOP + 0.1f;
    poll_gga_frame();
    poll_rmc_frame();
    TEST_ASSERT(0 == car_gps_valid, "excessive GGA HDOP invalidates RMC position");

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    nav_uptime_ms = 100;
    set_gnss_fix(lat0, lon0, 0.0f, 90.0f);
    poll_gga_frame();
    nav_uptime_ms += CAR_GPS_GGA_STALE_MS + 1U;
    poll_rmc_frame();
    TEST_ASSERT(0 == car_gps_valid, "stale GGA metadata invalidates a newer RMC frame");

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    car_ekf_init_state(0.0f, 0.0f, 0.0f, 0.0f);
    nav_uptime_ms = 100;
    set_gnss_fix(lat0, lon0, 0.0f, 90.0f);
    poll_gga_frame();
    nav_uptime_ms = 200;
    poll_rmc_frame();
    TEST_ASSERT(1 == car_gps_valid, "fresh quality GGA plus valid RMC publishes GPS validity");
    TEST_ASSERT(1 == gps_update_pending, "valid RMC publishes one pending snapshot");
    car_ekf_apply_pending_gps();
    accepted_x = car_gps_x_m;
    accepted_y = car_gps_y_m;
    accepted_fix_count = car_gps_fix_count;
    accepted_sequence = gps_publish_sequence;

    datum_offset(lat0, lon0, 3.0, 0.0, &jump_lat, &jump_lon);
    nav_uptime_ms = 300;
    set_gnss_fix(jump_lat, jump_lon, 0.0f, 90.0f);
    poll_rmc_frame();
    TEST_ASSERT(0 == gps_update_pending, "isolated three-metre jump is not published");
    TEST_ASSERT(accepted_sequence == gps_publish_sequence,
            "rejected jump cannot modify snapshot publication sequence");
    TEST_ASSERT(accepted_fix_count == car_gps_fix_count,
            "rejected jump cannot increment consumed GPS fix count");
    TEST_NEAR(car_gps_x_m, accepted_x, 0.0001f, "rejected jump preserves prior GPS x");
    TEST_NEAR(car_gps_y_m, accepted_y, 0.0001f, "rejected jump preserves prior GPS y");
    TEST_ASSERT(0 == gps_valid_fix_streak,
            "rejected jump breaks the consecutive-good-fix readiness streak");

    nav_uptime_ms = 200U + CAR_GPS_STALE_MS + 1U;
    gnss_flag = 0;
    car_nav_gnss_poll();
    TEST_ASSERT(0 == car_gps_valid,
            "accepted-fix watchdog invalidates GPS 600ms after an unrecovered jump");
    TEST_ASSERT(0 == gps_update_pending,
            "stale watchdog clears any unconsumed GPS snapshot");
}

static void test_pending_gps_snapshot_sequence(void)
{
    float expected_x;
    float expected_y;
    uint32 fix_count;

    reset_navigation_globals();
    car_local_datum_lock(31.84, 117.25);
    car_ekf_init_state(0.0f, 0.0f, 0.0f, 0.0f);
    gps_pending_x_m = 1.0f;
    gps_pending_y_m = 2.0f;
    gps_pending_speed_mps = 0.5f;
    gps_pending_yaw_deg = 0.0f;
    gps_pending_heading_yaw_deg = 0.0f;
    gps_pending_heading_valid = 0;
    gps_update_pending = 1;
    gps_publish_sequence = 2;
    car_gps_measurement_to_reference(gps_pending_x_m, gps_pending_y_m,
            gps_pending_speed_mps, gps_pending_yaw_deg, car_yaw_deg,
            &expected_x, &expected_y);
    car_ekf_apply_pending_gps();
    TEST_ASSERT(0 == gps_update_pending, "one ISR pass consumes one complete even-sequence snapshot");
    TEST_NEAR(car_gps_x_m, expected_x, 0.0001f, "snapshot x is consumed coherently");
    TEST_NEAR(car_gps_y_m, expected_y, 0.0001f, "snapshot y is consumed coherently");
    TEST_NEAR(car_gps_speed_mps, 0.5f, 0.0001f, "snapshot speed is consumed coherently");
    TEST_NEAR(car_gps_direction_deg, 0.0f, 0.0001f, "snapshot yaw is consumed coherently");
    fix_count = car_gps_fix_count;
    car_ekf_apply_pending_gps();
    TEST_ASSERT(fix_count == car_gps_fix_count,
            "same published snapshot cannot be consumed twice");

    gps_pending_x_m = 10.0f;
    gps_pending_y_m = 20.0f;
    gps_pending_speed_mps = 0.2f;
    gps_pending_yaw_deg = 90.0f;
    gps_pending_heading_yaw_deg = 95.0f;
    gps_pending_heading_valid = 1;
    gps_update_pending = 1;
    gps_publish_sequence = 3;
    car_ekf_apply_pending_gps();
    TEST_ASSERT(1 == gps_update_pending,
            "ISR defers an in-progress odd-sequence GPS publication");
    TEST_ASSERT(fix_count == car_gps_fix_count,
            "in-progress snapshot cannot increment fix count");
    TEST_NEAR(car_gps_x_m, expected_x, 0.0001f,
            "in-progress publication cannot tear published x");
    TEST_NEAR(car_gps_y_m, expected_y, 0.0001f,
            "in-progress publication cannot tear published y");

    gps_publish_sequence = 4;
    car_gps_measurement_to_reference(gps_pending_x_m, gps_pending_y_m,
            gps_pending_speed_mps, gps_pending_yaw_deg, car_yaw_deg,
            &expected_x, &expected_y);
    car_ekf_apply_pending_gps();
    TEST_ASSERT(0 == gps_update_pending, "completed publication is consumed on the next ISR pass");
    TEST_ASSERT(fix_count + 1U == car_gps_fix_count,
            "completed second snapshot increments fix count exactly once");
    TEST_NEAR(car_gps_x_m, expected_x, 0.0001f, "second coherent snapshot x");
    TEST_NEAR(car_gps_y_m, expected_y, 0.0001f, "second coherent snapshot y");
    TEST_NEAR(car_gps_speed_mps, 0.2f, 0.0001f, "second coherent snapshot speed");
    TEST_NEAR(car_gps_direction_deg, 90.0f, 0.0001f, "second coherent snapshot yaw");
}
#if defined(CAR_NEAREST_MAX_REGRESS_PT)
#define TEST_NEAREST_MAX_ROLLBACK_PT   ((uint32)CAR_NEAREST_MAX_REGRESS_PT)
#else
#define TEST_NEAREST_MAX_ROLLBACK_PT   (3U)
#endif

void gpio_init(gpio_pin_enum pin, int mode, gpio_level_enum level, int config)
{ (void)pin; (void)mode; (void)level; (void)config; }
void gpio_set_level(gpio_pin_enum pin, gpio_level_enum level)
{ (void)pin; (void)level; }
gpio_level_enum gpio_get_level(gpio_pin_enum pin)
{
    if(pin == CAR_KEY_A_PIN) return test_key_a;
    if(pin == CAR_KEY_B_PIN) return test_key_b;
    return GPIO_LOW;
}
void pwm_init(pwm_channel_enum channel, uint32 frequency, uint32 duty)
{
    (void)frequency;
    if(channel == CAR_LEFT_MOTOR_PWM_CH) test_left_pwm_duty = duty;
    if(channel == CAR_RIGHT_MOTOR_PWM_CH) test_right_pwm_duty = duty;
    if(channel == CAR_STEER_MOTOR_PWM_CH) test_steer_pwm_duty = duty;
}
void pwm_set_duty(pwm_channel_enum channel, uint32 duty)
{
    if(channel == CAR_LEFT_MOTOR_PWM_CH) test_left_pwm_duty = duty;
    if(channel == CAR_RIGHT_MOTOR_PWM_CH) test_right_pwm_duty = duty;
    if(channel == CAR_STEER_MOTOR_PWM_CH) test_steer_pwm_duty = duty;
}
void encoder_dir_init(encoder_index_enum index, int ch1, int ch2)
{ (void)index; (void)ch1; (void)ch2; }
void encoder_quad_init(encoder_index_enum index, int ch1, int ch2)
{ (void)index; (void)ch1; (void)ch2; }
int16 encoder_get_count(encoder_index_enum index)
{ int16 value = test_encoder_delta; (void)index; test_encoder_delta = 0; return value; }
void encoder_clear_count(encoder_index_enum index) { (void)index; }
int mpu6050_init(void) { return 0; }
void mpu6050_get_gyro(void) { }
void mpu6050_get_acc(void) { }
float mpu6050_gyro_transition(int16 raw) { return (float)raw; }
float mpu6050_acc_transition(int16 raw) { return (float)raw; }
void gnss_init(int device) { (void)device; }
uint8 gnss_data_parse(void) { return 1; }
void system_delay_ms(uint32 ms) { (void)ms; }
void ips200_set_dir(int dir) { (void)dir; }
void ips200_set_color(uint16 foreground, uint16 background) { (void)foreground; (void)background; }
void ips200_init(int type) { (void)type; }
void ips200_clear(void) { }
void ips200_show_string(uint16 x, uint16 y, const char *text) { (void)x; (void)y; (void)text; }
void ips200_show_uint(uint16 x, uint16 y, uint32 value, uint8 width) { (void)x; (void)y; (void)value; (void)width; }
void ips200_show_int(uint16 x, uint16 y, int32 value, uint8 width) { (void)x; (void)y; (void)value; (void)width; }
void ips200_show_float(uint16 x, uint16 y, float value, uint8 int_width, uint8 frac_width)
{ (void)x; (void)y; (void)value; (void)int_width; (void)frac_width; }
void ips200_draw_point(uint16 x, uint16 y, uint16 color) { (void)x; (void)y; (void)color; }
void ips200_draw_line(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color)
{ (void)x1; (void)y1; (void)x2; (void)y2; (void)color; }

static void reset_navigation_globals(void)
{
    memset(car_path, 0, sizeof(car_path));
    memset(ekf_state, 0, sizeof(ekf_state));
    memset(ekf_p, 0, sizeof(ekf_p));
    memset(&gnss, 0, sizeof(gnss));
    car_state = CAR_STATE_WAIT_START;
    car_replay_mode = CAR_REPLAY_NONE;
    car_path_count = 0;
    car_path_prepared = 0;
    car_path_has_gps = 0;
    car_local_datum_ready = 0;
    car_local_datum_sample_count = 0;
    car_local_datum_reject_count = 0;
    car_gps_valid = 0;
    car_gps_satellites = 0;
    car_gps_fix_count = 0;
    car_gps_x_m = 0.0f;
    car_gps_y_m = 0.0f;
    car_gps_speed_mps = 0.0f;
    car_gps_direction_deg = 0.0f;
    car_mpu6050_ok = 1;
    gps_valid_fix_streak = 0;
    gps_last_update_ms = 0;
    nav_uptime_ms = 0;
    gps_seen_gga_count = 0;
    gps_gga_seen = 0;
    gps_last_gga_ms = 0;
    gps_publish_sequence = 0;
    gps_last_raw_x = 0.0f;
    gps_last_raw_y = 0.0f;
    gps_last_raw_ms = 0;
    gps_raw_initialized = 0;
    gps_heading_anchor_initialized = 0;
    gps_heading_anchor_x = 0.0f;
    gps_heading_anchor_y = 0.0f;
    gps_stop_last_fix = 0;
    gps_priority_stop_count = 0;
    gps_update_pending = 0;
    gps_pending_x_m = 0.0f;
    gps_pending_y_m = 0.0f;
    gps_pending_speed_mps = 0.0f;
    gps_pending_yaw_deg = 0.0f;
    gps_pending_heading_yaw_deg = 0.0f;
    gps_pending_heading_valid = 0;
    ekf_pos_reject_count = 0;
    ekf_yaw_observed = 0;
    car_pose_x_m = 0.0f;
    car_pose_y_m = 0.0f;
    car_yaw_deg = 0.0f;
    car_speed_mps = 0.0f;
    car_steer_encoder_count = 0;
    car_nearest_index = 0;
    car_target_index = 0;
    car_cross_track_error_m = 0.0f;
    car_heading_error_deg = 0.0f;
    car_track_correction_active = 0;
    car_elapsed_ms = 0;
    car_path_duration_ms = 0;
    car_error_code = CAR_ERROR_NONE;
    car_key1_level = CAR_KEY_RELEASE_LEVEL;
    car_key2_level = CAR_KEY_RELEASE_LEVEL;
    rear_pwm_ramped = 0;
    imu_still_count = 0;
    steer_stall_count = 0;
    steer_wrong_direction_count = 0;
    guide_progress_anchor_s_m = 0.0f;
    guide_progress_last_ms = 0;
    finish_endpoint_fix_count = 0;
    finish_last_fix = 0;
    key_last_a_level = CAR_KEY_RELEASE_LEVEL;
    key_last_b_level = CAR_KEY_RELEASE_LEVEL;
    key_a_cooldown = 0;
    key_b_cooldown = 0;
    ready_pending_key = 0;
    ready_pending_count = 0;
    ready_chord_count = 0;
    ready_wait_release = 0;
    test_key_a = CAR_KEY_RELEASE_LEVEL;
    test_key_b = CAR_KEY_RELEASE_LEVEL;
    test_encoder_delta = 0;
    test_left_pwm_duty = 0;
    test_right_pwm_duty = 0;
    test_steer_pwm_duty = 0;
    gnss_flag = 0;
    gnss_rmc_flag = 0;
    gnss_gga_flag = 0;
    gnss_rmc_count = 0;
    gnss_gga_count = 0;
    car_pid_clear(&track_pid);
    car_pid_clear(&steer_pos_pid);
#if !CAR_LOCAL_DATUM_FIXED_ENABLE
    car_local_datum_candidate_reset();
#endif
}

static void datum_offset(double latitude_deg, double longitude_deg,
        double east_m, double north_m, double *latitude_out, double *longitude_out)
{
    double east_m_per_rad;
    double north_m_per_rad;

    car_wgs84_local_scales(latitude_deg, &east_m_per_rad, &north_m_per_rad);
    *latitude_out = latitude_deg + north_m / north_m_per_rad / (double)CAR_DEG_TO_RAD;
    *longitude_out = longitude_deg + east_m / east_m_per_rad / (double)CAR_DEG_TO_RAD;
}

static void assert_path_is_finite(const char *label)
{
    uint32 i;

    TEST_ASSERT(car_path_count <= CAR_PATH_MAX_POINTS, "prepared path count stays within storage");
    for(i = 0; i < car_path_count; i++)
    {
        if(!(isfinite(car_path[i].x) && isfinite(car_path[i].y)
                && isfinite(car_path[i].s_m) && isfinite(car_path[i].yaw_deg)
                && isfinite(car_path[i].yaw_rate_dps) && isfinite(car_path[i].speed_mps)
                && isfinite(car_path[i].steer_count) && isfinite(car_path[i].curvature)))
        {
            fprintf(stderr, "FAIL: %s has non-finite value at index %u (line %d)\n",
                    label, (unsigned)i, __LINE__);
            test_failures++;
            return;
        }
    }
}

static void load_straight_path(uint32 count, float spacing_m)
{
    uint32 i;

    car_path_count = count;
    for(i = 0; i < count; i++)
    {
        car_path[i].x = (float)i * spacing_m;
        car_path[i].y = 0.0f;
        car_path[i].s_m = (float)i * spacing_m;
        car_path[i].yaw_deg = 0.0f;
        car_path[i].speed_mps = CAR_LOW_SPEED_MPS;
        car_path[i].steer_count = 0.0f;
        car_path[i].gps_valid = 1;
    }
    car_path_prepared = 1;
    car_path_has_gps = 1;
}

static void control_tick_with_fresh_gps(void)
{
    gps_last_update_ms = nav_uptime_ms;
    car_gps_valid = 1;
    if(gps_valid_fix_streak < CAR_GPS_READY_FIX_COUNT)
    {
        gps_valid_fix_streak = CAR_GPS_READY_FIX_COUNT;
    }
    car_nav_control_10ms();
}

static void release_buttons(void)
{
    test_key_a = CAR_KEY_RELEASE_LEVEL;
    test_key_b = CAR_KEY_RELEASE_LEVEL;
}

static void set_gnss_fix(double latitude_deg, double longitude_deg,
        float speed_mps, float course_deg)
{
    gnss.state = 1;
    gnss.satellite_used = 10;
    gnss.fix_quality = 1;
    gnss.hdop = 1.0f;
    gnss.latitude = fabs(latitude_deg);
    gnss.longitude = fabs(longitude_deg);
    gnss.ns = (latitude_deg < 0.0) ? 'S' : 'N';
    gnss.ew = (longitude_deg < 0.0) ? 'W' : 'E';
    gnss.speed = speed_mps * 3.6f;
    gnss.direction = course_deg;
}

static void poll_gga_frame(void)
{
    gnss_gga_count++;
    gnss_gga_flag = 1;
    gnss_flag = 1;
    car_nav_gnss_poll();
}

static void poll_rmc_frame(void)
{
    gnss_rmc_count++;
    gnss_rmc_flag = 1;
    gnss_flag = 1;
    car_nav_gnss_poll();
}

static void assert_all_pwm_stopped(const char *message)
{
    test_checks++;
    if(!((0U == test_left_pwm_duty) && (0U == test_right_pwm_duty)
            && (0U == test_steer_pwm_duty) && (0 == car_rear_pwm_output)
            && (0 == car_steer_pwm_output)))
    {
        fprintf(stderr, "FAIL: %s: hw=(%u,%u,%u) sw=(%d,%d) (line %d)\n",
                message, (unsigned)test_left_pwm_duty, (unsigned)test_right_pwm_duty,
                (unsigned)test_steer_pwm_duty, (int)car_rear_pwm_output,
                (int)car_steer_pwm_output, __LINE__);
        test_failures++;
    }
}

static void test_local_coordinates(void)
{
    float x;
    float y;
    double east_scale;
    double north_scale;
    const double lat0 = 31.84;
    const double lon0 = 117.25;

    reset_navigation_globals();
    car_local_datum_lock(lat0, lon0);
    east_scale = gps_origin_east_m_per_rad;
    north_scale = gps_origin_north_m_per_rad;
    car_gps_to_local(lat0 + 1.0 / north_scale / CAR_DEG_TO_RAD,
            lon0 + 1.0 / east_scale / CAR_DEG_TO_RAD, &x, &y);
    TEST_NEAR(x, 1.0f, 0.0001f, "WGS84 east metre");
    TEST_NEAR(y, 1.0f, 0.0001f, "WGS84 north metre");

    car_local_datum_lock(0.0, 179.999999);
    car_gps_to_local(0.0, -179.999999, &x, &y);
    TEST_NEAR(x, 0.222639f, 0.001f, "date-line unwrap");
    TEST_NEAR(y, 0.0f, 0.0001f, "date-line north");
}

#if !CAR_LOCAL_DATUM_FIXED_ENABLE
static void test_datum_exact_sample_count(void)
{
    uint32 i;
    float x;
    float y;
    const double lat0 = 31.840000;
    const double lon0 = 117.250000;

    reset_navigation_globals();
    for(i = 0; i + 1U < CAR_LOCAL_DATUM_SAMPLE_COUNT; i++)
    {
        TEST_ASSERT(0 == car_local_datum_update(lat0, lon0, 0.0f),
                "datum must not lock before the configured accepted-fix count");
        TEST_ASSERT(0 == car_local_datum_ready, "datum remains unready before frame 50");
        TEST_ASSERT(car_local_datum_sample_count == i + 1U,
                "datum accepted-fix counter increments exactly once per frame");
    }
    TEST_ASSERT(1 == car_local_datum_update(lat0, lon0, 0.0f),
            "datum locks on the configured accepted-fix count");
    TEST_ASSERT(1 == car_local_datum_ready, "datum ready after frame 50");
    TEST_ASSERT(CAR_LOCAL_DATUM_SAMPLE_COUNT == car_local_datum_sample_count,
            "datum sample count saturates at configured count");
    car_gps_to_local(lat0, lon0, &x, &y);
    TEST_NEAR(x, 0.0f, 0.001f, "stationary datum origin east");
    TEST_NEAR(y, 0.0f, 0.001f, "stationary datum origin north");
}

static void test_datum_jump_reseed_and_motion_reset(void)
{
    uint32 i;
    float x;
    float y;
    double shifted_lat;
    double shifted_lon;
    const double lat0 = 31.840000;
    const double lon0 = 117.250000;

    reset_navigation_globals();
    for(i = 0; i < 10U; i++)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    datum_offset(lat0, lon0, 8.0, 0.0, &shifted_lat, &shifted_lon);
    TEST_ASSERT(0 == car_local_datum_update(shifted_lat, shifted_lon, 0.0f),
            "isolated datum jump is rejected");
    TEST_ASSERT(10U == car_local_datum_sample_count,
            "isolated datum jump does not consume an accepted sample");
    TEST_ASSERT(1U == car_local_datum_reject_count,
            "isolated datum jump increments rejection telemetry");
    (void)car_local_datum_update(lat0, lon0, 0.0f);
    TEST_ASSERT(11U == car_local_datum_sample_count,
            "good fix after an isolated jump resumes original cluster");
    while(!car_local_datum_ready)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    car_gps_to_local(lat0, lon0, &x, &y);
    TEST_NEAR(x, 0.0f, 0.01f, "isolated jump cannot bias datum east");
    TEST_NEAR(y, 0.0f, 0.01f, "isolated jump cannot bias datum north");

    reset_navigation_globals();
    for(i = 0; i < 10U; i++)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    datum_offset(lat0, lon0, 6.0, -2.0, &shifted_lat, &shifted_lon);
    for(i = 0; i < CAR_LOCAL_DATUM_RESEED_REJECTS; i++)
    {
        (void)car_local_datum_update(shifted_lat, shifted_lon, 0.0f);
    }
    TEST_ASSERT(1U == car_local_datum_sample_count,
            "sustained shifted cluster reseeds on the configured rejection streak");
    TEST_ASSERT(CAR_LOCAL_DATUM_RESEED_REJECTS == car_local_datum_reject_count,
            "sustained shift rejection telemetry is preserved across reseed");
    while(!car_local_datum_ready)
    {
        (void)car_local_datum_update(shifted_lat, shifted_lon, 0.0f);
    }
    car_gps_to_local(shifted_lat, shifted_lon, &x, &y);
    TEST_NEAR(x, 0.0f, 0.01f, "reseeded datum locks shifted east origin");
    TEST_NEAR(y, 0.0f, 0.01f, "reseeded datum locks shifted north origin");

    reset_navigation_globals();
    for(i = 0; i < 10U; i++)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    TEST_ASSERT(0 == car_local_datum_update(lat0, lon0,
                    CAR_LOCAL_DATUM_MAX_SPEED_MPS + 0.01f),
            "reported motion prevents datum acquisition");
    TEST_ASSERT(0U == car_local_datum_sample_count,
            "reported motion resets the in-progress datum cluster");
    TEST_ASSERT(0 == car_local_datum_ready, "motion reset cannot publish a datum");
    for(i = 0; i < CAR_LOCAL_DATUM_SAMPLE_COUNT; i++)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    TEST_ASSERT(1 == car_local_datum_ready,
            "a fresh complete stationary cluster locks after motion reset");

    reset_navigation_globals();
    for(i = 0; i < CAR_LOCAL_DATUM_SAMPLE_COUNT; i++)
    {
        double drift_fraction = (double)i / (double)(CAR_LOCAL_DATUM_SAMPLE_COUNT - 1U);
        datum_offset(lat0, lon0, 1.4 * drift_fraction, 0.0, &shifted_lat, &shifted_lon);
        (void)car_local_datum_update(shifted_lat, shifted_lon, 0.0f);
    }
    TEST_ASSERT(0 == car_local_datum_ready,
            "slow positional drift cannot masquerade as a stationary datum cluster");
    TEST_ASSERT(car_local_datum_sample_count < CAR_LOCAL_DATUM_SAMPLE_COUNT,
            "centroid drift rejection restarts datum acquisition");
    for(i = 0; i < CAR_LOCAL_DATUM_SAMPLE_COUNT; i++)
    {
        (void)car_local_datum_update(lat0, lon0, 0.0f);
    }
    TEST_ASSERT(1 == car_local_datum_ready,
            "stable cluster locks after slow-drift rejection");
}
#endif

static void test_cte_and_return_signs(void)
{
    float cte;
    float correction;

    reset_navigation_globals();
    load_straight_path(20, 0.1f);
    car_pose_x_m = 0.5f;
    car_pose_y_m = 0.5f;
    car_replay_mode = CAR_REPLAY_A_FORWARD;
    cte = car_signed_cross_track(5);
    TEST_NEAR(cte, 0.5f, 0.0001f, "forward left-side CTE");
    TEST_ASSERT(car_track_pid_correction(cte) > 0.0f,
            "forward left-side error must command right-turn positive encoder");

    car_pid_clear(&track_pid);
    car_replay_mode = CAR_REPLAY_B_RETURN;
    cte = car_signed_cross_track(5);
    TEST_NEAR(cte, -0.5f, 0.0001f, "return travel-frame CTE sign");
    TEST_ASSERT(car_track_pid_correction(cte) < 0.0f,
            "return-frame right-side error must command left-turn negative encoder");

    car_pid_clear(&track_pid);
    car_track_correction_active = 0;
    car_heading_error_deg = 0.0f;
    correction = car_track_pid_correction(CAR_TRACK_TOLERANCE_M);
    TEST_NEAR(correction, 0.0f, 0.0001f,
            "fresh correction stays zero exactly on tolerance boundary");
    TEST_ASSERT(0 == car_track_correction_active,
            "fresh correction remains inactive on tolerance boundary");

    correction = car_track_pid_correction(CAR_TRACK_TOLERANCE_M + 0.08f);
    TEST_ASSERT(correction > 0.0f, "positive excursion activates positive correction");
    TEST_ASSERT(1 == car_track_correction_active, "outer correction latches active");
    (void)car_track_pid_correction(CAR_TRACK_TOLERANCE_M - 0.01f);
    TEST_ASSERT(1 == car_track_correction_active,
            "correction hysteresis does not chatter just inside entry threshold");
    correction = car_track_pid_correction(CAR_TRACK_TOLERANCE_M * 0.50f);
    TEST_NEAR(correction, 0.0f, 0.0001f,
            "correction releases after returning well inside corridor");
    TEST_ASSERT(0 == car_track_correction_active,
            "correction active flag clears at inner release threshold");

    car_pid_clear(&track_pid);
    correction = car_track_pid_correction(-CAR_TRACK_TOLERANCE_M - 0.08f);
    TEST_ASSERT(correction < 0.0f, "negative excursion activates negative correction");
    TEST_ASSERT(1 == car_track_correction_active,
            "negative correction uses the same hysteresis latch");
}

static void test_path_prepare_straight(void)
{
    reset_navigation_globals();
    car_path_count = 101;
    for(uint32 i = 0; i < car_path_count; i++)
    {
        car_path[i].x = (float)i * 0.02f;
        car_path[i].y = (i & 1U) ? 0.03f : -0.03f;
        car_path[i].steer_count = 0.0f;
        car_path[i].gps_valid = 1;
    }
    car_path_prepare();
    TEST_ASSERT(car_path_count >= CAR_START_MIN_POINTS, "straight path survives compaction");
    TEST_NEAR(car_path[0].x, 0.0f, 0.0001f, "path first x retained");
    TEST_NEAR(car_path[car_path_count - 1].x, 2.0f, 0.0001f, "path final x retained");
    assert_path_is_finite("alternating-noise straight path");
    for(uint32 i = 0; i < car_path_count; i++)
    {
        float heading_error = fabsf(car_wrap_deg(car_path[i].yaw_deg));
        if(i > 0U)
        {
            TEST_ASSERT(car_path[i].s_m > car_path[i - 1].s_m,
                    "path arc length strictly increases");
        }
        if(heading_error >= 15.0f)
        {
            fprintf(stderr, "straight heading index=%u yaw=%.3f x=%.3f y=%.3f count=%u\n",
                    (unsigned)i, car_path[i].yaw_deg, car_path[i].x, car_path[i].y,
                    (unsigned)car_path_count);
            TEST_ASSERT(0, "smoothed straight path heading remains near east");
        }
    }
}

static void test_path_prepare_boundaries_and_short_tail(void)
{
    uint32 i;

    reset_navigation_globals();
    car_path_prepare();
    TEST_ASSERT(0U == car_path_count, "empty path prepare is a no-op");

    car_path_count = 1;
    car_path[0].x = 2.0f;
    car_path[0].y = -1.0f;
    car_path[0].steer_count = 7.0f;
    car_path_prepare();
    TEST_ASSERT(1U == car_path_count, "single-point path prepare stays in bounds");
    TEST_NEAR(car_path[0].x, 2.0f, 0.0001f, "single-point x retained");

    reset_navigation_globals();
    load_straight_path(2, 0.1f);
    car_path_prepare();
    TEST_ASSERT(2U == car_path_count, "two-point path prepare retains both endpoints");
    assert_path_is_finite("two-point path");
    TEST_NEAR(car_path[0].yaw_deg, 0.0f, 0.001f, "two-point start yaw");
    TEST_NEAR(car_path[1].yaw_deg, 0.0f, 0.001f, "two-point end yaw");

    reset_navigation_globals();
    car_path_count = 12;
    for(i = 0; i + 1U < car_path_count; i++)
    {
        car_path[i].x = (float)i * 0.1f;
        car_path[i].y = 0.0f;
        car_path[i].steer_count = 0.0f;
        car_path[i].gps_valid = 1;
    }
    car_path[car_path_count - 1U].x = 1.02f;
    car_path[car_path_count - 1U].y = 0.03f;
    car_path[car_path_count - 1U].steer_count = 0.0f;
    car_path[car_path_count - 1U].gps_valid = 1;
    car_path_prepare();
    TEST_NEAR(car_path[car_path_count - 1U].x, 1.02f, 0.0001f,
            "short final segment preserves taught endpoint x");
    TEST_NEAR(car_path[car_path_count - 1U].y, 0.03f, 0.0001f,
            "short final segment preserves taught endpoint y");
    TEST_ASSERT(fabsf(car_wrap_deg(car_path[car_path_count - 1U].yaw_deg)) < 15.0f,
            "short noisy tail cannot rotate endpoint heading excessively");
    assert_path_is_finite("short-tail path");

    reset_navigation_globals();
    load_straight_path(CAR_PATH_MAX_POINTS, 0.02f);
    car_path_prepare();
    TEST_ASSERT(car_path_count >= CAR_START_MIN_POINTS,
            "maximum-capacity input survives bounded compaction");
    TEST_ASSERT(car_path_count <= CAR_PATH_MAX_POINTS,
            "maximum-capacity prepare never writes beyond array");
    assert_path_is_finite("maximum-capacity path");
    for(i = 1; i < car_path_count; i++)
    {
        TEST_ASSERT(car_path[i].s_m > car_path[i - 1U].s_m,
                "maximum-capacity prepared arc length is strictly increasing");
    }
}

static void test_path_prepare_true_spatial_resampling(void)
{
    uint32 i;
    float dx;
    float dy;
    float segment_m;

    reset_navigation_globals();
    car_path_has_gps = 1;
    car_path_count = 3;
    car_path[0].x = 0.0f;
    car_path[0].y = 0.0f;
    car_path[0].steer_count = 0.0f;
    car_path[1].x = 0.8f;
    car_path[1].y = 0.0f;
    car_path[1].steer_count = 20.0f;
    car_path[2].x = 1.6f;
    car_path[2].y = 0.0f;
    car_path[2].steer_count = 40.0f;

    car_path_prepare();
    TEST_ASSERT(car_path_count >= 16U,
            "sparse raw segments are interpolated instead of merely compacted");
    TEST_NEAR(car_path[car_path_count - 1U].x, 1.6f, 0.0001f,
            "true resampling preserves the taught endpoint");
    for(i = 1; i < car_path_count; i++)
    {
        dx = car_path[i].x - car_path[i - 1U].x;
        dy = car_path[i].y - car_path[i - 1U].y;
        segment_m = sqrtf(dx * dx + dy * dy);
        TEST_ASSERT(segment_m <= CAR_PATH_RESAMPLE_M * 1.36f,
                "resampled path has a bounded maximum spatial gap");
    }
}

static void test_path_prepare_rejects_nonfinite_points(void)
{
    uint32 i;

    reset_navigation_globals();
    load_straight_path(30, 0.1f);
    car_path[7].x = NAN;
    car_path[13].y = INFINITY;
    car_path[21].steer_count = -INFINITY;
    car_path_prepare();
    TEST_ASSERT(car_path_count >= CAR_START_MIN_POINTS,
            "isolated corrupt samples do not destroy an otherwise valid route");
    assert_path_is_finite("path with rejected corrupt samples");
    for(i = 1; i < car_path_count; i++)
    {
        TEST_ASSERT(car_path[i].s_m > car_path[i - 1U].s_m,
                "corrupt-sample filtering leaves strictly increasing arc length");
    }
}

static void test_nearest_progress_limits_and_rollback(void)
{
    uint32 nearest;

    reset_navigation_globals();
    load_straight_path(100, 0.1f);
    car_replay_mode = CAR_REPLAY_A_FORWARD;
    car_yaw_deg = 0.0f;
    car_nearest_index = 10;
    car_pose_x_m = 2.8f;
    car_pose_y_m = 0.0f;
    nearest = car_find_nearest_index();
    TEST_ASSERT(nearest > 10U, "forward nearest search can make progress");
    TEST_ASSERT(nearest <= 10U + TEST_NEAREST_MAX_PROGRESS_PT,
            "one noisy pose update cannot jump far ahead on the route");

    car_nearest_index = 20;
    car_pose_x_m = 1.8f;
    nearest = car_find_nearest_index();
    TEST_ASSERT(nearest < 20U,
            "forward replay allows a small rollback to recover from prior index overshoot");
    TEST_ASSERT(nearest + TEST_NEAREST_MAX_ROLLBACK_PT >= 20U,
            "forward rollback is bounded");

    car_replay_mode = CAR_REPLAY_B_RETURN;
    car_yaw_deg = 180.0f;
    car_nearest_index = 80;
    car_pose_x_m = 6.2f;
    nearest = car_find_nearest_index();
    TEST_ASSERT(nearest < 80U, "return nearest search can make reverse-index progress");
    TEST_ASSERT(nearest + TEST_NEAREST_MAX_PROGRESS_PT >= 80U,
            "one noisy return pose update cannot jump far ahead toward index zero");

    car_nearest_index = 70;
    car_pose_x_m = 7.2f;
    nearest = car_find_nearest_index();
    TEST_ASSERT(nearest > 70U,
            "return replay allows a small index increase to recover from overshoot");
    TEST_ASSERT(nearest <= 70U + TEST_NEAREST_MAX_ROLLBACK_PT,
            "return rollback is bounded");
}

static void test_gps_priority_uses_local_route_window(void)
{
    uint32 i;
    uint32 nearest;
    uint8 stop = 0;
    float distance_m = 0.0f;

    reset_navigation_globals();
    car_path_count = 110;
    for(i = 0; i < 50U; i++)
    {
        car_path[i].x = (float)i * 0.1f;
        car_path[i].y = 0.0f;
        car_path[i].yaw_deg = 0.0f;
        car_path[i].gps_valid = 1;
    }
    for(i = 50U; i < 60U; i++)
    {
        car_path[i].x = 4.9f;
        car_path[i].y = (float)(i - 49U) * 0.3f;
        car_path[i].yaw_deg = 90.0f;
        car_path[i].gps_valid = 1;
    }
    for(i = 60U; i < car_path_count; i++)
    {
        car_path[i].x = (float)(109U - i) * 0.1f;
        car_path[i].y = 3.0f;
        car_path[i].yaw_deg = 180.0f;
        car_path[i].gps_valid = 1;
    }
    car_path_prepared = 1;
    car_path_has_gps = 1;
    car_gps_valid = 1;
    car_replay_mode = CAR_REPLAY_A_FORWARD;
    car_nearest_index = 10;
    car_gps_x_m = 1.0f;
    car_gps_y_m = 3.0f;

    nearest = car_find_gps_nearest_index(&distance_m);
    TEST_ASSERT(nearest < 40U,
            "GPS safety search stays near current route progress at a parallel branch");
    TEST_ASSERT(distance_m > 2.5f,
            "GPS safety distance measures departure from current branch, not a later crossing");

    for(i = 1; i <= CAR_GPS_PRIORITY_STOP_COUNT; i++)
    {
        car_gps_fix_count = i;
        stop = car_gps_priority_should_stop();
        if(i < CAR_GPS_PRIORITY_STOP_COUNT)
        {
            TEST_ASSERT(0 == stop, "GPS corridor stop requires consecutive new fixes");
        }
    }
    TEST_ASSERT(1 == stop,
            "GPS corridor safety stops after sustained departure from current route branch");

    car_gps_x_m = 1.0f;
    car_gps_y_m = 0.2f;
    car_gps_fix_count++;
    TEST_ASSERT(0 == car_gps_priority_should_stop(),
            "one in-corridor new fix clears GPS priority stop streak");
}

static void prepare_ready_button_fixture(uint8 at_return_endpoint)
{
    float x_m;
    float yaw_deg;

    reset_navigation_globals();
    load_straight_path(20, 0.1f);
    car_local_datum_ready = 1;
    car_gps_valid = 1;
    gps_valid_fix_streak = CAR_GPS_READY_FIX_COUNT;
    x_m = at_return_endpoint ? car_path[car_path_count - 1U].x : car_path[0].x;
    yaw_deg = at_return_endpoint ? 180.0f : 0.0f;
    car_gps_x_m = x_m;
    car_gps_y_m = 0.0f;
    car_gps_speed_mps = 0.0f;
    car_gps_direction_deg = yaw_deg;
    car_reset_runtime_pose_at(x_m, 0.0f, yaw_deg, 0);
    car_state = CAR_STATE_READY;
}

static void test_button_state_machine(void)
{
    uint32 i;

    reset_navigation_globals();
    car_local_datum_ready = 1;
    car_gps_valid = 1;
    gps_valid_fix_streak = CAR_GPS_READY_FIX_COUNT;
    car_gps_x_m = 0.0f;
    car_gps_y_m = 0.0f;
    car_reset_runtime_pose_at(0.0f, 0.0f, 0.0f, 0);
    car_state = CAR_STATE_WAIT_START;
    test_key_b = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_WAIT_START == car_state,
            "B cannot start the first teaching run from power-on wait");

    reset_navigation_globals();
    car_local_datum_ready = 1;
    car_gps_valid = 1;
    gps_valid_fix_streak = CAR_GPS_READY_FIX_COUNT;
    car_reset_runtime_pose_at(0.0f, 0.0f, 0.0f, 0);
    car_state = CAR_STATE_WAIT_START;
    test_key_a = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_RECORDING == car_state,
            "A starts the first teaching run after datum and GPS readiness");

    prepare_ready_button_fixture(0);
    test_key_a = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_READY == car_state,
            "single A is delayed during the A+B chord window");
    release_buttons();
    for(i = 0; i < CAR_KEY_CHORD_WINDOW_COUNT; i++)
    {
        control_tick_with_fresh_gps();
    }
#if CAR_GUIDE_ARM_CONFIRM_ENABLE
    TEST_ASSERT(CAR_STATE_ARMED == car_state, "single A arms forward replay after chord window");
    TEST_ASSERT(CAR_REPLAY_A_FORWARD == car_replay_mode, "A selects forward replay");
    for(i = 0; i <= (CAR_GUIDE_ARM_MIN_MS / CAR_CONTROL_PERIOD_MS); i++)
    {
        control_tick_with_fresh_gps();
    }
    test_key_a = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_GUIDE == car_state,
            "second A starts an armed forward replay after minimum arm delay");
#else
    TEST_ASSERT(CAR_STATE_GUIDE == car_state, "single A starts forward replay");
#endif

    prepare_ready_button_fixture(1);
    test_key_b = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    release_buttons();
    for(i = 0; i < CAR_KEY_CHORD_WINDOW_COUNT; i++)
    {
        control_tick_with_fresh_gps();
    }
#if CAR_GUIDE_ARM_CONFIRM_ENABLE
    TEST_ASSERT(CAR_STATE_ARMED == car_state, "single B arms return replay after chord window");
    TEST_ASSERT(CAR_REPLAY_B_RETURN == car_replay_mode, "B selects return replay");
    for(i = 0; i <= (CAR_GUIDE_ARM_MIN_MS / CAR_CONTROL_PERIOD_MS); i++)
    {
        control_tick_with_fresh_gps();
    }
    test_key_b = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_GUIDE == car_state,
            "second B starts an armed return replay after minimum arm delay");
#else
    TEST_ASSERT(CAR_STATE_GUIDE == car_state, "single B starts return replay");
#endif

    prepare_ready_button_fixture(0);
    test_key_a = CAR_KEY_ACTIVE_LEVEL;
    test_key_b = CAR_KEY_ACTIVE_LEVEL;
    for(i = 0; i < CAR_KEY_CHORD_HOLD_COUNT; i++)
    {
        control_tick_with_fresh_gps();
    }
    TEST_ASSERT(CAR_STATE_WAIT_START == car_state,
            "holding A+B clears route and returns to wait state");
    TEST_ASSERT(0U == car_path_count, "holding A+B clears all taught path points");

    prepare_ready_button_fixture(0);
    test_key_a = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    release_buttons();
    control_tick_with_fresh_gps();
    test_key_b = CAR_KEY_ACTIVE_LEVEL;
    control_tick_with_fresh_gps();
    TEST_ASSERT(CAR_STATE_READY == car_state,
            "near-simultaneous A then B launches neither replay mode");
    TEST_ASSERT(CAR_REPLAY_NONE == car_replay_mode,
            "ambiguous A/B press leaves replay selection empty");
}

static void test_encoder_fault_watchdogs(void)
{
    uint32 i;
    uint8 ok;

    reset_navigation_globals();
    car_state = CAR_STATE_GUIDE;
    car_rear_pwm_output = CAR_LOW_SPEED_REAR_PWM;
    car_steer_pwm_output = CAR_STEER_MOVE_MIN_DUTY;
    pwm_set_duty(CAR_LEFT_MOTOR_PWM_CH, CAR_LOW_SPEED_REAR_PWM);
    pwm_set_duty(CAR_RIGHT_MOTOR_PWM_CH, CAR_LOW_SPEED_REAR_PWM);
    pwm_set_duty(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOVE_MIN_DUTY);
    test_encoder_delta = (int16)(CAR_STEER_ENCODER_MAX_DELTA_10MS + 1);
    ok = car_encoder_update();
    TEST_ASSERT(0 == ok, "implausible one-cycle encoder jump is rejected");
    TEST_ASSERT((CAR_STATE_ERROR == car_state) && (CAR_ERROR_ENCODER == car_error_code),
            "encoder jump enters latched encoder error state");
    assert_all_pwm_stopped("encoder jump immediately stops every motor PWM");

    reset_navigation_globals();
    car_state = CAR_STATE_GUIDE;
    car_steer_pwm_output = CAR_STEER_MOVE_MIN_DUTY;
    pwm_set_duty(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOVE_MIN_DUTY);
    for(i = 0; i < CAR_STEER_WRONG_DIR_COUNT; i++)
    {
        test_encoder_delta = -1;
        ok = car_encoder_update();
        if(i + 1U < CAR_STEER_WRONG_DIR_COUNT)
        {
            TEST_ASSERT(1 == ok, "wrong-direction watchdog requires a consecutive count");
        }
    }
    TEST_ASSERT(0 == ok, "persistent encoder motion opposite PWM direction is rejected");
    TEST_ASSERT((CAR_STATE_ERROR == car_state) && (CAR_ERROR_ENCODER == car_error_code),
            "wrong-direction encoder enters encoder error state");
    assert_all_pwm_stopped("wrong-direction encoder fault stops every motor PWM");

    reset_navigation_globals();
    car_state = CAR_STATE_GUIDE;
    car_rear_pwm_output = CAR_LOW_SPEED_REAR_PWM;
    car_steer_pwm_output = CAR_STEER_MOVE_MIN_DUTY;
    pwm_set_duty(CAR_LEFT_MOTOR_PWM_CH, CAR_LOW_SPEED_REAR_PWM);
    pwm_set_duty(CAR_RIGHT_MOTOR_PWM_CH, CAR_LOW_SPEED_REAR_PWM);
    pwm_set_duty(CAR_STEER_MOTOR_PWM_CH, CAR_STEER_MOVE_MIN_DUTY);
    ok = 1;
    for(i = 0; i < CAR_STEER_STALL_COUNT; i++)
    {
        test_encoder_delta = 0;
        ok = car_encoder_update();
        if(i + 1U < CAR_STEER_STALL_COUNT)
        {
            TEST_ASSERT(1 == ok, "steering stall watchdog waits for configured one-second count");
        }
    }
    TEST_ASSERT(0 == ok, "one second of commanded steering without encoder motion faults");
    TEST_ASSERT((CAR_STATE_ERROR == car_state) && (CAR_ERROR_ENCODER == car_error_code),
            "steering stall enters encoder error state");
    assert_all_pwm_stopped("steering stall fault stops every motor PWM");

    reset_navigation_globals();
    car_state = CAR_STATE_GUIDE;
    test_encoder_delta = 80;
    TEST_ASSERT(1 == car_encoder_update(), "large but plausible encoder increment is accepted");
    test_encoder_delta = 80;
    TEST_ASSERT(0 == car_encoder_update(), "cumulative encoder hard-limit violation is rejected");
    TEST_ASSERT(CAR_ERROR_ENCODER == car_error_code,
            "encoder hard-limit violation reports encoder error");
    assert_all_pwm_stopped("encoder hard-limit fault stops every motor PWM");
}

static void test_ekf_long_run_covariance(void)
{
    uint32 step;
    uint8 i;
    uint8 j;
    uint8 axis;
    float h[CAR_EKF_STATE_DIM];
    float scale;

    reset_navigation_globals();
    car_ekf_init_state(0.0f, 0.0f, 0.0f, CAR_LOW_SPEED_MPS);
    for(step = 0; step < 100000U; step++)
    {
        car_steer_encoder_count = ((step / 200U) & 1U) ? 5 : -5;
        car_ekf_predict(((step / 100U) & 1U) ? 1.5f : -1.5f,
                ((step / 50U) & 1U) ? 0.02f : -0.02f);
        if(0U == (step % 10U))
        {
            for(i = 0; i < CAR_EKF_STATE_DIM; i++)
            {
                h[i] = 0.0f;
            }
            axis = (uint8)((step / 10U) % CAR_EKF_STATE_DIM);
            h[axis] = 1.0f;
            car_ekf_update_scalar(h, (step & 16U) ? 0.001f : -0.001f, 0.05f);
        }
        if(0U == (step % 1000U))
        {
            TEST_ASSERT(1 == car_ekf_numeric_ok(),
                    "EKF state and covariance stay finite during 100k-step run");
            for(i = 0; i < CAR_EKF_STATE_DIM; i++)
            {
                for(j = i + 1U; j < CAR_EKF_STATE_DIM; j++)
                {
                    scale = 1.0f + fmaxf(fabsf(ekf_p[i][j]), fabsf(ekf_p[j][i]));
                    TEST_ASSERT(fabsf(ekf_p[i][j] - ekf_p[j][i]) <= 0.00001f * scale,
                            "Joseph covariance remains symmetric during long run");
                }
            }
        }
    }
    TEST_ASSERT(1 == car_ekf_numeric_ok(), "EKF remains numerically valid after 100k steps");
    for(i = 0; i < CAR_EKF_STATE_DIM; i++)
    {
        TEST_ASSERT(ekf_p[i][i] >= CAR_EKF_MIN_COV,
                "EKF covariance diagonal remains positive after 100k steps");
        for(j = i + 1U; j < CAR_EKF_STATE_DIM; j++)
        {
            scale = 1.0f + fmaxf(fabsf(ekf_p[i][j]), fabsf(ekf_p[j][i]));
            TEST_ASSERT(fabsf(ekf_p[i][j] - ekf_p[j][i]) <= 0.00001f * scale,
                    "Joseph covariance is symmetric after 100k steps");
        }
    }
}

static void test_guide_precheck_and_no_progress_stop(void)
{
    uint32 i;
    float start_yaw_deg = 0.0f;

    prepare_ready_button_fixture(0);
    TEST_ASSERT(1 == car_guide_precheck(CAR_REPLAY_A_FORWARD, &start_yaw_deg),
            "valid A start pose passes distance and heading precheck");
    TEST_NEAR(start_yaw_deg, 0.0f, 0.001f, "valid A precheck returns forward route yaw");

    car_gps_x_m = CAR_GUIDE_START_MAX_DIST_M + 0.01f;
    car_gps_y_m = 0.0f;
    car_error_code = CAR_ERROR_NONE;
    TEST_ASSERT(0 == car_guide_precheck(CAR_REPLAY_A_FORWARD, &start_yaw_deg),
            "start outside configured distance is rejected");
    TEST_ASSERT(CAR_ERROR_START_TOO_FAR == car_error_code,
            "distance precheck reports start-too-far error");

    prepare_ready_button_fixture(0);
    car_yaw_deg = CAR_GUIDE_START_HEADING_MAX_DEG + 1.0f;
    car_error_code = CAR_ERROR_NONE;
    TEST_ASSERT(0 == car_guide_precheck(CAR_REPLAY_A_FORWARD, &start_yaw_deg),
            "start facing beyond heading gate is rejected");
    TEST_ASSERT(CAR_ERROR_START_HEADING == car_error_code,
            "heading precheck reports start-heading error");

    prepare_ready_button_fixture(1);
    TEST_ASSERT(1 == car_guide_precheck(CAR_REPLAY_B_RETURN, &start_yaw_deg),
            "valid B endpoint and reverse-facing yaw pass precheck");
    TEST_NEAR(car_wrap_deg(start_yaw_deg - 180.0f), 0.0f, 0.001f,
            "B precheck reverses route yaw by 180 degrees");

    prepare_ready_button_fixture(0);
    car_start_guide(CAR_REPLAY_A_FORWARD);
    TEST_ASSERT(CAR_STATE_GUIDE == car_state, "no-progress test enters guide state");
    for(i = 1; i <= (CAR_GUIDE_NO_PROGRESS_MS / CAR_CONTROL_PERIOD_MS); i++)
    {
        car_elapsed_ms = i * CAR_CONTROL_PERIOD_MS;
        car_gps_valid = 1;
        car_guide_update();
        if(i + 1U < (CAR_GUIDE_NO_PROGRESS_MS / CAR_CONTROL_PERIOD_MS))
        {
            TEST_ASSERT(CAR_STATE_GUIDE == car_state,
                    "no-progress watchdog does not fire before configured duration");
        }
    }
    TEST_ASSERT(CAR_STATE_ERROR == car_state,
            "four seconds without route-index progress enters error state");
    TEST_ASSERT(CAR_ERROR_NO_PROGRESS == car_error_code,
            "no-progress watchdog reports dedicated error code");
    assert_all_pwm_stopped("no-progress watchdog stops every motor PWM");
}

int main(void)
{
    test_gps_quality_jump_and_stale_watchdogs();
    test_pending_gps_snapshot_sequence();
    test_local_coordinates();
#if !CAR_LOCAL_DATUM_FIXED_ENABLE
    test_datum_exact_sample_count();
    test_datum_jump_reseed_and_motion_reset();
#endif
    test_cte_and_return_signs();
    test_path_prepare_straight();
    test_path_prepare_boundaries_and_short_tail();
    test_path_prepare_true_spatial_resampling();
    test_path_prepare_rejects_nonfinite_points();
    test_nearest_progress_limits_and_rollback();
    test_gps_priority_uses_local_route_window();
    test_button_state_machine();
    test_encoder_fault_watchdogs();
    test_ekf_long_run_covariance();
    test_guide_precheck_and_no_progress_stop();

    if(test_failures)
    {
        fprintf(stderr, "%d host navigation test(s) failed\n", test_failures);
        return 1;
    }
    printf("all host navigation tests passed (%u checks)\n", (unsigned)test_checks);
    return 0;
}
