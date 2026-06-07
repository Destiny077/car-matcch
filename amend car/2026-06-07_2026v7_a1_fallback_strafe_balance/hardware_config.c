/*
 * Central hardware and competition-tuning parameters.
 *
 * Values in this file are copied from the current sample program and rulebook
 * notes. Real-car measurements should replace them during hardware tuning.
 */

#ifndef _HARDWARE_CONFIG_
#define _HARDWARE_CONFIG_

/* Drive base */
#define ROBOT_WHEEL_DIAMETER_CM 6.0
#define ROBOT_TRACK_WIDTH_CM 26.5
#define ROBOT_ENCODER_TICKS_PER_REV 2048

/* Motor direction map: M1 front-left, M2 front-right, M3 rear-left, M4 rear-right. */
#define ROBOT_MOTOR_DIR_M1 1
#define ROBOT_MOTOR_DIR_M2 0
#define ROBOT_MOTOR_DIR_M3 1
#define ROBOT_MOTOR_DIR_M4 0

/* Motor closed-loop constants used by SetMotorConstSpeedValue. */
#define ROBOT_MOTOR_PID_P 1.5
#define ROBOT_MOTOR_PID_I 1.2
#define ROBOT_MOTOR_PID_D 0.001

/* AI camera */
#define ROBOT_AI_CAM_PORT 7
#define ROBOT_AI_CAM_MODEL "zm_hxkk_26"
#define ROBOT_TAG_LOST_CRAWL_SPEED 0

/* Conservative chassis profile for route-drift testing. */
#define ROBOT_ROUTE_MAX_SPEED 18.0
#define ROBOT_ROUTE_TURN_SPEED_LIMIT 22.0
#define ROBOT_ROUTE_SETTLE_SEC 0.25
#define ROBOT_ROUTE_SEGMENT_CM 12.0
#define ROBOT_ROUTE_ENCODER_BALANCE_GAIN 0.06
#define ROBOT_ROUTE_ENCODER_BALANCE_LIMIT 7
#define ROBOT_MOVE_CONTROL_PERIOD_SEC 0.01
#define ROBOT_ROUTE_RAMP_CM 6.0
#define ROBOT_ROUTE_RAMP_MIN_SPEED 10
#define ROBOT_ROUTE_RETRY_COUNT 1

/* AprilTag alignment profile. */
#define ROBOT_TAG_DIST_THRESHOLD 35.0
#define ROBOT_TAG_SPEED_HIGH 10.0
#define ROBOT_TAG_SPEED_LOW 2.0
#define ROBOT_TAG_VW_LIMIT 8.0
#define ROBOT_TAG_X_TOLERANCE 12.0
#define ROBOT_TAG_Y_TOLERANCE 12.0
#define ROBOT_TAG_A_TOLERANCE 6.0
#define ROBOT_A1_TAG_ID 1
#define ROBOT_A1_SEEN_CONFIRM_LOOPS 6
#define ROBOT_A1_TAG_WARMUP_SEC 0.15
#define ROBOT_TAG_RETRY_COUNT 2
#define ROBOT_TAG_NAV_CONFIRM_LOOPS 2
#define ROBOT_PID_INTEGRAL_LIMIT 80.0
#define ROBOT_PID_OUTPUT_LIMIT 8.0
#define ROBOT_PID_KP_MAX 5.0
#define ROBOT_PID_KI_MAX 2.0
#define ROBOT_PID_KD_MAX 2.0
#define ROBOT_VISION_SAMPLE_MIN_X 5.0
#define ROBOT_VISION_SAMPLE_MAX_X 315.0
#define ROBOT_VISION_SAMPLE_MIN_Y 5.0
#define ROBOT_VISION_SAMPLE_MAX_Y 235.0

/* Stored PID parameter addresses. Values are stored scaled by 100. */
#define ROBOT_PIDW_KP_ADDR 9
#define ROBOT_PIDW_KI_ADDR 10
#define ROBOT_PIDW_KD_ADDR 11
#define ROBOT_PID_DATA_SCALE 100.0

/* Motion safety timeouts. */
#define ROBOT_MOVE_TIMEOUT_BASE_MS 3000
#define ROBOT_MOVE_TIMEOUT_PER_CM_MS 220
#define ROBOT_TURN_TIMEOUT_BASE_MS 2000
#define ROBOT_TURN_TIMEOUT_PER_DEG_MS 70

/* Arm geometry and default positions. */
#define ROBOT_ARM_L0 100
#define ROBOT_ARM_L1 106
#define ROBOT_ARM_L2 85
#define ROBOT_ARM_L3 160
#define ROBOT_ARM_INIT_X 0
#define ROBOT_ARM_INIT_Y 100
#define ROBOT_ARM_INIT_Z 100

/* Servo defaults. */
#define ROBOT_SERVO_AUX_ID 4
#define ROBOT_SERVO_PAW_ID 5
#define ROBOT_SERVO_AUX_INIT 1500
#define ROBOT_SERVO_PAW_INIT 1200
#define ROBOT_PAW_OPEN 1100
#define ROBOT_PAW_GRIP 1700
#define ROBOT_SERVO_MIN_PWM 500
#define ROBOT_SERVO_MAX_PWM 2500
#define ROBOT_STAGE_DISPLAY_LINE 6

#endif
