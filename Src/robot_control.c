#include "robot_control.h"
#include <stdint.h>
#include <stdio.h>   /* snprintf: integer-only serial telemetry */

/* Peripheral handles created by CubeMX in main.c */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;    /* 1 MHz free-running timer: HC-SR04 timing */
extern TIM_HandleTypeDef htim3;    /* 50 Hz PWM CH1: SG90 on PA6 */
extern I2C_HandleTypeDef hi2c1;   /* Right TCS34725 (original role): PB6/PB7 */

/* Hardware (adjust if your actual connections differ):
   PA0: Maker Line AN (3.3 V powered sensor)
   PA2: USART2_TX -> HC-05 RXD; PA3: USART2_RX <- HC-05 TXD
   PA8: TIM1_CH1 -> TB6612 PWMA; PA9: TIM1_CH2 -> TB6612 PWMB
   PB12: AIN1; PB13: AIN2; PB14: BIN1; PB15: BIN2; PB5: STBY */
#define MOTOR_A_IS_LEFT        1U  /* 0 if motor A is the right motor */
#define MOTOR_A_REVERSED       0U  /* 1 if motor A direction is reversed */
#define MOTOR_B_REVERSED       0U  /* 1 if motor B direction is reversed */
#define SENSOR_REVERSED        0U  /* 1 if analog left/right is inverted */

/* Speeds in permille: 0 = 0%, 1000 = 100% PWM duty. */
#define MANUAL_SPEED           550   /* Forward: base 55% PWM */
#define MANUAL_REVERSE_SPEED   650   /* Backward only: base 65% PWM */
#define MANUAL_TURN_SPEED      500
/* Forward straight-line PWM calibration: the right wheel is boosted by
   25% relative to the 55% base command, as in the user's previous version.
   REVERSE deliberately uses EQUAL 65% PWM for both motors (no trim).
   If reverse still curves, calibrate the physical wheel speeds separately.
   No trim is applied to PID, L/R pivots, or automatic pickup approach.
   MOTOR_A_IS_LEFT must match the physical motor wiring. */
#define MANUAL_LEFT_TRIM_PCT   100U
#define MANUAL_RIGHT_TRIM_PCT  125U  /* Forward only */
#define FOLLOW_SPEED          470
#define SEARCH_SPEED          330
#define MAX_PID_CORRECTION     520.0f

/* ADC regions: see Cytron MAKER-LINE datasheet Table 4.
   12-bit ADC values for a sensor powered at approx 3.3 V. */
#define ADC_CENTER            2048
#define ADC_LOST_UNKNOWN_MAX   204   /* < 0.05 * Vcc */
#define ADC_LOST_LEFT_MAX      409   /* < 0.10 * Vcc */
#define ADC_LOST_RIGHT_MIN    3686   /* > 0.90 * Vcc */
#define ADC_CROSS_MIN         3891   /* > 0.95 * Vcc */
#define ADC_POSITION_SCALE  1638.0f /* 0.4 * 4095 */

#define FOLLOW_PERIOD_MS        5U
#define GAP_STRAIGHT_MS       110U
/* A centred-line loss is more likely a dash than a corner. */
#define DASH_BRIDGE_MS        300U
#define CORNER_SIDE_ADC       2750 /* normal orientation; left mirror 1346 */
#define CORNER_HINT_MS        350U
#define CORNER_CENTER_STABLE_MS 25U
#define CORNER_ALIGN_SPEED    270
#define OUTER_DASH_MAX_MS     300U /* max blind bridge, then right search on known map */
#define OUTER_GAP_SPEED       350  /* reduced forward speed while line is lost */
#define OUTER_RIGHT_EDGE_ADC  2900 /* strong right-side position, normal orientation */
#define OUTER_RIGHT_CONFIRM_MS  20U
#define OUTER_RIGHT_LOSS_MS     35U
#define OUTER_CORNER_REARM_MS  150U
#define BOOT_COMMAND_GUARD_MS 2500U
/* 0: preserve working H standalone mode; 1: H launches full mission. */
#define H_FULL_MISSION        1U
/* OMFAER Carino Bluetooth app touch protocol:
   F/B = start linear movement; releasing sends S (stop).
   L/R = start pivot; releasing sends C (circle stop).
   Motor stays commanded until explicit S/C release; this is NOT a heartbeat.
   WARNING: a disconnected HC-05 or lost S/C leaves the last motor command
   active. Test raised off the floor and provide an accessible power cutoff.
   There is no reliable software-only link-loss stop using this protocol. */
#define PID_INTEGRAL_LIMIT    1.50f /* Integral of normalized error in seconds */
#define D_FILTER_ALPHA        0.25f /* Low-pass filter (0..1) on derivative */

/* ---- Normal-map mission (autonomous mode). All values are starting points:
   tune them on the real map using Live Expressions (robot_line_adc etc.). ---- */
#define MISSION_OFF               0U
#define MISSION_READ_COLOUR       1U
#define MISSION_COUNT_JUNCTIONS   2U
#define MISSION_PASS_JUNCTION     3U
#define MISSION_TURN_FORWARD      4U
#define MISSION_TURN_LEFT         5U
#define MISSION_FOLLOW_BRANCH     6U
#define MISSION_WAIT_BALL         7U
#define MISSION_PICKUP_APPROACH   8U
#define MISSION_PICKUP_CLOSE      9U
/* Return-to-unload route on the supplied map (append IDs for compatibility). */
#define MISSION_OUTER_ROUTE      10U
#define MISSION_OUTER_RIGHT      11U
#define MISSION_RETURN_BACKOFF   12U
#define MISSION_TURN_AROUND      13U
#define MISSION_RETURN_BRANCH    14U
#define MISSION_RETURN_JOIN      15U
#define MISSION_RETURN_LEFT      16U
#define MISSION_RETURN_MAIN      17U
#define MISSION_RETURN_PASS      18U
#define MISSION_UNLOAD_ENTER     19U
#define MISSION_UNLOAD_RELEASE   20U
/* Explicit staged full-black markers (not the branch junction counter). */
#define MISSION_CONFIRM_START    21U
#define MISSION_CONFIRM_UNLOAD   22U

/* Mission result: 0=not started, 1=running, 2=delivered, 3=aborted. */
#define MISSION_RESULT_IDLE      0U
#define MISSION_RESULT_RUNNING   1U
#define MISSION_RESULT_SUCCESS   2U
#define MISSION_RESULT_FAILED    3U
/* Failure code 0=none, 1=hardware, 2=colour, 3=outer corner/route,
   4=outbound branch/turn, 5=pickup, 6=turnaround, 7=return branch,
   8=return to main, 9=unload not found, 10=ADC, 11=total time. */
#define FAIL_HARDWARE  1U
#define FAIL_COLOUR    2U
#define FAIL_OUTER     3U
#define FAIL_BRANCH    4U
#define FAIL_PICKUP    5U
#define FAIL_UTURN     6U
#define FAIL_RETURN_BRANCH 7U
#define FAIL_RETURN_JOIN   8U
#define FAIL_UNLOAD    9U
#define FAIL_ADC      10U
#define FAIL_TIMEOUT  11U
#define FAIL_START    12U

#define COLOUR_CONFIRM_MS       150U  /* Same colour must be seen this long */
#define COLOUR_TIMEOUT_MS     20000U  /* Give up if no colour card is found */
/* A branch on the LEFT pulls the sensor reading low suddenly. Normal PID
   tracking stays above this value, a branch drops below it. */
#define JUNCTION_LEFT_ADC_MAX   950   /* about 0.23 * Vcc */
#define JUNCTION_CONFIRM_MS      15U  /* Reading must stay low this long */
#define JUNCTION_REARM_MS        35U  /* Continuous centred line after skipped entrance */
#define JUNCTION_PASS_TIMEOUT_MS 900U /* Stop rather than blindly pass multiple entrances */
#define OUTBOUND_BRANCH_COUNT     3U
#define OUTBOUND_REQUIRED_RIGHTS  2U
/* 1: full-black is also a junction on the main collection straight.
   Set 0 only if all real entrances give a left-position reading. */
#define OUTBOUND_ACCEPT_CROSS     1U
#define TURN_FORWARD_MS         100U  /* Roll forward so the wheels reach the branch */
#define TURN_LEFT_MIN_MS        250U  /* Pivot at least this long before looking */
#define TURN_LEFT_TIMEOUT_MS   1500U
#define TURN_CENTER_ADC_MIN    1500   /* Line is roughly centred again */
#define TURN_CENTER_ADC_MAX    2600
#define CROSS_CONFIRM_MS         20U  /* Bar (all black) must last this long */
#define MARKER_CLEAR_MS          90U  /* Must leave black before counting again */
#define START_MARKER_TIMEOUT_MS 3000U  /* START is under sensor on pressing A */
#define UNLOAD_MARKER_TIMEOUT_MS 400U  /* Confirm full-black unload, else fail */
#define BRANCH_TIMEOUT_MS      8000U

/* Map: START -> upper straight -> two RIGHT 90-degree corners ->
   lower straight, where ball branches occur in order 1, 2, 3.
   Use a strong right-side position before loss, or the right-loss code.
   Short unknown gaps are bridged; a longer gap starts a bounded right
   search on this known two-right-corner map. Calibrate dash duration. */
#define OUTER_CORNER_LOST_MS       80U
#define OUTER_CORNER_MIN_TURN_MS 100U
#define OUTER_CORNER_TIMEOUT_MS  1700U
#define OUTER_ROUTE_TIMEOUT_MS  20000U
#define MISSION_TOTAL_TIMEOUT_MS 90000U

/* Map: target branch -> ball; after gripping, optionally reverse, U-turn,
   follow branch toward main line, turn LEFT, pass remaining branches,
   enter the 20x20 cm UNLOAD box and open gripper. No encoders exist. */
#define RETURN_BACKOFF_MS           0U  /* Direct U-turn; restore 180U if clearance is needed */
#define RETURN_BACKOFF_SPEED      400
#define UTURN_MIN_MS              750U
#define UTURN_TIMEOUT_MS         2300U
#define UTURN_SPEED              400
#define RETURN_BRANCH_BLIND_MS    600U
#define RETURN_BRANCH_TIMEOUT_MS 7000U
#define RETURN_T_CONFIRM_MS       30U
#define RETURN_JOIN_FORWARD_MS    100U
#define RETURN_LEFT_MIN_MS        230U
#define RETURN_LEFT_TIMEOUT_MS   1700U
#define RETURN_MAIN_BLIND_MS      400U
#define RETURN_MAIN_TIMEOUT_MS  16000U
#define RETURN_REARM_MS           160U
#define RETURN_PASS_MS            190U
#define RETURN_UNLOAD_ENTRY_MS    220U
#define RETURN_ENTRY_SPEED        320
#define RETURN_UNLOAD_HOLD_MS     850U

/* Automatic pickup at the selected branch, after the full-black endpoint.
   Measured on this robot: sensor height 9 cm, floor echo 52.5 cm,
   ping-pong ball echo 13.5 cm, physical gripper-to-ball gap 13-15 cm.
   The 10..<18 cm window rejects the measured floor echo, but cannot
   identify a ball uniquely. Require three distinct fresh echoes and
   at least 180 ms of valid range before moving.
   Initial timed travel: 130 mm (lower end of measured gripper gap).
   The 120 mm/s wheel speed is UNVERIFIED; calibrate on the track with
   an empty gripper and no ball. Adjust travel to the actual capture zone.
   The tilted HC-SR04 echo is NOT odometry and may vanish during approach. */
#define PICKUP_DETECT_MIN_CM       10.0f
#define PICKUP_DETECT_MAX_CM       18.0f /* EXCLUSIVE upper bound */
#define PICKUP_CONFIRM_MS        180U
#define PICKUP_MIN_ECHO_SAMPLES    3U
#define PICKUP_ECHO_STALE_MS     250U
#define PICKUP_WAIT_TIMEOUT_MS  5000U
#define PICKUP_APPROACH_SPEED    450    /* 45% PWM, both motors equal */
#define PICKUP_TRAVEL_TARGET_MM  130U    /* Initial trial travel: 13 cm */
#define PICKUP_CALIBRATED_MM_PER_S 120U  /* INITIAL GUESS at 45% PWM; CALIBRATE */
#define PICKUP_APPROACH_MS \
    ((PICKUP_TRAVEL_TARGET_MM * 1000U + PICKUP_CALIBRATED_MM_PER_S / 2U) / \
     PICKUP_CALIBRATED_MM_PER_S)
#define PICKUP_GRIP_HOLD_MS      600U

/* Obstacle stop (HC-SR04). Set OBSTACLE_STOP_ENABLE to 0 to disable. */
#define OBSTACLE_STOP_ENABLE      0U /* Map mission uses HC-SR04 only at PICKUP.
    Previously the 10 cm early-navigation stop could immobilize the robot
    at START because of floor echoes. Disabling it removes obstacle avoidance:
    run only on a clear, supervised track. */
#define OBSTACLE_STOP_CM       10.0f
#define OBSTACLE_CLEAR_CM      15.0f

/* SG90: adjust these after observing the physical gripper. Pulse widths do
   NOT uniquely determine angle without mounting/orientation calibration. */
#define SERVO_SAFE_MIN_US       1000U
#define SERVO_SAFE_MAX_US       2000U
#define SERVO_CENTER_US         1500U
/* Physical gripper startup correction (2026-11-27):
   The gripper was observed to move CLOSED when the previous OPEN command
   (1060 us) became active.  Use the opposite servo position for OPEN.
   Nominal Servo.h angles for these pulses are approximately 110/50 deg.
   Confirm these endpoints do not mechanically strain the gripper. */
#define GRIPPER_OPEN_US         1678U
#define GRIPPER_CLOSE_US        1060U

/* HC-SR04: TIM2 must tick every 1us and roll over at 65535. */
#define ULTRASONIC_PERIOD_MS     100U
#define ULTRASONIC_TIMEOUT_US  30000U
#define ULTRASONIC_TRIGGER_US     12U

/* Only RIGHT TCS34725 on I2C1 (PB6 SCL, PB7 SDA) for route markers.
   The front/centre TCS34725 is removed. HC-SR04 handles pickup range. */
#define TCS_ADDR              (0x29U << 1) /* STM32 HAL expects shifted address */
#define TCS_CMD              0x80U
#define TCS_AUTO_INC         0x20U
#define TCS_ENABLE           0x00U
#define TCS_ATIME            0x01U
#define TCS_CONTROL          0x0FU
#define TCS_STATUS           0x13U
#define TCS_RGBC_START       0x14U
#define RGB_POLL_PERIOD_MS     60U
#define RGB_I2C_TIMEOUT_MS      5U
#define RGB_RETRY_PERIOD_MS   3000U  /* Retry unplugged I2C sensors */

/* COLOR CALIBRATION: starting points only; tune from live RGB readings.
   Blue is deliberately less strict than the earlier 35% dominance rule.
   The clear and saturation limits reject very dark / nearly neutral surfaces. */
#define COLOR_MIN_CLEAR         40U
#define COLOR_MIN_RGB_SUM       80U
#define COLOR_MIN_SAT_PCT       10U
#define BLUE_MIN_PERCENT        34U
#define BLUE_VS_RED_PCT        125U /* B >= 1.25 * R */
#define BLUE_VS_GREEN_PCT       92U /* B >= 0.92 * G */

/* Non-blocking UART2 terminal reporting. 9600 baud, 8-N-1.
   At 500ms intervals the whole line finishes well before the next report. */
#define TELEMETRY_PERIOD_MS    500U
#define TELEMETRY_BUFFER_SIZE  384U

/* Tune these by modifying the initial values or using Live Expressions. */
volatile float robot_kp = 310.0f;
volatile float robot_ki =  40.0f;
volatile float robot_kd =   0.425f;
volatile float robot_pid_error = 0.0f;
volatile float robot_pid_integral = 0.0f;
volatile float robot_pid_output = 0.0f;
volatile uint16_t robot_line_adc = 0U;

volatile RobotRgbReading_t robot_rgb_right = {0}; /* Right marker sensor, I2C1 */
volatile float robot_ultrasonic_cm = -1.0f;
volatile uint8_t robot_ultrasonic_valid = 0U;
/* Gripper starts OPEN; O toggles OPEN/CLOSED. */
volatile RobotGripperState_t robot_gripper_state = ROBOT_GRIPPER_OPEN;
/* Serial terminal telemetry is always enabled (500 ms period). */
static volatile uint8_t telemetry_tx_busy = 0U;
static uint8_t telemetry_tx_buffer[TELEMETRY_BUFFER_SIZE];
static uint32_t telemetry_last_ms = 0U;

/* All these tasks run in the main loop, preserving existing UART ISR logic. */
typedef enum {
    ULTRA_IDLE = 0,
    ULTRA_TRIGGER_HIGH,
    ULTRA_WAIT_RISE,
    ULTRA_WAIT_FALL
} UltraState;
static UltraState ultra_state = ULTRA_IDLE;
static uint16_t ultra_state_start_us = 0U;
static uint16_t ultra_rise_us = 0U;
static uint32_t ultra_last_ping_ms = 0U;
static uint32_t rgb_last_poll_ms = 0U;
static uint32_t rgb_last_retry_ms = 0U;
/* Updated on each valid HC-SR04 echo, independently of Robot_Task timing. */
static uint32_t ultrasonic_last_valid_ms = 0U;
/* Useful when diagnosing a missing HC-SR04 echo vs. a blocked mission. */
static uint32_t ultrasonic_good_echo_count = 0U;
static uint32_t ultrasonic_timeout_count = 0U;

/* These can be inspected in STM32CubeIDE Live Expressions.
   Bits: 0=TIM1/PWM, 1=ADC calibration, 2=TIM3/servo,
         3=TIM2/ultrasonic, 4=UART receive. */
volatile uint8_t robot_hw_fault_flags = 0U;


/* UART ring buffer: fast sequences of characters are not overwritten. */
#define RX_BUFFER_SIZE  16U
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0U;
static volatile uint8_t rx_tail = 0U;
static uint8_t rx_byte = 0U;
static uint32_t robot_boot_ms = 0U;

typedef enum {
    MODE_MANUAL = 0,
    MODE_LINE_FOLLOW = 1
} RobotMode;
static RobotMode current_mode = MODE_MANUAL;

/* Prevent duplicate H/O toggle messages arriving in one tap from immediately
   switching the state back. Configure BlueBee to send ONE character/tap. */
#define TOGGLE_GUARD_MS       250U
static uint8_t h_toggle_seen = 0U;
static uint8_t o_toggle_seen = 0U;
static uint32_t last_h_toggle_ms = 0U;
static uint32_t last_o_toggle_ms = 0U;

static uint8_t manual_moving = 0U;
/* OMFAER app sends one direction message on touch and S/C on release.
   CMD_RX counts direction presses; REL_RX counts release messages. */
volatile uint32_t robot_manual_command_count = 0U;
static uint32_t manual_release_command_count = 0U;
/* Debug UART command reception; monitor LAST and AGE in telemetry. */
static char last_manual_direction = '-';
static uint8_t line_seen_once = 0U;
static int8_t last_turn = 1;  /* -1 left, +1 right */
static uint32_t last_manual_ms = 0U;
static uint32_t last_follow_ms = 0U;
static uint32_t last_line_ms = 0U;
static int8_t corner_side = 0;
static uint32_t corner_side_ms = 0U;
static uint8_t turn_departed = 0U;
static uint8_t turn_center_candidate = 0U;
static uint32_t turn_center_ms = 0U;

/* Mission state (also visible in Live Expressions and in the telemetry). */
volatile uint8_t robot_mission_state = MISSION_OFF;
volatile uint8_t robot_target_ball = 0U;
volatile uint8_t robot_junction_count = 0U;
volatile uint8_t robot_outer_corner_count = 0U;
volatile uint8_t robot_return_junction_count = 0U;
/* 0=none; 1=START; 2=selected ball endpoint; 3=UNLOAD/END.
   Unlike J/RJ, this is deliberately counted only in mission-specific states. */
volatile uint8_t robot_full_black_count = 0U;
volatile uint8_t robot_mission_result = MISSION_RESULT_IDLE;
volatile uint8_t robot_mission_fault = 0U;
static uint32_t mission_state_ms = 0U;
static uint32_t mission_started_ms = 0U;
static uint32_t outer_route_started_ms = 0U;
static uint32_t outer_last_line_ms = 0U;
static uint32_t outer_second_corner_ms = 0U;
static uint8_t outer_line_seen = 0U;
static uint8_t outer_right_candidate = 0U;
static uint32_t outer_right_candidate_ms = 0U;
static uint32_t outer_last_corner_ms = 0U;
static uint8_t return_armed = 0U;
static uint8_t return_candidate = 0U;
static uint32_t return_candidate_ms = 0U;
static uint32_t return_last_cross_ms = 0U;
static uint32_t return_main_start_ms = 0U;
static RobotColor_t colour_candidate = ROBOT_COLOR_UNKNOWN;
static uint32_t colour_candidate_ms = 0U;
static uint8_t junction_armed = 0U;
static uint8_t junction_candidate = 0U;
static uint32_t junction_first_ms = 0U;
static uint8_t junction_clear_candidate = 0U;
static uint32_t junction_clear_ms = 0U;
static uint8_t cross_candidate = 0U;
/* Debounced full-black detector. One continuous bar counts only once.
   Separate J/RJ counters still locate the chosen branch on the map. */
static uint8_t marker_latched = 0U;
static uint8_t marker_candidate = 0U;
static uint8_t marker_clear_candidate = 0U;
static uint8_t marker_just_confirmed = 0U;
static uint32_t marker_candidate_ms = 0U;
static uint32_t marker_clear_ms = 0U;
static uint32_t cross_first_ms = 0U;
static uint8_t obstacle_blocked = 0U;
static uint8_t pickup_condition_seen = 0U;
static uint32_t pickup_condition_ms = 0U;
static uint32_t pickup_last_checked_echo_ms = 0U;
static uint8_t pickup_valid_echo_count = 0U;

static uint8_t pid_initialized = 0U;
static uint32_t pid_last_ms = 0U;
static float pid_previous_error = 0.0f;
static float pid_filtered_derivative = 0.0f;

static int32_t clamp_int(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void drive_motor(GPIO_TypeDef *port, uint16_t in1, uint16_t in2,
                        uint32_t channel, int32_t command, uint8_t reverse)
{
    int32_t speed = clamp_int(command, -1000, 1000);
    if (reverse) speed = -speed;

    if (speed > 0) {
        HAL_GPIO_WritePin(port, in1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port, in2, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(port, in1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, in2, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(port, in1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, in2, GPIO_PIN_RESET);
    }

    uint32_t magnitude = (uint32_t)((speed < 0) ? -speed : speed);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t compare = ((period + 1U) * magnitude) / 1000U;
    if (compare > period) compare = period;
    __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

static void set_motors(int32_t left, int32_t right)
{
    int32_t a = MOTOR_A_IS_LEFT ? left : right;
    int32_t b = MOTOR_A_IS_LEFT ? right : left;

    drive_motor(GPIOB, GPIO_PIN_12, GPIO_PIN_13,
                TIM_CHANNEL_1, a, MOTOR_A_REVERSED);
    drive_motor(GPIOB, GPIO_PIN_14, GPIO_PIN_15,
                TIM_CHANNEL_2, b, MOTOR_B_REVERSED);
}

static void stop_motors(void)
{
    set_motors(0, 0);
}

/* Independently trim the two wheels during MANUAL straight driving only.
   clamp_int prevents PWM from exceeding the allowed 0..1000 range. */
static int32_t manual_left_speed(int32_t base_speed)
{
    return clamp_int((base_speed * (int32_t)MANUAL_LEFT_TRIM_PCT) / 100,
                     0, 1000);
}

static int32_t manual_right_speed(int32_t base_speed)
{
    return clamp_int((base_speed * (int32_t)MANUAL_RIGHT_TRIM_PCT) / 100,
                     0, 1000);
}

static void reset_pid(void)
{
    pid_initialized = 0U;
    pid_previous_error = 0.0f;
    pid_filtered_derivative = 0.0f;
    robot_pid_error = 0.0f;
    robot_pid_integral = 0.0f;
    robot_pid_output = 0.0f;
}

static uint8_t read_line_adc(uint16_t *result)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0U;

    if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    *result = (uint16_t)HAL_ADC_GetValue(&hadc1);
    robot_line_adc = *result;
    (void)HAL_ADC_Stop(&hadc1);
    return 1U;
}

/* ================================================================
 * SG90 gripper: TIM3_CH1 on PA6, 1 MHz counter, ARR = 19999.
 * 1500 means a 1500us-high pulse in a 20ms PWM period (50 Hz).
 * ================================================================ */
void Robot_Servo_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < SERVO_SAFE_MIN_US) pulse_us = SERVO_SAFE_MIN_US;
    if (pulse_us > SERVO_SAFE_MAX_US) pulse_us = SERVO_SAFE_MAX_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

void Robot_Gripper_Center(void)
{
    Robot_Servo_SetPulseUs(SERVO_CENTER_US);
    robot_gripper_state = ROBOT_GRIPPER_CENTER;
}

void Robot_Gripper_Open(void)
{
    Robot_Servo_SetPulseUs(GRIPPER_OPEN_US);
    robot_gripper_state = ROBOT_GRIPPER_OPEN;
}

void Robot_Gripper_Close(void)
{
    Robot_Servo_SetPulseUs(GRIPPER_CLOSE_US);
    robot_gripper_state = ROBOT_GRIPPER_CLOSED;
}

/* First O press closes (startup is OPEN), next O opens, and so on. */
void Robot_Gripper_Toggle(void)
{
    if (robot_gripper_state == ROBOT_GRIPPER_OPEN) {
        Robot_Gripper_Close();
    } else {
        Robot_Gripper_Open();
    }
}

/* ================================================================
 * HC-SR04 on PB0/TRIG and PB1/ECHO. TIM2: 1us tick, ARR 65535.
 * No HAL_Delay, no wait-for-echo loop. Robot_Task must be called
 * continuously for reliable edge polling; I2C is deferred during ping.
 * ================================================================ */
static uint16_t micros16(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static uint16_t micro_elapsed(uint16_t start, uint16_t current)
{
    return (uint16_t)(current - start); /* Works across 16-bit wrap */
}

static void ultrasonic_timeout(void)
{
    ++ultrasonic_timeout_count;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    robot_ultrasonic_valid = 0U;
    robot_ultrasonic_cm = -1.0f;
    ultra_state = ULTRA_IDLE;
}

static void ultrasonic_task(uint32_t now_ms)
{
    uint16_t t = micros16();

    switch (ultra_state) {
    case ULTRA_IDLE:
        /* Stuck-high ECHO must not leave an old distance marked valid. */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET &&
            (uint32_t)(now_ms - ultra_last_ping_ms) >
                (2U * ULTRASONIC_PERIOD_MS)) {
            robot_ultrasonic_valid = 0U;
            robot_ultrasonic_cm = -1.0f;
        }
        if ((uint32_t)(now_ms - ultra_last_ping_ms) >= ULTRASONIC_PERIOD_MS &&
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
            ultra_last_ping_ms = now_ms;
            ultra_state_start_us = t;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            ultra_state = ULTRA_TRIGGER_HIGH;
        }
        break;

    case ULTRA_TRIGGER_HIGH:
        if (micro_elapsed(ultra_state_start_us, t) >= ULTRASONIC_TRIGGER_US) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            ultra_state_start_us = t;
            ultra_state = ULTRA_WAIT_RISE;
        }
        break;

    case ULTRA_WAIT_RISE:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
            ultra_rise_us = t;
            ultra_state = ULTRA_WAIT_FALL;
        } else if (micro_elapsed(ultra_state_start_us, t) > ULTRASONIC_TIMEOUT_US) {
            ultrasonic_timeout();
        }
        break;

    case ULTRA_WAIT_FALL:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
            uint16_t pulse_us = micro_elapsed(ultra_rise_us, t);
            if (pulse_us >= 116U && pulse_us <= 25000U) {
                robot_ultrasonic_cm = (float)pulse_us / 58.0f;
                robot_ultrasonic_valid = 1U;
                ultrasonic_last_valid_ms = now_ms;
                ++ultrasonic_good_echo_count;
            } else {
                robot_ultrasonic_cm = -1.0f;
                robot_ultrasonic_valid = 0U;
            }
            ultra_state = ULTRA_IDLE;
        } else if (micro_elapsed(ultra_rise_us, t) > ULTRASONIC_TIMEOUT_US) {
            ultrasonic_timeout();
        }
        break;

    default:
        ultrasonic_timeout();
        break;
    }
}

/* ================================================================
 * RIGHT TCS34725 on I2C1: original marker classifier is unchanged.
 * The front/centre colour sensor has been removed from this firmware.
 * ================================================================ */
static uint8_t tcs_write(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(bus, TCS_ADDR, (uint16_t)(TCS_CMD | reg),
        I2C_MEMADD_SIZE_8BIT, &value, 1U, RGB_I2C_TIMEOUT_MS) == HAL_OK;
}

static uint8_t tcs_read(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(bus, TCS_ADDR, (uint16_t)(TCS_CMD | reg),
        I2C_MEMADD_SIZE_8BIT, value, 1U, RGB_I2C_TIMEOUT_MS) == HAL_OK;
}

static uint8_t tcs_init(I2C_HandleTypeDef *bus)
{
    /* Missing optional sensor should not stop motor/PID operation. */
    if (HAL_I2C_IsDeviceReady(bus, TCS_ADDR, 2U, 5U) != HAL_OK) return 0U;

    /* ~50.4ms RGBC integration, 4x gain (starting point). */
    if (!tcs_write(bus, TCS_ENABLE, 0x00U) ||
        !tcs_write(bus, TCS_ATIME, 0xEBU) ||
        !tcs_write(bus, TCS_CONTROL, 0x01U) ||
        !tcs_write(bus, TCS_ENABLE, 0x01U)) return 0U;

    HAL_Delay(3U); /* Power-on oscillator warmup; only once in Robot_Init. */
    return tcs_write(bus, TCS_ENABLE, 0x03U); /* PON + AEN */
}

static RobotColor_t classify_color(uint16_t clear, uint16_t red,
                                   uint16_t green, uint16_t blue)
{
    uint32_t sum = (uint32_t)red + (uint32_t)green + (uint32_t)blue;
    if (clear < COLOR_MIN_CLEAR || sum < COLOR_MIN_RGB_SUM)
        return ROBOT_COLOR_UNKNOWN;

    uint16_t highest = red, lowest = red;
    if (green > highest) highest = green;
    if (blue > highest) highest = blue;
    if (green < lowest) lowest = green;
    if (blue < lowest) lowest = blue;

    /* Reject near-neutral white/grey and very weakly coloured surfaces. */
    if ((uint32_t)(highest - lowest) * 100U <
        sum * COLOR_MIN_SAT_PCT)
        return ROBOT_COLOR_UNKNOWN;

    /* Integer ratios avoid floating printf and brightness dependence.
       Use uint32_t throughout to avoid intermediate overflow. */
    if ((uint32_t)red * 100U >= sum * 39U &&
        (uint32_t)red * 100U >= (uint32_t)green * 125U &&
        (uint32_t)red * 100U >= (uint32_t)blue * 125U)
        return ROBOT_COLOR_RED;

    /* Blue surfaces sometimes have G close to or slightly above B. */
    if ((uint32_t)blue * 100U >= sum * BLUE_MIN_PERCENT &&
        (uint32_t)blue * 100U >= (uint32_t)red * BLUE_VS_RED_PCT &&
        (uint32_t)blue * 100U >= (uint32_t)green * BLUE_VS_GREEN_PCT)
        return ROBOT_COLOR_BLUE;

    if ((uint32_t)green * 100U >= sum * 38U &&
        (uint32_t)green * 100U >= (uint32_t)red * 115U &&
        (uint32_t)green * 100U >= (uint32_t)blue * 118U)
        return ROBOT_COLOR_GREEN;

    return ROBOT_COLOR_UNKNOWN;
}

static const char *color_name(RobotColor_t color)
{
    switch (color) {
    case ROBOT_COLOR_RED: return "RED";
    case ROBOT_COLOR_GREEN: return "GREEN";
    case ROBOT_COLOR_BLUE: return "BLUE";
    default: return "UNKNOWN";
    }
}

static void tcs_sample(I2C_HandleTypeDef *bus,
                       volatile RobotRgbReading_t *out)
{
    if (!out->connected) return;

    uint8_t status = 0U;
    if (!tcs_read(bus, TCS_STATUS, &status)) {
        out->valid = 0U;
        out->connected = 0U; /* Retry after 3 seconds */
        out->color = ROBOT_COLOR_UNKNOWN;
        return;
    }
    if (!(status & 0x01U)) return; /* New RGBC sample not ready */

    uint8_t raw[8];
    if (HAL_I2C_Mem_Read(bus, TCS_ADDR,
            (uint16_t)(TCS_CMD | TCS_AUTO_INC | TCS_RGBC_START),
            I2C_MEMADD_SIZE_8BIT, raw, 8U, RGB_I2C_TIMEOUT_MS) != HAL_OK) {
        out->valid = 0U;
        out->connected = 0U;
        out->color = ROBOT_COLOR_UNKNOWN;
        return;
    }
    uint16_t clear = (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    uint16_t red   = (uint16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    uint16_t green = (uint16_t)(raw[4] | ((uint16_t)raw[5] << 8));
    uint16_t blue  = (uint16_t)(raw[6] | ((uint16_t)raw[7] << 8));

    out->clear = clear;
    out->red = red;
    out->green = green;
    out->blue = blue;
    out->color = classify_color(clear, red, green, blue);
    out->valid = 1U;
}

static void rgb_task(uint32_t now_ms)
{
    /* Keep the original RIGHT sensor active; I2C must not interrupt a ping. */
    if (ultra_state != ULTRA_IDLE) return;

    if ((uint32_t)(now_ms - rgb_last_retry_ms) >= RGB_RETRY_PERIOD_MS) {
        rgb_last_retry_ms = now_ms;
        if (!robot_rgb_right.connected) {
            robot_rgb_right.valid = 0U;
            robot_rgb_right.color = ROBOT_COLOR_UNKNOWN;
            robot_rgb_right.connected = tcs_init(&hi2c1);
        }
    }
    if ((uint32_t)(now_ms - rgb_last_poll_ms) < RGB_POLL_PERIOD_MS) return;
    rgb_last_poll_ms = now_ms;
    tcs_sample(&hi2c1, &robot_rgb_right);
}

/* Normalize against total RGB counts rather than using fixed raw thresholds.
   Percentages make it easier to calibrate under different lighting. */
static unsigned int rgb_ratio_pct(uint16_t channel,
                                  const volatile RobotRgbReading_t *sample)
{
    uint32_t sum = (uint32_t)sample->red + (uint32_t)sample->green +
                   (uint32_t)sample->blue;
    if (!sample->valid || sum == 0U) return 0U;
    return (unsigned int)(((uint32_t)channel * 100U + sum / 2U) / sum);
}

static const char *mission_name(uint8_t state)
{
    switch (state) {
    case MISSION_READ_COLOUR:     return "COLOUR";
    case MISSION_COUNT_JUNCTIONS: return "COUNT";
    case MISSION_PASS_JUNCTION:   return "PASS";
    case MISSION_TURN_FORWARD:    return "FWD";
    case MISSION_TURN_LEFT:       return "TURN";
    case MISSION_FOLLOW_BRANCH:   return "BRANCH";
    case MISSION_WAIT_BALL:       return "WAIT_BALL";
    case MISSION_PICKUP_APPROACH: return "APPROACH";
    case MISSION_PICKUP_CLOSE:    return "GRIPPING";
    case MISSION_OUTER_ROUTE:     return "OUTER";
    case MISSION_OUTER_RIGHT:     return "RIGHT90";
    case MISSION_RETURN_BACKOFF:  return "BACKOFF";
    case MISSION_TURN_AROUND:     return "UTURN180";
    case MISSION_RETURN_BRANCH:   return "BACK_BRANCH";
    case MISSION_RETURN_JOIN:     return "JOIN_FWD";
    case MISSION_RETURN_LEFT:     return "JOIN_LEFT";
    case MISSION_RETURN_MAIN:     return "TO_UNLOAD";
    case MISSION_RETURN_PASS:     return "PASS_RETURN";
    case MISSION_UNLOAD_ENTER:    return "ENTER_UNLOAD";
    case MISSION_UNLOAD_RELEASE:  return "RELEASE";
    case MISSION_CONFIRM_START:    return "START_MARK";
    case MISSION_CONFIRM_UNLOAD:   return "END_MARK";
    default:                      return "OFF";
    }
}

/* Create one short snapshot every 500ms. UART transmission uses interrupts,
   so neither UART baudrate nor snprintf blocks the PID or echo-polling loop.
   Keep telemetry_tx_buffer unchanged until the TX-complete interrupt. */
static void telemetry_task(uint32_t now_ms)
{
    if (telemetry_tx_busy) return;
    if (ultra_state != ULTRA_IDLE) return; /* Do not delay echo edge polling */
    if ((uint32_t)(now_ms - telemetry_last_ms) < TELEMETRY_PERIOD_MS) return;
    telemetry_last_ms = now_ms;

    const char *rs = !robot_rgb_right.connected ? "NO_SENSOR" :
                     !robot_rgb_right.valid     ? "NO_DATA"   : "OK";

    char range_text[24];
    if (robot_ultrasonic_valid && robot_ultrasonic_cm >= 0.0f) {
        /* Avoid %f: embedded newlib-nano often disables floating printf. */
        uint32_t tenths = (uint32_t)(robot_ultrasonic_cm * 10.0f + 0.5f);
        (void)snprintf(range_text, sizeof(range_text), "%lu.%lucm",
                       (unsigned long)(tenths / 10U),
                       (unsigned long)(tenths % 10U));
    } else {
        (void)snprintf(range_text, sizeof(range_text), "NO_ECHO");
    }

    int n = snprintf((char *)telemetry_tx_buffer,
        sizeof(telemetry_tx_buffer),
        "%s | R[%s] %s C=%u R=%u G=%u B=%u (%u/%u/%u%%) | "
        "US=%s UOK=%lu UTO=%lu PC=%u/%u | GRIP=%s | ERR=%u | MIS=%s M=%u T=%u J=%u K=%u RJ=%u LINE=%u SIDE=%d ARM=%u RES=%u FAULT=%u | CMD_RX=%lu REL_RX=%lu LAST=%c AGE=%lums\r\n",
        current_mode == MODE_MANUAL ? "MAN" : "PID",
        rs, color_name(robot_rgb_right.valid ? robot_rgb_right.color : ROBOT_COLOR_UNKNOWN),
        (unsigned int)robot_rgb_right.clear,
        (unsigned int)robot_rgb_right.red,
        (unsigned int)robot_rgb_right.green,
        (unsigned int)robot_rgb_right.blue,
        rgb_ratio_pct(robot_rgb_right.red, &robot_rgb_right),
        rgb_ratio_pct(robot_rgb_right.green, &robot_rgb_right),
        rgb_ratio_pct(robot_rgb_right.blue, &robot_rgb_right),
        range_text,
        (unsigned long)ultrasonic_good_echo_count,
        (unsigned long)ultrasonic_timeout_count,
        (unsigned int)pickup_valid_echo_count,
        (unsigned int)PICKUP_MIN_ECHO_SAMPLES,
        robot_gripper_state == ROBOT_GRIPPER_OPEN ? "OPEN" :
        robot_gripper_state == ROBOT_GRIPPER_CLOSED ? "CLOSED" : "CENTER",
        (unsigned int)robot_hw_fault_flags,
        mission_name(robot_mission_state),
        (unsigned int)robot_full_black_count,
        (unsigned int)robot_target_ball,
        (unsigned int)robot_junction_count,
        (unsigned int)robot_outer_corner_count,
        (unsigned int)robot_return_junction_count,
        (unsigned int)robot_line_adc, (int)corner_side,
        (unsigned int)junction_armed,
        (unsigned int)robot_mission_result,
        (unsigned int)robot_mission_fault,
        (unsigned long)robot_manual_command_count,
        (unsigned long)manual_release_command_count,
        last_manual_direction,
        (unsigned long)((uint32_t)(now_ms - last_manual_ms)));

    if (n <= 0 || (unsigned int)n >= sizeof(telemetry_tx_buffer)) return;
    telemetry_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart2, telemetry_tx_buffer,
                             (uint16_t)n) != HAL_OK) {
        telemetry_tx_busy = 0U;
    }
}

static void process_command(uint8_t c)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - robot_boot_ms) < BOOT_COMMAND_GUARD_MS) return;
    /* Manual overrides cancel the active mission. */
    if (c == 'P' || c == 'F' || c == 'B' || c == 'L' || c == 'R') {
        if (robot_mission_state != MISSION_OFF) {
            robot_mission_result = MISSION_RESULT_FAILED;
            robot_mission_fault = 0U; /* User manual override, not a fault. */
        }
        robot_mission_state = MISSION_OFF;
    }

    switch (c) {
    case 'A':  /* Start the normal-map autonomous mission */
        if (robot_mission_state != MISSION_OFF) break;
        Robot_Mission_Start();
        break;

    case 'h':  /* OMFAER headlight OFF: stop either automatic mode */
        if (robot_mission_state != MISSION_OFF) {
            robot_mission_result = MISSION_RESULT_FAILED;
            robot_mission_fault = 0U;
        }
        robot_mission_state = MISSION_OFF;
        current_mode = MODE_MANUAL;
        manual_moving = 0U;
        stop_motors();
        reset_pid();
        corner_side = 0;
        break;

    case 'H':
#if H_FULL_MISSION
        if (robot_mission_state == MISSION_OFF) Robot_Mission_Start();
        break;
#else
        /* Preserve the uploaded working H toggle, including cancellation. */
        if (robot_mission_state != MISSION_OFF) {
            robot_mission_result = MISSION_RESULT_FAILED;
            robot_mission_fault = 0U;
            robot_mission_state = MISSION_OFF;
        }
#endif
        /* fall through */
    case 'P':  /* Standalone PID toggle */
        if (h_toggle_seen &&
            (uint32_t)(now - last_h_toggle_ms) < TOGGLE_GUARD_MS) break;
        h_toggle_seen = 1U;
        last_h_toggle_ms = now;
        stop_motors();
        manual_moving = 0U;
        reset_pid();
        if (current_mode == MODE_MANUAL) {
            current_mode = MODE_LINE_FOLLOW;
            line_seen_once = 0U;
            corner_side = 0;
            last_turn = 1;
            last_line_ms = now;
            last_follow_ms = now - FOLLOW_PERIOD_MS;
        } else {
            current_mode = MODE_MANUAL;
        }
        break;

    case 'O':  /* Toggle gripper OPEN <-> CLOSED; starts OPEN */
        if (o_toggle_seen &&
            (uint32_t)(now - last_o_toggle_ms) < TOGGLE_GUARD_MS) break;
        o_toggle_seen = 1U;
        last_o_toggle_ms = now;
        Robot_Gripper_Toggle();
        break;


    case 'F':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(manual_left_speed(MANUAL_SPEED),
                   manual_right_speed(MANUAL_SPEED));
        last_manual_ms = now; /* Last movement command, for telemetry. */
        manual_moving = 1U;
        last_manual_direction = 'F';
        robot_manual_command_count++;
        break;

    case 'B':
        current_mode = MODE_MANUAL;
        reset_pid();
        /* Equal reverse PWM: 65% left, 65% right. Forward trim is separate. */
        set_motors(-MANUAL_REVERSE_SPEED, -MANUAL_REVERSE_SPEED);
        last_manual_ms = now; /* Last movement command, for telemetry. */
        manual_moving = 1U;
        last_manual_direction = 'B';
        robot_manual_command_count++;
        break;

    case 'L':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(-MANUAL_TURN_SPEED, MANUAL_TURN_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
        last_manual_direction = 'L';
        robot_manual_command_count++;
        break;

    case 'R':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(MANUAL_TURN_SPEED, -MANUAL_TURN_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
        last_manual_direction = 'R';
        robot_manual_command_count++;
        break;

    case 'S':  /* OMFAER: released F or B (no visible S button needed) */
    case 'C':  /* OMFAER: released L or R (no visible C button needed) */
        if (current_mode == MODE_MANUAL) {
            stop_motors();
            manual_moving = 0U;
            manual_release_command_count++;
            last_manual_direction = (char)c;
            last_manual_ms = now;
        }
        break;

    default:
        /* Ignore CR/LF and other characters from the phone. */
        break;
    }
}

/* PID error: -1 = far left, 0 = centred, +1 = far right.
   Controller output is a differential PWM command in permille. */
static float calculate_pid(float error, uint32_t now)
{
    float dt = (float)FOLLOW_PERIOD_MS / 1000.0f;

    if (pid_initialized) {
        uint32_t elapsed_ms = (uint32_t)(now - pid_last_ms);
        if (elapsed_ms > 0U && elapsed_ms <= 30U) {
            dt = (float)elapsed_ms / 1000.0f;
        } else {
            /* Avoid derivative spike after a long pause. */
            pid_initialized = 0U;
            pid_filtered_derivative = 0.0f;
        }
    }

    float derivative = 0.0f;
    if (pid_initialized) {
        derivative = (error - pid_previous_error) / dt;
    }

    pid_filtered_derivative +=
        D_FILTER_ALPHA * (derivative - pid_filtered_derivative);

    float proposed_integral = clamp_float(
        robot_pid_integral + error * dt,
        -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    float p = robot_kp * error;
    float d = robot_kd * pid_filtered_derivative;
    float proposed_output = p + robot_ki * proposed_integral + d;

    /* Conditional integration anti-windup: do not accumulate
       error while the output is already saturated in that direction. */
    if (!((proposed_output > MAX_PID_CORRECTION && error > 0.0f) ||
          (proposed_output < -MAX_PID_CORRECTION && error < 0.0f))) {
        robot_pid_integral = proposed_integral;
    }

    float output = clamp_float(
        p + robot_ki * robot_pid_integral + d,
        -MAX_PID_CORRECTION, MAX_PID_CORRECTION);

    pid_last_ms = now;
    pid_previous_error = error;
    pid_initialized = 1U;
    robot_pid_error = error;
    robot_pid_output = output;
    return output;
}

/* Convert normal POSITION readings only; never mirror black/lost codes. */
static int32_t position_adc(uint16_t adc)
{
    return SENSOR_REVERSED ? 4096 - (int32_t)adc : (int32_t)adc;
}

static void remember_corner_side(uint32_t now, uint16_t adc)
{
    if (adc >= ADC_CROSS_MIN) { corner_side = 0; return; }
    if (adc < ADC_LOST_LEFT_MAX || adc > ADC_LOST_RIGHT_MIN) return;
    int32_t pos = position_adc(adc);
    if (pos >= CORNER_SIDE_ADC) { corner_side = 1; corner_side_ms = now; }
    else if (pos <= 4096 - CORNER_SIDE_ADC) { corner_side = -1; corner_side_ms = now; }
    else if (pos >= TURN_CENTER_ADC_MIN && pos <= TURN_CENTER_ADC_MAX)
        corner_side = 0;
}

static void follow_line_adc(uint32_t now, uint16_t adc)
{
    remember_corner_side(now, adc);
    /* The sensor reports all-black separately from normal position. */
    if (adc >= ADC_CROSS_MIN) {
        line_seen_once = 1U;
        last_line_ms = now;
        reset_pid();
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        return;
    }

    uint8_t lost = 0U;
    if (adc < ADC_LOST_UNKNOWN_MAX) {
        lost = 1U; /* Unknown: keep previous search direction. */
    } else if (adc < ADC_LOST_LEFT_MAX) {
        lost = 1U;
        /* Keep the last valid position, not a line-loss code. */
    } else if (adc > ADC_LOST_RIGHT_MIN) {
        lost = 1U;
    }

    if (lost) {
        reset_pid();

        /* Do not drive off the mat if automatic mode starts with no line. */
        if (!line_seen_once) {
            stop_motors();
        } else {
            uint8_t hint = corner_side != 0 &&
                (uint32_t)(last_line_ms - corner_side_ms) <= CORNER_HINT_MS;
            int8_t direction = hint ? corner_side : last_turn;
            uint32_t bridge = hint ? GAP_STRAIGHT_MS : DASH_BRIDGE_MS;
            if ((uint32_t)(now - last_line_ms) < bridge)
                set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
            else if (robot_mission_state == MISSION_OFF &&
                     (uint32_t)(now - last_line_ms) >= bridge + OUTER_CORNER_TIMEOUT_MS)
                stop_motors(); /* bounded standalone search; wait for line */
            else if (direction < 0)
                set_motors(-SEARCH_SPEED, SEARCH_SPEED);
            else
                set_motors(SEARCH_SPEED, -SEARCH_SPEED);
        }
        return;
    }

    line_seen_once = 1U;
    last_line_ms = now;

    float error = ((float)adc - (float)ADC_CENTER) / ADC_POSITION_SCALE;
    error = clamp_float(error, -1.0f, 1.0f);
    if (SENSOR_REVERSED) error = -error;

    if (error < -0.06f) last_turn = -1;
    if (error >  0.06f) last_turn =  1;

    float correction = calculate_pid(error, now);
    int32_t left = (int32_t)((float)FOLLOW_SPEED + correction);
    int32_t right = (int32_t)((float)FOLLOW_SPEED - correction);
    set_motors(left, right);
}

static void follow_line(uint32_t now)
{
    uint16_t adc;
    if (!read_line_adc(&adc)) {
        reset_pid();
        stop_motors();
        return;
    }
    follow_line_adc(now, adc);
}

/* ================================================================
 * FULL USER-MAP MISSION, launched by Bluetooth 'A'
 *
 * FIRST full-black START marker -> right marker RED(1) / BLUE(2) / GREEN(3) -> two right
 * 90-degree outer corners -> choose ball branch from right to left
 * -> left into branch -> SECOND full-black ball endpoint -> confirm range
 * -> timed approach -> grip -> back off -> U-turn
 * -> follow branch north to T -> left onto westbound main line
 * -> skip remaining branches -> THIRD full-black UNLOAD marker -> open
 * gripper -> stop.
 *
 * This is sensor/timer based, NOT odometry. The Maker Line ANALOG output
 * encodes both position and cross/all-black (~ADC >= 3891). Robustness
 * depends on calibrated black-mark geometry and real speed.
 * ================================================================ */
static RobotColor_t mission_read_colour(void)
{
    return (robot_rgb_right.valid &&
            robot_rgb_right.color != ROBOT_COLOR_UNKNOWN) ?
        robot_rgb_right.color : ROBOT_COLOR_UNKNOWN;
}

/* Independent destination latch: never reset route progress when colour arrives. */
static void outbound_colour_update(uint32_t now)
{
    if (robot_target_ball != 0U) return;
    RobotColor_t c = mission_read_colour();
    if (c != colour_candidate) {
        colour_candidate = c;
        colour_candidate_ms = now;
    } else if (c != ROBOT_COLOR_UNKNOWN &&
               (uint32_t)(now - colour_candidate_ms) >= COLOUR_CONFIRM_MS) {
        robot_target_ball = (c == ROBOT_COLOR_RED) ? 1U :
                            (c == ROBOT_COLOR_BLUE) ? 2U : 3U;
    }
}

static void mission_set_state(uint8_t state, uint32_t now)
{
    robot_mission_state = state;
    mission_state_ms = now;
    turn_departed = 0U;
    turn_center_candidate = 0U;
    turn_center_ms = now;
}

static void mission_finish(uint8_t success, uint8_t failure_code)
{
    stop_motors();
    reset_pid();
    manual_moving = 0U;
    robot_mission_state = MISSION_OFF;
    robot_mission_result = success ? MISSION_RESULT_SUCCESS : MISSION_RESULT_FAILED;
    robot_mission_fault = success ? 0U : failure_code;
    current_mode = MODE_MANUAL;
}

static void mission_abort(uint8_t code)
{
    mission_finish(0U, code);
}

static uint8_t is_line_lost(uint16_t adc)
{
    /* Maker Line: below ~10% Vcc = off line. Above ~90% Vcc may
       instead mean all-black; do NOT confuse CROSS with right loss. */
    return (adc < ADC_LOST_LEFT_MAX ||
            (adc > ADC_LOST_RIGHT_MIN && adc < ADC_CROSS_MIN));
}

static uint8_t is_line_centered(uint16_t adc)
{
    return adc >= TURN_CENTER_ADC_MIN && adc <= TURN_CENTER_ADC_MAX;
}

/* Dedicated 90-degree pivot. Require departure from the previous centre,
   then a stable new centre. Pause while confirming to avoid overshooting.
   Return 1 only after confirmation; caller owns timeout and mission state. */
static uint8_t turn90_step(uint32_t now, uint16_t adc, int8_t direction,
                           uint32_t minimum_ms)
{
    uint8_t centered = is_line_centered(adc);
    if (!centered) {
        turn_departed = 1U;
        turn_center_candidate = 0U;
    }
    if (turn_departed && centered &&
        (uint32_t)(now - mission_state_ms) >= minimum_ms) {
        stop_motors();
        if (!turn_center_candidate) {
            turn_center_candidate = 1U;
            turn_center_ms = now;
        } else if ((uint32_t)(now - turn_center_ms) >= CORNER_CENTER_STABLE_MS) {
            return 1U;
        }
        return 0U;
    }
    turn_center_candidate = 0U;
    int32_t speed = SEARCH_SPEED;
    if (turn_departed && !is_line_lost(adc) && adc < ADC_CROSS_MIN &&
        (uint32_t)(now - mission_state_ms) >= minimum_ms / 2U)
        speed = CORNER_ALIGN_SPEED;
    set_motors(direction * speed, -direction * speed);
    return 0U;
}

static uint8_t is_map_junction(uint16_t adc)
{
    /* Low signal (left side of line) OR all-black intersection.
       The 5-channel analog output alone cannot distinguish every T
       intersection from a wide marker or off-centre curve. */
    return (adc >= ADC_LOST_LEFT_MAX && adc < JUNCTION_LEFT_ADC_MAX) ||
            adc >= ADC_CROSS_MIN;
}

/* Outbound left entrances: do not count white/lost-line codes. */
static uint8_t is_outbound_entrance(uint16_t adc)
{
    if (adc >= ADC_CROSS_MIN) return OUTBOUND_ACCEPT_CROSS;
    if (is_line_lost(adc)) return 0U;
    return position_adc(adc) < JUNCTION_LEFT_ADC_MAX;
}

static uint8_t is_t_marker(uint16_t adc)
{
    /* Return branch T / unload entrance: requires a wide black junction
       crossing the whole sensor array. This MUST be tested on the map. */
    return adc >= ADC_CROSS_MIN;
}


/* Debounce all-black (Maker Line analogue ADC >= ADC_CROSS_MIN).
   The returned event is a NEW stable bar, not the same bar on every 5ms tick.
   A continuous black region must be exited for MARKER_CLEAR_MS to rearm. */
static uint8_t mission_marker_update(uint32_t now, uint8_t black)
{
    marker_just_confirmed = 0U;
    if (black) {
        marker_clear_candidate = 0U;
        if (!marker_latched) {
            if (!marker_candidate) {
                marker_candidate = 1U;
                marker_candidate_ms = now;
            } else if ((uint32_t)(now - marker_candidate_ms) >= CROSS_CONFIRM_MS) {
                marker_latched = 1U;
                marker_candidate = 0U;
                marker_just_confirmed = 1U;
            }
        }
    } else {
        marker_candidate = 0U;
        if (marker_latched) {
            if (!marker_clear_candidate) {
                marker_clear_candidate = 1U;
                marker_clear_ms = now;
            } else if ((uint32_t)(now - marker_clear_ms) >= MARKER_CLEAR_MS) {
                marker_latched = 0U;
                marker_clear_candidate = 0U;
            }
        }
    }
    return marker_just_confirmed;
}

static uint8_t obstacle_ahead(void)
{
#if OBSTACLE_STOP_ENABLE
    if (!robot_ultrasonic_valid || robot_ultrasonic_cm < 0.0f)
        return obstacle_blocked;
    if (obstacle_blocked) {
        if (robot_ultrasonic_cm >= OBSTACLE_CLEAR_CM) obstacle_blocked = 0U;
    } else if (robot_ultrasonic_cm < OBSTACLE_STOP_CM) {
        obstacle_blocked = 1U;
    }
    return obstacle_blocked;
#else
    return 0U;
#endif
}

void Robot_Mission_Start(void)
{
    uint32_t now = HAL_GetTick();
    stop_motors();
    manual_moving = 0U;
    reset_pid();
    robot_target_ball = 0U;
    robot_junction_count = 0U;
    robot_outer_corner_count = 0U;
    robot_return_junction_count = 0U;
    robot_full_black_count = 0U;
    marker_latched = 0U;
    marker_candidate = 0U;
    marker_clear_candidate = 0U;
    marker_just_confirmed = 0U;
    marker_candidate_ms = now;
    marker_clear_ms = now;
    robot_mission_result = MISSION_RESULT_RUNNING;
    robot_mission_fault = 0U;
    mission_started_ms = now;
    colour_candidate = ROBOT_COLOR_UNKNOWN;
    colour_candidate_ms = now;
    junction_armed = 0U;
    junction_candidate = 0U;
    junction_clear_candidate = 0U;
    junction_clear_ms = now;
    cross_candidate = 0U;
    obstacle_blocked = 0U;
    pickup_condition_seen = 0U;
    pickup_condition_ms = now;
    pickup_valid_echo_count = 0U;
    pickup_last_checked_echo_ms = ultrasonic_last_valid_ms;
    return_armed = 0U;
    return_candidate = 0U;
    return_last_cross_ms = now;
    return_main_start_ms = now;
    outer_line_seen = 0U;
    outer_right_candidate = 0U;
    outer_right_candidate_ms = now;
    outer_last_corner_ms = now;
    corner_side = 0;
    outer_last_line_ms = now;
    outer_route_started_ms = now;
    outer_second_corner_ms = now;
    line_seen_once = 0U;
    last_line_ms = now;
    last_turn = 1; /* Both outer-map corners are RIGHT. */
    last_follow_ms = now - FOLLOW_PERIOD_MS;
    Robot_Gripper_Open();
    current_mode = MODE_LINE_FOLLOW;
    if ((robot_hw_fault_flags & (1U | 2U | 4U | 8U)) != 0U) {
        mission_abort(FAIL_HARDWARE);
    } else {
        mission_set_state(MISSION_CONFIRM_START, now);
    }
}

static uint8_t pickup_ready(uint32_t now)
{
    if ((robot_hw_fault_flags & (4U | 8U)) != 0U) {
        pickup_condition_seen = 0U;
        pickup_valid_echo_count = 0U;
        return 0U;
    }
    uint8_t distance_ok = robot_ultrasonic_valid &&
        ((uint32_t)(now - ultrasonic_last_valid_ms) <= PICKUP_ECHO_STALE_MS) &&
        ((uint32_t)(now - ultrasonic_last_valid_ms) <=
         (uint32_t)(now - mission_state_ms)) &&
        robot_ultrasonic_cm >= PICKUP_DETECT_MIN_CM &&
        robot_ultrasonic_cm < PICKUP_DETECT_MAX_CM;
    if (!distance_ok) {
        pickup_condition_seen = 0U;
        pickup_valid_echo_count = 0U;
        pickup_last_checked_echo_ms = ultrasonic_last_valid_ms;
        return 0U;
    }
    if (ultrasonic_last_valid_ms != pickup_last_checked_echo_ms) {
        pickup_last_checked_echo_ms = ultrasonic_last_valid_ms;
        if (pickup_valid_echo_count < PICKUP_MIN_ECHO_SAMPLES)
            ++pickup_valid_echo_count;
    }
    if (!pickup_condition_seen) {
        pickup_condition_seen = 1U;
        pickup_condition_ms = now;
        return 0U;
    }
    return pickup_valid_echo_count >= PICKUP_MIN_ECHO_SAMPLES &&
           (uint32_t)(now - pickup_condition_ms) >= PICKUP_CONFIRM_MS;
}

static void pickup_begin_approach(uint32_t now)
{
    /* Called ONLY after three independent, valid 10..<18 cm echoes.
       Do not use later echo distances as odometry: the ball may move out
       of the ultrasonic beam during the short, bounded approach. */
    stop_motors();
    reset_pid();
    mission_set_state(MISSION_PICKUP_APPROACH, now);
    set_motors(PICKUP_APPROACH_SPEED, PICKUP_APPROACH_SPEED);
}

static void mission_step(uint32_t now)
{
    if ((uint32_t)(now - mission_started_ms) > MISSION_TOTAL_TIMEOUT_MS) {
        mission_abort(FAIL_TIMEOUT);
        return;
    }
    /* These timed stages do not require current line sensor readings. */
    if (robot_mission_state == MISSION_WAIT_BALL) {
        stop_motors();
        if (pickup_ready(now)) {
            pickup_begin_approach(now);
        } else if ((uint32_t)(now - mission_state_ms) >= PICKUP_WAIT_TIMEOUT_MS) {
            mission_abort(FAIL_PICKUP);
        }
        return;
    }
    if (robot_mission_state == MISSION_PICKUP_APPROACH) {
        /* Timed wheel travel: HC-SR04 range is NOT a movement encoder.
           Continue for the calibrated interval even if the close ball
           vanishes from the echo beam. Manual F/B/L/R/H still aborts mission.
           Hardware PWM failure stops mission; approach is always time-bounded. */
        if ((robot_hw_fault_flags & (1U | 4U | 8U)) != 0U) {
            mission_abort(FAIL_HARDWARE);
        } else if ((uint32_t)(now - mission_state_ms) >= PICKUP_APPROACH_MS) {
            stop_motors();
            Robot_Gripper_Close();
            mission_set_state(MISSION_PICKUP_CLOSE, now);
        } else {
            set_motors(PICKUP_APPROACH_SPEED, PICKUP_APPROACH_SPEED);
        }
        return;
    }
    if (robot_mission_state == MISSION_PICKUP_CLOSE) {
        stop_motors();
        if ((uint32_t)(now - mission_state_ms) >= PICKUP_GRIP_HOLD_MS) {
            reset_pid();
#if RETURN_BACKOFF_MS > 0U
            mission_set_state(MISSION_RETURN_BACKOFF, now);
#else
            mission_set_state(MISSION_TURN_AROUND, now);
#endif
        }
        return;
    }
    if (robot_mission_state == MISSION_RETURN_BACKOFF) {
#if RETURN_BACKOFF_MS > 0U
        if ((uint32_t)(now - mission_state_ms) >= RETURN_BACKOFF_MS) {
            stop_motors();
            reset_pid();
            mission_set_state(MISSION_TURN_AROUND, now);
        } else {
            /* Equal reverse backoff: avoid unwanted turning after pickup. */
            set_motors(-RETURN_BACKOFF_SPEED, -RETURN_BACKOFF_SPEED);
        }
#else
        stop_motors();
        reset_pid();
        mission_set_state(MISSION_TURN_AROUND, now);
#endif
        return;
    }
    if (robot_mission_state == MISSION_UNLOAD_ENTER) {
        if ((uint32_t)(now - mission_state_ms) >= RETURN_UNLOAD_ENTRY_MS) {
            stop_motors();
            Robot_Gripper_Open();
            mission_set_state(MISSION_UNLOAD_RELEASE, now);
        } else {
            set_motors(RETURN_ENTRY_SPEED, RETURN_ENTRY_SPEED);
        }
        return;
    }
    if (robot_mission_state == MISSION_UNLOAD_RELEASE) {
        stop_motors();
        if ((uint32_t)(now - mission_state_ms) >= RETURN_UNLOAD_HOLD_MS)
            mission_finish(1U, 0U);
        return;
    }

    uint16_t adc = 0U;
    if (!read_line_adc(&adc)) {
        mission_abort(FAIL_ADC);
        return;
    }
    uint8_t junction = is_map_junction(adc);
    uint8_t cross = is_t_marker(adc);
    uint8_t new_marker = mission_marker_update(now, cross);

    switch (robot_mission_state) {
    case MISSION_CONFIRM_START:
        /* On 'A' the front Maker Line sensor must already touch the START bar.
           Do not drive until the FIRST full-black marker is confirmed. */
        stop_motors();
        if (new_marker && robot_full_black_count == 0U) {
            robot_full_black_count = 1U;
            reset_pid();
            line_seen_once = 1U;
            last_line_ms = now;
            outer_route_started_ms = now;
            mission_set_state(MISSION_READ_COLOUR, now);
        } else if ((uint32_t)(now - mission_state_ms) >= START_MARKER_TIMEOUT_MS) {
            mission_abort(FAIL_START); /* START marker not confirmed */
        }
        break;

    case MISSION_CONFIRM_UNLOAD:
        /* Third full-black marker must be physically beneath the sensor. */
        stop_motors();
        if (cross && marker_latched && robot_full_black_count == 2U) {
            robot_full_black_count = 3U;
            mission_set_state(MISSION_UNLOAD_ENTER, now);
        } else if ((uint32_t)(now - mission_state_ms) >= UNLOAD_MARKER_TIMEOUT_MS) {
            mission_abort(FAIL_UNLOAD);
        }
        break;

    case MISSION_READ_COLOUR:
    case MISSION_OUTER_ROUTE: {
        /* The same two-corner navigation runs even before colour is known. */
        outbound_colour_update(now);
        if (robot_target_ball != 0U && robot_mission_state == MISSION_READ_COLOUR)
            mission_set_state(MISSION_OUTER_ROUTE, now);
        if (obstacle_ahead()) { stop_motors(); reset_pid(); return; }
        if ((uint32_t)(now - outer_route_started_ms) >= OUTER_ROUTE_TIMEOUT_MS) {
            mission_abort(FAIL_OUTER);
            return;
        }
        uint8_t armed = robot_outer_corner_count == 0U ||
            (uint32_t)(now - outer_last_corner_ms) >= OUTER_CORNER_REARM_MS;
        uint8_t lost = is_line_lost(adc);
        uint8_t turn_now = 0U;
        if (!lost) {
            outer_line_seen = 1U;
            outer_last_line_ms = now;
            if (armed && adc < ADC_CROSS_MIN && position_adc(adc) >= OUTER_RIGHT_EDGE_ADC) {
                if (!outer_right_candidate) {
                    outer_right_candidate = 1U;
                    outer_right_candidate_ms = now;
                } else if ((uint32_t)(now - outer_right_candidate_ms) >= OUTER_RIGHT_CONFIRM_MS) {
                    turn_now = 1U;
                }
                /* Slow and steer right immediately instead of driving past the edge. */
                remember_corner_side(now, adc);
                line_seen_once = 1U;
                last_line_ms = now;
                reset_pid();
                if (!turn_now) set_motors(OUTER_GAP_SPEED, 0);
            } else {
                outer_right_candidate = 0U;
                follow_line_adc(now, adc);
            }
        } else {
            uint8_t edge_pending = outer_right_candidate;
            outer_right_candidate = 0U;
            reset_pid();
            if (!outer_line_seen) {
                if (!line_seen_once) { stop_motors(); break; }
                outer_line_seen = 1U;
                outer_last_line_ms = last_line_ms;
            }
            uint32_t lost_ms = (uint32_t)(now - outer_last_line_ms);
            uint8_t right_loss = SENSOR_REVERSED ?
                (adc >= ADC_LOST_UNKNOWN_MAX && adc < ADC_LOST_LEFT_MAX) :
                (adc > ADC_LOST_RIGHT_MIN && adc < ADC_CROSS_MIN);
            uint8_t right_hint = corner_side > 0 &&
                (uint32_t)(outer_last_line_ms - corner_side_ms) <= CORNER_HINT_MS;
            if (armed && (edge_pending ||
                (right_loss && lost_ms >= OUTER_RIGHT_LOSS_MS) ||
                (right_hint && lost_ms >= OUTER_CORNER_LOST_MS) ||
                lost_ms >= OUTER_DASH_MAX_MS)) {
                turn_now = 1U;
            } else if (lost_ms >= OUTER_DASH_MAX_MS) {
                mission_abort(FAIL_OUTER);
                break;
            } else {
                set_motors(OUTER_GAP_SPEED, OUTER_GAP_SPEED);
            }
        }
        if (turn_now) {
            reset_pid();
            corner_side = 0;
            outer_right_candidate = 0U;
            mission_set_state(MISSION_OUTER_RIGHT, now);
            turn_departed = 1U; /* already outside the old centre */
            set_motors(SEARCH_SPEED, -SEARCH_SPEED); /* same direction as manual R */
        }
        break;
    }

    case MISSION_OUTER_RIGHT:
        outbound_colour_update(now);
        /* The physical right direction matches the working manual R. */
        if (turn90_step(now, adc, 1, OUTER_CORNER_MIN_TURN_MS)) {
            ++robot_outer_corner_count;
            outer_last_corner_ms = now;
            outer_right_candidate = 0U;
            reset_pid();
            line_seen_once = 1U;
            last_line_ms = now;
            last_turn = 1;
            corner_side = 0;
            outer_line_seen = 1U;
            outer_last_line_ms = now;
            if (robot_outer_corner_count == OUTBOUND_REQUIRED_RIGHTS) {
                outer_second_corner_ms = now;
                /* turn90_step already confirmed the new centre for 25 ms.
                   Arm immediately on that known centre, not after a blind delay. */
                robot_junction_count = 0U;
                junction_armed = 1U;
                junction_candidate = 0U;
                junction_clear_candidate = 0U;
                mission_set_state(MISSION_COUNT_JUNCTIONS, now);
            } else {
                mission_set_state(MISSION_OUTER_ROUTE, now);
            }
        } else if ((uint32_t)(now - mission_state_ms) >= OUTER_CORNER_TIMEOUT_MS) {
            mission_abort(FAIL_OUTER);
        }
        break;

    case MISSION_COUNT_JUNCTIONS: {
        /* Hard route order: two confirmed RIGHT turns, then LEFT entrances. */
        if (robot_outer_corner_count != OUTBOUND_REQUIRED_RIGHTS) {
            mission_abort(FAIL_OUTER);
            break;
        }
        outbound_colour_update(now);
        if (robot_target_ball != 0U && robot_target_ball <= robot_junction_count) {
            mission_abort(FAIL_COLOUR); /* target already passed */
            break;
        }
        if (robot_target_ball == 0U &&
            (uint32_t)(now - outer_second_corner_ms) >= COLOUR_TIMEOUT_MS) {
            mission_abort(FAIL_COLOUR);
            break;
        }
        if (obstacle_ahead()) { stop_motors(); reset_pid(); return; }

        if (!is_outbound_entrance(adc)) {
            junction_candidate = 0U;
            follow_line_adc(now, adc);
            break;
        }
        /* Do not let PID pull the robot into a branch before counting it. */
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if (!junction_armed) break;
        if (!junction_candidate) {
            junction_candidate = 1U;
            junction_first_ms = now;
            break;
        }
        if ((uint32_t)(now - junction_first_ms) < JUNCTION_CONFIRM_MS) break;
        junction_candidate = 0U;
        junction_armed = 0U;
        junction_clear_candidate = 0U;
        ++robot_junction_count;
        reset_pid();
        if (robot_target_ball != 0U && robot_junction_count == robot_target_ball) {
            mission_set_state(MISSION_TURN_FORWARD, now);
        } else if (robot_junction_count >= OUTBOUND_BRANCH_COUNT) {
            mission_abort(FAIL_COLOUR);
        } else {
            mission_set_state(MISSION_PASS_JUNCTION, now);
        }
        break;
    }
    case MISSION_PASS_JUNCTION:
        outbound_colour_update(now);
        if (robot_target_ball != 0U && robot_target_ball <= robot_junction_count) {
            mission_abort(FAIL_COLOUR);
            break;
        }
        if ((uint32_t)(now - mission_state_ms) >= JUNCTION_PASS_TIMEOUT_MS) {
            mission_abort(FAIL_BRANCH);
            break;
        }
        /* One physical entrance can count only once. Remain straight until
           the sensor returns to a continuous centred line; a white gap is
           NOT evidence that the entrance has been cleared. */
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if (is_line_centered(adc)) {
            if (!junction_clear_candidate) {
                junction_clear_candidate = 1U;
                junction_clear_ms = now;
            } else if ((uint32_t)(now - junction_clear_ms) >= JUNCTION_REARM_MS) {
                junction_clear_candidate = 0U;
                junction_candidate = 0U;
                junction_armed = 1U;
                reset_pid();
                line_seen_once = 1U;
                last_line_ms = now;
                mission_set_state(MISSION_COUNT_JUNCTIONS, now);
            }
        } else {
            junction_clear_candidate = 0U;
        }
        break;

    case MISSION_TURN_FORWARD:
        if (robot_outer_corner_count != OUTBOUND_REQUIRED_RIGHTS ||
            robot_target_ball == 0U || robot_target_ball != robot_junction_count) {
            mission_abort(FAIL_BRANCH);
            break;
        }
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if ((uint32_t)(now - mission_state_ms) >= TURN_FORWARD_MS)
            mission_set_state(MISSION_TURN_LEFT, now);
        break;
    case MISSION_TURN_LEFT:
        if (turn90_step(now, adc, -1, TURN_LEFT_MIN_MS)) {
            reset_pid();
            line_seen_once = 1U;
            last_line_ms = now;
            corner_side = 0;
            cross_candidate = 0U;
            mission_set_state(MISSION_FOLLOW_BRANCH, now);
        } else if ((uint32_t)(now - mission_state_ms) > TURN_LEFT_TIMEOUT_MS) {
            mission_abort(FAIL_BRANCH);
        }
        break;

    case MISSION_FOLLOW_BRANCH:
        if ((uint32_t)(now - mission_state_ms) >= BRANCH_TIMEOUT_MS) {
            mission_abort(FAIL_BRANCH);
            break;
        }
        /* The dedicated left pivot already confirmed a centred branch.
           Stop on the FIRST full-black reading here, then verify while
           stationary. Do not drive across the endpoint during debounce. */
        if (cross && robot_full_black_count == 1U) {
            stop_motors();
            reset_pid();
            if (!cross_candidate) {
                cross_candidate = 1U;
                cross_first_ms = now;
            } else if ((uint32_t)(now - cross_first_ms) >= CROSS_CONFIRM_MS) {
                robot_full_black_count = 2U;
                pickup_condition_seen = 0U;
                pickup_valid_echo_count = 0U;
                pickup_last_checked_echo_ms = ultrasonic_last_valid_ms;
                mission_set_state(MISSION_WAIT_BALL, now);
            }
            break;
        }
        cross_candidate = 0U;
        follow_line_adc(now, adc);
        break;

    case MISSION_TURN_AROUND:
        /* Back-to-base orientation: LEFT 180-degree pivot, then reacquire
           the same branch with the robot facing toward the main route. */
        /* Require genuine line departure, then a stable centre after the
           minimum pivot time. A timer alone cannot measure 180 degrees. */
        if ((uint32_t)(now - mission_state_ms) >= UTURN_TIMEOUT_MS) {
            mission_abort(FAIL_UTURN);
            break;
        }
        if (is_line_lost(adc)) turn_departed = 1U;
        if (turn_departed &&
            (uint32_t)(now - mission_state_ms) >= UTURN_MIN_MS &&
            is_line_centered(adc)) {
            stop_motors();
            if (!turn_center_candidate) {
                turn_center_candidate = 1U;
                turn_center_ms = now;
                break;
            }
            if ((uint32_t)(now - turn_center_ms) < CORNER_CENTER_STABLE_MS)
                break;
            reset_pid();
            last_turn = -1;
            line_seen_once = 1U;
            last_line_ms = now;
            corner_side = 0;
            cross_candidate = 0U;
            mission_set_state(MISSION_RETURN_BRANCH, now);
        } else {
            turn_center_candidate = 0U;
            set_motors(-UTURN_SPEED, UTURN_SPEED);
        }
        break;

    case MISSION_RETURN_BRANCH:
        /* T junction to MAIN must be an all-black cross marker. A
           600ms blind interval avoids the ball-end black bar. */
        if ((uint32_t)(now - mission_state_ms) >= RETURN_BRANCH_BLIND_MS &&
            cross) {
            if (!cross_candidate) {
                cross_candidate = 1U;
                cross_first_ms = now;
            } else if ((uint32_t)(now - cross_first_ms) >= RETURN_T_CONFIRM_MS) {
                stop_motors();
                reset_pid();
                cross_candidate = 0U;
                mission_set_state(MISSION_RETURN_JOIN, now);
                break;
            }
        } else {
            cross_candidate = 0U;
        }
        follow_line_adc(now, adc);
        if ((uint32_t)(now - mission_state_ms) >= RETURN_BRANCH_TIMEOUT_MS)
            mission_abort(FAIL_RETURN_BRANCH);
        break;

    case MISSION_RETURN_JOIN:
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if ((uint32_t)(now - mission_state_ms) >= RETURN_JOIN_FORWARD_MS)
            mission_set_state(MISSION_RETURN_LEFT, now);
        break;
    case MISSION_RETURN_LEFT:
        if (turn90_step(now, adc, -1, RETURN_LEFT_MIN_MS)) {
            stop_motors();
            reset_pid();
            line_seen_once = 1U;
            last_line_ms = now;
            corner_side = 0;
            return_armed = 0U;
            return_candidate = 0U;
            return_last_cross_ms = now;
            return_main_start_ms = now;
            mission_set_state(MISSION_RETURN_MAIN, now);
        } else if ((uint32_t)(now - mission_state_ms) >= RETURN_LEFT_TIMEOUT_MS) {
            mission_abort(FAIL_RETURN_JOIN);
        }
        break;

    case MISSION_RETURN_MAIN: {
        /* Ball1: remaining ball2+ball3+unload = 3 marks.
           Ball2: remaining ball3+unload = 2 marks.
           Ball3: unload is first mark = 1 mark.
           Number of expected marks = 4 - robot_target_ball. */
        if ((uint32_t)(now - return_main_start_ms) >= RETURN_MAIN_TIMEOUT_MS) {
            mission_abort(FAIL_UNLOAD);
            return;
        }
        if ((uint32_t)(now - return_main_start_ms) < RETURN_MAIN_BLIND_MS) {
            follow_line_adc(now, adc);
            break;
        }
        if (!junction) {
            return_candidate = 0U;
            if (!return_armed &&
                (uint32_t)(now - return_last_cross_ms) >= RETURN_REARM_MS)
                return_armed = 1U;
            follow_line_adc(now, adc);
        } else {
            return_last_cross_ms = now;
            /* Go straight through branch, don't let PID turn into it. */
            set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
            if (return_armed) {
                if (!return_candidate) {
                    return_candidate = 1U;
                    return_candidate_ms = now;
                } else if ((uint32_t)(now - return_candidate_ms) >=
                           JUNCTION_CONFIRM_MS) {
                    return_candidate = 0U;
                    return_armed = 0U;
                    ++robot_return_junction_count;
                    reset_pid();
                    if (robot_return_junction_count >=
                        (uint8_t)(4U - robot_target_ball)) {
                        /* Destination must also be a CONFIRMED full-black
                           bar. Stop on it; do not release at an ordinary
                           side-branch reading. */
                        if (cross && robot_full_black_count == 2U) {
                            stop_motors();
                            mission_set_state(MISSION_CONFIRM_UNLOAD, now);
                        } else {
                            mission_abort(FAIL_UNLOAD);
                        }
                    } else {
                        mission_set_state(MISSION_RETURN_PASS, now);
                    }
                }
            }
        }
        break;
    }
    case MISSION_RETURN_PASS:
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if ((uint32_t)(now - mission_state_ms) >= RETURN_PASS_MS) {
            return_last_cross_ms = now;
            reset_pid();
            mission_set_state(MISSION_RETURN_MAIN, now);
        }
        break;

    default:
        mission_abort(FAIL_TIMEOUT);
        break;
    }
}

void Robot_Init(void)
{
    /* Always boot in MANUAL mode, with motors not moving. */
    current_mode = MODE_MANUAL;
    manual_moving = 0U;
    robot_manual_command_count = 0U;
    manual_release_command_count = 0U;
    last_manual_direction = '-';
    h_toggle_seen = 0U;
    o_toggle_seen = 0U;
    last_h_toggle_ms = 0U;
    last_o_toggle_ms = 0U;
    telemetry_tx_busy = 0U;
    telemetry_last_ms = HAL_GetTick();
    robot_hw_fault_flags = 0U;
    /* Early startup banner: makes it possible to confirm UART TX even if a
       sensor or timer later fails. UART2 must have been initialized by CubeMX. */
    static const uint8_t banner[] =
        "ROBOT BOOT | CORNER_EARLY_V3 | MANUAL | GRIP=OPEN | USART2=9600\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)banner,
                            (uint16_t)(sizeof(banner) - 1U), 100U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    stop_motors();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
        robot_hw_fault_flags |= 1U;
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        robot_hw_fault_flags |= 2U;

    /* Command the calibrated physical OPEN pulse before enabling PWM on PA6.
       This explicitly resets the state for the first O press. */
    Robot_Gripper_Open();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
        robot_hw_fault_flags |= 4U;

    /* Free-running 1 MHz 16-bit counter for HC-SR04 pulse measurement. */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
        robot_hw_fault_flags |= 8U;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    ultra_state = ULTRA_IDLE;
    ultra_last_ping_ms = HAL_GetTick() - ULTRASONIC_PERIOD_MS;
    ultrasonic_last_valid_ms = 0U;
    ultrasonic_good_echo_count = 0U;
    ultrasonic_timeout_count = 0U;
    robot_ultrasonic_valid = 0U;
    robot_ultrasonic_cm = -1.0f;

    /* Only RIGHT marker sensor remains on I2C1. */
    robot_rgb_right.connected = tcs_init(&hi2c1);
    robot_rgb_right.valid = 0U;
    robot_rgb_right.color = ROBOT_COLOR_UNKNOWN;
    rgb_last_poll_ms = HAL_GetTick();
    rgb_last_retry_ms = HAL_GetTick();

    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK)
        robot_hw_fault_flags |= 16U;

    /* Do not enable motors if motor PWM failed to start. */
    if ((robot_hw_fault_flags & 1U) == 0U)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    stop_motors();
    reset_pid();
    last_manual_ms = HAL_GetTick();
    robot_mission_state = MISSION_OFF;
    robot_target_ball = 0U;
    robot_junction_count = 0U;
    robot_outer_corner_count = 0U;
    robot_return_junction_count = 0U;
    robot_full_black_count = 0U;
    marker_latched = 0U;
    marker_candidate = 0U;
    marker_clear_candidate = 0U;
    marker_just_confirmed = 0U;
    marker_candidate_ms = HAL_GetTick();
    marker_clear_ms = HAL_GetTick();
    robot_mission_result = MISSION_RESULT_IDLE;
    robot_mission_fault = 0U;
    obstacle_blocked = 0U;
    pickup_condition_seen = 0U;
    pickup_condition_ms = HAL_GetTick();
    pickup_valid_echo_count = 0U;
    pickup_last_checked_echo_ms = ultrasonic_last_valid_ms;
    corner_side = 0;
    robot_boot_ms = HAL_GetTick();
}

void Robot_Task(void)
{
    /* Consume all received Bluetooth characters outside the ISR. */
    while (rx_tail != rx_head) {
        uint8_t c = rx_buffer[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) % RX_BUFFER_SIZE);
        process_command(c);
    }

    uint32_t now = HAL_GetTick();
    if ((robot_hw_fault_flags & 8U) == 0U)
        ultrasonic_task(now);
    if (current_mode == MODE_LINE_FOLLOW) {
        if ((uint32_t)(now - last_follow_ms) >= FOLLOW_PERIOD_MS) {
            last_follow_ms = now;
            if ((robot_hw_fault_flags & 3U) == 0U) {
                if (robot_mission_state != MISSION_OFF)
                    mission_step(now);
                else
                    follow_line(now);
            } else
                stop_motors();
        }
    }
    /* Manual motion is latched by F/B/L/R until OMFAER sends S/C on release.
       Do not add a fixed 400ms hold timeout: this app doesn't repeat commands.
       WARNING: lost release packets / Bluetooth disconnect cannot stop motors. */

    /* Run colour sampling after motor PID and only when no ping is active. */
    rgb_task(HAL_GetTick());
    telemetry_task(HAL_GetTick());
}

void Robot_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        telemetry_tx_busy = 0U;
    }
}

void Robot_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t next = (uint8_t)((rx_head + 1U) % RX_BUFFER_SIZE);
        if (next != rx_tail) {
            rx_buffer[rx_head] = rx_byte;
            rx_head = next;
        }
        (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}

void Robot_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}
