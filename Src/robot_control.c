#include "robot_control.h"
#include <stdint.h>
#include <stdio.h>   /* snprintf: integer-only serial telemetry */

/* Peripheral handles created by CubeMX in main.c */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;    /* 1 MHz free-running timer: HC-SR04 timing */
extern TIM_HandleTypeDef htim3;    /* 50 Hz PWM CH1: SG90 on PA6 */
extern I2C_HandleTypeDef hi2c1;   /* Right TCS34725: PB6/PB7 */
extern I2C_HandleTypeDef hi2c2;   /* Left TCS34725: PB10/PB11 */

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
#define MANUAL_SPEED           550
#define MANUAL_TURN_SPEED      500
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
#define MANUAL_TIMEOUT_MS     250U
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

#define COLOUR_CONFIRM_MS       150U  /* Same colour must be seen this long */
#define COLOUR_TIMEOUT_MS     20000U  /* Give up if no colour card is found */
/* A branch on the LEFT pulls the sensor reading low suddenly. Normal PID
   tracking stays above this value, a branch drops below it. */
#define JUNCTION_LEFT_ADC_MAX   950   /* about 0.23 * Vcc */
#define JUNCTION_CONFIRM_MS      15U  /* Reading must stay low this long */
#define JUNCTION_PASS_MS        180U  /* Drive straight over a skipped branch */
#define JUNCTION_REARM_MS       150U  /* Need a normal reading before next one */
#define TURN_FORWARD_MS         100U  /* Roll forward so the wheels reach the branch */
#define TURN_LEFT_MIN_MS        250U  /* Pivot at least this long before looking */
#define TURN_LEFT_TIMEOUT_MS   1500U
#define TURN_CENTER_ADC_MIN    1500   /* Line is roughly centred again */
#define TURN_CENTER_ADC_MAX    2600
#define BRANCH_BLIND_MS         500U  /* Ignore the bar right after the turn */
#define CROSS_CONFIRM_MS         20U  /* Bar (all black) must last this long */
#define BRANCH_TIMEOUT_MS      8000U

/* Obstacle stop (HC-SR04). Set OBSTACLE_STOP_ENABLE to 0 to disable. */
#define OBSTACLE_STOP_ENABLE      1U
#define OBSTACLE_STOP_CM       10.0f
#define OBSTACLE_CLEAR_CM      15.0f

/* SG90: adjust these after observing the physical gripper. Pulse widths do
   NOT uniquely determine angle without mounting/orientation calibration. */
#define SERVO_SAFE_MIN_US       1000U
#define SERVO_SAFE_MAX_US       2000U
#define SERVO_CENTER_US         1500U
/* Based on the working Arduino Servo.h test:
   gripper.write(50)  -> about 1060 us = OPEN (default)
   gripper.write(110) -> about 1678 us = CLOSED
   Arduino Servo.h default mapping: 544..2400 us for 0..180 degrees.
   If your actual mounting is reversed, swap these two values. */
#define GRIPPER_OPEN_US         1060U
#define GRIPPER_CLOSE_US        1678U

/* HC-SR04: TIM2 must tick every 1us and roll over at 65535. */
#define ULTRASONIC_PERIOD_MS     100U
#define ULTRASONIC_TIMEOUT_US  30000U
#define ULTRASONIC_TRIGGER_US     12U

/* TCS34725: separate I2C buses, same 7-bit I2C address 0x29. */
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
#define TELEMETRY_BUFFER_SIZE  320U

/* Tune these by modifying the initial values or using Live Expressions. */
volatile float robot_kp = 310.0f;
volatile float robot_ki =  40.0f;
volatile float robot_kd =   0.425f;
volatile float robot_pid_error = 0.0f;
volatile float robot_pid_integral = 0.0f;
volatile float robot_pid_output = 0.0f;
volatile uint16_t robot_line_adc = 0U;

volatile RobotRgbReading_t robot_rgb_left  = {0};
volatile RobotRgbReading_t robot_rgb_right = {0};
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
static uint8_t rgb_next_left = 1U;
static uint32_t rgb_last_retry_ms = 0U;

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
static uint8_t line_seen_once = 0U;
static int8_t last_turn = 1;  /* -1 left, +1 right */
static uint32_t last_manual_ms = 0U;
static uint32_t last_follow_ms = 0U;
static uint32_t last_line_ms = 0U;

/* Mission state (also visible in Live Expressions and in the telemetry). */
volatile uint8_t robot_mission_state = MISSION_OFF;
volatile uint8_t robot_target_ball = 0U;
volatile uint8_t robot_junction_count = 0U;
static uint32_t mission_state_ms = 0U;
static RobotColor_t colour_candidate = ROBOT_COLOR_UNKNOWN;
static uint32_t colour_candidate_ms = 0U;
static uint8_t junction_armed = 0U;
static uint8_t junction_candidate = 0U;
static uint32_t junction_first_ms = 0U;
static uint32_t junction_last_ms = 0U;
static uint8_t cross_candidate = 0U;
static uint32_t cross_first_ms = 0U;
static uint8_t obstacle_blocked = 0U;

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
 * TCS34725: read clear/red/green/blue 16-bit values separately.
 * Right uses I2C1; left uses I2C2. The classification is tentative.
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

static void tcs_sample(I2C_HandleTypeDef *bus, volatile RobotRgbReading_t *out)
{
    if (!out->connected) return;

    uint8_t status = 0U;
    if (!tcs_read(bus, TCS_STATUS, &status)) {
        out->valid = 0U;
        out->connected = 0U; /* Attempt reconnection after 3 seconds */
        out->color = ROBOT_COLOR_UNKNOWN;
        return;
    }
    if (!(status & 0x01U)) return; /* New sample not ready */

    uint8_t raw[8];
    /* Command bit 7 and auto-increment bit 5 allow an 8-byte burst. */
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
    /* Sensor transactions only between ultrasonic pings. */
    if (ultra_state != ULTRA_IDLE) return;

    if ((uint32_t)(now_ms - rgb_last_retry_ms) >= RGB_RETRY_PERIOD_MS) {
        rgb_last_retry_ms = now_ms;
        if (!robot_rgb_left.connected) {
            robot_rgb_left.valid = 0U;
            robot_rgb_left.color = ROBOT_COLOR_UNKNOWN;
            robot_rgb_left.connected = tcs_init(&hi2c2);
        }
        if (!robot_rgb_right.connected) {
            robot_rgb_right.valid = 0U;
            robot_rgb_right.color = ROBOT_COLOR_UNKNOWN;
            robot_rgb_right.connected = tcs_init(&hi2c1);
        }
    }

    if ((uint32_t)(now_ms - rgb_last_poll_ms) < RGB_POLL_PERIOD_MS)
        return;

    rgb_last_poll_ms = now_ms;
    if (rgb_next_left) {
        tcs_sample(&hi2c2, &robot_rgb_left);
    } else {
        tcs_sample(&hi2c1, &robot_rgb_right);
    }
    rgb_next_left = !rgb_next_left;
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

    const char *ls = !robot_rgb_left.connected ? "NO_SENSOR" :
                     !robot_rgb_left.valid     ? "NO_DATA"   : "OK";
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
        "%s | L[%s] %s C=%u R=%u G=%u B=%u (%u/%u/%u%%) | "
        "R[%s] %s C=%u R=%u G=%u B=%u (%u/%u/%u%%) | US=%s | GRIP=%s | ERR=%u | MIS=%s T=%u J=%u\r\n",
        current_mode == MODE_MANUAL ? "MAN" : "PID", ls,
        color_name(robot_rgb_left.valid ? robot_rgb_left.color : ROBOT_COLOR_UNKNOWN),
        (unsigned int)robot_rgb_left.clear,
        (unsigned int)robot_rgb_left.red,
        (unsigned int)robot_rgb_left.green,
        (unsigned int)robot_rgb_left.blue,
        rgb_ratio_pct(robot_rgb_left.red, &robot_rgb_left),
        rgb_ratio_pct(robot_rgb_left.green, &robot_rgb_left),
        rgb_ratio_pct(robot_rgb_left.blue, &robot_rgb_left),
        rs, color_name(robot_rgb_right.valid ? robot_rgb_right.color : ROBOT_COLOR_UNKNOWN),
        (unsigned int)robot_rgb_right.clear,
        (unsigned int)robot_rgb_right.red,
        (unsigned int)robot_rgb_right.green,
        (unsigned int)robot_rgb_right.blue,
        rgb_ratio_pct(robot_rgb_right.red, &robot_rgb_right),
        rgb_ratio_pct(robot_rgb_right.green, &robot_rgb_right),
        rgb_ratio_pct(robot_rgb_right.blue, &robot_rgb_right),
        range_text,
        robot_gripper_state == ROBOT_GRIPPER_OPEN ? "OPEN" :
        robot_gripper_state == ROBOT_GRIPPER_CLOSED ? "CLOSED" : "CENTER",
        (unsigned int)robot_hw_fault_flags,
        mission_name(robot_mission_state),
        (unsigned int)robot_target_ball,
        (unsigned int)robot_junction_count);

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

    /* Any manual or stop command cancels the autonomous mission. */
    if (c == 'H' || c == 'S' || c == 'F' || c == 'B' || c == 'L' || c == 'R')
        robot_mission_state = MISSION_OFF;

    switch (c) {
    case 'A':  /* Start the normal-map autonomous mission */
        if (robot_mission_state != MISSION_OFF) break;
        Robot_Mission_Start();
        break;

    case 'H':  /* Toggle MANUAL <-> automatic PID follow */
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


    case 'S':   // Stop when direction button is released
        current_mode = MODE_MANUAL;
        manual_moving = 0U;
        reset_pid();
        stop_motors();
        break;

    case 'F':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(MANUAL_SPEED, MANUAL_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
        break;

    case 'B':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(-MANUAL_SPEED, -MANUAL_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
        break;

    case 'L':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(-MANUAL_TURN_SPEED, MANUAL_TURN_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
        break;

    case 'R':
        current_mode = MODE_MANUAL;
        reset_pid();
        set_motors(MANUAL_TURN_SPEED, -MANUAL_TURN_SPEED);
        last_manual_ms = now;
        manual_moving = 1U;
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

static void follow_line_adc(uint32_t now, uint16_t adc)
{
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
        last_turn = -1;
    } else if (adc > ADC_LOST_RIGHT_MIN) {
        lost = 1U;
        last_turn = 1;
    }

    if (lost) {
        reset_pid();

        /* Do not drive off the mat if automatic mode starts with no line. */
        if (!line_seen_once) {
            stop_motors();
        } else if ((uint32_t)(now - last_line_ms) < GAP_STRAIGHT_MS) {
            set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        } else if (last_turn < 0) {
            set_motors(-SEARCH_SPEED, SEARCH_SPEED);
        } else {
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
 * NORMAL-MAP MISSION (autonomous mode), started with command 'A'.
 *  1. Follow the line from the start box and read the colour card:
 *     RED -> ball 1, BLUE -> ball 2, GREEN -> ball 3.
 *  2. The bottom straight has three branch lines on the LEFT, met in the
 *     order ball 1, ball 2, ball 3. Skip (target - 1) branches, then turn
 *     left into the target branch.
 *  3. Follow the branch until the full black bar (all sensors black),
 *     stop, and switch to MANUAL so the user picks up the ball.
 * Every corner on this map is a right turn, so a sudden LEFT reading
 * can only be a branch line.
 * ================================================================ */
static RobotColor_t mission_read_colour(void)
{
    if (robot_rgb_right.valid && robot_rgb_right.color != ROBOT_COLOR_UNKNOWN)
        return robot_rgb_right.color;
    if (robot_rgb_left.valid && robot_rgb_left.color != ROBOT_COLOR_UNKNOWN)
        return robot_rgb_left.color;
    return ROBOT_COLOR_UNKNOWN;
}

static void mission_set_state(uint8_t state, uint32_t now)
{
    robot_mission_state = state;
    mission_state_ms = now;
}

/* End of mission: stop and hand over to the user (MANUAL mode). */
static void mission_finish(void)
{
    stop_motors();
    reset_pid();
    manual_moving = 0U;
    robot_mission_state = MISSION_OFF;
    current_mode = MODE_MANUAL;
}

static uint8_t obstacle_ahead(void)
{
#if OBSTACLE_STOP_ENABLE
    if (!robot_ultrasonic_valid || robot_ultrasonic_cm < 0.0f)
        return obstacle_blocked; /* No echo: keep the last decision */
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
    colour_candidate = ROBOT_COLOR_UNKNOWN;
    colour_candidate_ms = now;
    junction_armed = 0U;
    junction_candidate = 0U;
    junction_last_ms = now;
    cross_candidate = 0U;
    obstacle_blocked = 0U;
    line_seen_once = 0U;
    last_line_ms = now;
    last_follow_ms = now - FOLLOW_PERIOD_MS;
    current_mode = MODE_LINE_FOLLOW;
    mission_set_state(MISSION_READ_COLOUR, now);
}

static void mission_step(uint32_t now)
{
    uint16_t adc;
    if (!read_line_adc(&adc)) {
        reset_pid();
        stop_motors();
        return;
    }

    /* Stop for an obstacle only before the branch (the ball at the end of
       the branch must not trigger it). */
    if ((robot_mission_state == MISSION_READ_COLOUR ||
         robot_mission_state == MISSION_COUNT_JUNCTIONS) && obstacle_ahead()) {
        stop_motors();
        reset_pid();
        return;
    }

    uint8_t junction = (adc >= ADC_LOST_LEFT_MAX &&
                        adc <  JUNCTION_LEFT_ADC_MAX) ? 1U : 0U;

    switch (robot_mission_state) {
    case MISSION_READ_COLOUR: {
        follow_line_adc(now, adc);
        RobotColor_t c = mission_read_colour();
        if (c != colour_candidate) {
            colour_candidate = c;
            colour_candidate_ms = now;
        } else if (c != ROBOT_COLOR_UNKNOWN &&
                   (uint32_t)(now - colour_candidate_ms) >= COLOUR_CONFIRM_MS) {
            robot_target_ball = (c == ROBOT_COLOR_RED)  ? 1U :
                                (c == ROBOT_COLOR_BLUE) ? 2U : 3U;
            junction_armed = 0U;
            junction_candidate = 0U;
            junction_last_ms = now;
            mission_set_state(MISSION_COUNT_JUNCTIONS, now);
        } else if ((uint32_t)(now - mission_state_ms) > COLOUR_TIMEOUT_MS) {
            mission_finish();
        }
        break;
    }

    case MISSION_COUNT_JUNCTIONS:
        if (!junction) {
            junction_candidate = 0U;
            if (!junction_armed &&
                (uint32_t)(now - junction_last_ms) >= JUNCTION_REARM_MS)
                junction_armed = 1U;
            follow_line_adc(now, adc);
        } else {
            junction_last_ms = now;
            /* Go straight while a branch is under the sensor, so the PID
               does not steer into it by mistake. */
            set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
            if (junction_armed) {
                if (!junction_candidate) {
                    junction_candidate = 1U;
                    junction_first_ms = now;
                } else if ((uint32_t)(now - junction_first_ms) >=
                           JUNCTION_CONFIRM_MS) {
                    junction_candidate = 0U;
                    junction_armed = 0U;
                    robot_junction_count =
                        (uint8_t)(robot_junction_count + 1U);
                    reset_pid();
                    if (robot_junction_count >= robot_target_ball)
                        mission_set_state(MISSION_TURN_FORWARD, now);
                    else
                        mission_set_state(MISSION_PASS_JUNCTION, now);
                }
            }
        }
        break;

    case MISSION_PASS_JUNCTION:
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if ((uint32_t)(now - mission_state_ms) >= JUNCTION_PASS_MS) {
            junction_last_ms = now;
            reset_pid();
            mission_set_state(MISSION_COUNT_JUNCTIONS, now);
        }
        break;

    case MISSION_TURN_FORWARD:
        set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
        if ((uint32_t)(now - mission_state_ms) >= TURN_FORWARD_MS)
            mission_set_state(MISSION_TURN_LEFT, now);
        break;

    case MISSION_TURN_LEFT:
        set_motors(-SEARCH_SPEED, SEARCH_SPEED); /* pivot left */
        if ((uint32_t)(now - mission_state_ms) >= TURN_LEFT_MIN_MS &&
            adc >= TURN_CENTER_ADC_MIN && adc <= TURN_CENTER_ADC_MAX) {
            reset_pid();
            line_seen_once = 1U;
            last_line_ms = now;
            cross_candidate = 0U;
            mission_set_state(MISSION_FOLLOW_BRANCH, now);
        } else if ((uint32_t)(now - mission_state_ms) > TURN_LEFT_TIMEOUT_MS) {
            mission_finish();
        }
        break;

    case MISSION_FOLLOW_BRANCH:
        follow_line_adc(now, adc);
        if ((uint32_t)(now - mission_state_ms) >= BRANCH_BLIND_MS) {
            if (adc >= ADC_CROSS_MIN) {
                if (!cross_candidate) {
                    cross_candidate = 1U;
                    cross_first_ms = now;
                } else if ((uint32_t)(now - cross_first_ms) >= CROSS_CONFIRM_MS) {
                    mission_finish(); /* Reached the bar: user takes over */
                    break;
                }
            } else {
                cross_candidate = 0U;
            }
        }
        if ((uint32_t)(now - mission_state_ms) > BRANCH_TIMEOUT_MS)
            mission_finish();
        break;

    default:
        mission_finish();
        break;
    }
}

void Robot_Init(void)
{
    /* Always boot in MANUAL mode, with motors not moving. */
    current_mode = MODE_MANUAL;
    manual_moving = 0U;
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
        "ROBOT BOOT | MANUAL | GRIP=OPEN | USART2=9600\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)banner,
                            (uint16_t)(sizeof(banner) - 1U), 100U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    stop_motors();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
        robot_hw_fault_flags |= 1U;
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        robot_hw_fault_flags |= 2U;

    /* Command OPEN (Arduino 50 degrees) before enabling PWM on PA6.
       This also explicitly resets the state for the first O press. */
    Robot_Gripper_Open();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
        robot_hw_fault_flags |= 4U;

    /* Free-running 1 MHz 16-bit counter for HC-SR04 pulse measurement. */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
        robot_hw_fault_flags |= 8U;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    ultra_state = ULTRA_IDLE;
    ultra_last_ping_ms = HAL_GetTick() - ULTRASONIC_PERIOD_MS;
    robot_ultrasonic_valid = 0U;
    robot_ultrasonic_cm = -1.0f;

    /* Each TCS34725 is on a separate I2C bus: no address collision. */
    robot_rgb_left.connected = tcs_init(&hi2c2);
    robot_rgb_right.connected = tcs_init(&hi2c1);
    robot_rgb_left.valid = 0U;
    robot_rgb_right.valid = 0U;
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
    obstacle_blocked = 0U;
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
    } else if (manual_moving &&
               (uint32_t)(now - last_manual_ms) >= MANUAL_TIMEOUT_MS) {
        stop_motors();
        manual_moving = 0U;
    }

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
