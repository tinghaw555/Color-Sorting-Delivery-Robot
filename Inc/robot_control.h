#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include "main.h"
#include <stdint.h>

/* Two TCS34725 modules use their own I2C buses (fixed 7-bit address 0x29). */
typedef enum {
    ROBOT_COLOR_UNKNOWN = 0,
    ROBOT_COLOR_RED,
    ROBOT_COLOR_GREEN,
    ROBOT_COLOR_BLUE
} RobotColor_t;

typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint8_t connected;     /* Device responded during initialization */
    uint8_t valid;         /* Most recent reading is valid */
    RobotColor_t color;    /* Normalized classification: tune from real samples */
} RobotRgbReading_t;

/* Live Expressions: updates occur in Robot_Task(), not in an interrupt. */
extern volatile RobotRgbReading_t robot_rgb_left;
extern volatile RobotRgbReading_t robot_rgb_right;
extern volatile float robot_ultrasonic_cm;  /* -1.0f means no valid echo */
extern volatile uint8_t robot_ultrasonic_valid;
/* Initialization status bitmask shown as ERR in telemetry (0 = OK). */
extern volatile uint8_t robot_hw_fault_flags;

/* Commanded servo state: starts OPEN; 'O' toggles CLOSED/OPEN.
   This variable does not measure the physical servo position. */
typedef enum {
    ROBOT_GRIPPER_OPEN = 0,
    ROBOT_GRIPPER_CLOSED = 1,
    ROBOT_GRIPPER_CENTER = 2
} RobotGripperState_t;
extern volatile RobotGripperState_t robot_gripper_state;

/* SG90: 50 Hz PWM on TIM3_CH1, PA6. Arduino-tested 50? OPEN / 110? CLOSED mapped to approximately 1060 / 1678 us pulses. */
void Robot_Servo_SetPulseUs(uint16_t pulse_us);
void Robot_Gripper_Open(void);
void Robot_Gripper_Close(void);
void Robot_Gripper_Toggle(void); /* Called by Bluetooth O command */
void Robot_Gripper_Center(void);


/* Call after CubeMX-generated GPIO, ADC1, TIM1, TIM2, TIM3,
   I2C1, I2C2 and USART2 initialization functions. */
void Robot_Init(void);

/* Call continuously in main while(1). Ultrasonic uses a polling state
   machine; RGB I2C transactions are brief but synchronous. */
void Robot_Task(void);

/* Forward the Cube HAL UART callbacks to these functions. */
void Robot_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Robot_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Robot_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* Normal-map autonomous mission (start it with the Bluetooth command 'A').
   robot_mission_state: 0=OFF 1=READ COLOUR 2=COUNT BRANCHES 3=PASS BRANCH
   4=TURN FORWARD 5=TURN LEFT 6=FOLLOW BRANCH. When the mission ends the robot
   stops and switches to MANUAL mode by itself. */
extern volatile uint8_t robot_mission_state;
extern volatile uint8_t robot_target_ball;     /* 0 = not read yet, else 1..3 */
extern volatile uint8_t robot_junction_count;  /* left branches counted so far */
void Robot_Mission_Start(void);

/* Optional STM32CubeIDE Live Expressions: adjustable PID gains. */
extern volatile float robot_kp;
extern volatile float robot_ki;
extern volatile float robot_kd;
extern volatile float robot_pid_error;
extern volatile float robot_pid_integral;
extern volatile float robot_pid_output;
extern volatile uint16_t robot_line_adc;

#endif /* ROBOT_CONTROL_H_ */
