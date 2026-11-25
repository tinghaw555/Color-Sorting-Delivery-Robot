# Color-Sorting-Delivery-Robot

# Color-Sorting Delivery Robot

**STM32-Based Autonomous Mobile Robot | PID Control | Finite State Machine | Embedded Systems**

## Project Overview

The Color-Sorting Delivery Robot is an STM32F103C8T6-based mobile robot developed to integrate autonomous line following, colour-based target identification, and Bluetooth-controlled navigation.

The system combines a **PID controller** for line tracking with a **Finite State Machine (FSM)** for autonomous navigation. By detecting coloured markers, the robot determines its target location and navigates through predefined track junctions.

The robot also supports manual control through Bluetooth, ultrasonic distance sensing, and a servo-operated gripper.

## Key Features

- **PID-Based Line Following:** Uses an analogue infrared line sensor and differential motor speed control for line tracking.
- **Finite State Machine (FSM):** Organizes autonomous navigation into different operating states.
- **Colour-Based Target Selection:** Identifies red, blue, and green markers using TCS34725 colour sensors.
- **Junction Detection and Counting:** Navigates to a selected branch based on the detected target.
- **Bluetooth Remote Control:** Supports manual movement and operating-mode selection through an HC-05 module.
- **Ultrasonic Obstacle Detection:** Uses an HC-SR04 sensor to detect nearby obstacles during selected autonomous navigation stages.
- **Servo Gripper Control:** Operates an SG90 servo for opening and closing the gripper.
- **Serial Telemetry:** Reports sensor readings, operating states, and diagnostic information.

## Hardware Components

| Component | Description |
|---|---|
| STM32F103C8T6 Blue Pill | Main microcontroller |
| TB6612FNG | Dual DC motor driver |
| DC TT Motors | Differential-drive movement |
| Cytron Maker Line | Analogue infrared line sensor |
| TCS34725 × 2 | Colour detection |
| HC-SR04 | Ultrasonic distance sensor |
| HC-05 | Bluetooth communication |
| SG90 | Servo gripper |
| LM2596 | DC-DC voltage regulator |

## Software and Development Tools

- **Programming Language:** Embedded C
- **IDE:** STM32CubeIDE
- **Framework:** STM32 HAL
- **Microcontroller Configuration:** STM32CubeMX
- **Programming and Debugging:** ST-LINK V2

## Control Algorithms

### 1. PID Line-Following Control

The robot uses a Proportional–Integral–Derivative (PID) controller to correct its position relative to the detected line.

The control output is calculated using:

\[
u(t)=K_pe(t)+K_i\int e(t)\,dt+K_d\frac{de(t)}{dt}
\]

The resulting correction is applied to the left and right motors:

```c
left_speed  = base_speed + correction;
right_speed = base_speed - correction;
```

Initial PID parameters:

| Parameter | Value |
|---|---|
| Kp | 310.0 |
| Ki | 40.0 |
| Kd | 0.425 |
| PID update interval | 5 ms |
| Base motor speed | 47% PWM |

The controller also incorporates derivative filtering and integral anti-windup. Parameters require tuning according to the physical robot and track conditions.

### 2. Finite State Machine (FSM) Navigation

The autonomous mission uses a state-based algorithm to coordinate colour detection, junction counting, and navigation.

**Autonomous mission sequence:**

1. Start autonomous operation.
2. Follow the line using PID control.
3. Detect a colour marker.
4. Determine the target ball location.
5. Detect and count junctions.
6. Navigate into the selected branch.
7. Follow the branch until the designated black stopping bar.
8. Stop and switch to manual mode for ball collection.

**Colour-to-target mapping:**

| Detected Colour | Target |
|---|---|
| Red | Ball 1 |
| Blue | Ball 2 |
| Green | Ball 3 |

The autonomous mission is organized into the following states:

- `MISSION_READ_COLOUR`
- `MISSION_COUNT_JUNCTIONS`
- `MISSION_PASS_JUNCTION`
- `MISSION_TURN_FORWARD`
- `MISSION_TURN_LEFT`
- `MISSION_FOLLOW_BRANCH`

A mission-off state is used when autonomous operation is inactive or completed.

### 3. Bluetooth Manual Control

The robot communicates with a mobile application through the HC-05 Bluetooth module using USART2.

| Command | Function |
|---|---|
| `F` | Move forward |
| `B` | Move backward |
| `L` | Turn left |
| `R` | Turn right |
| `H` | Toggle manual / PID line-following mode |
| `O` | Toggle gripper open / close |
| `A` | Start autonomous mission |

Manual movement uses a 250 ms command timeout to stop the motors when no new directional command is received. Continuous press-and-hold movement depends on the mobile application's transmission behaviour.

## STM32 Pin Configuration

| STM32 Pin | Function |
|---|---|
| PA0 | Maker Line analogue input |
| PA2 / PA3 | HC-05 USART2 TX / RX |
| PA6 | SG90 servo — TIM3 CH1 |
| PA8 / PA9 | TB6612FNG PWMA / PWMB |
| PB0 / PB1 | HC-SR04 TRIG / ECHO |
| PB5 | TB6612FNG STBY |
| PB6 / PB7 | TCS34725 right — I2C1 SCL / SDA |
| PB10 / PB11 | TCS34725 left — I2C2 SCL / SDA |
| PB12 / PB13 | TB6612FNG AIN1 / AIN2 |
| PB14 / PB15 | TB6612FNG BIN1 / BIN2 |

## Repository Structure

```text
Color-Sorting-Delivery-Robot/
├── Inc/
│   ├── main.h
│   └── robot_control.h
│
├── Src/
│   ├── main.c
│   ├── robot_control.c
│   ├── stm32f1xx_hal_msp.c
│   └── stm32f1xx_it.c
│
├── .gitignore
└── README.md
```

### Source Code Description

- **main.c:** System clock configuration, peripheral initialization, and main application loop.
- **main.h:** Hardware definitions and main program declarations.
- **robot_control.c:** Motor control, PID algorithms, Bluetooth communication, sensor integration, and autonomous mission logic.
- **robot_control.h:** Function prototypes, shared variables, and data structures.
- **stm32f1xx_hal_msp.c:** Peripheral hardware support and GPIO configuration.
- **stm32f1xx_it.c:** Interrupt service routines.

## Development Status

The repository contains the implemented firmware for manual control, PID line following, sensor integration, and FSM-based autonomous navigation.

Physical calibration and testing are required to validate the complete autonomous mission, particularly colour thresholds, junction detection, turning timing, and motor responsiveness.

The current autonomous mission navigates to a target collection point and switches to manual control. A complete autonomous pickup, sorting, and delivery cycle is not yet implemented.

## Notes

This is a **source-code-only repository**. It does not contain the complete STM32CubeIDE project, STM32 HAL dependencies, startup files, or CubeMX `.ioc` configuration required for a standalone build.
