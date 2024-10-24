# stm32-robot
This repository contains the code and setup for a robot control system using the **STM32F1** microcontroller. The system communicates with an ESP32 through UART and manages motor control, proximity sensor data, and light signals. The robot can be controlled via commands sent from the ESP32, and the system provides feedback on obstacles detected using proximity sensors.

## 1. Hardware

### Microcontroller:
- **STM32F103**: Cortex-M3 based microcontroller.

### Motors:
- **Left Motor**: Controlled through GPIO pins for forward and backward motion.
- **Right Motor**: Controlled similarly via GPIO.

### Proximity Sensors:
- **LEFT Prox Sensor**: Detects obstacles on the left side.
- **MID Prox Sensor**: Detects obstacles in front.
- **RIGHT Prox Sensor**: Detects obstacles on the right side.

### Lights:
- **Left Red/Green Light**: Visual feedback for left motor status.
- **Right Red/Green Light**: Visual feedback for right motor status.

### UART Communication:
- **ESP32 Communication**: The STM32 communicates with the ESP32 through UART at a baud rate of 115200. This connection is used to receive movement commands and send proximity data.

## 2. System Tasks

The system is based on FreeRTOS and handles multiple tasks in parallel. The main tasks are as follows:

- **Motor Control Task**: Controls the direction and movement of the motors based on commands received from the ESP32.
- **Light Control Task**: Manages the status lights based on motor movements.
- **Default Task**: Runs basic housekeeping functions.
- **Watchdog Timer**: Ensures that the motors stop after a specified period without receiving new commands.

### Task Flow

1. **MotorControlTask**:
   - Receives commands from the ESP32 via a UART interrupt.
   - Controls motor movements based on the received commands.
   - Communicates proximity sensor status back to the ESP32.

2. **LightControlTask**:
   - Controls the LEDs to reflect the current motor status.
   - Toggles red/green lights based on motor actions.

3. **Default Task**:
   - A basic task that runs in the background for system maintenance.

4. **Watchdog Timer**:
   - Automatically stops the motors if no new command is received within a timeout period.

## 3. Communication Protocol

The system uses a simple command-response protocol with the ESP32. Below is a breakdown of the data exchanged between the STM32 and ESP32.

### Commands from ESP32:

- **CMD_STOP**: Stop the motors.
- **CMD_FORWARD**: Move the robot forward.
- **CMD_CLOCKWISE_ROTATE**: Rotate the robot clockwise.
- **CMD_COUNTERCLOCKWISE_ROTATE**: Rotate the robot counterclockwise.

### Status to ESP32:

- **NO_OBSTACLE**: No obstacle detected.
- **LEFT_OBSTACLE**: Obstacle detected on the left.
- **MID_OBSTACLE**: Obstacle detected in front.
- **RIGHT_OBSTACLE**: Obstacle detected on the right.

## 4. Pinout

- **Motor Pins**:
  - Left Motor Forward: **GPIOB_PIN_11**
  - Left Motor Backward: **GPIOB_PIN_12**
  - Right Motor Forward: **GPIOB_PIN_13**
  - Right Motor Backward: **GPIOB_PIN_14**

- **Proximity Sensors**:
  - Left Prox: **GPIOC_PIN_0**
  - Mid Prox: **GPIOC_PIN_1**
  - Right Prox: **GPIOC_PIN_2**

- **Lights**:
  - Left Red Light: **GPIOB_PIN_8**
  - Left Green Light: **GPIOC_PIN_9**
  - Right Red Light: **GPIOC_PIN_6**
  - Right Green Light: **GPIOC_PIN_8**

- **UART1**:
  - TX: **GPIOA_PIN_9**
  - RX: **GPIOA_PIN_10**

## 5. Error Handling

If an error occurs in the system, the `Error_Handler` function disables all interrupts and enters an infinite loop. This ensures that the error can be debugged safely.

## 6. License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
