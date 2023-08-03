
# Embedded-systems

This repository contains the code for a microcontroller-based system, possibly a robot or a similar device, implemented on a DSPIC30F4011 board. The system uses a scheduler to manage different tasks at different frequencies. The tasks are responsible for reading and processing commands, controlling motors, managing system status, and communicating with the user.

## Board

The system is implemented on a DSPIC30F4011 board.

## Implementation

The system is implemented using several key features of the DSPIC30F4011 microcontroller:

1.  **Timers**: The system uses the microcontroller's timers to manage the execution of tasks at different frequencies.
    
2.  **Scheduling**: A simple scheduler is used to manage the execution of tasks. Each task is associated with a `heartbeat` structure, which keeps track of how often the task should be run.
    
3.  **ADC (Analog-to-Digital Converter)**: The ADC is used to read the temperature sensor.
    
4.  **PWM (Pulse Width Modulation)**: The PWM is used to control the motors.
    
5.  **UART (Universal Asynchronous Receiver/Transmitter)**: The UART is used for communication with the user. The system can receive commands over UART and send feedback about the motor speeds, system status, and temperature readings.
    
6.  **SPI (Serial Peripheral Interface)**: The SPI is used to control an LCD display, which shows the system status and motor speeds.
    

The system operates in one of three modes: control mode, safe mode, and timeout mode. In control mode, the system processes commands to control the motors. In safe mode, triggered by pressing a button, the motors are disabled. In timeout mode, if no commands are received for a certain period, the motors are also disabled. The system status is displayed on the LCD and indicated by an LED.

## Authors

-   [aldoprogra](https://github.com/aldoprogra)
-   [Imdimark](https://github.com/Imdimark)
-   [youssefattia98](https://github.com/youssefattia98)

Please refer to the source code for more details about the implementation.
