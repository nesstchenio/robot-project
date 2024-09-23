# Robot Project

## Project Overview
This project involves the design, programming, and testing of an autonomous robot that interacts with its environment using integrated sensors and actuators. The robot is programmed to detect obstacles and navigate towards them, with specific speed and distance requirements.

The main goal of the robot is to perform the following tasks:

- Start upon pressing a blue push button.
- Perform a battery check every 5 seconds.
- Detect obstacles and move towards them at a speed of 20 cm/s.
- Stop at a distance of 20 cm from the obstacle, with a precision of +/- 2 cm.

## Language, Software and Hardware 
- Programming Language: C
- Software Used: STM32CubeIDE
- Hardware: NUCLEO-L476RG microcontroller

## Project Specifications 
### Functional Requirements
- Obstacle Detection: The robot detects obstacles using a sonar sensor, rotating in place until it finds one within a range of 300 cm.
- Movement: Once an obstacle is detected, the robot moves toward it at 20 cm/s, using a PWM frequency of 5 kHz to control its treads.
- Battery Monitoring: The robot performs a battery check every 5 seconds. If the battery voltage falls below 6V, a warning LED is activated.
- Precision: The robot stops when it is 20 cm away from the obstacle, ensuring accuracy within +/- 2 cm.

### Key Features:
- Sonar System: The robot uses sonar to calculate the distance to the obstacle, regularly checking every 100 ms.
- Battery Surveillance: A timer triggers battery checks, and the robot notifies the user if the voltage is insufficient for operation.
- PWM-Controlled Treads: The robot moves at 20 cm/s using PWM with a frequency of 5 kHz. The treads are also responsible for rotating the robot when searching for obstacles.

## System Architecture
### Main Components
- Button: Starts the robot and handles interrupt-based stopping via a push button (blue button).
- Sonar Sensor: Detects obstacles and measures distances.
- PWM Control: Regulates the speed of the robot’s treads to ensure precise movement.
- Battery Monitor (ADC): Monitors the battery voltage and triggers LED warnings when the battery is low.

### Mathematical Calculations
- Battery Monitoring: The ADC is configured to read the battery voltage, converting values into 8-bit digital data. A threshold is set to warn the user when the battery drops below 6V.
- PWM Calculations: The prescaler and timer values are set to generate the 5 kHz PWM needed for precise tread control. The PWM duty cycle is adjusted to achieve the desired speed of 20 cm/s.
- Sonar Distance Calculation: The sonar measures the time taken for ultrasonic waves to reflect back from an obstacle, using this duration to calculate the distance.

## Project Files
- main.c: Contains the full code implementation, including initialization of peripherals and main logic.
- .ioc file: Configuration file used with STM32CubeIDE to set up timers, ADC, PWM, and GPIOs for the push button, treads, and sonar sensor.

## Key Functions:
### Initialization:
Configures the button, ADC, PWM, and sonar sensor.
Sets timers for periodic battery checks and distance measurements.

### Main Loop:
The robot continuously checks for obstacles and moves toward them.
Regularly monitors the battery level and takes action if low.

### Interrupt Handlers:
Handle push-button input to start/stop the robot.
Manage sonar readings and battery checks using timers.

## Compilation and Simulation
To compile the project:
Click the hammer icon in the top left corner of STM32CubeIDE.
Press the green run button to upload the code to the STM32 microcontroller.
Start the robot by pressing the blue push button.

## Future Improvements
There are plans to further enhance the robot’s precision and performance, particularly in optimizing sonar detection accuracy and improving speed control during movement and rotation.

