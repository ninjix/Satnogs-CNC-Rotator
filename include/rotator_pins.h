/*!
* @file rotator_pins.h
*
* It is a header file for pin mapping.
*
* Licensed under the GPLv3
*
*
*  Adruino CNC shield v3 pinout
*  Based on GRBL pinout https://github.com/grbl/grbl/issues/390
* 
* A0   Reset/Abort 
* A1   Feed Hold 
* A2   Cycle Start/Resume
* A3   Coolant Enable
* A4   <RESERVED> SDA
* A5   <RESERVED> SCL
* 
* D2   Step pulse x-axis
* D3   Step pulse y-axis
* D4   Step pulse z-axis
* D5   Direction x-axis
* D6   Direction y-axis
* D7   Direction z-axis
* D8   Stepper enable/disable
* D9   Limit x-axis
* D10  Limit y-axis
* D11  Limit z-axis
* D12  Spindle enable
* D13  Spindle direction
* 
*/

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

// Azimuth Motor (Y-Axis)
#define M1IN1 3 ///< Motor 1 PWM pin, pulse
#define M1IN2 6 ///< Motor 1 PWM pin, direction
// #define M1SF 7  ///< Motor 1 digital input
// #define M1FB A1 ///< Motor 1 analog input

// Elevation Motor (X-Axis)
#define M2IN1 2 ///< Motor 2 PWM pin, pulse
#define M2IN2 5 ///< Motor 2 PWM pin, direction
// #define M2SF 7  ///< Motor 2 digital input
// #define M2FB A0 ///< Motor 2 analog input

#define MOTOR_EN 8 ///< Digital output, to enable the motors

#define SW1 10  ///< Digital input, to read the status of end-stop for motor 1 (limit-y-axis)
#define SW2 9 ///< Digital input, to read the status of end-stop for motor 2 (limit-x-axis)

#define RS485_DIR 2 ///< Digital output, to set the direction of RS485 communication

#define SDA_PIN 3 ///< I2C data pin
#define SCL_PIN 4 ///< I2C clock pin

#define PIN12 12 ///< General purpose I/O pin
#define PIN13 13 ///< General purpose I/O pin
#define A2 A2    ///< General purpose I/O & analog pin
#define A3 A3    ///< General purpose I/O & analog pin

#endif /* ROTATOR_PINS_H_ */
