/**
 * @file example_modes.h
 * @brief STS3032 Servo Driver - Additional Examples
 * 
 * This header file contains declarations for various demonstration examples
 * showing different use cases of the STS3032 servo driver.
 * 
 * These are OPTIONAL examples. The basic usage is shown in main.c
 */

#ifndef EXAMPLE_MODES_H
#define EXAMPLE_MODES_H

/**
 * @brief Configuration Example
 * 
 * Use this example when setting up new servos or changing IDs.
 * Demonstrates:
 * - Using the wizard to assign specific IDs
 * - Auto-detecting all servos on the bus
 * - Manually changing servo IDs
 * 
 * To use: Include this file and call example_servo_configuration()
 */
void example_servo_configuration(void);

/**
 * @brief Single Servo Test Example
 * 
 * A simple test showing how to control a single servo.
 * Demonstrates:
 * - Scanning for servos
 * - Enabling torque
 * - Moving to different positions
 * - Reading current position
 * 
 * To use: Include this file and call example_single_servo()
 */
void example_single_servo(void);

/**
 * @brief Multi-Servo Example
 * 
 * Shows how to control multiple servos simultaneously.
 * Demonstrates:
 * - Managing multiple servo IDs
 * - Coordinated movement patterns
 * - Sequential positioning
 * - Creating smooth motion sequences
 * 
 * To use: Include this file and call example_multi_servo()
 */
void example_multi_servo(void);

#endif // EXAMPLE_MODES_H
