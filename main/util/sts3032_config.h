/**
 * @file sts3032_config.h
 * @brief STS3032 Configuration and Setup Utilities
 * 
 * Provides utilities for servo ID configuration and setup wizards.
 */

#ifndef STS3032_CONFIG_H
#define STS3032_CONFIG_H

#include <stdint.h>

/**
 * @brief Run the ID configuration wizard
 * 
 * Interactive wizard to help assign unique IDs to servos.
 * 
 * Steps:
 * 1. Disconnect ALL servos from bus
 * 2. Connect ONLY the first servo
 * 3. Power cycle the ESP32
 * 4. This wizard will guide you through assigning IDs
 * 5. Repeat for each servo
 * 
 * @param new_id The ID to assign to the currently connected servo
 */
void sts_config_wizard(uint8_t new_id);

/**
 * @brief Run automatic servo detection and configuration
 * 
 * This function will:
 * - Scan the bus for servos
 * - Report which IDs are in use
 * - Optionally guide ID reassignment
 */
void sts_config_auto_detect(void);

#endif // STS3032_CONFIG_H
