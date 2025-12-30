/**
 * @file walk_forward_reaction.h
 * @brief Walk forward animation triggered by front push
 */

#ifndef WALK_FORWARD_REACTION_H
#define WALK_FORWARD_REACTION_H

#include <stdint.h>

/**
 * @brief Execute the walk forward animation for the specified number of cycles
 * @param cycles Number of times to repeat the full animation (default: 3)
 */
void walk_forward_play(uint8_t cycles);

#endif // WALK_FORWARD_REACTION_H
