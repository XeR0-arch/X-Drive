/**
 * @file    encoders.h
 * @brief   Encoder reading and per-motor RPM computation
 */
#ifndef __ENCODERS_H
#define __ENCODERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motors.h"

/**
 * @brief  Read the raw encoder counter for a motor.
 * @param  motor  Pointer to motor struct (uses side to select timer)
 * @return Current counter value as int32_t
 */
int32_t Encoders_GetValue(Motor_t *motor);

/**
 * @brief  Update a single motor's encoder state.
 *         Reads counter, computes delta (with 16-bit wrap handling for
 *         TIM3/TIM4), calculates RPM, and applies low-pass filter.
 *         Call this once per PID period for each motor.
 * @param  motor  Pointer to motor struct to update
 */
void Encoders_Update(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODERS_H */
