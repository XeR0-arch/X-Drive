/**
 * @file    encoders.c
 * @brief   Encoder reading and per-motor RPM computation (4-wheel X-drive)
 *
 * Reads quadrature encoder counters from TIM5/TIM2/TIM3/TIM4,
 * computes delta counts (with 16-bit wrap handling for TIM3/TIM4),
 * converts to distance and RPM, and applies a Butterworth-inspired
 * low-pass filter for smooth PID feedback.
 *
 * No odometry / position update here — that belongs in holonomic control.
 */
#include "encoders.h"
#include "tim.h"

/**
 * @brief  Read the raw encoder counter for a motor's timer.
 *
 * Timer mapping:
 *   Front-Left  → TIM5 (32-bit)
 *   Front-Right → TIM2 (32-bit)
 *   Back-Left   → TIM3 (16-bit)
 *   Back-Right  → TIM4 (16-bit)
 */
int32_t Encoders_GetValue(Motor_t *motor)
{
    switch (motor->side)
    {
        case MOTOR_FRONT_LEFT:  return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        case MOTOR_FRONT_RIGHT: return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        case MOTOR_BACK_LEFT:   return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        case MOTOR_BACK_RIGHT:  return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        default:                return 0;
    }
}

/**
 * @brief  Update encoder state, RPM, and filtered RPM for one motor.
 *
 * 1. Read current counter, compute delta from previous
 * 2. For 16-bit timers (TIM3, TIM4): cast delta through (int16_t) to
 *    handle counter wrap at 0xFFFF correctly
 * 3. Convert to distance (mm) and instantaneous RPM
 * 4. Apply 2nd-order Butterworth low-pass filter (~15 Hz cutoff at 1 kHz)
 */
void Encoders_Update(Motor_t *motor)
{
    /* ---- Save previous and read current encoder count ---- */
    motor->encPrev = motor->enc;
    motor->enc     = Encoders_GetValue(motor);

    /* ---- Compute delta with proper wrap handling ---- */
    int32_t raw_diff = motor->enc - motor->encPrev;

    /*
     * TIM3 and TIM4 are 16-bit timers: counter wraps at 0xFFFF.
     * Casting through (int16_t) handles the wrap-around correctly.
     * TIM2 and TIM5 are 32-bit — int32_t subtraction is already correct.
     */
    if (motor->side == MOTOR_BACK_LEFT || motor->side == MOTOR_BACK_RIGHT)
    {
        raw_diff = (int32_t)(int16_t)(uint16_t)raw_diff;
    }

    motor->encDiff = -raw_diff;

    /* ---- Distance this step (mm) ---- */
    motor->dist = (float)motor->encDiff / ENCODER_CPR * WHEEL_CIRCUMFERENCE_MM;
    motor->totalDist += motor->dist;

    /* ---- Instantaneous RPM ---- */
    /* RPM = (counts / step) × (1 / step_time_in_seconds) × (60 / CPR) */
    motor->act_rpm = ((float)motor->encDiff / PID_TIME_STEP * 60.0f) / ENCODER_CPR;

    /* ---- Low-pass filter on RPM ---- */
    /* Butterworth-inspired 2nd-order IIR, ~15 Hz cutoff at 1 kHz sample rate.
     * Coefficients from the reference code. */
    motor->act_rpm_filtered = 0.854f  * motor->act_rpm_filtered
                            + 0.0728f * motor->act_rpm
                            + 0.0728f * motor->prev_rpm;
    motor->prev_rpm = motor->act_rpm;
}
