/**
 * @file    pid.c
 * @brief   Velocity PID controller for 4-wheel X-drive
 *
 * Classic PID on RPM error with integral wind-up clamping.
 * Output is normalised to [-1.0 … +1.0] for the motor driver.
 *
 * Ported from 2-wheel reference code — identical PID algorithm,
 * now applied independently to each of the 4 wheels.
 */
#include "pid.h"
#include "motors.h"

/* ======================================================================== */
/*  Global motor instances                                                  */
/* ======================================================================== */
Motor_t motorFL;    /* Front-Left   (TIM5) */
Motor_t motorFR;    /* Front-Right  (TIM2) */
Motor_t motorBL;    /* Back-Left    (TIM3) */
Motor_t motorBR;    /* Back-Right   (TIM4) */

/* ======================================================================== */
/*  PID init / controller                                                   */
/* ======================================================================== */

void PID_Init(Motor_t *motor, MotorSide_t side, float kp, float ki, float kd)
{
    motor->side      = side;
    motor->pidEnable = false;
    motor->kp        = kp;
    motor->ki        = ki;
    motor->kd        = kd;

    motor->set_rpm          = 0.0f;
    motor->act_rpm          = 0.0f;
    motor->act_rpm_filtered = 0.0f;
    motor->prev_rpm         = 0.0f;
    motor->e                = 0.0f;
    motor->e_prev           = 0.0f;
    motor->e_total          = 0.0f;
    motor->out              = 0.0f;
    motor->enc              = 0;
    motor->encPrev          = 0;
    motor->encDiff          = 0;
    motor->totalDist        = 0.0f;
    motor->dist             = 0.0f;
}

/**
 * @brief  Execute one PID iteration for a motor.
 *
 * Algorithm:
 *   error       = set_rpm − act_rpm_filtered
 *   integral   += error  (clamped to ±PID_INTEGRAL_CLAMP)
 *   derivative  = (error − prev_error) / dt
 *   output      = Kp·e + Ki·integral·dt + Kd·derivative
 *
 * Output is clamped to [-1.0, +1.0] then sent to Motors_SetSpeed().
 * The sign inversion (−output) matches the reference code convention.
 */
void PID_Controller(Motor_t *motor)
{
    motor->e_prev  = motor->e;
    motor->e       = motor->set_rpm - motor->act_rpm_filtered;
    motor->e_total += motor->e;

    /* Integral clamping (anti-windup) */
    if (motor->e_total > PID_INTEGRAL_CLAMP)
        motor->e_total = PID_INTEGRAL_CLAMP;
    else if (motor->e_total < -PID_INTEGRAL_CLAMP)
        motor->e_total = -PID_INTEGRAL_CLAMP;

    motor->out = motor->kp * motor->e
               + motor->ki * motor->e_total * PID_TIME_STEP
               + motor->kd * (motor->e - motor->e_prev) / PID_TIME_STEP;

    /* Clamp output to normalised range */
    if (motor->out > 1.0f)
        motor->out = 1.0f;
    else if (motor->out < -1.0f)
        motor->out = -1.0f;

    Motors_SetSpeed(motor, -motor->out);
}

/* ======================================================================== */
/*  PID enable / disable / query / reset                                    */
/* ======================================================================== */

void PID_Enable(Motor_t *motor)
{
    motor->pidEnable = true;
}

void PID_Disable(Motor_t *motor)
{
    motor->pidEnable = false;
}

bool PID_IsEnabled(Motor_t *motor)
{
    return motor->pidEnable;
}

void PID_ResetState(Motor_t *motor)
{
    motor->e       = 0.0f;
    motor->e_prev  = 0.0f;
    motor->e_total = 0.0f;
    motor->out     = 0.0f;
}
