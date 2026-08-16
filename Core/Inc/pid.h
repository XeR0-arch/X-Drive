/**
 * @file    pid.h
 * @brief   Velocity PID controller prototypes and global motor instances
 */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motors.h"

/* ======================================================================== */
/*  Global motor instances — one per wheel                                  */
/* ======================================================================== */
extern Motor_t motorFL;    /* Front-Left   (TIM5) */
extern Motor_t motorFR;    /* Front-Right  (TIM2) */
extern Motor_t motorBL;    /* Back-Left    (TIM3) */
extern Motor_t motorBR;    /* Back-Right   (TIM4) */

/* ======================================================================== */
/*  Function prototypes                                                     */
/* ======================================================================== */

/**
 * @brief  Initialise a motor's PID state and gains.
 * @param  motor  Pointer to motor struct to initialise
 * @param  side   Which wheel this motor drives
 * @param  kp     Proportional gain
 * @param  ki     Integral gain
 * @param  kd     Derivative gain
 */
void PID_Init(Motor_t *motor, MotorSide_t side, float kp, float ki, float kd);

/**
 * @brief  Execute one PID iteration.
 *         Reads act_rpm_filtered, computes PID output, calls Motors_SetSpeed().
 */
void PID_Controller(Motor_t *motor);

/** Enable PID control for this motor.  */
void PID_Enable(Motor_t *motor);

/** Disable PID control for this motor (output will freeze). */
void PID_Disable(Motor_t *motor);

/** Check if PID is enabled for this motor. */
bool PID_IsEnabled(Motor_t *motor);

/** Reset PID error accumulators and output. */
void PID_ResetState(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
