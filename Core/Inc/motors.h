/**
 * @file    motors.h
 * @brief   Motor types, constants, and driver prototypes for 4-wheel X-drive
 *
 * Hardware mapping (from IOC):
 *   Motor            Timer   Encoder Pins         PWM (CH3)   Dir (GPIO)
 *   Front-Left       TIM5    PA0(CH1) + PA1(CH2)  PA2         PA3
 *   Front-Right      TIM2    PA5(CH1) + PB3(CH2)  PB10        PB11
 *   Back-Left        TIM3    PA6(CH1) + PA7(CH2)  PB0         PB1
 *   Back-Right       TIM4    PD12(CH1)+ PD13(CH2) PD14        PD15
 *
 * Each timer uses CH1+CH2 for quadrature encoder. 
 * Speed is controlled via PWM on CH3. Direction is via plain GPIO.
 * TIM14 is the PID control-loop interrupt timer (1 kHz).
 */
#ifndef __MOTORS_H
#define __MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* ======================================================================== */
/*  Motor identification                                                    */
/* ======================================================================== */
typedef enum
{
    MOTOR_FRONT_LEFT = 0,   /* TIM5 */
    MOTOR_FRONT_RIGHT,      /* TIM2 */
    MOTOR_BACK_LEFT,        /* TIM3 */
    MOTOR_BACK_RIGHT,       /* TIM4 */
    MOTOR_COUNT
} MotorSide_t;

/* ======================================================================== */
/*  Motor data structure — one per wheel                                    */
/* ======================================================================== */
typedef struct
{
    MotorSide_t side;
    bool        pidEnable;

    /* PID gains */
    float kp;
    float ki;
    float kd;

    /* Setpoint & measured feedback */
    float set_rpm;
    float act_rpm;
    float act_rpm_filtered;
    float prev_rpm;

    /* PID error state */
    float e;
    float e_prev;
    float e_total;
    float out;          /* normalised output [-1.0 … +1.0] */

    /* Encoder state */
    int32_t enc;
    int32_t encPrev;
    int32_t encDiff;
    float   dist;       /* distance this step (mm) */
    float   totalDist;  /* cumulative distance (mm) */
} Motor_t;

/* ======================================================================== */
/*  Hardware constants  — ADJUST THESE FOR YOUR ROBOT                       */
/* ======================================================================== */

/** Encoder counts per revolution (with 4× quadrature decoding).
 *  Example: 360 PPR encoder → 1440 CPR in TI12 mode.
 *  TODO: set this to your actual encoder CPR. */
#define ENCODER_CPR             260.0f

/** PID loop period in seconds (1 kHz → 1 ms). */
#define PID_TIME_STEP           0.001f

/** Motor deadband as a fraction of max PWM [0.0 … 1.0].
 *  Below this duty cycle the motor stalls, so we snap to zero.
 *  Reference used 1200/5000 ≈ 0.24.  Tune per your motors. */
#define MOTOR_DEADBAND_FRAC     0.24f

/** Wheel physical parameters (adjust to your robot). */
#define WHEEL_DIAMETER_MM       60.0f
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265f)

/** Integral clamp for PID anti-windup. */
#define PID_INTEGRAL_CLAMP      1750.0f

/* ---- Math helpers ---- */
#define DEG_TO_RAD  (3.14159265f / 180.0f)
#define RAD_TO_DEG  (180.0f / 3.14159265f)

/* ======================================================================== */
/*  Function prototypes                                                     */
/* ======================================================================== */

/**
 * @brief  Initialise all motor peripherals.
 *         Starts encoder timers, PWM outputs, and TIM14 control-loop interrupt.
 */
void Motors_Init(void);

/**
 * @brief  Set motor speed from normalised value.
 * @param  motor  Pointer to motor struct
 * @param  speed  Normalised speed [-1.0 … +1.0]
 *                Positive = forward (CH3=0, CH4=PWM)
 *                Negative = reverse (CH3=PWM, CH4=0)
 */
void Motors_SetSpeed(Motor_t *motor, float speed);

/**
 * @brief  Stop all 4 motors immediately (all PWM outputs to zero).
 */
void Motors_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H */
