/**
 * @file    motors.c
 * @brief   Motor driver for 4-wheel holonomic X-drive
 *
 * Initialises encoder timers, PWM outputs, and the TIM14 control-loop
 * interrupt.  Motors_SetSpeed() drives H-bridge via CH3+CH4 per timer.
 *
 * Timer / pin mapping (from IOC):
 *   Front-Left  : TIM5  — Enc PA0+PA1,  PWM PA2+PA3
 *   Front-Right : TIM2  — Enc PA5+PB3,  PWM PB10+PB11
 *   Back-Left   : TIM3  — Enc PA6+PA7,  PWM PB0+PB1
 *   Back-Right  : TIM4  — Enc PD12+PD13,PWM PD14+PD15
 *   Control loop: TIM14 — 1 kHz interrupt (Prescaler 83, Period 999)
 */
#include "motors.h"
#include "tim.h"
#include <math.h>

/* ======================================================================== */
/*  Internal helpers                                                        */
/* ======================================================================== */

/** Get the HAL timer handle for a motor's timer. */
static TIM_HandleTypeDef* Motors_GetTimerHandle(MotorSide_t side)
{
    switch (side)
    {
        case MOTOR_FRONT_LEFT:  return &htim5;
        case MOTOR_FRONT_RIGHT: return &htim2;
        case MOTOR_BACK_LEFT:   return &htim3;
        case MOTOR_BACK_RIGHT:  return &htim4;
        default:                return NULL;
    }
}

/** Get the max PWM compare value (= timer ARR) for a motor's timer. */
static uint32_t Motors_GetMaxPWM(MotorSide_t side)
{
    TIM_HandleTypeDef *htim = Motors_GetTimerHandle(side);
    if (htim != NULL)
        return __HAL_TIM_GET_AUTORELOAD(htim);
    return 0;
}

/* ======================================================================== */
/*  Public API                                                              */
/* ======================================================================== */

void Motors_Init(void)
{
    /* ---- Start encoder timers (CH1+CH2 quadrature input) ---- */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   /* Front-Left  */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   /* Front-Right */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* Back-Left   */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   /* Back-Right  */

    /* ---- Ensure motors are stopped before enabling PWM ---- */
    Motors_Stop();

    /* ---- Start PWM outputs (CH3 for speed, CH4 is now GPIO dir) ---- */
    /* Ensure compare is 0 before starting PWM to avoid MDDS10 boot error */
    
    /* Front-Left: TIM5 CH3 (PA2), Dir PA3 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    /* Front-Right: TIM2 CH3 (PB10), Dir PB11 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    /* Back-Left: TIM3 CH3 (PB0), Dir PB1 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* Back-Right: TIM4 CH3 (PD14), Dir PD15 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    /* ---- Configure TIM14 for 1 kHz PID control-loop interrupt ----
     * APB1 timer clock = 84 MHz.
     * Prescaler = 83 (already set in CubeMX) → 1 MHz tick.
     * Override period to 999 → 1000 ticks → 1 ms → 1 kHz.
     * (CubeMX default was 65535 → ~15 Hz, too slow for PID.)
     */
    __HAL_TIM_SET_AUTORELOAD(&htim14, 999);
    __HAL_TIM_SET_COUNTER(&htim14, 0);

    /* Enable TIM14 interrupt in NVIC
     * (TIM14 shares IRQ vector with TIM8_TRG_COM on STM32F407) */
    HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

    /* Start TIM14 with update interrupt */
    HAL_TIM_Base_Start_IT(&htim14);
}

/**
 * @brief  Set motor speed from normalised value.
 *
 * Converts [-1.0 … +1.0] to the correct PWM compare value on CH3 and direction on GPIO.
 */
void Motors_SetSpeed(Motor_t *motor, float speed)
{
    TIM_HandleTypeDef *htim = Motors_GetTimerHandle(motor->side);
    if (htim == NULL) return;

    uint32_t maxPwm  = Motors_GetMaxPWM(motor->side);
    uint32_t pwm     = (uint32_t)(fabsf(speed) * (float)maxPwm);
    uint32_t deadband = (uint32_t)((float)maxPwm * MOTOR_DEADBAND_FRAC);

    /* Apply deadband and clamp */
    if (pwm < deadband)
        pwm = 0;
    else if (pwm > maxPwm)
        pwm = maxPwm;

    GPIO_TypeDef* dirPort = NULL;
    uint16_t dirPin = 0;

    switch (motor->side)
    {
        case MOTOR_FRONT_LEFT:  dirPort = GPIOA; dirPin = GPIO_PIN_3; break;
        case MOTOR_FRONT_RIGHT: dirPort = GPIOB; dirPin = GPIO_PIN_11; break;
        case MOTOR_BACK_LEFT:   dirPort = GPIOB; dirPin = GPIO_PIN_1; break;
        case MOTOR_BACK_RIGHT:  dirPort = GPIOD; dirPin = GPIO_PIN_15; break;
        default: return;
    }

    if (speed >= 0.0f)
    {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET); /* Forward */
    }
    else
    {
        HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);   /* Reverse */
    }

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, pwm);
}

void Motors_Stop(void)
{
    /* Zero all PWM outputs — CH3 on every motor timer */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}
