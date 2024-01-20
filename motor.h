/**
 * @file motor.h
 * @author Marko
 * @brief Header file for the Motor class and related constants and functions.
 * @version 1.0
 * @date 2024-01-19
 * @copyright Copyright (c) 2024
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "pid.h"
#include "filter.h"

/**
 * @brief Number of counts per turn for the motor encoder.
 */
#define MOTOR_ENCODER_COUNTS_PER_TURN 100.0f

/**
 * @brief Conversion factor from encoder pulses to radians per pulse.
 */
#define MOTOR_ENCODER_RADS_PER_PULSE (2 * PI / MOTOR_ENCODER_COUNTS_PER_TURN)

/**
 * @brief Time division factor for time-related calculations.
 */
#define TIME_DIV 1000000.0f


/**
 * @brief Pin assignments for motor control and encoder.
 */
#define PIN_A 0
#define PIN_B 1

/**
 * @brief PID constants for motor speed control.
 */
#define MOTOR_PID_RANGE 255.0f
#define MOTOR_PID_P 40.0f
#define MOTOR_PID_I 15.0f
#define MOTOR_PID_D 0.1f

/**
 * @brief Motor pin assignments.
 */
#define MOTOR_A 9
#define MOTOR_B 10
#define MOTOR_BOOST_ENABLE 11
#define ENCODER_A 8
#define ENCODER_B 18

/**
 * @brief Interrupt handler for motor0's encoder.
 */
void motor0InterruptHandler();

/**
 * @brief Motor class for controlling a DC motor with an encoder.
 */
class Motor
{
private:
    unsigned char _motorA;           /**< Motor A pin. */
    unsigned char _motorB;           /**< Motor B pin. */
    unsigned char _motorBoostEnable; /**< Motor boost enable pin. */
    unsigned char _encA;             /**< Encoder A pin. */
    unsigned char _encB;             /**< Encoder B pin. */

    /**
     * @brief Configures PWM settings for the motor.
     */
    void _configurePWM();

    /**
     * @brief Sets up interrupts for the motor's encoder.
     */
    void _setInterrupts();

public:
    /**
     * @brief Constructor for the Motor class.
     * @param directionInverse Set to true if motor direction is inverted, false otherwise.
     * @param motorA Motor A pin.
     * @param motorB Motor B pin.
     * @param motorBoostEnable Motor boost enable pin.
     * @param encA Encoder A pin.
     * @param encB Encoder B pin.
     */
    Motor(bool directionInverse = false, unsigned char motorA = MOTOR_A, unsigned char motorB = MOTOR_B,
          unsigned char motorBoostEnable = MOTOR_BOOST_ENABLE, unsigned char encA = ENCODER_A, unsigned char encB = ENCODER_B);

    /**
     * @brief Destructor for the Motor class.
     */
    ~Motor();

    /**
     * @brief Function pointer for the encoder interrupt handler.
     */
    void (*encoderInterruptHandler)();

    /**
     * @brief Initializes the motor and encoder.
     */
    void init();

    /**
     * @brief Sets the motor power and direction.
     * @param dutyCycle Duty cycle for PWM (0-255).
     * @param direction Motor direction (true for forward, false for reverse).
     */
    void setPower(unsigned char dutyCycle, bool direction);

    /**
     * @brief Enables the motor.
     */
    void enable();

    /**
     * @brief Disables the motor.
     */
    void disable();

    /**
     * @brief Applies brake to the motor.
     */
    void brake();

    /**
     * @brief Updates the motor state based on elapsed time.
     * @param time Elapsed time since the last update.
     */
    void tick(unsigned long time);

    /**
     * @brief Sets the interrupt handler for the encoder.
     * @param handler Function pointer to the interrupt handler.
     */
    void setInterruptHandler(void (*handler)());

    /**
     * @brief Gets the encoder pin based on the specified pin.
     * @param pin Pin number.
     * @return Encoder pin corresponding to the specified pin.
     */
    unsigned char getEncPins(unsigned char pin);

    PID pid;                           /**< PID controller for motor speed control. */
    float speed;                       /**< Motor speed in radians per second. */
    bool _dirInverse;                  /**< Flag indicating whether motor direction is inverted. */
    unsigned long _lastEncPulse;       /**< Timestamp of the last encoder pulse. */
};

/**
 * @brief External instance of the Motor class for general use.
 */
extern Motor motor0;

#endif
