/**
 * @file motor.cpp
 * @author Marko
 * @brief Implementation file for the Motor class and related functions.
 * @version 0.1
 * @date 2024-01-20
 * @copyright Copyright (c) 2024
 */

#include "motor.h"

/**
 * @brief Motor class constructor.
 * @param directionInverse Set to true if motor direction is inverted, false otherwise.
 * @param motorA Motor A pin.
 * @param motorB Motor B pin.
 * @param motorBoostEnable Motor boost enable pin.
 * @param encA Encoder A pin.
 * @param encB Encoder B pin.
 */
Motor::Motor(bool directionInverse, unsigned char motorA, unsigned char motorB, unsigned char motorBoostEnable,
             unsigned char encA, unsigned char encB) : _motorA(motorA),
                                                       _motorB(motorB),
                                                       _motorBoostEnable(motorBoostEnable),
                                                       _encA(encA),
                                                       _encB(encB),
                                                       _dirInverse(directionInverse),
                                                       _lastEncPulse(0),
                                                       pid(MOTOR_PID_P, MOTOR_PID_I, MOTOR_PID_D, -MOTOR_PID_RANGE, MOTOR_PID_RANGE)
{
}

/**
 * @brief Motor class destructor.
 */
Motor::~Motor()
{
}

/**
 * @brief Initializes the motor and encoder pins.
 */
void Motor::init()
{
    pinMode(_motorBoostEnable, OUTPUT);
    pinMode(_encA, INPUT);
    pinMode(_encB, INPUT);
    _setInterrupts();
    disable();
}

/**
 * @brief Enables the motor by configuring the boost enable pin (pin needs to be left floating).
 */
void Motor::enable()
{
    pinMode(_motorBoostEnable, INPUT);
}

/**
 * @brief Disables the motor by setting the boost enable pin to a high level.
 */
void Motor::disable()
{
    pinMode(_motorBoostEnable, OUTPUT);
    digitalWrite(_motorBoostEnable, HIGH);
}

/**
 * @brief Configures PWM settings for the motor.
 */
void Motor::_configurePWM()
{
    pinMode(_motorA, OUTPUT);
    pinMode(_motorB, OUTPUT);
    analogWrite(_motorA, 0);
    analogWrite(_motorB, 0);
}

/**
 * @brief Sets up interrupts for the motor's encoder.
 */
void Motor::_setInterrupts()
{
    attachInterrupt(_encA, encoderInterruptHandler, RISING);
}

/**
 * @brief Sets the motor power and direction based on duty cycle and direction parameters.
 * @param dutyCycle Duty cycle for PWM (0-255).
 * @param direction Motor direction (true for forward, false for reverse).
 */
void Motor::setPower(unsigned char dutyCycle, bool direction)
{
    bool correctedDirection = (!_dirInverse) ? !direction : direction;

    if (correctedDirection)
    {
        analogWrite(_motorA, dutyCycle);
        analogWrite(_motorB, 0);
    }
    else
    {
        analogWrite(_motorA, 0);
        analogWrite(_motorB, dutyCycle);
    }
}

/**
 * @brief Applies brake to the motor by setting both PWM outputs to maximum.
 */
void Motor::brake()
{
    analogWrite(_motorA, 255);
    analogWrite(_motorB, 255);
}

/**
 * @brief Gets the encoder pin based on the specified pin.
 * @param pin Pin number (default is PIN_A).
 * @return Encoder pin corresponding to the specified pin.
 */
unsigned char Motor::getEncPins(unsigned char pin /* = PIN_A */)
{
    return (!pin) ? _encA : _encB;
}

/**
 * @brief Sets the interrupt handler for the encoder.
 * @param handler Function pointer to the interrupt handler.
 */
void Motor::setInterruptHandler(void (*handler)())
{
    encoderInterruptHandler = handler;
}

/**
 * @brief Updates the motor state based on elapsed time using the PID controller.
 * @param time Elapsed time since the last update.
 */
void Motor::tick(unsigned long time)
{
    float pidValue = pid.tick(speed, time);
    if (pidValue <= 0)
    {
        setPower((unsigned char)-pidValue, true);
    }
    else if (pidValue > 0)
    {
        setPower((unsigned char)pidValue, false);
    }
}

/**
 * @brief Interrupt handler for motor0's encoder.
 */
void motor0InterruptHandler()
{
    bool direction;
    if (digitalRead(motor0.getEncPins(PIN_B)))
    {
        direction = false;
    }
    else
    {
        direction = true;
    }

    unsigned long now = micros();
    double dt = (now - motor0._lastEncPulse);
    motor0._lastEncPulse = now;
    double fdt = dt / TIME_DIV;

    direction = (motor0._dirInverse) ? !direction : direction;
    motor0.speed = (direction) ? -(MOTOR_ENCODER_RADS_PER_PULSE / fdt) : (MOTOR_ENCODER_RADS_PER_PULSE / fdt);
}

/**
 * @brief External instance of the Motor class for general use.
 */
Motor motor0(false);
