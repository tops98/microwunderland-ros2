#ifndef PIPWMCONTROLLER_HPP
#define PIPWMCONTROLLER_HPP

// package
#include "servo_controller/AbstractPwmController.hpp"
// external package
#include <wiringPi.h>

#define PI_OSCILATOR_FREQ 19200000

using namespace std;


class PiPwmController: public AbstractPwmController{

    /// Possible pwm modes on Pi
    enum EpwmMode{
        strict = PWM_MODE_MS, ///< Strict mode sends pulse at the start of the period
        balanced = PWM_MODE_BAL ///< Balanced mode divides the pulse in smaller pulses and distrinutes the over the period evenly 
    };

    // methods:
    public:
        PiPwmController(
            uint64_t oscilatorFrequency=PI_OSCILATOR_FREQ,
            uint32_t pwmFrequency=DEFAULT_PWM_FREQU,
            uint32_t resolution=DEFAULT_RESOLUTION);

        /**
         * Sets the pwm mode for the pi's pwm controller.
         * The Raspberry pi support strict and Balanced mode.
         * In Strict mode the Pulse is sent at the beginning of the period without
         * interruptions. In Balanced mode the Pulse is choped up into
         * multiple smaller pulses which are getting evenly distributed over the period.
         * The sum of the pulses equals the desired pulse length
        */
        void setPwmMode(EpwmMode pwmMode);

        /**
         * Enables or disables pwm on a given pin.
         * @param pin selected gpio pin
         * @param pwmOn true= pwm on; false= pwm off
        */
        void enablePwmPin(uint8_t pin, bool pwmOn)override;
 
        /**
         * Set target pulse width on the selected pwm output.
         * Note: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pulseWidth Pulse width in microseconds.
         * @param pin Selected PWM pin (numbering according to wiringPi).
         */
        void setPulseWidth( uint8_t pin, uint32_t pulseWidth) override;

    private:
        /**
         * Sets the value of the prescaler register
         * @param prescalerVal value of the prescaler register
        */
        void setPrescaler(uint32_t prescalerVal) override;

        /**
         * Sets the value of the the range register
         * @param resolution resolution/range
        */
        void setResolution(uint32_t resolution) override;
};
#endif