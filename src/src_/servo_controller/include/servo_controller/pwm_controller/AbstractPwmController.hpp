#ifndef ABSTRACTPWMCONTROLLER_HPP
#define ABSTRACTPWMCONTROLLER_HPP

// std
#include <stdint.h>
#include <math.h>

#define DEFAULT_RESOLUTION 4096
#define DEFAULT_PWM_FREQU 50
#define GET_DIVIDER_FOR_FREQ(baseFreq,newFreq,div1) std::round((_Float64)baseFreq / ( newFreq * div1))-1

using namespace std;


class AbstractPwmController{
    //member varibales:
    protected:
        // frequnecy of the clock used for pwm control
        uint32_t oscilatorFrequency_;
        // divider used for mapping the oscilatorFrequency_ to target frequency and resolution
        uint32_t prescaler_;
        // the resultution of pwm signal
        uint32_t resolution_;
        // factor for converting pulseWidth to resulution
        _Float64 unitsPerMicrosecond_;

    // methods:
    public:
        AbstractPwmController(
            uint64_t oscilatorFrequency,
            uint32_t pwmFrequency=DEFAULT_PWM_FREQU,
            uint32_t resolution=DEFAULT_RESOLUTION);

        /**
         * Set base frequency of the pwm signal.
         * The prescaler value will be ajusted automaticly.
         * NOTE: The actual frequency might deviate sligthly from the
         * desired frequency due to rounding errors.
         * @param frequency Target base frequeny.
         */
        void setPwmFrequency(uint32_t frequency);

        /**
         * Returns the base frequency of the pwm controller.
         */
        _Float64 getFrequency();


        /**
         * Returns the bresolution of the pwm controller.
         */
        uint32_t getResolution();

        /**
         * Enables or disables pwm on a given pin.
         * @param pin selected gpio pin
         * @param pwmOn true= pwm on; false= pwm off
        */
        virtual void enablePwmPin(uint8_t pin, bool pwmOn)=0;
        
        /**
         * Set target pulse width on the selected pwm output.
         * Note: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pulseWidth Pulse width in microseconds.
         * @param pin Selected PWM pin.
         */
        virtual void setPulseWidth( uint8_t pin, uint32_t pulseWidth)=0;

    protected:
        /**
         * Sets the value of the prescaler register
         * @param prescalerVal value of the prescaler register
        */
        virtual void setPrescaler(uint32_t prescalerVal)=0;

        /**
         * Sets the value of the the range register
         * @param resolution resolution/range
        */
        virtual void setResolution(uint32_t resolution)=0;

        /**
         * Converts a desired Pulse width in microseconds
         * to a number within the range of the available resolution.
         * @param pulseWidth Pulse width in microseconds.
         * @return Pulse width maped to a value within 0 and resolution
         */
        uint32_t mapPulseWidthToResulution(uint32_t pulseWidth);

        /**
         * Recalculates the constant used for the conversion
         * of a given pulse width to the choosen resultution 
         * of the pwm signal
        */
        void recalculateUnitsPerMicrosecond();                
};
#endif