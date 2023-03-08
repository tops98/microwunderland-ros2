#ifndef PCA9586PWMCONTROLLER_HPP
#define PCA9586PWMCONTROLLER_HPP

// package
#include "AbstractPwmController.hpp"
// external package
#include <wiringPiI2C.h>

#define PCA_OSCILATOR_FREQ 25000000

using namespace std;


class Pca9586PwmController: public AbstractPwmController{

    // methods:
    public:
        Pca9586PwmController(
            uint8_t deviceId,
            uint64_t oscilatorFrequency=PCA_OSCILATOR_FREQ,
            uint32_t pwmFrequency=DEFAULT_PWM_FREQU,
            uint32_t resolution=DEFAULT_RESOLUTION);
        
        /**
         * Set base frequency of the pwm signal.
         * The prescaler value will be ajusted automaticly.
         * NOTE: The actual frequency might deviate sligthly from the
         * desired frequency due to rounding errors.
         * @param frequency Target base frequeny.
         * @param resolution Target resolution if not set 4096 will be used as default.
         */
        void setPwmFrequency(uint32_t frequency, uint32_t resolution=DEFAULT_RESOLUTION) override;

        /**
         * Sets the selected pin to pwm mode.
         * @param pin selected gpio pin
        */
        void setPinToPwmMode(uint8_t pin) override;
 
        /**
         * Set target pulse width on the selected pwm output.
         * Note: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pulseWidth Pulse width in microseconds.
         * @param pin Selected PWM pin.
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