#ifndef MOCKUPPWMCONTROLLER_HPP
#define MOCKUPPWMCONTROLLER_HPP

#include <servo_controller/pwm_controller/AbstractPwmController.hpp>
#include <servo_controller/utils/DisableWaring.h>
#include <iostream>


class MockupPwmController: public AbstractPwmController{
    //member varibales:
    private:
        
    public:
        
    // methods:
    public:
        MockupPwmController()
        :AbstractPwmController(19200,DEFAULT_PWM_FREQU,DEFAULT_RESOLUTION){};

DISABLE_WARNING_PUSH
DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER
        /**
         * Enables or disables pwm on a given pin.
         * @param pin selected gpio pin
         * @param pwmOn true= pwm on; false= pwm off
        */
        virtual void enablePwmPin(uint8_t pin, bool pwmOn){
        };

        /**
         * Set target pulse width on the selected pwm output.
         * Note: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pulseWidth Pulse width in microseconds.
         * @param pin Selected PWM pin.
         */
        void setPulseWidth( uint8_t pin, uint32_t pulseWidth){
        }

    private:
        /**
         * Sets the value of the prescaler register
         * @param prescalerVal value of the prescaler register
        */
        void setPrescaler(uint32_t prescalerVal)override{
            AbstractPwmController::prescaler_=prescalerVal;
        }

        /**
         * Sets the value of the the range register
         * @param resolution resolution/range
        */
        void setResolution(uint32_t resolution) override{
        };     
DISABLE_WARNING_POP   
};
#endif