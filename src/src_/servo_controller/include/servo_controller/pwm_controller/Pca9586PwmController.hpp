#ifndef PCA9586PWMCONTROLLER_HPP
#define PCA9586PWMCONTROLLER_HPP

// std
#include <unordered_map>
// package
#include <servo_controller/pi_hardware_adapter/hardware_adapter.hpp>
#include <servo_controller/pwm_controller/AbstractPwmController.hpp>
#include <servo_controller/utils/DisableWaring.h>
#include "servo_controller/pwm_controller/PCA9586_Registers.hpp"
// external package

#define PCA_OSCILATOR_FREQ 25000000
#define PCA_PWM_RESOLUTION 4096
#define STARTUP_DELAY 500
#define SLEEP_MODE_ENABLE_BIT_POS 0x4
#define RESTART_MODE_BIT_POS 0x7
#define LED_ALL_CALL_ENABLE_BIT_POS 0x1
#define PWM_FULL_OFF_POS 0x4
#define AUTO_INCREMENT_BIT_POS 0x4
#define PWM_ON_REG_COUNT_MASK 0xf000
#define PWM_OFF_REG_MASK 0xe000
#define DEFAULT_DUTY_CYLCLE_OFFSET 408 // 10% offset

using namespace std;


class Pca9586PwmController: public AbstractPwmController{

    // member variables
    private:
        BasePIi2cAdapter* i2cAdapter_;
        BasePiHardwareAdapter* hardwareAdapter_;
        int deviceFileHandle_;

    // methods:
    public:
        Pca9586PwmController(
            uint8_t deviceId,
            uint64_t oscilatorFrequency=PCA_OSCILATOR_FREQ,
            uint32_t pwmFrequency=DEFAULT_PWM_FREQU);

        /**
         * Set base frequency of the pwm signal.
         * The prescaler value will be ajusted automaticly.
         * NOTE: The actual frequency might deviate sligthly from the
         * desired frequency due to rounding errors.
         * @param frequency Target base frequeny.
         */
        void setPwmFrequency(uint32_t frequency);

        /**
         * Enables or disables pwm on a given pin.
         * @param pin selected gpio pin
         * @param pwmOn true= pwm on; false= pwm off
        */
        void enablePwmPin(uint8_t pin, bool pwmOn) override;
        
        /**
         * Set target pulse width on the selected pwm output.
         * NOTE: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pin Selected PWM pin.
         * @param pulseWidth Pulse width in microseconds.
         * @throw invalid_argument if pulse width is longer than period
         */
        void setPulseWidth( uint8_t pin, uint32_t pulseWidth) override;

        /**
         * Set target pulse width on the selected pwm output, and
         * offset the start and end of the duty cylce.
         * NOTE: The pulse width shoud not be longer than
         * the period of the base frequency.
         * @param pin Selected PWM pin.
         * @param pulseWidth Pulse width in microseconds.
         * @param offset offset in microseconds
         * @throw invalid_argument if pulse width is longer than period
         */
        void setPulseWidth( uint8_t pin, uint32_t pulseWidth, uint16_t offset);

        /**
         * Set the start and stop time of a pwm duty cycle
         * for a selected pwm channel.
         * NOTE: Start and stop time cant be greater or equal
         * to the period length. E.g: for pwm signal with a frequency
         * of 50hz the period is 1000ms/50hz = 20ms meaning that start and
         * stop time have to be smaller than 20ms
         * @param pin Selected PWM pin.
         * @param startTime Start of duty cycle in microseconds.
         * @param stopTime End of duty cycle in microseconds.
         * @throw invalid_argument if start or stop time is greater than 
         * period lentgh
         */
        void setPulse( uint8_t pin, uint16_t startTime, uint16_t stopTime);

    private:

        /**
         * Sets the value of the prescaler register
         * @param prescalerVal value of the prescaler register
        */
        void setPrescaler(uint32_t prescalerVal) override;

DISABLE_WARNING_PUSH
DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER
        /**
         * Sets the value of the the range register
         * NOTE: The PCA9586 does not support variable resolution
         * so calling this function wont do anything!!!
         * @param resolution resolution/range
        */
        void setResolution(uint32_t resolution)override{};
DISABLE_WARNING_POP

        /**
         * Enables or dissables the sleep mode of the pca9685.
         * NOTE: On startup sleep mode is always enabled.
         * NOTE: The pca9685 needs 500us to leave sleep mode.
         * Calls durring the startup will result in undefined behavior
         * @param enable True sleep mode will be enabled; 
         * False sleep mode will be dissabled 
        */
        void setSleepMode(bool enable);

        /**
         * Sets a signle bit in an 8 bit register, while leaving
         * all other bits unchanged
         * @param address address of the reigster
         * @param positon position of the bit
         * @param value value of the bit (true=1, false=0)
        */
        void setSingleBitReg8(uint8_t address, uint8_t position, bool value);

        /**
         * Sets a signle bit in an 8 bit register, while leaving
         * all other bits unchanged
         * @param address address of the reigster
         * @param value value of the register
        */
        void setReg16(uint16_t address, uint16_t value, uint16_t mask=0x0000);

        /**
         * Returns the a structure holding all relevant register for a given pwm channel
         * @param uint8_t channel must be inbetween 0 and 15
         * @return PwmOutput_t
         * @throw invalid_argument error if channel does not exist
        */
        PwmOutput_t getPwmChannelRegisters(uint8_t channel);
        
};
#endif