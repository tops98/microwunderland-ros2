#ifndef BASE_PI_HARDWARE_ADAPTER_HPP
#define BASE_PI_HARDWARE_ADAPTER_HPP

#include <servo_controller/utils/DisableWaring.h>

// Pin modes
#define	INPUT			     0
#define	OUTPUT			     1
#define	PWM_OUTPUT		     2
#define	GPIO_CLOCK		     3
#define	SOFT_PWM_OUTPUT		 4
#define	SOFT_TONE_OUTPUT	 5
#define	PWM_TONE_OUTPUT		 6


class BasePiHardwareAdapter {

    private:
        // pointer to instance of the hardware adapter
        static BasePiHardwareAdapter* instance_;
    
    public:
        /**
         * returns a pointer to the BasePiHardwareAdapter instance.
         * If the instance hasen't been create it will be create
         * @return BasePiHardwareAdapter
        */
        static BasePiHardwareAdapter* getInstance();

// Disable warnigs for unused formal parameter since this just a mockup for testing
DISABLE_WARNING_PUSH
DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER

        /**
         * Initialize the hardware interfaces
         * @return 0 if sucess non zerro digit if error
        */
        virtual void setup() { };
        // Base functions
        
        /**
         * set the mode of a selected gpio pin (e.g INPUT, OUTPUT, PWM)
         * @param pin pin number
         * @param mode selected mode
        */
        virtual void gpio_pinMode (int pin, int mode) { };
        /**
         * sets the pull-up or pull-down resistor mode on the given pin,
         * which should be set as an input
         * @param pin pin number
         * @param pud mode
        */
        virtual void gpio_pullUpDownControl (int pin, int pud) { };
        virtual void gpio_digitalWrite (int pin, int value) { };
        virtual void gpio_pwmWrite (int pin, int value) { };
        virtual int gpio_digitalRead (int pin) { return 0;};
        virtual int gpio_analogRead (int pin) { return 0;};
        virtual void gpio_analogWrite (int pin, int value) { };
        // Pi specific functions
        virtual void gpio_digitalWriteByte (int value) { };
        virtual void gpio_pwmSetMode (int mode) { };
        virtual void gpio_pwmSetRange (unsigned int range) { };
        virtual void gpio_pwmSetClock (int divisor) { };
        // Timing
        virtual unsigned int gpio_millis (void) { return 0;};
        virtual unsigned int gpio_micros (void) { return 0;};
        virtual void gpio_delay (unsigned int howLong) { };
        virtual void gpio_delayMicroseconds (unsigned int howLong) { };

// enable warnings again
DISABLE_WARNING_POP  

    protected:
        BasePiHardwareAdapter(){};
};      
#endif