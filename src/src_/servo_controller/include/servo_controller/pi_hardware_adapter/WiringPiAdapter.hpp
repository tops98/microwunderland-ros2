#ifndef WIRINGPIADAPTER_HPP
#define WIRINGPIADAPTER_HPP

#include <wiringPi.h>
#include <servo_controller/pi_hardware_adapter/base_pi_hardware_adapter.hpp>


class WiringPiAdapter: public BasePiHardwareAdapter{


    private:
        static WiringPiAdapter* instance_;
    public:
        static  WiringPiAdapter* getInstance();

        void setup (void) override{ 
            if (wiringPiSetup () == -1){
                throw "setting up wiringPi failed!";
            }
        };

        // Base functions
        void gpio_pinMode (int pin, int mode) override{ 
            pinMode(pin, mode);
        };
        void gpio_pullUpDownControl(int pin, int pud) override{ 
            pullUpDnControl(pin,pud);
        };
        void gpio_digitalWrite (int pin, int value) override{ 
            digitalWrite(pin, value);
        };
        void gpio_pwmWrite (int pin, int value) override{ 
            pwmWrite(pin, value);
        };
        int gpio_digitalRead (int pin) override{ 
            return digitalRead(pin);
        };
        int gpio_analogRead (int pin) override{ 
            return analogRead(pin);
        };
        void gpio_analogWrite (int pin, int value) override{
            analogWrite(pin,value);
        };
        // Pi specific functions
        void gpio_digitalWriteByte (int value) override{ 
            digitalWriteByte(value);
        };
        void gpio_pwmSetMode (int mode) override{ 
            pwmSetMode(mode);
        };
        void gpio_pwmSetRange (unsigned int range) override{ 
            pwmSetRange(range);
        };
        void gpio_pwmSetClock (int divisor) override{ 
            pwmSetClock(divisor);
        };
        // Timing
        unsigned int gpio_millis (void) override{ return millis();};
        unsigned int gpio_micros (void) override{ return micros();};
        void gpio_delay (unsigned int howLong) { delay(howLong);};
        void gpio_delayMicroseconds (unsigned int howLong) override{ delayMicroseconds(howLong);};

        private:
            WiringPiAdapter(){};
};
#endif
