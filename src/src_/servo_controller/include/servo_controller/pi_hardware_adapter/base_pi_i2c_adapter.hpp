#ifndef BASE_PI_I2C_ADAPTER_HPP
#define BASE_PI_I2C_ADAPTER_HPP

#include <servo_controller/utils/DisableWaring.h>


class BasePIi2cAdapter{

    private:
        static BasePIi2cAdapter* instance_;
    public:
        static BasePIi2cAdapter* getInstance();

DISABLE_WARNING_PUSH
DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER

        virtual void setup(int deviceId) {
        };
        virtual int write (int data){
			return 0;
		};
        virtual int read (int fd){
			return 0;
		};
        virtual int writeReg8 (int reg, int data){
			return 0;
		};
        virtual int writeReg16 (int reg, int data){
			return 0;
		};
        virtual int readReg8 (int reg){
			return 0;
		};
        virtual int readReg16 (int reg){
			return 0;
		};

DISABLE_WARNING_POP

    protected:
        BasePIi2cAdapter(){};
};
#endif