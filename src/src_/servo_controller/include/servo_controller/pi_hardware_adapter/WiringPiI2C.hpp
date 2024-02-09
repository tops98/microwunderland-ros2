#ifndef WIRINGPII2C_HPP
#define WIRINGPII2C_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <servo_controller/pi_hardware_adapter/base_pi_i2c_adapter.hpp>


class WiringPiI2CAdapter: public BasePIi2cAdapter{
    
    private:
        int deviceFileHandle_ = -1;
        static WiringPiI2CAdapter* instance_;
    
    // methods:
    public:
        static WiringPiI2CAdapter* getInstance();

        void setup(int deviceId) override{
            deviceFileHandle_ = wiringPiI2CSetup (deviceId);
            if (deviceFileHandle_ == -1){
                throw "setting up wiringPiI2C failed!";
            }
        };
        int write (int data) override{
            checkForDevice();
			return wiringPiI2CWrite(deviceFileHandle_,data);
		};
        int read (){
            checkForDevice();
			return wiringPiI2CRead(deviceFileHandle_);
		};
        int writeReg8 (int reg, int data) override{
            checkForDevice();
			return wiringPiI2CWriteReg8(deviceFileHandle_,reg, data);
		};
        int writeReg16 (int reg, int data) override{
            checkForDevice();
			return wiringPiI2CWriteReg16(deviceFileHandle_,reg,data);
		};
        int readReg8 (int reg) override{
            checkForDevice();
			return wiringPiI2CReadReg8(deviceFileHandle_, reg);
		};
        int readReg16 (int reg) override{
            checkForDevice();
			return wiringPiI2CReadReg16(deviceFileHandle_, reg);
		};

    private:
        WiringPiI2CAdapter(){};
        void checkForDevice(){
            if(deviceFileHandle_ == -1){
                throw "no device has been assigned;\
                please call setup(deviceId before using any other method\
                then getInstance)\n";
            }
        };
};
#endif
