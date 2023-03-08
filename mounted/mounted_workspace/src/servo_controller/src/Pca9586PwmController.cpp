#include "servo_controller/Pca9586PwmController.hpp"


Pca9586PwmController::Pca9586PwmController(
uint8_t deviceId,
uint64_t oscilatorFrequency,
uint32_t pwmFrequency,
uint32_t resolution):
AbstractPwmController(oscilatorFrequency,pwmFrequency,resolution){
    if (wiringPiI2CSetup (deviceId) == -1){
        throw "setting up wiringPiI2C failed!";
    }
}

void Pca9586PwmController::setPwmFrequency(uint32_t frequency, uint32_t resolution){
    AbstractPwmController::setPwmFrequency(frequency,resolution);
    setPrescaler(prescaler_);
    setResolution(resolution_);
}

void Pca9586PwmController::setPrescaler(uint32_t prescalerVal){
    
}

void Pca9586PwmController::setResolution(uint32_t resolution){
    
}

void Pca9586PwmController::setPinToPwmMode(uint8_t pin){

}

void Pca9586PwmController::setPulseWidth(uint8_t pin, uint32_t pulseWidth){
    uint32_t pulseInUnits = mapPulseWidthToResulution(pulseWidth);

}