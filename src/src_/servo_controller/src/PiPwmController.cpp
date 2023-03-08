#include "servo_controller/PiPwmController.hpp"


PiPwmController::PiPwmController(
uint64_t oscilatorFrequency,
uint32_t pwmFrequency,
uint32_t resolution):
AbstractPwmController(oscilatorFrequency,pwmFrequency,resolution){
    if (wiringPiSetup () == -1){
        throw "setting up wiringPi failed!";
    }
}

void PiPwmController::setPwmFrequency(uint32_t frequency, uint32_t resolution){
    AbstractPwmController::setPwmFrequency(frequency,resolution);
    setPrescaler(prescaler_);
    setResolution(resolution_);
}

void PiPwmController::setPwmMode(EpwmMode pwmMode){
    pwmSetMode(pwmMode);
}

void PiPwmController::setPrescaler(uint32_t prescalerVal){
    pwmSetClock(prescalerVal);
}

void PiPwmController::setResolution(uint32_t resolution){
    pwmSetRange(resolution-1);
}

void PiPwmController::setPinToPwmMode(uint8_t pin){
    pinMode(pin,PWM_OUTPUT);
    setPwmMode(EpwmMode::strict);
    setPrescaler(prescaler_);
    setResolution(resolution_);
}

void PiPwmController::setPulseWidth(uint8_t pin, uint32_t pulseWidth){
    uint32_t pulseInUnits = mapPulseWidthToResulution(pulseWidth);
    pwmWrite(pin,pulseInUnits);
}