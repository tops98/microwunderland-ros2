#include "servo_controller/pwm_controller/PiPwmController.hpp"


PiPwmController::PiPwmController(
uint64_t oscilatorFrequency,
uint32_t pwmFrequency,
uint32_t resolution):
AbstractPwmController(oscilatorFrequency,pwmFrequency,resolution){
    hardwareAdapter_ = GET_HARDWARE_ADAPTER;
    hardwareAdapter_->setup();
}

void PiPwmController::setPwmFrequency(uint32_t frequency){
    AbstractPwmController::setPwmFrequency(frequency);
    setPrescaler(prescaler_);
    setResolution(resolution_);
}

void PiPwmController::setPwmMode(EpwmMode pwmMode){
    hardwareAdapter_->gpio_pwmSetMode(pwmMode);
}

void PiPwmController::setPrescaler(uint32_t prescalerVal){
    hardwareAdapter_->gpio_pwmSetClock(prescalerVal);
}

void PiPwmController::setResolution(uint32_t resolution){
    hardwareAdapter_->gpio_pwmSetRange(resolution-1);
}

void PiPwmController::enablePwmPin(uint8_t pin, bool pwmOn){
    if(pwmOn){
        hardwareAdapter_->gpio_pinMode(pin,PWM_OUTPUT);
        setPwmMode(EpwmMode::strict);
        setPrescaler(prescaler_);
        setResolution(resolution_);
    }else{
        hardwareAdapter_->gpio_pinMode(pin,INPUT);
    }
}

void PiPwmController::setPulseWidth(uint8_t pin, uint32_t pulseWidth){
    uint32_t pulseInUnits = mapPulseWidthToResulution(pulseWidth);
    hardwareAdapter_->gpio_pwmWrite(pin,pulseInUnits);
}