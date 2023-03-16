#include "servo_controller/AbstractPwmController.hpp"


AbstractPwmController::AbstractPwmController(uint64_t oscilatorFrequency, uint32_t pwmFrequency, uint32_t resolution){
    oscilatorFrequency_ = oscilatorFrequency;
    resolution_ = resolution;
    setPwmFrequency(pwmFrequency);
}

void AbstractPwmController::setPwmFrequency(uint32_t frequency){
    prescaler_ = GET_DIVIDER_FOR_FREQ(oscilatorFrequency_,frequency,resolution_);
    recalculateUnitsPerMicrosecond();
    setPrescaler(prescaler_);
    setResolution(resolution_);
}

_Float64 AbstractPwmController::getFrequency(){
    return (_Float64)oscilatorFrequency_/prescaler_/resolution_;
}

uint32_t AbstractPwmController::getResolution(){
    return resolution_;
}

uint32_t AbstractPwmController::mapPulseWidthToResulution(uint32_t pulseWidth){
    return round( pulseWidth * unitsPerMicrosecond_);
}

void AbstractPwmController::recalculateUnitsPerMicrosecond(){
    _Float64 periodLength = 1000000/getFrequency(); // in microseconds
    unitsPerMicrosecond_ = resolution_ / periodLength;
}