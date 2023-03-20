#include "servo_controller/servomotor/Servomotor.hpp"

using namespace std;


Servomotor::Servomotor(
std::shared_ptr<AbstractPwmController> pwmController, uint8_t servoPin,
uint32_t minPulse,
uint32_t maxPulse,
uint16_t actuationRange,
uint32_t frequnecy){

    minPulse_ = minPulse;
    maxPulse_ = maxPulse;
    actuationRange_ = actuationRange;
    frequency_ = frequnecy;
    servoPin_ = servoPin;
    pwmController_ = pwmController;

    pwmController_->enablePwmPin(servoPin,true);
    pwmController_->setPwmFrequency(frequency_);
    recalculatePulseLengthPerDegreeConst();
}

Servomotor::Servomotor(std::shared_ptr<AbstractPwmController> pwmController, ServomotorConfig_t config):
Servomotor(
    pwmController,
    config.pin_number,
    config.min_pulse,
    config.max_pulse,
    config.actuation_range,
    config.working_frequency
){}

Servomotor::~Servomotor(){
    pwmController_->enablePwmPin(servoPin_,false);
}

void Servomotor::recalculatePulseLengthPerDegreeConst(){
    uint32_t range = maxPulse_ - minPulse_;
    pulseLengthPerDegree_ = ((_Float64)range / actuationRange_);
}

void Servomotor::setMinPulse(uint32_t pulseLength){
    minPulse_  = pulseLength;
    recalculatePulseLengthPerDegreeConst();
}

void Servomotor::setMaxPulse(uint32_t pulseLength){
    maxPulse_  = pulseLength;
    recalculatePulseLengthPerDegreeConst();
}

void Servomotor::setActuationRange(uint16_t actuationRange){
    if(actuationRange > 360){
        throw invalid_argument("actuation range cant be greater than 360 degrees");
    }
    actuationRange_ = actuationRange;
    recalculatePulseLengthPerDegreeConst();
}

void Servomotor::setFrequency(uint32_t frequency){
    frequency_ = frequency;
    pwmController_->setPwmFrequency(frequency);
}   


void Servomotor::setAngle(uint16_t angle){
    if(angle >= actuationRange_){
        throw invalid_argument("angle must be within actuation range");
    }
    uint32_t pulse = pulseLengthPerDegree_* angle + minPulse_;
    pwmController_->setPulseWidth(servoPin_, pulse);
}

uint16_t Servomotor::getActuationRange(){
    return actuationRange_;
}