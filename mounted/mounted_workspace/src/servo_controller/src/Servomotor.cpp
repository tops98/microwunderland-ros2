#include "servo_controller/Servomotor.hpp"

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

    pwmController_->setPinToPwmMode(servoPin);
    pwmController_->setPwmFrequency(frequency_);
    recalculatePulseLengthPerDegreeConst();
}

void Servomotor::recalculatePulseLengthPerDegreeConst(){
    uint32_t range = maxPulse_ - minPulse_;
    pulseLengthPerDegree_ = ((_Float64)range / actuationRange_);
}

void Servomotor::setAngle(uint16_t angle){
    uint32_t pulse = pulseLengthPerDegree_* angle + minPulse_;
    pwmController_->setPulseWidth(servoPin_, pulse);
}