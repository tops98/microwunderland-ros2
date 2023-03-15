#include "servo_controller/Switch.hpp"
// std
#include <stdexcept>

using namespace std;


Switch::Switch(
shared_ptr<Servomotor> servo,
unordered_map<string,uint16_t> states,
string initialState){
    
    if(servo == nullptr){
        throw invalid_argument("servo motor is null");
    }
    if(states.find(initialState) == states.end()){
        throw invalid_argument("inital state not found in available states");
    }
    checkStates(states);
    states_ = states;
    initialState_ = initialState;
    servo_ =  servo;
    servo_->setAngle(states_[initialState_]);
}

Switch::Switch(std::shared_ptr<Servomotor> servo, SwitchConfig_t config):
Switch(servo, *config.states.get(), config.initial_state){}

void Switch::setState(string state){
    auto stateElement = states_.find(state);
    if( stateElement == states_.end()){
        throw invalid_argument("unkown state"); 
    }
    servo_->setAngle(stateElement->second);
    currentState_ = state;
}

void Switch::setAvailableStates(std::unordered_map<std::string,std::uint16_t> newStates){
    checkStates(newStates);
    states_ = newStates;
}

unordered_map<string,uint16_t> Switch::getAvailableStates(){
    return states_;
}

string Switch::getCurrentState(){
    return currentState_;
}

void Switch::checkStates(const unordered_map<string,uint16_t> &states){
    uint16_t actuationRange = servo_->getActuationRange();
    for(auto it = states.begin(); it != states.end(); it++){
        if(it->second >= actuationRange){
            throw out_of_range("postion of state can not be greater than actuation range");
        }
    }
}