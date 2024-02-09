#include "servo_controller/switch/Switch.hpp"
// std
#include <stdexcept>
#include <iostream>
#include <sstream>

using namespace std;


Switch::Switch(
shared_ptr<Servomotor> servo,
unordered_map<string,uint16_t> states,
string initialState,
std::string name){
    
    checkParams(states, initialState, servo);
    servo_ =  servo;
    states_ = states;
    initialState_ = initialState;
    name_ = name;
    servo_->setAngle(states_[initialState_]);
}

Switch::Switch(std::shared_ptr<Servomotor> servo, SwitchConfig_t config):
Switch(servo, *(config.states.get()), config.initial_state, config.switch_name){}

void Switch::setState(string state){
    auto stateElement = states_.find(state);
    if( stateElement == states_.end()){
        throw invalid_argument("unkown state"); 
    }
    servo_->setAngle(stateElement->second);
    currentState_ = state;
}

string Switch::getName(){
    return name_;
}

string Switch::toString(){
    ostringstream oss;
    oss<<"switch_name: " <<name_<<"\n";
    oss<<"initial_state: "<<initialState_;
    oss<<"current_state: "<<currentState_;
    oss<<"\r\tstates:\n";

    for(auto state : states_){
        oss<<"\r\t\t"<<state.first<<": "<<state.second<<"\n";
    }
    return oss.str();
}

void Switch::setAvailableStates(std::unordered_map<std::string,std::uint16_t> newStates){
    checkStates(newStates,servo_->getActuationRange());
    states_ = newStates;
}

unordered_map<string,uint16_t> Switch::getAvailableStates(){
    return states_;
}

string Switch::getCurrentState(){
    return currentState_;
}

void Switch::checkParams(const unordered_map<string,uint16_t> &states, const string initialState, shared_ptr<Servomotor> servo){
    uint16_t actuationRange = 0;

    // check if servo pointer is correct
    if(servo.get() == nullptr){
        throw invalid_argument("servo motor is null");
    }
    
    actuationRange = servo->getActuationRange();
    
    // check if initial state is a valid state
    if(states.find(initialState) == states.end()){
        throw invalid_argument("inital state not found in available states");
    }

    checkStates(states,actuationRange);
}

void Switch::checkStates(const unordered_map<string,uint16_t> &states, uint16_t actuationRange){
    // check if all states are within actuation range
    for(auto it = states.begin(); it != states.end(); it++){
        if(it->second > actuationRange){
            throw out_of_range("postion of state can not be greater than actuation range");
        }
    }
}