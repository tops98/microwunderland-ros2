#ifndef SWITCH_HPP
#define SWITCH_HPP

// package
#include <servo_controller/servomotor/Servomotor.hpp>
// std
#include <unordered_map>
#include <string>
#include <memory>
#include <stdint.h>


typedef struct SwitchConfig_t{
    std::string switch_name;
    ServomotorConfig_t servomotor;
    std::string initial_state;
    std::shared_ptr<std::unordered_map<std::string,std::uint16_t>> states;
}SwitchConfig_t;


class Switch{
    //member varibales:
    private:
        std::shared_ptr<Servomotor> servo_;
        std::unordered_map<std::string,std::uint16_t> states_;
        std::string currentState_;
        std::string initialState_;
        std::string name_;
        
    // methods:
    public:
        Switch(
            std::shared_ptr<Servomotor> servo,
            std::unordered_map<std::string,std::uint16_t> states,
            std::string initialState,
            std::string name
            );
        
        Switch(std::shared_ptr<Servomotor> servo, SwitchConfig_t config);

        /**
         * Changes the state of the switch to a desired new State
         * @param state name of the new state
         * @throws invalid_argument if the name of the state is unkown 
        */
        void setState(std::string state);

        /**
         * Replace the old list of states with a new one
         * @param newStates map with all possible states composed of a name and a position
        */
        void setAvailableStates(std::unordered_map<std::string,std::uint16_t> newStates);

        /**
         * Get all available states of the switch.
         * States are composed of a name and a corresponding position
         * @return unordered_map<std::string,std::uint16_t>
        */
        std::unordered_map<std::string,std::uint16_t> getAvailableStates();
        
        /**
         * Get the name of thr currently active state of the switch
         * @return string
        */
        std::string getCurrentState();

        /**
         * Returns the name of the switch
         * @return string
        */
        std::string getName();

        /**
         * Returns the configuration of the switch as a string
         * @return string
        */
        std::string toString();

    private:
        /**
         * Checks if all params of the switch are okay
         * @param states states to be checked
         * @param initialState initial state of the switch
         * @param servo servomotor used
         * @throw out_of_range when a states position is not within 
         * the servomotors actuation range
         * @throw invlaid_argument when initial state can not be found in states,
         * or servomor is a nullptr
         * 
        */
        void checkParams(const unordered_map<string,uint16_t> &states, const string initialState, shared_ptr<Servomotor> servo);

        /**
         * Checks if all states are within the actuation range of the used servo
         * @param states states to be checked
         * @param actuationRange actuation range of the servo
         * @throw out_of_range when a states position is not within 
         * the servomotors actuation range
        */
        void checkStates(const unordered_map<string,uint16_t> &states, uint16_t actuationRange);
        
};
#endif