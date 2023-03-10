#ifndef SWITCH_HPP
#define SWITCH_HPP

// package
#include <servo_controller/Servomotor.hpp>
// std
#include <unordered_map>
#include <string>
#include <memory>
#include <stdint.h>

class Switch{
    //member varibales:
    private:
        std::shared_ptr<Servomotor> servo_;
        std::unordered_map<std::string,std::uint16_t> states_;
        std::string currentState_;
        std::string initialState_;
        
    // methods:
    public:
        Switch(
            std::shared_ptr<Servomotor> servo,
            std::unordered_map<std::string,std::uint16_t> states,
            std::string initialState);

        /**
         * Changes the state of the switch to a desired new State
         * @param state name of the new state
         * @throws invalid_argument if the name of the state is unkown 
        */
        void setState(std::string state);

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

    private:
        /**
         * Checks if all states are within the actuation range of the used servo
         * @param states states to be checked
         * @throw out_of_range when a states position is not within 
         * the servomotors actuation range
        */
        void checkStates(const std::unordered_map<std::string,std::uint16_t> &states);
        
};
#endif