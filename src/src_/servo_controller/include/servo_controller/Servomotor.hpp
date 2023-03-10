#ifndef SERVOMOTOR_HPP
#define SERVOMOTOR_HPP

// package
#include <servo_controller/AbstractPwmController.hpp>
// std
#include <memory>
#include <stdexcept>


class Servomotor{
    //member varibales:
    private:
        // microseconds/degree
        _Float64 pulseLengthPerDegree_;
        // gpio pin number of servo pin
        uint8_t servoPin_;
        // min pulse length for min position in microseconds
        uint32_t minPulse_;
        // max pulse length for max position in microseconds
        uint32_t maxPulse_;
        // actuation range in degree
        uint16_t actuationRange_;
        // frequency in hz
        uint32_t frequency_;
        // pointer to pwm controller implementation
        std::shared_ptr<AbstractPwmController> pwmController_;
    
    // methods:
    public:
        Servomotor(
            std::shared_ptr<AbstractPwmController> pwmController, uint8_t servoPin,
            uint32_t minPulse = 500,
            uint32_t maxPulse = 2500,
            uint16_t actuationRange = 180,
            uint32_t frequnecy = 50);

        /**
         * Sets the new position of the servomotor
         * @param angle new position in degree
         * @throw invalid_argument exception if angle is not within actuation range
        */
        void setAngle(uint16_t angle);

        /**
         * Get actuation range of the servomotor
         * @return range in degree
        */
        uint16_t getActuationRange();

    private:
        /**
         * Recalculates the factor used for converting angle degrees
         * to a pulse length in microseconds
        */
        void recalculatePulseLengthPerDegreeConst(); 
};
#endif