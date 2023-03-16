#ifndef SERVOMOTOR_HPP
#define SERVOMOTOR_HPP

// package
#include <servo_controller/AbstractPwmController.hpp>
// std
#include <memory>
#include <stdexcept>


typedef struct ServomotorConfig_t{
    std::uint32_t min_pulse;         
    std::uint32_t max_pulse;        
    std::uint16_t actuation_range;
    std::uint32_t working_frequency;
    std::uint8_t  pin_number;
}ServomotorConfig_t;


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

        Servomotor(std::shared_ptr<AbstractPwmController> pwmController, ServomotorConfig_t config);

        ~Servomotor();

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

        /**
         * Set min pulse length
         * @param pulseLength length of the shortest pulse length allowed in microseconds
        */
        void setMinPulse(uint32_t pulseLength);
        /**
         * Set max pulse length
         * @param pulseLength length of the longest pulse length allowed in microseconds
        */
        void setMaxPulse(uint32_t pulseLength);
        /**
         * Sets the Actuation range of the servo
         * @param actuationRange maximal range of motion in degree 
        */
        void setActuationRange(uint16_t actuationRange);
        /**
         * Set the pwm frequency used for communicating with the servomotor
         * @param frequency frequency in hz
        */
        void setFrequency(uint32_t frequency);

    private:
        /**
         * Recalculates the factor used for converting angle degrees
         * to a pulse length in microseconds
        */
        void recalculatePulseLengthPerDegreeConst(); 
};
#endif