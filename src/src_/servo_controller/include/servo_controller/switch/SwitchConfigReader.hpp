#ifndef SWITCHCONFIGREADER_HPP
#define SWITCHCONFIGREADER_HPP

#include <yaml-cpp/yaml.h>
#include <servo_controller/servomotor/Servomotor.hpp>
#include <servo_controller/switch/Switch.hpp>

class SwitchConfigReader{
    
    public:
        static std::vector<SwitchConfig_t> readSwitchConfig(std::string pathToConfig);
    private:
        SwitchConfigReader(){};
        
};
#endif