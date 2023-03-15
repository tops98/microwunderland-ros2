#ifndef SWITCHCONFIGREADER_HPP
#define SWITCHCONFIGREADER_HPP

#include <yaml-cpp/yaml.h>
#include <servo_controller/Servomotor.hpp>
#include <servo_controller/Switch.hpp>

class SwitchConfigReader{
    
    public:
        static std::vector<SwitchConfig_t> readSwitchConfig(std::string pathToConfig);
    private:
        SwitchConfigReader(){};
        
};
#endif