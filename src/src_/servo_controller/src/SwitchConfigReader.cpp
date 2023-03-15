#include "servo_controller/SwitchConfigReader.hpp"
#include <iostream>
#include <stdint.h>

using namespace std;


std::vector<SwitchConfig_t> SwitchConfigReader::readSwitchConfig(std::string pathToConfig){
    vector<SwitchConfig_t> configs;
    YAML::Node configFile = YAML::LoadFile(pathToConfig);
    auto servoConfigs = configFile["servomotor_configurations"];
    auto switchConfigs = configFile["switch_configurations"];

    for(auto currentConfig : switchConfigs){
        SwitchConfig_t config;
        auto  states = make_shared<unordered_map<string,uint16_t>>();
        auto servoName = currentConfig["servo_config"].as<string>();
        auto servoConfig = servoConfigs[servoName];
        
        config.switch_name = currentConfig["name"].as<string>();
        config.initial_state = currentConfig["initial_state"].as<string>();

        config.servomotor.pin_number = currentConfig["pwm_pin"].as<uint8_t>();
        config.servomotor.actuation_range = servoConfig["actuation_range"].as<uint16_t>();
        config.servomotor.min_pulse = servoConfig["min_pulse"].as<uint16_t>();
        config.servomotor.max_pulse = servoConfig["max_pulse"].as<uint16_t>();
        config.servomotor.working_frequency = servoConfig["frequency"].as<uint16_t>();

        for(auto it = currentConfig["states"].begin(); it !=currentConfig["states"].end(); it++){
            states->insert({it->first.as<string>(),it->second.as<uint16_t>()});
        }
        config.states = states;
        configs.push_back(config);
    }

    return configs;
}
