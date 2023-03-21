#ifndef SWITCHSERVICE_HPP
#define SWITCHSERVICE_HPP

// ros2
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
// package
#include <servo_controller/switch/Switch.hpp>
#include <servo_controller/srv/get_available_states.hpp>
#include <servo_controller/srv/get_current_state.hpp>
#include <servo_controller/srv/set_state.hpp>
//std
#include <unordered_map>
#include <string>
#include <vector>

typedef servo_controller::srv::GetAvailableStates GetAvailableStatesService;
typedef servo_controller::srv::GetCurrentState GetCurrentStateService;
typedef servo_controller::srv::SetState SetStateService;

#define HELP_TEXT "\
Switch service node\n\
Usage: ros2 run servo_controller SwitchService [path to switch config] [pwm controller type: pi| pca9586 | mockup] [i2c adress in hex if pca9586 is used]\n\n\
This node creates a service to remote control all in the switch_config.yaml file\
specified switches if connected to the device\n"


typedef struct arguments_t{
    std::shared_ptr<AbstractPwmController> pwm_controller;
    std::string path_to_config;
    string erro_msg;
    bool success;
}arguments_t;


class SwitchService: public rclcpp::Node{
    //member varibales:
    private:
        std::unordered_map<std::string,std::shared_ptr<Switch>> switches_;

        rclcpp::Service<GetAvailableStatesService>::SharedPtr getAvailableStatesService_;
        rclcpp::Service<GetCurrentStateService>::SharedPtr getCurrentStateService_;
        rclcpp::Service<SetStateService>::SharedPtr setStateService_;
        
    // methods:
    public:
        SwitchService(std::unordered_map<std::string,std::shared_ptr<Switch>> switches);
    private:
        void getAvailableStatesCallback(const GetAvailableStatesService::Request::SharedPtr request, GetAvailableStatesService::Response::SharedPtr response);
        void getCurrentStateCallback(const GetCurrentStateService::Request::SharedPtr request, GetCurrentStateService::Response::SharedPtr response);
        void setStateCallback(const SetStateService::Request::SharedPtr request, SetStateService::Response::SharedPtr response);
};
#endif