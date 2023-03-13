#ifndef SWITCHSERVICE_HPP
#define SWITCHSERVICE_HPP

// ros2
#include <rclcpp/rclcpp.hpp>
// package
#include <servo_controller/Switch.hpp>
#include <servo_controller/srv/get_available_states.hpp>
#include <servo_controller/srv/get_current_state.hpp>
#include <servo_controller/srv/set_state.hpp>
//std
#include <unordered_map>
#include <string>

typedef servo_controller::srv::GetAvailableStates GetAvailableStatesService;
typedef servo_controller::srv::GetCurrentState GetCurrentStateService;
typedef servo_controller::srv::SetState SetStateService;


class SwitchService: public rclcpp::Node{
    //member varibales:
    private:
        std::unordered_map<std::string,std::shared_ptr<Switch>> switches_;

        rclcpp::Service<GetAvailableStatesService>::SharedPtr getAvailableStatesService_;
        rclcpp::Service<GetCurrentStateService>::SharedPtr getCurrentStateService_;
        rclcpp::Service<SetStateService>::SharedPtr setStateService_;
        
    // methods:
    public:
        SwitchService();
    private:
        void getAvailableStatesCallback(const GetAvailableStatesService::Request::SharedPtr request, GetAvailableStatesService::Response::SharedPtr response);
        void getCurrentStateCallback(const GetCurrentStateService::Request::SharedPtr request, GetCurrentStateService::Response::SharedPtr response);
        void setStateCallback(const SetStateService::Request::SharedPtr request, SetStateService::Response::SharedPtr response);
};
#endif