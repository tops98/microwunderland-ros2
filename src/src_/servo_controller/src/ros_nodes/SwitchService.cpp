#include "servo_controller/ros_nodes/SwitchService.hpp"
#include <servo_controller/msg/string_uint16_pair.hpp>
#include <servo_controller/msg/switch_config.hpp>
// std
#include <vector>
#include <functional>

typedef servo_controller::msg::StringUint16Pair SwitchState;

using namespace std;


SwitchService::SwitchService(unordered_map<string,shared_ptr<Switch>> switches):
Node("Switch_Service"){

    switches_ = switches;

    getAvailableStatesService_ = create_service<GetAvailableStatesService>("get_available_states",
        bind(&SwitchService::getAvailableStatesCallback,
        this, placeholders::_1, placeholders::_2)
    );

    getCurrentStateService_ = create_service<GetCurrentStateService>("get_current_state",
        bind(&SwitchService::getCurrentStateCallback,
        this, placeholders::_1, placeholders::_2)
    );

    setStateService_ = create_service<SetStateService>("set_state_service",
        bind(&SwitchService::setStateCallback,
        this, placeholders::_1, placeholders::_2)
    );
}

void SwitchService::getAvailableStatesCallback(const GetAvailableStatesService::Request::SharedPtr request,
 GetAvailableStatesService::Response::SharedPtr response){
    vector<SwitchState> states;
    string switchName = request->switch_name;
    
    if( switches_.find(switchName) == switches_.end()){
        response->states = states;
        response->status = 1;
        return;
    }

    auto statesMap = switches_[switchName]->getAvailableStates();
    for(auto it = statesMap.begin(); it != statesMap.end(); it++){
        SwitchState data = SwitchState();
        data.key = it->first;
        data.value = it->second;
        states.push_back(data);
    }
    response->states = states;
    response->status = 0;
}

void SwitchService::getCurrentStateCallback(const GetCurrentStateService::Request::SharedPtr request,
 GetCurrentStateService::Response::SharedPtr response){
    
    if( switches_.find(request->switch_name) == switches_.end()){
        response->state_name = "";
        response->status = 1;
        return;
    }

    string currentState = switches_[request->switch_name]->getCurrentState();
    response->state_name = currentState;
    response->status = 0;
}

void SwitchService::setStateCallback(const SetStateService::Request::SharedPtr request,
 SetStateService::Response::SharedPtr response){
    if( switches_.find(request->switch_name) == switches_.end()){
        response->status = 1;
        return;
    }

    switches_[request->switch_name]->setState(request->state_name);
    response->status = 0;
}

int main(int argc, char* argv[]){
    string configPath = string(argv[1]).substr(string(argv[1]).find("--servo_config"));
    rclcpp::init(argc,argv);
    // rclcpp::spin(make_shared<SwitchService>());
    rclcpp::shutdown();

    return 0;
}