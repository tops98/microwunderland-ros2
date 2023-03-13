#include "servo_controller/SwitchService.hpp"
#include <servo_controller/msg/string_uint16_pair.hpp>
// std
#include <vector>

typedef servo_controller::msg::StringUint16Pair SwitchState;

using namespace std;


SwitchService::SwitchService(): Node("Switch_Service"){
    getAvailableStatesService_ = create_service<GetAvailableStatesService>("get_available_states",
        std::bind(&SwitchService::getAvailableStatesCallback,
        this, std::placeholders::_1, std::placeholders::_2)
    );

    getCurrentStateService_ = create_service<GetCurrentStateService>("get_current_state",
        std::bind(&SwitchService::getCurrentStateCallback,
        this, std::placeholders::_1, std::placeholders::_2)
    );

    setStateService_ = create_service<SetStateService>("set_state_service",
        std::bind(&SwitchService::setStateCallback,
        this, std::placeholders::_1, std::placeholders::_2)
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
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SwitchService>());
    rclcpp::shutdown();

    return 0;
}