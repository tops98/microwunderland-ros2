#include "servo_controller/ros_nodes/SwitchService.hpp"

// package
#include <servo_controller/msg/string_uint16_pair.hpp>
#include <servo_controller/servomotor/Servomotor.hpp>
#include <servo_controller/pwm_controller/MockupPwmController.hpp>
#include <servo_controller/pwm_controller/Pca9586PwmController.hpp>
#include <servo_controller/pwm_controller/PiPwmController.hpp>
#include <servo_controller/switch/SwitchConfigReader.hpp>
// std
#include <vector>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <iostream>

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

    RCLCPP_INFO(this->get_logger(), "SwitchService ready");
}

void SwitchService::printState(){
    std::ostringstream oss;
    oss<<"+++ Switch service configuration +++\n";
    for( auto sw : switches_){
        oss<<sw.second->toString()<<"\n";
    }
    RCLCPP_INFO(this->get_logger(), oss.str().c_str());
}

void SwitchService::getAvailableStatesCallback(const GetAvailableStatesService::Request::SharedPtr request,
 GetAvailableStatesService::Response::SharedPtr response){
    vector<SwitchState> states;
    string switchName = request->switch_name;
    RCLCPP_INFO_STREAM(this->get_logger(), "Getting available states for switch: " << switchName);
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
    
    string switchName = request->switch_name;
    RCLCPP_INFO_STREAM(this->get_logger(), "Getting current states for switch: " << switchName);
    if( switches_.find(switchName) == switches_.end()){
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

    string switchName = request->switch_name;
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting state to "<<switchName<<" for switch: " << switchName);
    if( switches_.find(switchName) == switches_.end()){
        response->status = 1;
        return;
    }

    switches_[request->switch_name]->setState(request->state_name);
    response->status = 0;
}

unordered_map<string,shared_ptr<Switch>> switchFactory(shared_ptr<AbstractPwmController> pwmController, vector<SwitchConfig_t> configs){
    unordered_map<string,shared_ptr<Switch>> switchMap;
    for( auto config : configs){
        auto servo = make_shared<Servomotor>(pwmController,config.servomotor);
        auto a = make_shared<Switch>(servo,config);
        switchMap[a->getName()] = a;
    }
    return switchMap;
}

arguments_t parseArguments(int argc, char* argv[]){
    string controllerType ="";
    arguments_t result;
    result.success = false;
    if(argc < 3){
        result.erro_msg = "invalid number of arguments";
        return result;
    }

    result.path_to_config = argv[1];
    controllerType = argv[2];
    if(controllerType == "mockup"){
        result.pwm_controller = make_shared<MockupPwmController>();
        result.success = true;
    }
    if(controllerType == "pi"){
        result.pwm_controller = make_shared<PiPwmController>();
        result.success = true;
    }
    if(controllerType == "pca9586"){
        int address = -1;

        if(argc <4){
            result.erro_msg = "missing argument; 3. argument is required if controller type is pca9586";
        }
        try{
            address = stoi(string(argv[3]), 0, 16);
        }catch( const exception& ex){
            result.erro_msg = "invalid i2c address format; address is expected in hex";
        }
        result.pwm_controller = make_shared<Pca9586PwmController>(address);
        result.success = true;
    }
    return result;
}

int main(int argc, char* argv[]){

    auto args = parseArguments(argc,argv);

    if(!args.success){
        cout<<"Argument error!"<<endl;
        cout<<args.erro_msg<<endl;
        std::cout<<"\nHELP:\n"<<HELP_TEXT<<std::endl;
        return 0;
    }

    auto configs = SwitchConfigReader::readSwitchConfig(args.path_to_config);
    auto switches = switchFactory(args.pwm_controller,configs);
    
    rclcpp::init(argc,argv);
    rclcpp::spin(make_shared<SwitchService>(switches));
    rclcpp::shutdown();

    return 0;
}
