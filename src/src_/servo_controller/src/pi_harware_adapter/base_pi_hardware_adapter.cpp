#include <servo_controller/pi_hardware_adapter/base_pi_hardware_adapter.hpp>


BasePiHardwareAdapter*  BasePiHardwareAdapter::instance_ = nullptr;

BasePiHardwareAdapter* BasePiHardwareAdapter::getInstance(){
    if(instance_ == nullptr){
        instance_ = new BasePiHardwareAdapter();
    }
    return instance_;
};        