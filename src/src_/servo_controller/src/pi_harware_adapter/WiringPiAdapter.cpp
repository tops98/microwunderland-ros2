#ifdef WIRING_PI_ADAPTER
#include <servo_controller/pi_hardware_adapter/WiringPiAdapter.hpp>


WiringPiAdapter*  WirningPiAdapter::instance_ = nullptr;

WiringPiAdapter* WiringPiAdapter::getInstance(){
    if(instance_ == nullptr){
        instance_ = new WiringPiAdapter();
    }
    return instance_;
}
#endif