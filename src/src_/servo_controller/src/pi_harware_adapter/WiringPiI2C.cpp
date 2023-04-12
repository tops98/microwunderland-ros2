#ifdef WIRING_PI_ADAPTER
#include <servo_controller/pi_hardware_adapter/WiringPiI2C.hpp>


WiringPiI2CAdapter*  WiringPiI2CAdapter::instance_ = nullptr;

WiringPiI2CAdapter* WiringPiI2CAdapter::getInstance(){
    if(instance_ == nullptr){
        instance_ = new WiringPiI2CAdapter();
    }
    return instance_;
}
#endif