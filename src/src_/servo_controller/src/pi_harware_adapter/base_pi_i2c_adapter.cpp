
#include <servo_controller/pi_hardware_adapter/base_pi_i2c_adapter.hpp>



BasePIi2cAdapter* BasePIi2cAdapter::instance_ = nullptr;

BasePIi2cAdapter* BasePIi2cAdapter::getInstance(){
            if(instance_ == nullptr){
                instance_ = new BasePIi2cAdapter();
            }
            return instance_;
}