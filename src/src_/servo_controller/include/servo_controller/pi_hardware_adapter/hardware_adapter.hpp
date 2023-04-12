#ifndef HARDWARE_ADAPTER_HPP
#define HARDWARE_ADAPTER_HPP

#include <servo_controller/pi_hardware_adapter/base_pi_hardware_adapter.hpp>
#include <servo_controller/pi_hardware_adapter/base_pi_i2c_adapter.hpp>


#define BASE_PI_ADAPTER

#ifdef BASE_PI_ADAPTER
    #define GET_HARDWARE_ADAPTER BasePiHardwareAdapter::getInstance()
    #define GET_I2C_ADAPTER BasePIi2cAdapter::getInstance()
#endif

#ifdef WIRING_PI_ADAPTER
    #include <servo_controller/pi_hardware_adapter/WiringPiAdapter.hpp>
    #include <servo_controller/pi_hardware_adapter/WiringPiI2C.hpp>

    #define GET_HARDWARE_ADAPTER WiringPiAdapter::getInstance()
    #define GET_I2C_ADAPTER WiringPiI2C::getInstance()
#endif

#endif