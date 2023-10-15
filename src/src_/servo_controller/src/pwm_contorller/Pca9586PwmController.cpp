#include "servo_controller/pwm_controller/Pca9586PwmController.hpp"

// package
#include "servo_controller/pwm_controller/PCA9586_Registers.hpp"
// std
#include <stdexcept>
#include <sstream>
#include <iostream>


Pca9586PwmController::Pca9586PwmController(
uint8_t deviceId,
uint64_t oscilatorFrequency,
uint32_t pwmFrequency):
AbstractPwmController(oscilatorFrequency,pwmFrequency, PCA_PWM_RESOLUTION){
    hardwareAdapter_ = GET_HARDWARE_ADAPTER;
    i2cAdapter_ = GET_I2C_ADAPTER;
    i2cAdapter_->setup(deviceId);
    // Prescale register can only be changed during sleep mode!
    setPrescaler(prescaler_);
    setSleepMode(false);
}

void Pca9586PwmController::setPwmFrequency(uint32_t frequency){
    AbstractPwmController::setPwmFrequency(frequency);
    setPrescaler(prescaler_);
}

void Pca9586PwmController::setPrescaler(uint32_t prescalerVal){
    setSleepMode(true);
    i2cAdapter_->writeReg8(PRE_SCALE_REG_ADDRESS,prescalerVal);
    setSleepMode(false);
    hardwareAdapter_->gpio_delayMicroseconds(STARTUP_DELAY);
}

void Pca9586PwmController::enablePwmPin(uint8_t pin, bool pwmOn){
    setSingleBitReg8(PIN_TO_REGISTER[pin].off.high,PWM_FULL_OFF_POS,pwmOn);
}

void Pca9586PwmController::setPulseWidth(uint8_t pin, uint32_t pulseWidth){
    setPulseWidth(pin,pulseWidth,DEFAULT_DUTY_CYLCLE_OFFSET);
}

void Pca9586PwmController::setPulseWidth( uint8_t pin, uint32_t pulseWidth, uint16_t offset){
    setPulse(pin,offset,pulseWidth);
}

void Pca9586PwmController:: setPulse( uint8_t pin, uint16_t startTime, uint16_t stopTime){
    // map microseconds to resulution
    startTime = mapPulseWidthToResulution(startTime);
    stopTime = mapPulseWidthToResulution(stopTime);
    std::ostringstream oss;
    oss<<"time for pulse width cant be greater or equal to period time\n";
    oss<<"startTime = "<<startTime<<", stopTime= "<<stopTime<<", period time = "<<PCA_PWM_RESOLUTION; 
    std::cout<<oss.str()<<std::endl;
    if(startTime >= PCA_PWM_RESOLUTION || stopTime >= PCA_PWM_RESOLUTION){
        
        std::ostringstream oss;
        oss<<"time for pulse width cant be greater or equal to period time\n";
        oss<<"startTime = "<<startTime<<", stopTime= "<<stopTime<<", period time = "<<PCA_PWM_RESOLUTION; 
        throw std::invalid_argument(oss.str());
    }
    // write first 8 LSBs in low register
    setReg8(PIN_TO_REGISTER[pin].on.low,startTime);
    // write first 4 MSBs in first nibble of high register
    setReg8(PIN_TO_REGISTER[pin].on.high,startTime>>8,PWM_ON_HIGH_REG_COUNT_MASK);

    // write first 8 LSBs in low register
    setReg8(PIN_TO_REGISTER[pin].off.low,stopTime);

    // shift 4 MSBs of stop time to 4 LSBs and set fived bit(led full off Bit) to zero
    stopTime = (stopTime>>8) | (0x1<<4);
    setReg8(PIN_TO_REGISTER[pin].off.high,stopTime,PWM_OFF_HIGH_REG_MASK);
}

void Pca9586PwmController::setSleepMode(bool enable){
    // set bit 4 (sleep mode bit) in MODE_1 register to zero or one
    setSingleBitReg8(MODE1_REG_ADDRESS,SLEEP_MODE_ENABLE_BIT_POS,enable);
}

void Pca9586PwmController::setSingleBitReg8(uint8_t address, uint8_t position, bool value){
    uint8_t registerValue = i2cAdapter_->readReg8 (address);
    registerValue = (registerValue & (~(0x1<<position))) | (value<<position);
    i2cAdapter_->writeReg8( address, registerValue);
}

void Pca9586PwmController::setReg8(uint8_t address, uint8_t value, uint8_t mask){
    uint8_t registerValue = i2cAdapter_->readReg8 (address);
    registerValue = (registerValue & mask) | (value &  ~mask);
    i2cAdapter_->writeReg8(address, registerValue);
}