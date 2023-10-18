#include "servo_controller/pwm_controller/Pca9586PwmController.hpp"

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
    // make sure auto increment is enabled
    setSingleBitReg8(MODE1_REG_ADDRESS,AUTO_INCREMENT_BIT_POS,true);
    // Prescale register can only be changed during sleep mode!
    setPrescaler(prescaler_);
}

void Pca9586PwmController::setPwmFrequency(uint32_t frequency){
    AbstractPwmController::setPwmFrequency(frequency);
    setPrescaler(prescaler_);
}

void Pca9586PwmController::setPrescaler(uint32_t prescalerVal){
    int mode1 = i2cAdapter_->readReg8(MODE1_REG_ADDRESS) & 0x7F;    // Set restart bit to 0
	int sleep	= mode1 | 0x10;									    // Set sleep bit to 1
	int wake 	= mode1 & 0xEF;									    // Set sleep bit to 0
	int restart = wake | 0x80;

    i2cAdapter_->writeReg8(MODE1_REG_ADDRESS, sleep);
    i2cAdapter_->writeReg8(PRE_SCALE_REG_ADDRESS, prescalerVal);
    i2cAdapter_->writeReg8(MODE1_REG_ADDRESS, wake);

    hardwareAdapter_->gpio_delayMicroseconds(1000);

    i2cAdapter_->writeReg8(MODE1_REG_ADDRESS, restart);
    hardwareAdapter_->gpio_delayMicroseconds(1000);
}

void Pca9586PwmController::enablePwmPin(uint8_t pin, bool pwmOn){
    setSingleBitReg8(getPwmChannelRegisters(pin).off.high,PWM_FULL_OFF_POS,!pwmOn);
}

void Pca9586PwmController::setPulseWidth(uint8_t pin, uint32_t pulseWidth){
    setPulseWidth(pin,pulseWidth,0);
}

void Pca9586PwmController::setPulseWidth( uint8_t pin, uint32_t pulseWidth, uint16_t offset){
    setPulse(pin,offset,offset+pulseWidth);
}

void Pca9586PwmController:: setPulse( uint8_t pin, uint16_t startTime, uint16_t stopTime){
    // map microseconds to resulution
    startTime = mapPulseWidthToResulution(startTime);
    stopTime = mapPulseWidthToResulution(stopTime);

    if(startTime >= PCA_PWM_RESOLUTION || stopTime >= PCA_PWM_RESOLUTION){
        
        std::ostringstream oss;
        oss<<"time for pulse width cant be greater or equal to period time\n";
        oss<<"startTime = "<<startTime<<", stopTime= "<<stopTime<<", period time = "<<PCA_PWM_RESOLUTION; 
        throw std::invalid_argument(oss.str());
    }
    // write start time into reg
    setReg16(getPwmChannelRegisters(pin).on.low,startTime,PWM_ON_REG_COUNT_MASK);

    // set stop time and set 13th bit(led full off Bit) to zero
    // stopTime = (stopTime) & ~(0x1<<12);
    setReg16(getPwmChannelRegisters(pin).off.low,stopTime,PWM_ON_REG_COUNT_MASK);
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

void Pca9586PwmController::setReg16(uint16_t address, uint16_t value, uint16_t mask){
    uint16_t registerValue = i2cAdapter_->readReg16 (address);
    registerValue = (registerValue & mask) | (value &  ~mask);

    i2cAdapter_->writeReg16(address, registerValue);
}

PwmOutput_t Pca9586PwmController::getPwmChannelRegisters(uint8_t channel){
    if(channel > 15){
        std::ostringstream oss;
        oss<<"Channel ("<<channel<<") is out of range; only values from 0 to 15 are valid";
        throw invalid_argument(oss.str().c_str());
    }
    return PIN_TO_REGISTER[channel];
}