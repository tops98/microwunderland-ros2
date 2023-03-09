#ifndef PCA9586_REGISTERS_HPP
#define PCA9586_REGISTERS_HPP


#define MODE1_REG_ADDRESS 0x00			// Mode register 1
#define MODE2_REG_ADDRESS 0x01			// Mode register 2
#define SUBADR1_REG_ADDRESS 0x02		// I2 C-bus subaddress 1
#define SUBADR2_REG_ADDRESS 0x03		// I2 C-bus subaddress 2
#define SUBADR3_REG_ADDRESS 0x04		// I2 C-bus subaddress 3
#define ALLCALLADR_REG_ADDRESS 0x05		// LED All Call I2C-bus address
#define LED0_ON_L_REG_ADDRESS 0x06		// LED0 output and brightness control byte 0
#define LED0_ON_H_REG_ADDRESS 0x07		// LED0 output and brightness control byte 1
#define LED0_OFF_L_REG_ADDRESS 0x08		// LED0 output and brightness control byte 2
#define LED0_OFF_H_REG_ADDRESS 0x09		// LED0 output and brightness control byte 3
#define LED1_ON_L_REG_ADDRESS 0x0A		// LED1 output and brightness control byte 0
#define LED1_ON_H_REG_ADDRESS 0x0B		// LED1 output and brightness control byte 1
#define LED1_OFF_L_REG_ADDRESS 0x0C		// LED1 output and brightness control byte 2
#define LED1_OFF_H_REG_ADDRESS 0x0D		// LED1 output and brightness control byte 3
#define LED2_ON_L_REG_ADDRESS 0x0E		// LED2 output and brightness control byte 0
#define LED2_ON_H_REG_ADDRESS 0x0F		// LED2 output and brightness control byte 1
#define LED2_OFF_L_REG_ADDRESS 0x10		// LED2 output and brightness control byte 2
#define LED2_OFF_H_REG_ADDRESS 0x11		// LED2 output and brightness control byte 3
#define LED3_ON_L_REG_ADDRESS 0x12		// LED3 output and brightness control byte 0
#define LED3_ON_H_REG_ADDRESS 0x13		// LED3 output and brightness control byte 1
#define LED3_OFF_L_REG_ADDRESS 0x14		// LED3 output and brightness control byte 2
#define LED3_OFF_H_REG_ADDRESS 0x15		// LED3 output and brightness control byte 3
#define LED4_ON_L_REG_ADDRESS 0x16		// LED4 output and brightness control byte 0
#define LED4_ON_H_REG_ADDRESS 0x17		// LED4 output and brightness control byte 1
#define LED4_OFF_L_REG_ADDRESS 0x18		// LED4 output and brightness control byte 2
#define LED4_OFF_H_REG_ADDRESS 0x19		// LED4 output and brightness control byte 3
#define LED5_ON_L_REG_ADDRESS 0x1A		// LED5 output and brightness control byte 0
#define LED5_ON_H_REG_ADDRESS 0x1B		// LED5 output and brightness control byte 1
#define LED5_OFF_L_REG_ADDRESS 0x1C		// LED5 output and brightness control byte 2
#define LED5_OFF_H_REG_ADDRESS 0x1D		// LED5 output and brightness control byte 3
#define LED6_ON_L_REG_ADDRESS 0x1E		// LED6 output and brightness control byte 0
#define LED6_ON_H_REG_ADDRESS 0x1F		// LED6 output and brightness control byte 1
#define LED6_OFF_L_REG_ADDRESS 0x20		// LED6 output and brightness control byte 2
#define LED6_OFF_H_REG_ADDRESS 0x21		// LED6 output and brightness control byte 3
#define LED7_ON_L_REG_ADDRESS 0x22		// LED7 output and brightness control byte 0
#define LED7_ON_H_REG_ADDRESS 0x23		// LED7 output and brightness control byte 1
#define LED7_OFF_L_REG_ADDRESS 0x24		// LED7 output and brightness control byte 2
#define LED7_OFF_H_REG_ADDRESS 0x25		// LED7 output and brightness control byte 3
#define LED8_ON_L_REG_ADDRESS 0x26		// LED8 output and brightness control byte 0
#define LED8_ON_H_REG_ADDRESS 0x27		// LED8 output and brightness control byte 1
#define LED8_OFF_L_REG_ADDRESS 0x28		// LED8 output and brightness control byte 2
#define LED8_OFF_H_REG_ADDRESS 0x29		// LED8 output and brightness control byte 3
#define LED9_ON_L_REG_ADDRESS 0x2A		// LED9 output and brightness control byte 0
#define LED9_ON_H_REG_ADDRESS 0x2B		// LED9 output and brightness control byte 1
#define LED9_OFF_L_REG_ADDRESS 0x2C		// LED9 output and brightness control byte 2
#define LED9_OFF_H_REG_ADDRESS 0x2D		// LED9 output and brightness control byte 3
#define LED10_ON_L_REG_ADDRESS 0x2E		// LED10 output and brightness control byte 0
#define LED10_ON_H_REG_ADDRESS 0x2F		// LED10 output and brightness control byte 1
#define LED10_OFF_L_REG_ADDRESS 0x30	// LED10 output and brightness control byte 2
#define LED10_OFF_H_REG_ADDRESS 0x31	// LED10 output and brightness control byte 3
#define LED11_ON_L_REG_ADDRESS 0x32		// LED11 output and brightness control byte 0
#define LED11_ON_H_REG_ADDRESS 0x33		// LED11 output and brightness control byte 1
#define LED11_OFF_L_REG_ADDRESS 0x34	// LED11 output and brightness control byte 2
#define LED11_OFF_H_REG_ADDRESS 0x35	// LED11 output and brightness control byte 3
#define LED12_ON_L_REG_ADDRESS 0x36		// LED12 output and brightness control byte 0
#define LED12_ON_H_REG_ADDRESS 0x37		// LED12 output and brightness control byte 1
#define LED12_OFF_L_REG_ADDRESS 0x38	// LED12 output and brightness control byte 2
#define LED12_OFF_H_REG_ADDRESS 0x39	// LED12 output and brightness control byte 3
#define LED13_ON_L_REG_ADDRESS 0x3A		// LED13 output and brightness control byte 0
#define LED13_ON_H_REG_ADDRESS 0x3B		// LED13 output and brightness control byte 1
#define LED13_OFF_L_REG_ADDRESS 0x3C	// LED13 output and brightness control byte 2
#define LED13_OFF_H_REG_ADDRESS 0x3D	// LED13 output and brightness control byte 3
#define LED14_ON_L_REG_ADDRESS 0x3E		// LED14 output and brightness control byte 0
#define LED14_ON_H_REG_ADDRESS 0x3F		// LED14 output and brightness control byte 1
#define LED14_OFF_L_REG_ADDRESS 0x40	// LED14 output and brightness control byte 2
#define LED14_OFF_H_REG_ADDRESS 0x41	// LED14 output and brightness control byte 3
#define LED15_ON_L_REG_ADDRESS 0x42		// LED15 output and brightness control byte 0
#define LED15_ON_H_REG_ADDRESS 0x43		// LED15 output and brightness control byte 1
#define LED15_OFF_L_REG_ADDRESS 0x44	// LED15 output and brightness control byte 2
#define LED15_OFF_H_REG_ADDRESS 0x45	// LED15 output and brightness control byte 3
#define ALL_LED_ON_L_REG_ADDRESS 0xFA	// zero load all the LEDn_ON registers, byte 0
#define ALL_LED_ON_H_REG_ADDRESS 0xFB	// zero load all the LEDn_ON registers, byte 1
#define ALL_LED_OFF_L_REG_ADDRESS 0xFC	// zero load all the LEDn_OFF registers, byte 0
#define ALL_LED_OFF_H_REG_ADDRESS 0xFD	// zero load all the LEDn_OFF registers, byte 1
#define PRE_SCALE_REG_ADDRESS 0xFE		// prescaler for PWM output frequency
#define TestMode_REG_ADDRESS 0xFF		// defines the test mode to be entered

typedef struct PWM_REG_ADDRESSES{
    uint8_t low;
    uint8_t high;
}PwmRegAddresses_t;

typedef struct PWM_OUTPUT{
    PwmRegAddresses_t  on;
    PwmRegAddresses_t off;
}PwmOutput_t;


const PwmOutput_t PIN_TO_REGISTER[]={
    {
        {{LED0_ON_L_REG_ADDRESS},{LED0_ON_H_REG_ADDRESS}},
        {{LED0_OFF_L_REG_ADDRESS},{LED0_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED1_ON_L_REG_ADDRESS},{LED1_ON_H_REG_ADDRESS}},
        {{LED1_OFF_L_REG_ADDRESS},{LED1_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED2_ON_L_REG_ADDRESS},{LED2_ON_H_REG_ADDRESS}},
        {{LED2_OFF_L_REG_ADDRESS},{LED2_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED3_ON_L_REG_ADDRESS},{LED3_ON_H_REG_ADDRESS}},
        {{LED3_OFF_L_REG_ADDRESS},{LED3_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED4_ON_L_REG_ADDRESS},{LED4_ON_H_REG_ADDRESS}},
        {{LED4_OFF_L_REG_ADDRESS},{LED4_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED5_ON_L_REG_ADDRESS},{LED5_ON_H_REG_ADDRESS}},
        {{LED5_OFF_L_REG_ADDRESS},{LED5_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED6_ON_L_REG_ADDRESS},{LED6_ON_H_REG_ADDRESS}},
        {{LED6_OFF_L_REG_ADDRESS},{LED6_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED7_ON_L_REG_ADDRESS},{LED7_ON_H_REG_ADDRESS}},
        {{LED7_OFF_L_REG_ADDRESS},{LED7_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED8_ON_L_REG_ADDRESS},{LED8_ON_H_REG_ADDRESS}},
        {{LED8_OFF_L_REG_ADDRESS},{LED8_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED9_ON_L_REG_ADDRESS},{LED9_ON_H_REG_ADDRESS}},
        {{LED9_OFF_L_REG_ADDRESS},{LED9_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED10_ON_L_REG_ADDRESS},{LED10_ON_H_REG_ADDRESS}},
        {{LED10_OFF_L_REG_ADDRESS},{LED10_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED11_ON_L_REG_ADDRESS},{LED11_ON_H_REG_ADDRESS}},
        {{LED11_OFF_L_REG_ADDRESS},{LED11_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED12_ON_L_REG_ADDRESS},{LED12_ON_H_REG_ADDRESS}},
        {{LED12_OFF_L_REG_ADDRESS},{LED12_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED13_ON_L_REG_ADDRESS},{LED13_ON_H_REG_ADDRESS}},
        {{LED13_OFF_L_REG_ADDRESS},{LED13_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED14_ON_L_REG_ADDRESS},{LED14_ON_H_REG_ADDRESS}},
        {{LED14_OFF_L_REG_ADDRESS},{LED14_OFF_H_REG_ADDRESS}}
    },
    {
        {{LED15_ON_L_REG_ADDRESS},{LED15_ON_H_REG_ADDRESS}},
        {{LED15_OFF_L_REG_ADDRESS},{LED15_OFF_H_REG_ADDRESS}}
    }
};

#endif