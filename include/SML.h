/***********************************************
*SplitMind Library
*Version 0.1
************************************************/
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stdbool.h"
#include "math.h"
#include <stdint.h>

//DEFINE PORT LETTER
#define	PORT_A				            (2)
#define PORT_B                          (3)
#define PORT_C                          (4)
#define PORT_D                          (5)
#define PORT_E                          (6)
#define PORT_F                          (7)
#define PORT_G                          (8)

//DEFINE MODES OF PINS
#define INPUT							(0)
#define OUTPUT_2						(2)
#define OUTPUT_10						(1)
#define OUTPUT_50						(3)

//DEFINE CONFIG OF MODES FOR INPUT
#define INPUT_ANALOG					(0)
#define INPUT_FLOAT						(1)
#define INPUT_PULL_UP_DOWN				(2)

//DEFINE CONFIG OF MODES FOR OUTPUT
#define OUTPUT_GPO_PUSH_PULL			(0)
#define	OUTPUT_GPO_OPEN_DRAIN			(1)
#define OUTPUT_AF_PUSH_PULL				(2)
#define OUTPUT_AF_OPEN_DRAIN			(3)

// servo degrees parametre
#define SERVOMIN  						100 // this is the 'minimum' pulse length count (out of 4096)	//115
#define SERVOMID  						330 // this is the 'middle' pulse length count (out of 4096)
#define SERVOMAX  						565 // this is the 'maximum' pulse length count (out of 4096)	//550
//#define DEGREE_IN_PULSE				(SERVOMAX-SERVOMIN)/180

#define LED0_ON_L		                0x06  /**< LED0 on tick, low byte*/
#define LED0_ON_H		                0x07  /**< LED0 on tick, high byte*/
#define LED0_OFF_L	                    0x08 /**< LED0 off tick, low byte */
#define LED0_OFF_H	                    0x09 /**< LED0 off tick, high byte */

#define PCA9685_ADDRESS_1               0x80
#define PCA9685_ADDRESS_2               0x82

void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);

void digitalWrite(uint8_t port, uint8_t pin, bool value);