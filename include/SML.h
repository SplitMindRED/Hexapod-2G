/***********************************************
*SplitMind Library
*Version 0.3
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
#define PORT_A				            (2)
#define PORT_B                      (3)
#define PORT_C                      (4)
#define PORT_D                      (5)
#define PORT_E                      (6)
#define PORT_F                      (7)
#define PORT_G                      (8)

//DEFINE MODES OF PINS
#define INPUT							   (0)
#define OUTPUT_2						   (2)
#define OUTPUT_10						   (1)
#define OUTPUT_50						   (3)

//DEFINE CONFIG OF MODES FOR INPUT
#define INPUT_ANALOG					   (0)
#define INPUT_FLOAT						(1)
#define INPUT_PULL_UP_DOWN				(2)

//DEFINE CONFIG OF MODES FOR OUTPUT
#define OUTPUT_GPO_PUSH_PULL			(0)
#define OUTPUT_GPO_OPEN_DRAIN		   (1)
#define OUTPUT_AF_PUSH_PULL			(2)
#define OUTPUT_AF_OPEN_DRAIN			(3)

// servo degrees parametre
#define SERVOMIN  						100 // this is the 'minimum' pulse length count (out of 4096)	//115
#define SERVOMID  						330 // this is the 'middle' pulse length count (out of 4096)
#define SERVOMAX  						565 // this is the 'maximum' pulse length count (out of 4096)	//550
//#define DEGREE_IN_PULSE				(SERVOMAX-SERVOMIN)/180

#define LED0_ON_L		               0x06 //LED0 on tick, low byte
#define LED0_ON_H		               0x07 //LED0 on tick, high byte
#define LED0_OFF_L	               0x08 //LED0 off tick, low byte
#define LED0_OFF_H	               0x09 //LED0 off tick, high byte

#define PCA9685_ADDRESS_1           0x80
#define PCA9685_ADDRESS_2           0x82

extern unsigned long TimeFromStart;
extern unsigned long Millis;
extern unsigned long CurrentInterruptionTime;
extern uint16_t deltaInterruptionTime;
extern uint8_t ChannelCounter;
extern bool StartPackage;
extern float Channel[6];
extern uint16_t delay_count;
extern double pi;
extern double q0grad, q1grad, q2grad;
extern double q0, q1, q2, Q, Qgrad, Q0, Q0grad;

void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);
void digitalWrite(uint8_t port, uint8_t pin, bool value);
void delay(int millisec);
uint64_t pulseIN(uint8_t PIN);

//I2C STUFF------------------------------------------------------------------------------------------
void I2C1_init(void);
void I2C_WriteByte(uint8_t device_address, uint8_t address, uint8_t data);
void I2C_burst_write(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data);
//END OF I2C STUFF-----------------------------------------------------------------------------------

//PCA9685 STUFF--------------------------------------------------------------------------------------
void PCA9685_reset(uint8_t device_address);
void PCA9685_init(uint8_t device_address);
void PCA9685_setPWM(uint8_t device_address, uint8_t ServoNum, uint16_t on, uint16_t off);
void SetServoAngle(uint8_t n, double angle);
void SpeedControl_SetServoAngle(uint8_t n, double angle, uint8_t pause);
//END OF PCA9685 STUFF-------------------------------------------------------------------------------

//TIMERS STUFF--------------------------------------------------------------------------------------
//void TIMER3_Init_Millisec();
/* for ppm pin A7 takes TIM3_CH2*/
//function for SysTick timer interruption
void SysTick_Handler(void);
//END OF TIMERS STUFF-------------------------------------------------------------------------------

//EXTERNAL INTERRUPTIONS STUFF----------------------------------------------------------------------

void EXTI0_IRQHandler(void);

void EXTI0_init(void);
//END OF EXTERNAL INTERRUPTIONS STUFF---------------------------------------------------------------

void FindAngles(int x, int y, int z);
