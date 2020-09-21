/***********************************************
*  SplitMind Library
*  Version 0.4
************************************************/
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stdbool.h"
#include "math.h"
#include "stdint.h"

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

//Hexapod parametres
//offset from (0, 0) on XY plane in every local coordinate system of leg
#define X_OFFSET                    50
#define Y_OFFSET                    50

#define STARTHEIGHT                 70

//height of step
#define DELTAHEIGHT                 30

//diameter of step circle. distance of step
#define DIAMETER                    60

extern unsigned long time_from_start;
extern unsigned long current_interruption_time;
extern uint16_t delta_interruption_time;
extern uint8_t channel_counter;
extern bool start_package;
extern float channel[6];
extern float Vx, Vy, Vz;
extern float input_roll, input_pitch, input_yaw;
extern float current_roll, current_pitch, current_yaw;

extern uint16_t delay_count;

extern bool servo_enable;

//servo angles--------------------------
extern double q0, q1, q2;
//--------------------------------------

//movements and trajectory variables
extern uint8_t trajectory_step[6];
//local trajectory for each leg [step][xyz coord]
extern float local_current_leg_position[6][3];
extern float local_target_leg_position[6][3];
extern int16_t local_start_point[6][3];
extern bool flag_leg_ready[6];
extern bool phase[2];

extern float diameter;

extern float k;
extern float dH;
extern float H;

//x(t) function
extern float Xt[6], Yt[6], Zt[6];

extern unsigned long next_time;

extern struct Legs
{
   //f(t) coordinates functions
   float Xt, Yt, Zt;

   //current local position of leg (x, y, z)
   float current_x;
   float current_y;
   float current_z;

   //new local position of leg (x, y, z)
   float target_x;
   float target_y;
   float target_z;


} Leg[6];

//--------------------------------------------

void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config);
void digitalWrite(uint8_t port, uint8_t pin, bool value);
void delay(int millisec);
uint64_t pulseIN(uint8_t PIN);
float map(float have, float have_min, float have_max, float need_min, float need_max);

//I2C------------------------------------------------------------------------------------------
void I2C1_init(void);
void I2C_writeByte(uint8_t device_address, uint8_t address, uint8_t data);
void I2C_burstWrite(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data);
//END OF I2C-----------------------------------------------------------------------------------

//PCA9685--------------------------------------------------------------------------------------
void PCA9685_reset(uint8_t device_address);
void PCA9685_init(uint8_t device_address);
void PCA9685_setPWM(uint8_t device_address, uint8_t servo_num, uint16_t on, uint16_t off);
void setServoAngle(uint8_t servo_num, double angle);
bool phaseControl(uint8_t GroupNum);
//END OF PCA9685-------------------------------------------------------------------------------

//TIMERS---------------------------------------------------------------------------------------
//void TIMER3_Init_Millisec();
/* for ppm pin A7 takes TIM3_CH2*/
//function for SysTick timer interruption
void SysTick_Handler(void);
//END OF TIMERS-------------------------------------------------------------------------------

//EXTERNAL INTERRUPTIONS----------------------------------------------------------------------
//Interruptions for PPM 
void EXTI0_IRQHandler(void);
void EXTI0_init(void);
//END OF EXTERNAL INTERRUPTIONS---------------------------------------------------------------


//HEXAPOD MOVEMENTS---------------------------------------------------------------------------
void findAngles(uint8_t leg_num, double x, double y, double z);

void moveLeg(uint8_t leg_num, double x, double y, double z);
//END OF HEXAPOD MOVEMENTS--------------------------------------------------------------------
