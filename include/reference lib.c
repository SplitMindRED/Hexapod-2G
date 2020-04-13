#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "stdbool.h"
#include "math.h"

//DEFINE PORT CHAR
#define	PORT_A				(2)
#define PORT_B              (3)
#define PORT_C              (4)
#define PORT_D              (5)
#define PORT_E              (6)
#define PORT_F              (7)
#define PORT_G              (8)

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
#define OUTPUT_GPO_PUSH_PULL				(0)
#define	OUTPUT_GPO_OPEN_DRAIN				(1)
#define OUTPUT_AF_PUSH_PULL					(2)
#define OUTPUT_AF_OPEN_DRAIN				(3)

// servo degrees parametre
#define SERVOMIN  						100 // this is the 'minimum' pulse length count (out of 4096)	//115
#define SERVOMID  						330 // this is the 'middle' pulse length count (out of 4096)
#define SERVOMAX  						565 // this is the 'maximum' pulse length count (out of 4096)	//550
//#define DEGREE_IN_PULSE				(SERVOMAX-SERVOMIN)/180

#define LED0_ON_L		0x06  /**< LED0 on tick, low byte*/
#define LED0_ON_H		0x07  /**< LED0 on tick, high byte*/
#define LED0_OFF_L	0x08 /**< LED0 off tick, low byte */
#define LED0_OFF_H	0x09 /**< LED0 off tick, high byte */

#define PCA9685_ADDRESS  0x80


//bool led = true;
//unsigned long TimeFromStart = 0;
//uint16_t delay_count = 0;
//const int OA = 45;
//const int AB = 85;
//double pi = 3.14;
////int x, y, z, p;
//double q0grad, q1grad, q2grad;
//double q0, q1, q2, Q, Qgrad, Q0, Q0grad;
//float Vx, Vy, Vz;
//float InputX, InputY, InputZ;
//float Last_InputX, Last_InputY, Last_InputZ;
//float CoordX, CoordY, CoordZ;


//MY LIB--------------------------------------------------------------
void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config)
{
	
	//(int)A = 65
	//MODE: 0 -> input; 1 -> output	
	
	//enable clock on port
	RCC -> APB2ENR |= 1 << port; 
	
	//CRL
	if(pin <= 7)
	{
		//reset config
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> CRL &= ~( 1 << (pin * 4  + 2) );
		
		//set mode and config
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400) ) -> CRL |= mode << (pin * 4);
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400) ) -> CRL |= config << (pin * 4 + 2);
	}
	//CRH
	else
	{		
		//reset config
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> CRH &= ~(1 << ( ( (pin-8) * 4 ) + 2 ));
		
		//set mode and config
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> CRH |= mode << ( (pin-8) * 4 );
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> CRH |= config << ( ( (pin-8) * 4 ) + 2 );
	}	
}

void digitalWrite(uint8_t port, uint8_t pin, bool value)
{
	
	if(value == 1)
	{
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> ODR |= value << pin;
	}
	else if (value == 0)
	{
		((GPIO_TypeDef *) (GPIOA_BASE + (port - 2) * 0x0400 )) -> ODR &= ~(1 << pin);
	}	
}

void delay(int millisec)
{
	unsigned long start_time = TimeFromStart;
	while( TimeFromStart != (start_time + (millisec * 1000)) )
	{
		//waiting
	}
}


uint64_t pulseIN(uint8_t PIN)
{
	uint16_t PulseLength = 0;
	unsigned long StartCount = 0;
	bool HIGHsignal = false;
	
	while(1)			// !(GPIOA -> IDR & (1 << PIN)) ) //while 0
	{
//		//when comes 1 -> wait for 0
//		if( GPIOA -> IDR & (1 << PIN) )
//		{
//			StartCount[0] = TimeFromStart;
//			signal[0] = true;
//		}	
//		
//		if( (signal[0] == true) && !(GPIOA -> IDR & (1 << PIN)) )
//		{			
//			//uint32_t delta = TimeFromStart - StartCount[0];
//			//double degfrompulse = 0.18*delta - 180;
//			//SetServoAngle(0, (0.18*(TimeFromStart - StartCount[0]) - 180) );
//			//PulseLength = TimeFromStart - StartCount;
//			break;
//		}
		if( GPIOA -> IDR & (1<<(PIN)) && HIGHsignal == false)
		{
			StartCount = TimeFromStart;
			HIGHsignal = true;
		}				
			
			//check 7 down
		if( (HIGHsignal == true) && !(GPIOA -> IDR & (1<<(PIN))) )
		{			
			//uint32_t delta = TimeFromStart - StartCount[0];
			//double degfrompulse = 0.18*delta - 180;
			HIGHsignal = false;
			PulseLength = TimeFromStart - StartCount;
			break;
		}	
	}
	delay(1);
	return PulseLength;
}
//END OF MY LIB-----------------------------------------------------------------------

//TIMER----------------------------------------------------------------------------------
void SysTick_Handler(void)
{
//	if(delay_count > 0)
//	{
//		delay_count--;
//	}
	
	TimeFromStart++;
}
//END OF TIMER------------------------------------------------------------------------------


//I2C STUFF######################################################################
GPIO_InitTypeDef GPIO_InitStructure;

void I2C1_init(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure I2C_EE pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);
}

void I2C_WriteByte(uint8_t address, uint8_t data)
{
	//send START
  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	//send slave address
  I2C_Send7bitAddress(I2C1, PCA9685_ADDRESS, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	//send register address
  I2C_SendData(I2C1, address);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	//send data
  I2C_SendData(I2C1, data);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	//send STOP
  I2C_GenerateSTOP(I2C1, ENABLE);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void I2C_burst_write(uint8_t address, uint8_t n_data, uint8_t *data)
{
	I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1, PCA9685_ADDRESS, I2C_Direction_Transmitter);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(I2C1, address);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	while(n_data--) 
	{
		I2C_SendData(I2C1, *data++);
		n_data --;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	
	I2C_GenerateSTOP(I2C1, ENABLE);
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}

//END OF I2C STUFF###################################################################################

//PCA9685 STUFF--------------------------------------------------------------------------------------
void PCA9685_reset()
{
//	uint8_t reset_data[2];
//	reset_data[0] = 0x00;
//	reset_data[1] = 0x06;
	I2C_WriteByte(0x00, 0x06); 
}


void PCA9685_init()
{
	PCA9685_reset();
	
	//sleep
	I2C_WriteByte(0x00, 0b10000); 
	
	//prescale
	//I2C_WriteByte(0xFE, 0x3); MAX
	I2C_WriteByte(0xFE, 0x64); //60hz 
	
	//normal mode
	I2C_WriteByte(0x00, 0xA1);
}

void PCA9685_setPWM(uint8_t ServoNum, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[4] = {on, (on >> 8), off, (off >> 8)};
	//I2C_burst_write(0x06 + 4*ServoNum, 4, outputBuffer);
	//I2C_WriteByte(0x06 + 4*ServoNum, on);
	//I2C_WriteByte(0x06 + 4*ServoNum+1, on >> 8);
	I2C_WriteByte(0x06 + 4*ServoNum+2, off);
	I2C_WriteByte(0x06 + 4*ServoNum+3, off >> 8);
}

void SetServoAngle(uint8_t n, double angle)
{
	double deg_to_pulse = 100 + (SERVOMAX-SERVOMIN) * angle/180;
	PCA9685_setPWM(n, 0, deg_to_pulse);
}
//END OF PCA9685 STUFF-------------------------------------------------------------------------------------------------------


void FindAngles(int x, int y, int z)
{
	//long int TimeStart = micros();
	double p = sqrt(x*x + y*y);
	double L_OB = sqrt(p*p + z*z);

	if (x == 0)		q0 = acos(x / p);
	else			q0 = atan2(y, x);

	q0grad = q0 * 180 / pi;

	//Q = atan2(z, p);
	Q = atan(z / p);
	//Qgrad = Q * 180 / pi;
	//Serial.print("Q = ");	Serial.println(Q);

	Q0 = acos((pow(OA, 2) + pow(L_OB, 2) - pow(AB, 2)) / (2 * OA * L_OB));
	//Q0grad = Q0 * 180 / pi;
	//Serial.print("Q0 = ");	Serial.println(Q0);

	q1 = Q + Q0;
	q1grad = 180 - q1 * 180 / pi;

	q2 = acos((pow(OA, 2) + pow(AB, 2) - pow(L_OB, 2)) / (2 * OA * AB));
	q2grad = q2 * 180 / pi;

	//long int TimeFinish = micros();
	//Serial.print("TimeStart = ");	Serial.println(TimeStart);
	//Serial.print("TimeFinish = ");	Serial.println(TimeFinish);
	//Serial.println();
	/*delay(1000);
	int Time = millis();
	Serial.print("Time = ");	Serial.println(Time);*/
}

void DoStraightLine()
{
	int TimeDelay = 500;
	//----------straight line-----------------------------------------
	SetServoAngle(0, 90);//1 point x=0 y=80
	SetServoAngle(1, 100.5);
	SetServoAngle(2, 69.5);
	delay(TimeDelay);

	SetServoAngle(0, 90);//2 point y+10 x=0 y=100
	SetServoAngle(1, 123.5);
	SetServoAngle(2, 98);
	delay(TimeDelay);

	SetServoAngle(0, 90);//3 point y+20 x=0 y=120
	SetServoAngle(1, 151.5);
	SetServoAngle(2, 137);
	delay(TimeDelay);
//---------------------------------------------------------------
}

int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
//#######SETUP SYSTICK TIMER###########################
	//__disable_irq();
	__enable_irq();	
	
	//SysTick -> LOAD = SystemCoreClock/1000;			//1ms
	SysTick -> LOAD = SystemCoreClock/1000000;	//1mcs
	SysTick -> CTRL = 0b111;	//start count
//#######################################################
	
	pinMode(PORT_A, 7, INPUT, INPUT_PULL_UP_DOWN);
	pinMode(PORT_A, 6, INPUT, INPUT_PULL_UP_DOWN);
	pinMode(PORT_A, 5, INPUT, INPUT_PULL_UP_DOWN);
//	pinMode(PORT_A, 7, INPUT, INPUT_PULL_UP_DOWN);
//	pinMode(PORT_A, 6, INPUT, INPUT_PULL_UP_DOWN);
//	pinMode(PORT_A, 5, INPUT, INPUT_PULL_UP_DOWN);
	
	digitalWrite( 4, 13, 1);
	
	I2C1_init();
	PCA9685_init();
	
//Home position
//	SetServoAngle(0, 90);
//	SetServoAngle(1, 90);
//	SetServoAngle(2, 90);

//Home pos
	FindAngles(0, 84, 44);
	SetServoAngle(0, q0grad);
	SetServoAngle(1, q1grad);
	SetServoAngle(2, q2grad);
	delay(500);
	
	CoordX = 0;
	CoordY = 84;
	CoordZ = 44;
	
	InputX = 0;
	InputY = 0;
	InputZ = 0;
	
	//I2C_WriteByte(LED0_OFF_L, 0b01001100);
	//I2C_WriteByte(LED0_OFF_H, 0b0001);
	
	while(1)
	{		
//		SetServoAngle(0, 180);
//		SetServoAngle(1, 180);
//		delay(1000);
//		
//		SetServoAngle(0, 90);
//		SetServoAngle(1, 90);
//		delay(1000);
//		
//		SetServoAngle(0, 0);
//		SetServoAngle(1, 0);
//		delay(1000);
//		
//		SetServoAngle(0, 90);
//		SetServoAngle(1, 90);
//		delay(1000);

			//check A7 up
//			if( GPIOA -> IDR & (1<<(7)) && signal[0] == false)
//			{
//				StartCount[0] = TimeFromStart;
//				signal[0] = true;
//			}				
//			
//			//check 7 down
//			if( (signal[0] == true) && !(GPIOA -> IDR & (1<<(7))) )
//			{			
//				//uint32_t delta = TimeFromStart - StartCount[0];
//				//double degfrompulse = 0.18*delta - 180;
//				SetServoAngle(0, (0.18*(TimeFromStart - StartCount[0]) - 180) );
//				signal[0] = false;
//			}	

	float Throttle = pulseIN(5);
	float Pitch = pulseIN(6);
	float Roll = pulseIN(7);
	SetServoAngle(0, (Throttle*0.18 - 180));
	SetServoAngle(1, (Pitch*0.18 - 180));
	SetServoAngle(2, (Roll*0.18 - 180));
	
//	SetServoAngle(3, (a*0.18 - 180));
//	SetServoAngle(4, (b*0.18 - 180));
//	SetServoAngle(5, (c*0.18 - 180));

	//DoStraightLine();
	
//###########line#######################
//	FindAngles(0, 80, 0);	
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
//	delay(500);
//	
//	FindAngles(0, 100, 0);	
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
//	delay(500);
//	
//	FindAngles(0, 120, 0);	
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
//	delay(500);
//#######################################

////Home pos
//	FindAngles(0, 84, 44);
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
//	delay(500);
	
//	Last_InputX = InputX;
//	Last_InputY = InputY;
//	Last_InputZ = InputZ;
	
//	InputZ = pulseIN(5);
//	InputY = pulseIN(6);
//	InputX = pulseIN(7);
	
	
//	if(InputX - Last_InputX < 30)
//	{
//		InputX = Last_InputX;
//	}
//	if(InputY - Last_InputY < 30)
//	{
//		InputY = Last_InputY;
//	}
//	if(InputZ - Last_InputZ < 30)
//	{
//		InputZ = Last_InputZ;
//	}
	
//	Vx = -(InputX * 0.02 - 30);
//	Vy = -(InputY * 0.02 - 30);
//	Vz = InputZ * 0.02 - 30;
//	
//	CoordX = CoordX + Vx;
//	CoordY = CoordY + Vy;
//	CoordZ = CoordZ + Vz;
	
	
//	CoordX = -(InputX * 0.06 - 90);
//	CoordY = -(InputY * 0.06 - 90);
//	CoordZ = InputZ * 0.06 - 90;
//	
//	
//	FindAngles(CoordX/10, CoordY/10, CoordZ/10);
//	
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
//	delay(100);
	


	}
	
	return 0;
}
