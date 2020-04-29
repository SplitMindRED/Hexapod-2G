/***********************************************
*SplitMind Library
*Version 0.3
*
*WRITING BITs
*a |= 1 << 7;					//set 1 in 7th bit
*a &= ~(1 << 3);				//set 0 in 3th bit
*a ^= 1 << 5;					//inversion of 5th bit
*a |= 1 << 7 | 1 << 8		//sets 1 to 7th and 8th bits
*
*READING BITs
*if( a & (1<<7) )				//if 7th bit in "a" var equals 1 -> true
*a & ( 1 << 7 | 1 << 8 )	//checks 7th and 8th bits
*
************************************************/

#include "SML.h"

unsigned long TimeFromStart = 0;
unsigned long Millis = 0;
unsigned long CurrentInterruptionTime = 0;
uint16_t deltaInterruptionTime = 0;
uint8_t ChannelCounter = 0;
bool StartPackage = false;
float Channel[6];
uint16_t delay_count = 0;
bool ServoEnable = false;

const uint8_t OA = 37;
const uint8_t AB = 45;
const uint8_t BC = 85;
double pi = 3.14;
double q0rad, q1rad, q2rad;
double q0, q1, q2, Q, Qrad, Q0, Q0rad;
//set mode of pin of port. look to define for parameters
void pinMode(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config)
{
	//enable clock on port
	RCC->APB2ENR |= 1 << port;

	//CRL
	if (pin <= 7)
	{
		//reset config
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL &= ~(1 << (pin * 4 + 2));

		//set mode and config
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL |= mode << (pin * 4);
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRL |= config << (pin * 4 + 2);
	}
	//CRH
	else
	{
		//reset config
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH &= ~(1 << (((pin - 8) * 4) + 2));

		//set mode and config
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH |= mode << ((pin - 8) * 4);
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->CRH |= config << (((pin - 8) * 4) + 2);
	}
}

//Send 0 or 1 to pin
void digitalWrite(uint8_t port, uint8_t pin, bool value)
{
	if (value == 1)
	{
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->ODR |= value << pin;
	}
	else if (value == 0)
	{
		((GPIO_TypeDef*)(GPIOA_BASE + (port - 2) * 0x0400))->ODR &= ~(1 << pin);
	}
}

//hard delay, empty cycle
void delay(int millisec)
{
	unsigned long start_time = TimeFromStart;
	while (TimeFromStart != (start_time + (millisec * 1000)))
	{
		//waiting
	}
}

//count pulse length and return
uint64_t pulseIN(uint8_t PIN)
{
	uint16_t PulseLength = 0;
	unsigned long StartCount = 0;
	bool HIGHsignal = false;

	while (1)
	{
		//when comes 1 -> start count
		if (GPIOA->IDR & (1 << (PIN)) && HIGHsignal == false)
		{
			StartCount = TimeFromStart;
			HIGHsignal = true;
		}

		//when comes 0 -> stop count and exit from cycle
		if ((HIGHsignal == true) && !(GPIOA->IDR & (1 << (PIN))))
		{
			HIGHsignal = false;
			PulseLength = TimeFromStart - StartCount;
			break;
		}
	}

	delay(1);

	return PulseLength;
}

//I2C STUFF-----------------------------------------------------------------------------
GPIO_InitTypeDef GPIO_InitStructure;

void I2C1_init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);
}

void I2C_WriteByte(uint8_t device_address, uint8_t address, uint8_t data)
{
   uint32_t stop = 0;
	//send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	//send slave address
	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	//send register address
	I2C_SendData(I2C1, address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	//send data
	I2C_SendData(I2C1, data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	//send STOP
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
   {
      stop++;
      if(stop == 1000)
      {
         break;
      }
   }
}

void I2C_burst_write(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	while (n_data--)
	{
		I2C_SendData(I2C1, *data++);
		n_data--;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}
//END OF I2C STUFF-----------------------------------------------------------------------------

//PCA9685 STUFF--------------------------------------------------------------------------------------
void PCA9685_reset(uint8_t device_address)
{
	//	uint8_t reset_data[2];
	//	reset_data[0] = 0x00;
	//	reset_data[1] = 0x06;
	I2C_WriteByte(device_address, 0x00, 0x06);
}

void PCA9685_init(uint8_t device_address)
{
	PCA9685_reset(device_address);

	//sleep
	I2C_WriteByte(device_address, 0x00, 0b10000);

	//prescale
	//I2C_WriteByte(0xFE, 0x3); MAX
	I2C_WriteByte(device_address, 0xFE, 0x64); //60hz 

	//normal mode
	I2C_WriteByte(device_address, 0x00, 0xA1);
}

void PCA9685_setPWM(uint8_t device_address, uint8_t ServoNum, uint16_t on, uint16_t off)
{
	//uint8_t outputBuffer[4] = { on, (on >> 8), off, (off >> 8) };

	//I2C_burst_write(0x06 + 4*ServoNum, 4, outputBuffer);
	//I2C_WriteByte(0x06 + 4*ServoNum, on);
	//I2C_WriteByte(0x06 + 4*ServoNum+1, on >> 8);

	I2C_WriteByte(device_address, 0x06 + 4 * ServoNum + 2, off);
	I2C_WriteByte(device_address, 0x06 + 4 * ServoNum + 3, off >> 8);
}

void SetServoAngle(uint8_t n, double angle)
{
   if(angle < 0)
   {
      angle = 0;
   }
	double deg_to_pulse = 100 + (SERVOMAX - SERVOMIN) * angle / 180;
	double deg_to_pulse_left = 100 + (SERVOMAX - SERVOMIN) * (180-angle) / 180;

	if (n < 9)
	{
        if(n == 2 || n == 5 || n == 8)
        {
            PCA9685_setPWM(PCA9685_ADDRESS_1, n, 0, deg_to_pulse_left);
        }
        else
        {
            PCA9685_setPWM(PCA9685_ADDRESS_1, n, 0, deg_to_pulse);
        }		
	}
	else
	{
        if(n == 10 || n == 13 || n == 16)
        {
            PCA9685_setPWM(PCA9685_ADDRESS_2, n-2, 0, deg_to_pulse_left);
        }
        else
        {
            PCA9685_setPWM(PCA9685_ADDRESS_2, n-2, 0, deg_to_pulse);
        }		
	}	
}

void SpeedControl_SetServoAngle(uint8_t n, double angle, uint8_t pause)
{

}
//END OF PCA9685 STUFF-------------------------------------------------------------------------------------------------------

//TIMERS STUFF--------------------------------------------------------------------------------------
/*
void TIMER3_Init_Millisec()
{
	//example code for timer millisec counter

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef timer_base;                    //create struct instance
	TIM_TimeBaseStructInit(&timer_base);                   //fill with default values
	timer_base.TIM_Prescaler = 72 - 1;                     //fill your value (prescale from 0 to 65535, 1 000 000 Hz)
	timer_base.TIM_Period = 1000 - 1;							 //
	TIM_TimeBaseInit(TIM3, &timer_base);                   //initialize timer

	__disable_irq();													 //disable all interruptions
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);             //tune timers interruptions parametres
	NVIC_EnableIRQ(TIM3_IRQn);                             //enable interruptions of specific timer
	__enable_irq();													 //enable all interruptions
	TIM_Cmd(TIM3, ENABLE);                                 //start count
}

void TIM3_IRQHandler(void)
{
	TIM_ClearFlag(TIM3, TIM_IT_Update);							 //reset interruption flag
	Millis++;															 //increment every 1 millisec
}
*/

//function for SysTick timer interruption
void SysTick_Handler(void)
{
	//increments every 1 microsec

	TimeFromStart++;
}
//END OF TIMERS STUFF-------------------------------------------------------------------------------

//EXTERNAL INTERRUPTIONS STUFF----------------------------------------------------------------------

void EXTI0_IRQHandler(void)
{
	//reset interruption flag
	EXTI->PR = EXTI_PR_PR0;

	//if FRONT
	if (GPIOA->IDR & 1)
	{
		//get time
		CurrentInterruptionTime = TimeFromStart;
	}
	else
	{
		//if END -> evaluate delta
		deltaInterruptionTime = TimeFromStart - CurrentInterruptionTime;

		//if we have found start of package after 9100 mcs
		if (StartPackage == true)
		{
			//consistently store channel values to array 
			Channel[ChannelCounter] = deltaInterruptionTime;
			ChannelCounter++;
			if (ChannelCounter == 6)
			{
				//when package is fully gathered
				StartPackage = false;
			}
			else if (ChannelCounter == 5)
			{
				//for emergency stop check 4 channel with new value
				if (Channel[4] > 1100)
				{
					ServoEnable = 0;
					
				}
				else
				{
					ServoEnable = 1;
				}
				digitalWrite(PORT_A, 12, ServoEnable); //1-off 0-on     
				digitalWrite(PORT_C, 13, ServoEnable); //1-led off, 0-led on				
			}
		}
		else if (deltaInterruptionTime > 5000)
		{
			//if long HIGH level detected -> we found start of the package
			StartPackage = true;
			ChannelCounter = 0;
		}
	}
}

void EXTI0_init(void)
{
	//clock for PA0 pin and alternate function
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	//GPIO PA0 pin initialization
	GPIO_InitTypeDef gpio_cfg;
	GPIO_StructInit(&gpio_cfg);

	gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
	gpio_cfg.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &gpio_cfg);

	//connecting alternate funcion of PA0 pin to external interruptions
	AFIO->EXTICR[0] &= ~(AFIO_EXTICR1_EXTI0_PA);

	//some preparations...
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->FTSR |= EXTI_RTSR_TR0;
	EXTI->PR = EXTI_RTSR_TR0;
	EXTI->IMR |= EXTI_RTSR_TR0;

	//enable external interruptions of pin 0
	NVIC_EnableIRQ(EXTI0_IRQn);
}
//END OF EXTERNAL INTERRUPTIONS STUFF---------------------------------------------------------------

void FindAngles(int x, int y, int z)
{
	double p = sqrt( x*x + y*y );
	double OC = sqrt( p*p + z*z );
	double AC = sqrt( (p-OA)*(p-OA) + z*z );

	Qrad = atan(z / (p-OA));
	Q0rad = acos((AB * AB + AC * AC - BC * BC) / (2 * AB * AC));

	if (x == 0)
	{
		q0rad = 0;
	}		
	else
	{
		q0rad = atan2(y, x);
	}

	q0 = q0rad * 180 / pi + 45;

	q1rad = Qrad + Q0rad;
	q1 = q1rad * 180 / pi + 90;

	q2rad = acos( (AB*AB + BC*BC - AC*AC) / (2 * AB * BC) );
	q2 = q2rad * 180 / pi;
}

