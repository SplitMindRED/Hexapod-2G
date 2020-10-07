/***********************************************
*	SplitMind Library
*	Version 0.4
*	
*	WRITING BITs
*	a |= 1 << 7;					//set 1 in 7th bit
*	a &= ~(1 << 3);				//set 0 in 3th bit
*	a ^= 1 << 5;					//inversion of 5th bit
*	a |= 1 << 7 | 1 << 8		   //sets 1 to 7th and 8th bits
*	
*	READING BITs
*	if( a & (1<<7) )				//if 7th bit in "a" var equals 1 -> true
*  a & ( 1 << 7 | 1 << 8 )	   //checks 7th and 8th bits
*
************************************************/

#include "SML.h"

unsigned long time_from_start = 0;
unsigned long current_interruption_time = 0;
uint16_t delta_interruption_time = 0;
uint8_t channel_counter = 0;
bool start_package = false;
float channel[6];
float Vx = 0, Vy = 0, Vz = 0;
float input_roll = 0, input_pitch = 0, input_yaw = 0;
//float current_roll = 0, current_pitch = 0, current_yaw = 0;

uint16_t delay_count = 0;

bool servo_enable = false;

//geometry variables--------------------------
const uint8_t OA = 37;
const uint8_t AB = 44;
const uint8_t BC = 83;
double pi = 3.14;
double q0rad, q1rad, q2rad, Qrad, Q0rad;
double q0, q1, q2;
//--------------------------------------------

//movements and trajectory variables

int16_t local_start_point[6][3] = 
{
	 {X_OFFSET,       -Y_OFFSET,  -STARTHEIGHT},
	 {X_OFFSET+30,    0,          -STARTHEIGHT},
	 {X_OFFSET,       Y_OFFSET,   -STARTHEIGHT},
	 {-X_OFFSET,      Y_OFFSET,   -STARTHEIGHT},
	 {-X_OFFSET-30,   0,          -STARTHEIGHT},
	 {-X_OFFSET,      -Y_OFFSET,  -STARTHEIGHT},
};

bool phase[2] = { 0, 1 };

float diameter = DIAMETER;                   //60 mm aplitude in step

float k = 0;
float dH = DELTAHEIGHT;
float H = STARTHEIGHT;

unsigned long next_time = 1000;

struct Legs Leg[6];
//--------------------------------------------

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
	unsigned long start_time = time_from_start;
	while (time_from_start <= (start_time + (millisec * 1000)))
	{
		//waiting
	}
}

//count pulse length and return
uint64_t pulseIN(uint8_t PIN)
{
	uint16_t pulse_length = 0;
	unsigned long start_count = 0;
	bool high_signal = false;

	while (1)
	{
		//when comes 1 -> start count
		if (GPIOA->IDR & (1 << (PIN)) && high_signal == false)
		{
			start_count = time_from_start;
			high_signal = true;
		}

		//when comes 0 -> stop count and exit from cycle
		if ((high_signal == true) && !(GPIOA->IDR & (1 << (PIN))))
		{
			high_signal = false;
			pulse_length = time_from_start - start_count;
			break;
		}
	}

	delay(1);

	return pulse_length;
}

float map(float have, float have_min, float have_max, float need_min, float need_max)
{
	float ratio = 0;
	float add = 0;
	
	ratio = (need_max - need_min) / (have_max - have_min);
	add = need_max - (have_max * ratio);
	

	return (have * ratio + add);
}

//I2C----------------------------------------------------------------------------------------------------
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

void I2C_writeByte(uint8_t device_address, uint8_t address, uint8_t data)
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

void I2C_burstWrite(uint8_t device_address, uint8_t address, uint8_t n_data, uint8_t* data)
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
//END OF I2C---------------------------------------------------------------------------------------------

//PCA9685------------------------------------------------------------------------------------------------
void PCA9685_reset(uint8_t device_address)
{
	I2C_writeByte(device_address, 0x00, 0x06);
}

void PCA9685_init(uint8_t device_address)
{
	PCA9685_reset(device_address);

	//sleep
	I2C_writeByte(device_address, 0x00, 0b10000);

	//prescale
	//I2C_writeByte(0xFE, 0x3); MAX
	I2C_writeByte(device_address, 0xFE, 0x64); //60hz 

	//normal mode
	I2C_writeByte(device_address, 0x00, 0xA1);

	//0 PWM
	for (uint8_t servo_num = 0; servo_num < 18; servo_num++)
	{
		//TODO test 0 pwm
		//PCA9685_setPWM(PCA9685_ADDRESS_1, servo_num, 0, 1);
	}
	
}

void PCA9685_setPWM(uint8_t device_address, uint8_t servo_num, uint16_t on, uint16_t off)
{
	I2C_writeByte(device_address, 0x06 + 4 * servo_num + 2, off);
	I2C_writeByte(device_address, 0x06 + 4 * servo_num + 3, off >> 8);
}

void setServoAngle(uint8_t servo_num, double angle)
{
	if(angle < 0)
	{
		angle = 0;
	}
	double deg_to_pulse = 100 + (SERVOMAX - SERVOMIN) * angle / 180;
	double deg_to_pulse_left = 100 + (SERVOMAX - SERVOMIN) * (180-angle) / 180;

	if (servo_num < 9)
	{
		if(servo_num == 2 || servo_num == 5 || servo_num == 8)
		{
			PCA9685_setPWM(PCA9685_ADDRESS_1, servo_num, 0, deg_to_pulse_left);
		}
		else
		{
			PCA9685_setPWM(PCA9685_ADDRESS_1, servo_num, 0, deg_to_pulse);
		}		
	}
	else
	{
		if(servo_num == 10 || servo_num == 13 || servo_num == 16)
		{
			PCA9685_setPWM(PCA9685_ADDRESS_2, servo_num -2, 0, deg_to_pulse_left);
		}
		else
		{
			PCA9685_setPWM(PCA9685_ADDRESS_2, servo_num -2, 0, deg_to_pulse);
		}		
	}	
}

bool phaseControl(uint8_t group_num)
{
	float X, Y, Z;
	float x, y, r;

	X = Leg[group_num + 2].current_x;
	Y = Leg[group_num + 2].current_y;
	Z = Leg[group_num + 2].current_z;

	//parameters in circle formula (x+x0)^2 + (y+y0)^2 = r^2
	x = X - X_OFFSET + (2 * group_num * X_OFFSET);	//short form. if gropu=0 -> -X_OFFSET, if group = 1 -> +X_OFFSET
	y = Y - Y_OFFSET;
	r = diameter / 2;

	if (x*x + y*y >= r*r)
	{
		phase[group_num] = !phase[group_num];		
	}

	return phase[group_num];
}
//END OF PCA9685-----------------------------------------------------------------------------------------

//TIMERS-------------------------------------------------------------------------------------------------
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
	time_from_start++;
}
//END OF TIMERS------------------------------------------------------------------------------------------

//EXTERNAL INTERRUPTIONS---------------------------------------------------------------------------------
//Interruptions for PPM 
void EXTI0_IRQHandler(void)
{
	//if FRONT
	if (GPIOA->IDR & 1)
	{
		//get time
		current_interruption_time = time_from_start;
	}
	else
	{
		//if END -> evaluate delta
		delta_interruption_time = time_from_start - current_interruption_time;

		//if we have found start of package after 9100 mcs
		if (start_package == true)
		{
			//consistently store channel values to array 
			channel[channel_counter] = delta_interruption_time;
			channel_counter++;
			if (channel_counter == 6)
			{
				//when package is fully gathered
				start_package = false;
			}
			else if (channel_counter == 5)
			{
				//for emergency stop check 4 channel with new value
				if (channel[4] > 1100)
				{
					servo_enable = 0;					
				}
				else
				{
					servo_enable = 1;
				}
				digitalWrite(PORT_A, 12, servo_enable); //1-off 0-on     
				digitalWrite(PORT_C, 13, servo_enable); //1-led off, 0-led on				
			}
		}
		else if (delta_interruption_time > 5000)
		{
			//if long HIGH level detected -> we found start of the package
			start_package = true;
			channel_counter = 0;
		}
	}

	//reset interruption flag
	EXTI->PR = EXTI_PR_PR0;
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
//END OF EXTERNAL INTERRUPTIONS--------------------------------------------------------------------------

//HEXAPOD MOVEMENTS--------------------------------------------------------------------------------------
void findAngles(uint8_t leg_num, double x, double y, double z)
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
		//right side
		if (leg_num <= 2)
		{
			q0rad = atan2(y, x);
		}
		//left side
		else
		{
			q0rad = atan(y / x) + pi;
		}
	}

	switch (leg_num)
	{
	case 0:
		q0 = q0rad * 180 / pi + 135;		//back right
		break;

	case 1:
		q0 = q0rad * 180 / pi + 96;		//mid right
		break;

	case 2:
		q0 = q0rad * 180 / pi + 45;		//front right
		break;

	case 3:
		q0 = q0rad * 180 / pi - 45;		//front left
		break;

	case 4:
		q0 = q0rad * 180 / pi - 90;		//mid left
		break;

	case 5:
		q0 = q0rad * 180 / pi - 135;		//back left
		break;
	}

	//q0 = q0rad * 180 / pi + 45;		//front right
	//q0 = q0rad * 180 / pi + 96;		//mid right
	//q0 = q0rad * 180 / pi + 135;	//back right
	//q0 = q0rad * 180 / pi - 45;		//front left
	//q0 = q0rad * 180 / pi - 90;		//mid left
	//q0 = q0rad * 180 / pi - 135;	//back left

	q1rad = Qrad + Q0rad;
	q1 = q1rad * 180 / pi + 90;

	q2rad = acos( (AB*AB + BC*BC - AC*AC) / (2 * AB * BC) );
	q2 = q2rad * 180 / pi;
}

void moveLeg(uint8_t leg_num, double x, double y, double z)
{
	findAngles(leg_num, x, y, z);

	setServoAngle(leg_num * 3, q0);
	setServoAngle(leg_num * 3 + 1, q1);
	setServoAngle(leg_num * 3 + 2, q2);

	Leg[leg_num].current_x = x;
	Leg[leg_num].current_y = y;
	Leg[leg_num].current_z = z;
}
//END OF HEXAPOD MOVEMENTS-------------------------------------------------------------------------------
