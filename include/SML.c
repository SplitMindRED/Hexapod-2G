/***********************************************
*SplitMind Library
*Version 0.1
************************************************/

#include "SML.h"

unsigned long TimeFromStart = 0;
uint16_t delay_count = 0;
const int OA = 45;
const int AB = 85;
double pi = 3.14;
double q0grad, q1grad, q2grad;
double q0, q1, q2, Q, Qgrad, Q0, Q0grad;

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

//function for SysTick timer interruption
void SysTick_Handler(void)
{
	//increments every 1 microsec

	TimeFromStart++;
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
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);
}

void I2C_WriteByte(uint8_t device_address, uint8_t address, uint8_t data)
{
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

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
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
	double deg_to_pulse = 100 + (SERVOMAX - SERVOMIN) * angle / 180;

	if (n < 9)
	{
		PCA9685_setPWM(PCA9685_ADDRESS_1, n, 0, deg_to_pulse);
	}
	else
	{
		PCA9685_setPWM(PCA9685_ADDRESS_2, n-2, 0, deg_to_pulse);
	}	
}
//END OF PCA9685 STUFF-------------------------------------------------------------------------------------------------------

void FindAngles(int x, int y, int z)
{
	//long int TimeStart = micros();
	double p = sqrt(x * x + y * y);
	double L_OB = sqrt(p * p + z * z);

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

