#include "SML.h"

float Vx, Vy, Vz;
float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
bool ServoEnable = false;

int main()
{
//#######SETUP SYSTICK TIMER###########################
	//__disable_irq();
	__enable_irq();

	//SysTick -> LOAD = SystemCoreClock/1000;	//1ms
	SysTick->LOAD = SystemCoreClock / 1000000;	//1mcs

	SysTick->CTRL = 0b111;						//start count
//#######################################################

	//5 channels input
	pinMode(PORT_A, 7, INPUT, INPUT_PULL_UP_DOWN);
	pinMode(PORT_A, 6, INPUT, INPUT_PULL_UP_DOWN);
	pinMode(PORT_A, 5, INPUT, INPUT_PULL_UP_DOWN);
    pinMode(PORT_A, 4, INPUT, INPUT_PULL_UP_DOWN);
    pinMode(PORT_A, 3, INPUT, INPUT_PULL_UP_DOWN);
    
    //Servo Enable (OE - output enable)
    pinMode(PORT_A, 12, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);
    
	//onboard led
	pinMode(PORT_C, 13, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);

	//led off
	digitalWrite(PORT_C, 13, 1);
    
    digitalWrite(PORT_A, 12, 1); //1-off 0-on

	I2C1_init();
	PCA9685_init(PCA9685_ADDRESS_1);
    PCA9685_init(PCA9685_ADDRESS_2);

    //led on
	digitalWrite(PORT_C, 13, 1);
    
    delay(1000);
    
	//Home position
	SetServoAngle(0, 90);
	SetServoAngle(1, 90);
	SetServoAngle(2, 90);
    SetServoAngle(9, 90);
    SetServoAngle(10, 90);
    SetServoAngle(11, 90);

	//Home position
//	FindAngles(0, 84, 44);
//	SetServoAngle(0, q0grad);
//	SetServoAngle(1, q1grad);
//	SetServoAngle(2, q2grad);
    //digitalWrite(PORT_A, 12, 0);
	delay(500);

	CoordX = 0;
	CoordY = 84;
	CoordZ = 44;

	InputX = 0;
	InputY = 0;
	InputZ = 0;

	while (1)
	{
		//get 3 input signals
		float Throttle	= pulseIN(5);
		float Pitch		= pulseIN(6);
		float Roll		= pulseIN(7);
        float Yaw		= pulseIN(4);
		float Switch	= pulseIN(3);
        
        if(Switch > 1500)
        {
            ServoEnable = false;
        }
        else
        {
            ServoEnable = true;
        }
        digitalWrite(PORT_A, 12, ServoEnable); //1-off 0-on

		//set angle based on input
//		SetServoAngle(0, (Throttle * 0.18 - 180));
//		SetServoAngle(1, (Pitch * 0.18 - 180));
//		SetServoAngle(2, (Roll * 0.18 - 180));
//        
//        SetServoAngle(5, (Yaw * 0.18 - 180));
//        
        
        for(int i = 0; i<18; i+=3)
        {
            SetServoAngle(i, (Throttle * 0.18 - 180));
            SetServoAngle(i+1, (Pitch * 0.18 - 180));
            SetServoAngle(i+2, (Roll * 0.18 - 180));
        }
        //delay(500);
        
//        for(int i = 9; i<18; i++)
//        {
//            SetServoAngle(i, 45);
//        }
//        delay(500);
        
        
	}
}
