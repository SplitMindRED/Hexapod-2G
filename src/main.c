#include "SML.h"

float Vx, Vy, Vz;
float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;
bool ServoEnable = false;

void Setup()
{
//SETUP SYSTICK TIMER------------------------------------

   //__enable_irq();

   //SysTick -> LOAD = SystemCoreClock/1000;	//1ms
   //SysTick->LOAD = SystemCoreClock / 1000000;	//1mcs

   //SysTick->CTRL = 0b111;						      //start count
   SysTick_Config(SystemCoreClock / 1000000);
//------------------------------------------------------
    
    for (uint8_t i = 0; i < 6; i++)
    {
       Channel[i] = 0;
    }

    //5 channels input
    //pinMode(PORT_A, 7, INPUT, INPUT_PULL_UP_DOWN);
    //pinMode(PORT_A, 6, INPUT, INPUT_PULL_UP_DOWN);
    //pinMode(PORT_A, 5, INPUT, INPUT_PULL_UP_DOWN);
    //pinMode(PORT_A, 4, INPUT, INPUT_PULL_UP_DOWN);
    //pinMode(PORT_A, 3, INPUT, INPUT_PULL_UP_DOWN);

    //Servo Enable (OE - output enable)
    pinMode(PORT_A, 12, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);

    //onboard led
    pinMode(PORT_C, 13, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);

    //led off
    digitalWrite(PORT_C, 13, 1);

    //disable servo control
    digitalWrite(PORT_A, 12, 1); //1-off 0-on

    I2C1_init();
    PCA9685_init(PCA9685_ADDRESS_1);
    PCA9685_init(PCA9685_ADDRESS_2);
    
    EXTI0_init();
    
   __enable_irq();
   //led on
   digitalWrite(PORT_C, 13, 0);
}

void GetSignals()
{
   //get 3 input signals
   Throttle = pulseIN(5);
   Pitch = pulseIN(6);
   Roll = pulseIN(7);
   Yaw = pulseIN(4);
   Switch = pulseIN(3);

   if (Switch > 1500)
   {
      ServoEnable = false;
   }
   else
   {
      ServoEnable = true;
   }
   
   digitalWrite(PORT_A, 12, ServoEnable); //1-off 0-on
}

void ServoTest4()
{
    //set angle based on input
    SetServoAngle(0, (Throttle * 0.18 - 180));
    SetServoAngle(1, (Pitch * 0.18 - 180));
    SetServoAngle(2, (Roll * 0.18 - 180));        
    SetServoAngle(5, (Yaw * 0.18 - 180));

    //PCA9685_setPWM(PCA9685_ADDRESS_1, 0, 0, Throttle*0.465-365);
    //PCA9685_setPWM(PCA9685_ADDRESS_1, 1, 0, Pitch * 0.465 - 365);
    //PCA9685_setPWM(PCA9685_ADDRESS_1, 2, 0, Roll * 0.465 - 365);
}

void FullServoTest()
{
   for (int i = 0; i < 18; i += 3)
   {
      //SetServoAngle(i, (Throttle * 0.18 - 180));
      //SetServoAngle(i + 1, (Pitch * 0.18 - 180));
      //SetServoAngle(i + 2, (Roll * 0.18 - 180));
      
      SetServoAngle(i, (Channel[0] * 0.18 - 108));
      SetServoAngle(i + 1, (Channel[1] * 0.18 - 108));
      SetServoAngle(i + 2, (Channel[2] * 0.18 - 108));
   }
}

int main()
{
   Setup();

   //main loop
   while (1)
   {	       
      //GetSignals();
       
      //ServoTest4();
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
      
      FullServoTest();

   }
}
