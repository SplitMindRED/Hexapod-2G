#include "SML.h"

float Vx, Vy, Vz;
float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;


void Setup()
{
   //SETUP SYSTICK TIMER
   SysTick_Config(SystemCoreClock / 1000000);   //1 mcs
    
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

    for (uint8_t ServoNum = 0; ServoNum < 18; ServoNum++)
    {
       SetServoAngle(ServoNum, 90);
    }
    
   __enable_irq();

   delay(100);
   
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

void FullServoTest(bool Mode)
{
   if(Mode == 0)
   {
      for (int i = 0; i < 18; i += 3)
      {
         SetServoAngle(i, (Throttle * 0.18 - 180));
         SetServoAngle(i + 1, (Pitch * 0.18 - 180));
         SetServoAngle(i + 2, (Roll * 0.18 - 180));
      }
   }
   else
   {
      for (int i = 0; i < 18; i += 3)
      {
         
         SetServoAngle(i, (Channel[0] * 0.18 - 108));
         SetServoAngle(i + 1, (Channel[1] * 0.18 - 108));
         SetServoAngle(i + 2, (Channel[2] * 0.18 - 108));
      }
   }
}

void FrontRightLeg_square()
{
   int pause = 20;
   int delaypause = 2500;

   for (uint16_t Y = 0; Y < 50; Y++)
   {
      FindAngles(2, 50, Y, -50);

      SetServoAngle(6, q0);
      SetServoAngle(7, q1);
      SetServoAngle(8, q2);
      delay(pause);
   }

   delay(delaypause);

   for (uint16_t X = 50; X < 100; X++)
   {
      FindAngles(2, X, 50, -50);

      SetServoAngle(6, q0);
      SetServoAngle(7, q1);
      SetServoAngle(8, q2);
      delay(pause);
   }

   delay(delaypause);

   for (uint16_t Y = 50; Y > 0; Y--)
   {
      FindAngles(2, 100, Y, -50);

      SetServoAngle(6, q0);
      SetServoAngle(7, q1);
      SetServoAngle(8, q2);
      delay(pause);
   }

   delay(delaypause);

   for (uint16_t X = 100; X > 50; X--)
   {
      FindAngles(2, X, 0, -50);

      SetServoAngle(6, q0);
      SetServoAngle(7, q1);
      SetServoAngle(8, q2);
      delay(pause);
   }

   delay(delaypause);
}

int main()
{
   Setup();

   //main loop
   while (1)
   {
      //FullServoTest(1);      

      int pause = 20;
      int delaypause = 2500;

//line for mid right leg
      for (uint16_t Y = 0; Y < 50; Y++)
      {
         FindAngles(1, 50, Y, -50);

         SetServoAngle(3, q0);
         SetServoAngle(4, q1);
         SetServoAngle(5, q2);
         delay(pause);
      }
      
      delay(delaypause);

      for (uint16_t Y = 50; Y > 0; Y--)
      {
         FindAngles(1, 50, Y, -50);

         SetServoAngle(3, q0);
         SetServoAngle(4, q1);
         SetServoAngle(5, q2);
         delay(pause);
      }

      delay(delaypause);



//line for back right leg
      for (int16_t Y = 0; Y > -50; Y--)
      {
         FindAngles(0, 50, Y, -50);

         SetServoAngle(0, q0);
         SetServoAngle(1, q1);
         SetServoAngle(2, q2);
         delay(pause);
      }
      
      delay(delaypause);

      for (int16_t Y = -50; Y < 0; Y++)
      {
         FindAngles(0, 50, Y, -50);

         SetServoAngle(0, q0);
         SetServoAngle(1, q1);
         SetServoAngle(2, q2);
         delay(pause);
      }

      delay(delaypause);

//line for front left leg
//      for (int16_t Y = 0; Y < 50; Y++)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(9, q0);
//         SetServoAngle(10, q1);
//         SetServoAngle(11, q2);
//         delay(pause);
//      }

//      delay(delaypause);

//      for (int16_t Y = 50; Y > 0; Y--)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(9, q0);
//         SetServoAngle(10, q1);
//         SetServoAngle(11, q2);
//         delay(pause);
//      }

//      delay(delaypause);

//line for mid left leg
//      for (int16_t Y = 0; Y < 50; Y++)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(12, q0);
//         SetServoAngle(13, q1);
//         SetServoAngle(14, q2);
//         delay(pause);
//      }

//      delay(delaypause);

//      for (int16_t Y = 50; Y > 0; Y--)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(12, q0);
//         SetServoAngle(13, q1);
//         SetServoAngle(14, q2);
//         delay(pause);
//      }

//      delay(delaypause);
      
//line for back left leg
//      for (int16_t Y = 0; Y > -50; Y--)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(15, q0);
//         SetServoAngle(16, q1);
//         SetServoAngle(17, q2);
//         delay(pause);
//      }

//      delay(delaypause);

//      for (int16_t Y = -50; Y < 0; Y++)
//      {
//         FindAngles(-50, Y, -50);

//         SetServoAngle(15, q0);
//         SetServoAngle(16, q1);
//         SetServoAngle(17, q2);
//         delay(pause);
//      }

//      delay(delaypause);

   }
}
