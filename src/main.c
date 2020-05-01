#include "SML.h"

float Vx, Vy, Vz;
float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;
float Xt, Yt[6], Zt;

int16_t Y_amplitude = 0;


unsigned long NextTime = 1000;


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
    delay(100);
    
    EXTI0_init();
    
   __enable_irq();

   delay(1000);
   
//   for (uint8_t ServoNum = 0; ServoNum < 18; ServoNum++)
//    {
//       SetServoAngle(ServoNum, 90);
//    }
    
   for (uint8_t i = 0; i < 6; i++)
   {
      for (uint8_t j = 0; j < 3; j++)
      {
         LocalTargetLegPosition[i][j] = 0;
         LocalCurrentLegPosition[i][j] = 0;
      }
   }

   MoveLeg(0, 57.276, -57.276, -83);
   MoveLeg(1, 81, 0, -83);
   MoveLeg(2, 57.276, 57.276, -83);
   MoveLeg(3, -57.276, 57.276, -83);
   MoveLeg(4, -81, 0, -83);
   MoveLeg(5, -57.276, -57.276, -83);

   
   
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
   
   Vy = 30;
   uint8_t Phase = 1;
   Y_amplitude = 60;                   //60 mm aplitude in Y axis

   //main loop
   while (1)
   {
      //FullServoTest(1);  

      if ((TimeFromStart + 1000) >= NextTime)
      {

         switch (Phase)
         {
         case 1:

            //Y
            Yt[2] = LocalCurrentLegPosition[2][1] - Vy / 1000;

            MoveLeg(2, 60, Yt[2], -50);

            if (Yt[2] <= -30)
            {
               Phase = 2;
            }

            break;

         case 2:

            //Y
            Yt[2] = LocalCurrentLegPosition[2][1] + Vy / 1000;

            //Z
            Zt = -0.0166 * Yt[2] * Yt[2] - 35;

            MoveLeg(2, 60, Yt[2], Zt);

            if (Yt[2] >= 30)
            {
               Phase = 1;
            }

            break;
         }

         NextTime = TimeFromStart + 1000; //1 ms
      }

      
   }
}
