#include "SML.h"

float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;
float Xt[6], Yt[6], Zt[6];

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
       
   for (uint8_t i = 0; i < 6; i++)
   {
      for (uint8_t j = 0; j < 3; j++)
      {
         LocalTargetLegPosition[i][j] = 0;
         LocalCurrentLegPosition[i][j] = 0;
      }
   }

   MoveLeg(0, LocalStartPoint[0][0], LocalStartPoint[0][1], LocalStartPoint[0][2]);
   MoveLeg(1, LocalStartPoint[1][0], LocalStartPoint[1][1], LocalStartPoint[1][2]);
   MoveLeg(2, LocalStartPoint[2][0], LocalStartPoint[2][1], LocalStartPoint[2][2]);
   MoveLeg(3, LocalStartPoint[3][0], LocalStartPoint[3][1], LocalStartPoint[3][2]);
   MoveLeg(4, LocalStartPoint[4][0], LocalStartPoint[4][1], LocalStartPoint[4][2]);
   MoveLeg(5, LocalStartPoint[5][0], LocalStartPoint[5][1], LocalStartPoint[5][2]);   
   
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

void HeightTest(float H)
{
   MoveLeg(0, LocalStartPoint[0][0], LocalStartPoint[0][1], -H);
   MoveLeg(1, LocalStartPoint[1][0], LocalStartPoint[1][1], -H);
   MoveLeg(2, LocalStartPoint[2][0], LocalStartPoint[2][1], -H);
   MoveLeg(3, LocalStartPoint[3][0], LocalStartPoint[3][1], -H);
   MoveLeg(4, LocalStartPoint[4][0], LocalStartPoint[4][1], -H);
   MoveLeg(5, LocalStartPoint[5][0], LocalStartPoint[5][1], -H);
}

void HexapodMove()
{
   if ((TimeFromStart + 1000) >= NextTime)
   {
      if (fabs(Vx) <= 20 && fabs(Vy) <= 20)
      {
         MoveLeg(0, LocalStartPoint[0][0], LocalStartPoint[0][1], -H);
         MoveLeg(2, LocalStartPoint[2][0], LocalStartPoint[2][1], -H);
         MoveLeg(4, LocalStartPoint[4][0], LocalStartPoint[4][1], -H);
         Phase[0] = 0;

         MoveLeg(1, LocalStartPoint[1][0], LocalStartPoint[1][1], -H);
         MoveLeg(3, LocalStartPoint[3][0], LocalStartPoint[3][1], -H);
         MoveLeg(5, LocalStartPoint[5][0], LocalStartPoint[5][1], -H);
         Phase[1] = 1;
      }
      else
      {
         //group 0-------------------------------------------------------
         if (PhaseControl(0) == 0)
         {
            //X
            Xt[0] = LocalCurrentLegPosition[0][0] - Vx / 1000;
            Xt[2] = LocalCurrentLegPosition[2][0] - Vx / 1000;
            Xt[4] = LocalCurrentLegPosition[4][0] - Vx / 1000;

            //Y
            Yt[0] = LocalCurrentLegPosition[0][1] - Vy / 1000;
            Yt[2] = LocalCurrentLegPosition[2][1] - Vy / 1000;
            Yt[4] = LocalCurrentLegPosition[4][1] - Vy / 1000;

            //Z
            Zt[0] = -H;
            Zt[2] = -H;
            Zt[4] = -H;

            MoveLeg(0, Xt[0], Yt[0], Zt[0]);
            MoveLeg(2, Xt[2], Yt[2], Zt[2]);
            MoveLeg(4, Xt[4], Yt[4], Zt[4]);
         }
         else
         {
            //X
            Xt[0] = LocalCurrentLegPosition[0][0] + Vx / 1000;
            Xt[2] = LocalCurrentLegPosition[2][0] + Vx / 1000;
            Xt[4] = LocalCurrentLegPosition[4][0] + Vx / 1000;

            //Y
            Yt[0] = LocalCurrentLegPosition[0][1] + Vy / 1000;
            Yt[2] = LocalCurrentLegPosition[2][1] + Vy / 1000;
            Yt[4] = LocalCurrentLegPosition[4][1] + Vy / 1000;

            //Z
            Zt[0] = -k * (Yt[0] + Y_OFFSET) * (Yt[0] + Y_OFFSET) - k * (Xt[0] - X_OFFSET) * (Xt[0] - X_OFFSET) - H + dH;
            Zt[2] = -k * (Yt[2] - Y_OFFSET) * (Yt[2] - Y_OFFSET) - k * (Xt[2] - X_OFFSET) * (Xt[2] - X_OFFSET) - H + dH;
            Zt[4] = -k * Yt[4] * Yt[4] - k * (Xt[4] + X_OFFSET + 30) * (Xt[4] + X_OFFSET + 30) - H + dH;


            MoveLeg(0, Xt[0], Yt[0], Zt[0]);
            MoveLeg(2, Xt[2], Yt[2], Zt[2]);
            MoveLeg(4, Xt[4], Yt[4], Zt[4]);
         }

         //group 1---------------------------------------------------
         if (PhaseControl(1) == 0)
         {
            //X
            Xt[1] = LocalCurrentLegPosition[1][0] - Vx / 1000;
            Xt[3] = LocalCurrentLegPosition[3][0] - Vx / 1000;
            Xt[5] = LocalCurrentLegPosition[5][0] - Vx / 1000;

            //Y
            Yt[1] = LocalCurrentLegPosition[1][1] - Vy / 1000;
            Yt[3] = LocalCurrentLegPosition[3][1] - Vy / 1000;
            Yt[5] = LocalCurrentLegPosition[5][1] - Vy / 1000;

            //Z
            Zt[1] = -H;
            Zt[3] = -H;
            Zt[5] = -H;

            MoveLeg(1, Xt[1], Yt[1], Zt[1]);
            MoveLeg(3, Xt[3], Yt[3], Zt[3]);
            MoveLeg(5, Xt[5], Yt[5], Zt[5]);
         }
         else
         {
            //X
            Xt[1] = LocalCurrentLegPosition[1][0] + Vx / 1000;
            Xt[3] = LocalCurrentLegPosition[3][0] + Vx / 1000;
            Xt[5] = LocalCurrentLegPosition[5][0] + Vx / 1000;

            //Y
            Yt[1] = LocalCurrentLegPosition[1][1] + Vy / 1000;
            Yt[3] = LocalCurrentLegPosition[3][1] + Vy / 1000;
            Yt[5] = LocalCurrentLegPosition[5][1] + Vy / 1000;

            //Z
            Zt[1] = -k * Yt[1] * Yt[1] - k * (Xt[1] - X_OFFSET - 30)*(Xt[1] - X_OFFSET - 30) - H + dH;
            Zt[3] = -k * (Yt[3] - Y_OFFSET) * (Yt[3] - Y_OFFSET) - k * (Xt[3] + X_OFFSET) * (Xt[3] + X_OFFSET) - H + dH;
            Zt[5] = -k * (Yt[5] + Y_OFFSET) * (Yt[5] + Y_OFFSET) - k * (Xt[5] + X_OFFSET) * (Xt[5] + X_OFFSET) - H + dH;

            MoveLeg(1, Xt[1], Yt[1], Zt[1]);
            MoveLeg(3, Xt[3], Yt[3], Zt[3]);
            MoveLeg(5, Xt[5], Yt[5], Zt[5]);
         }
      }     

      NextTime = TimeFromStart + 1000; //1 ms
   }
}

int main()
{
   Setup();   

   //main loop
   while (1)
   {
      //Vy = (Channel[1] * 0.8 - 880);
      //H = (Channel[2] * 0.07 + 8);
      Vx = map(Channel[0], 600, 1600, -1100, 1100);

      if (fabs(Vx) < 20)
      {
         Vx == 0;         
      }

      Vy = map(Channel[1], 600, 1600, -1100, 1100);   

      if (fabs(Vy) < 20)
      {
         Vy == 0;
      }
      
      H = map(Channel[2], 600, 1600, 70, 120);

      k = 4 * dH / (Diameter * Diameter);

      //SWC switch mode
      if (Channel[5] > 1300)                                //low
      {
         FullServoTest(1);         
      }
      else if (Channel[5] < 1200 && Channel[5] > 900)       //mid
      {
         HeightTest(H);
      }
      else if (Channel[5] < 700)                            //high
      {
         HexapodMove();
      }   

   }
}
