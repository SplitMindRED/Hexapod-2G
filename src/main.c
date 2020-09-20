#include "SML.h"

float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;
//x(t) function
float Xt[6], Yt[6], Zt[6];

unsigned long next_time = 1000;

void setup()
{	
    //SETUP SYSTICK TIMER
    SysTick_Config(SystemCoreClock / 1000000);   //1 mcs
    
    for (uint8_t i = 0; i < 6; i++)
    {
       channel[i] = 0;
    }

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
         local_current_leg_position[i][j] = 0;
         local_target_leg_position[i][j] = 0;         
      }

      //set all legs start position
      moveLeg(i, local_start_point[i][0], local_start_point[i][1], local_start_point[i][2]);
   }
      
   //led on
   digitalWrite(PORT_C, 13, 0);
}

void fullServoTest()
{
   for (int i = 0; i < 18; i += 3)
   {         
      setServoAngle(i, (channel[0] * 0.18 - 108));
      setServoAngle(i + 1, (channel[1] * 0.18 - 108));
      setServoAngle(i + 2, (channel[2] * 0.18 - 108));
   }
}

void heightTest(float H)
{
   for (uint8_t i = 0; i < 6; i++)
   {
      moveLeg(i, local_start_point[i][0], local_start_point[i][1], -H);
   }
}

void hexapodMove()
{
   if ((time_from_start + 1000) >= next_time)
   {
      if (fabs(Vx) <= 20 && fabs(Vy) <= 20)
      {
         moveLeg(0, local_start_point[0][0], local_start_point[0][1], -H);
         moveLeg(2, local_start_point[2][0], local_start_point[2][1], -H);
         moveLeg(4, local_start_point[4][0], local_start_point[4][1], -H);
         phase[0] = 0;

         moveLeg(1, local_start_point[1][0], local_start_point[1][1], -H);
         moveLeg(3, local_start_point[3][0], local_start_point[3][1], -H);
         moveLeg(5, local_start_point[5][0], local_start_point[5][1], -H);
         phase[1] = 1;
      }
      else
      {
         //group 0-------------------------------------------------------
         if (phaseControl(0) == 0)
         {
            //X
            Xt[0] = local_current_leg_position[0][0] - Vx / 1000;
            Xt[2] = local_current_leg_position[2][0] - Vx / 1000;
            Xt[4] = local_current_leg_position[4][0] - Vx / 1000;

            //Y
            Yt[0] = local_current_leg_position[0][1] - Vy / 1000;
            Yt[2] = local_current_leg_position[2][1] - Vy / 1000;
            Yt[4] = local_current_leg_position[4][1] - Vy / 1000;

            //Z
            Zt[0] = -H;
            Zt[2] = -H;
            Zt[4] = -H;
         }
         else
         {
            //X
            Xt[0] = local_current_leg_position[0][0] + Vx / 1000;
            Xt[2] = local_current_leg_position[2][0] + Vx / 1000;
            Xt[4] = local_current_leg_position[4][0] + Vx / 1000;

            //Y
            Yt[0] = local_current_leg_position[0][1] + Vy / 1000;
            Yt[2] = local_current_leg_position[2][1] + Vy / 1000;
            Yt[4] = local_current_leg_position[4][1] + Vy / 1000;

            //Z
            Zt[0] = -k * (Yt[0] + Y_OFFSET) * (Yt[0] + Y_OFFSET) - k * (Xt[0] - X_OFFSET) * (Xt[0] - X_OFFSET) - H + dH;
            Zt[2] = -k * (Yt[2] - Y_OFFSET) * (Yt[2] - Y_OFFSET) - k * (Xt[2] - X_OFFSET) * (Xt[2] - X_OFFSET) - H + dH;
            Zt[4] = -k * Yt[4] * Yt[4] - k * (Xt[4] + X_OFFSET + 30) * (Xt[4] + X_OFFSET + 30) - H + dH;
         }

         //group 1---------------------------------------------------
         if (phaseControl(1) == 0)
         {
            //X
            Xt[1] = local_current_leg_position[1][0] - Vx / 1000;
            Xt[3] = local_current_leg_position[3][0] - Vx / 1000;
            Xt[5] = local_current_leg_position[5][0] - Vx / 1000;

            //Y
            Yt[1] = local_current_leg_position[1][1] - Vy / 1000;
            Yt[3] = local_current_leg_position[3][1] - Vy / 1000;
            Yt[5] = local_current_leg_position[5][1] - Vy / 1000;

            //Z
            Zt[1] = -H;
            Zt[3] = -H;
            Zt[5] = -H;
         }
         else
         {
            //X
            Xt[1] = local_current_leg_position[1][0] + Vx / 1000;
            Xt[3] = local_current_leg_position[3][0] + Vx / 1000;
            Xt[5] = local_current_leg_position[5][0] + Vx / 1000;

            //Y
            Yt[1] = local_current_leg_position[1][1] + Vy / 1000;
            Yt[3] = local_current_leg_position[3][1] + Vy / 1000;
            Yt[5] = local_current_leg_position[5][1] + Vy / 1000;

            //Z
            Zt[1] = -k * Yt[1] * Yt[1] - k * (Xt[1] - X_OFFSET - 30)*(Xt[1] - X_OFFSET - 30) - H + dH;
            Zt[3] = -k * (Yt[3] - Y_OFFSET) * (Yt[3] - Y_OFFSET) - k * (Xt[3] + X_OFFSET) * (Xt[3] + X_OFFSET) - H + dH;
            Zt[5] = -k * (Yt[5] + Y_OFFSET) * (Yt[5] + Y_OFFSET) - k * (Xt[5] + X_OFFSET) * (Xt[5] + X_OFFSET) - H + dH;        
         }

         for (uint8_t i = 0; i < 6; i++)
         {
            moveLeg(i, Xt[i], Yt[i], Zt[i]);
         }
      }     

      next_time = time_from_start + 1000; //1 ms
   }
}

int main()
{
   setup();   

   //main loop
   while (1)
   {
      //Vy = (channel[1] * 0.8 - 880);
      //H = (channel[2] * 0.07 + 8);
      Vx = map(channel[0], 600, 1600, -1100, 1100);

      if (fabs(Vx) < 20)
      {
         Vx = 0;         
      }

      Vy = map(channel[1], 600, 1600, -1100, 1100);   

      if (fabs(Vy) < 20)
      {
         Vy = 0;
      }
      
      H = map(channel[2], 600, 1600, 70, 110);
      dH = 30 * H * 1.1 / 70;

      k = 4 * dH / (diameter * diameter);

      //SWC switch mode
      if (channel[5] > 1300)                                //low
      {
         fullServoTest();         
      }
      else if (channel[5] < 1200 && channel[5] > 900)       //mid
      {
         heightTest(H);
      }
      else if (channel[5] < 700)                            //high
      {
         hexapodMove();
      }   
   }
}
