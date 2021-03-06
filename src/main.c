#include "SML.h"

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

void rotateBody()
{
   //leg tips's coordinates in central CS (coordinate system)
   float p_base[6][3];
   float p_base_new[6][3];
   float p_delta[6][3];
   float delta_roll = 0;

   delta_roll = input_roll - current_roll;

   //right back leg
   //X
   p_base[0][0] = Leg[2].current_x + 55.86;
   //Y
   p_base[0][1] = Leg[2].current_y - 90.86;
   //Z
   p_base[0][2] = Leg[2].current_z;

   //right middle leg
   //X
   p_base[1][0] = Leg[1].current_x + 64.5;
   //Y
   p_base[1][1] = Leg[1].current_y;
   //Z
   p_base[1][2] = Leg[1].current_z;   

   //right front leg
   //X
   p_base[2][0] = Leg[2].current_x + 55.86;
   //Y
   p_base[2][1] = Leg[2].current_y + 90.86;
   //Z
   p_base[2][2] = Leg[2].current_z;   

   //left front leg
   //X
   p_base[3][0] = Leg[3].current_x - 55.86;
   //Y
   p_base[3][1] = Leg[3].current_y + 90.86;
   //Z
   p_base[3][2] = Leg[3].current_z;

   //left middle leg
   //X
   p_base[4][0] = Leg[4].current_x - 64.5;
   //Y
   p_base[4][1] = Leg[4].current_y;
   //Z
   p_base[4][2] = Leg[4].current_z;   

   //left back leg
   //X
   p_base[5][0] = Leg[5].current_x - 55.86;
   //Y
   p_base[5][1] = Leg[5].current_y - 90.86;
   //Z
   p_base[5][2] = Leg[5].current_z;

   //new right back leg coordinates
   p_base_new[0][0] = p_base[0][0] * cos(-delta_roll) - p_base[0][2] * sin(-delta_roll);
   p_base_new[0][2] = p_base[0][0] * sin(-delta_roll) + p_base[0][2] * cos(-delta_roll);

   //new right middle leg coordinates
   p_base_new[1][0] = p_base[1][0] * cos(-delta_roll) - p_base[1][2] * sin(-delta_roll);
   p_base_new[1][2] = p_base[1][0] * sin(-delta_roll) + p_base[1][2] * cos(-delta_roll);

   //new right front leg coordinates
   p_base_new[2][0] = p_base[2][0] * cos(-delta_roll) - p_base[2][2] * sin(-delta_roll);
   p_base_new[2][2] = p_base[2][0] * sin(-delta_roll) + p_base[2][2] * cos(-delta_roll);

   //new left front leg coordinates
   p_base_new[3][0] = p_base[3][0] * cos(-delta_roll) - p_base[3][2] * sin(-delta_roll);
   p_base_new[3][2] = p_base[3][0] * sin(-delta_roll) + p_base[3][2] * cos(-delta_roll);

   //new left middle leg coordinates
   p_base_new[4][0] = p_base[4][0] * cos(-delta_roll) - p_base[4][2] * sin(-delta_roll);
   p_base_new[4][2] = p_base[4][0] * sin(-delta_roll) + p_base[4][2] * cos(-delta_roll); 

   //new left back leg coordinates
   p_base_new[5][0] = p_base[5][0] * cos(-delta_roll) - p_base[5][2] * sin(-delta_roll);
   p_base_new[5][2] = p_base[5][0] * sin(-delta_roll) + p_base[5][2] * cos(-delta_roll);

   p_delta[0][0] = p_base_new[0][0] - p_base[0][0];
   p_delta[0][2] = p_base_new[0][2] - p_base[0][2];

   p_delta[1][0] = p_base_new[1][0] - p_base[1][0];
   p_delta[1][2] = p_base_new[1][2] - p_base[1][2];

   p_delta[2][0] = p_base_new[2][0] - p_base[2][0];
   p_delta[2][2] = p_base_new[2][2] - p_base[2][2];

   p_delta[3][0] = p_base_new[3][0] - p_base[3][0];
   p_delta[3][2] = p_base_new[3][2] - p_base[3][2];

   p_delta[4][0] = p_base_new[4][0] - p_base[4][0];
   p_delta[4][2] = p_base_new[4][2] - p_base[4][2];   

   p_delta[5][0] = p_base_new[5][0] - p_base[5][0];
   p_delta[5][2] = p_base_new[5][2] - p_base[5][2];

   Leg[0].Xt = Leg[0].current_x + p_delta[0][0];
   Leg[0].Yt = Leg[0].current_y;
   Leg[0].Zt = Leg[0].current_z + p_delta[0][2];

   Leg[1].Xt = Leg[1].current_x + p_delta[1][0];
   Leg[1].Yt = Leg[1].current_y;
   Leg[1].Zt = Leg[1].current_z + p_delta[1][2];

   Leg[2].Xt = Leg[2].current_x + p_delta[2][0];
   Leg[2].Yt = Leg[2].current_y;
   Leg[2].Zt = Leg[2].current_z + p_delta[2][2];

   Leg[3].Xt = Leg[3].current_x + p_delta[3][0];
   Leg[3].Yt = Leg[3].current_y;
   Leg[3].Zt = Leg[3].current_z + p_delta[3][2];

   Leg[4].Xt = Leg[4].current_x + p_delta[4][0];
   Leg[4].Yt = Leg[4].current_y;
   Leg[4].Zt = Leg[4].current_z + p_delta[4][2];

   Leg[5].Xt = Leg[5].current_x + p_delta[5][0];
   Leg[5].Yt = Leg[5].current_y;
   Leg[5].Zt = Leg[5].current_z + p_delta[5][2];

   for (uint8_t i = 0; i < 6; i++)
   {
      moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
   }

  current_roll = input_roll;
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
            Leg[0].Xt = Leg[0].current_x - Vx / 1000;
            Leg[2].Xt = Leg[2].current_x - Vx / 1000;
            Leg[4].Xt = Leg[4].current_x - Vx / 1000;

            //Y
            Leg[0].Yt = Leg[0].current_y - Vy / 1000;
            Leg[2].Yt = Leg[2].current_y - Vy / 1000;
            Leg[4].Yt = Leg[4].current_y - Vy / 1000;

            //Z
            Leg[0].Zt = -H;
            Leg[2].Zt = -H;
            Leg[4].Zt = -H;
         }
         else
         {
            //X
            Leg[0].Xt = Leg[0].current_x + Vx / 1000;
            Leg[2].Xt = Leg[2].current_x + Vx / 1000;
            Leg[4].Xt = Leg[4].current_x + Vx / 1000;

            //Y
            Leg[0].Yt = Leg[0].current_y + Vy / 1000;
            Leg[2].Yt = Leg[2].current_y + Vy / 1000;
            Leg[4].Yt = Leg[4].current_y + Vy / 1000;

            //Z
            Leg[0].Zt = -k * (Leg[0].Yt + Y_OFFSET) * (Leg[0].Yt + Y_OFFSET) - k * (Leg[0].Xt - X_OFFSET) * (Leg[0].Xt - X_OFFSET) - H + dH;
            Leg[2].Zt = -k * (Leg[2].Yt - Y_OFFSET) * (Leg[2].Yt - Y_OFFSET) - k * (Leg[2].Xt - X_OFFSET) * (Leg[2].Xt - X_OFFSET) - H + dH;
            Leg[4].Zt = -k * Leg[4].Yt * Leg[4].Yt - k * (Leg[4].Xt + X_OFFSET + 30) * (Leg[4].Xt + X_OFFSET + 30) - H + dH;
         }

         //group 1---------------------------------------------------
         if (phaseControl(1) == 0)
         {
            //X
            Leg[1].Xt = Leg[1].current_x - Vx / 1000;
            Leg[3].Xt = Leg[3].current_x - Vx / 1000;
            Leg[5].Xt = Leg[5].current_x - Vx / 1000;

            //Y
            Leg[1].Yt = Leg[1].current_y - Vy / 1000;
            Leg[3].Yt = Leg[3].current_y - Vy / 1000;
            Leg[5].Yt = Leg[5].current_y - Vy / 1000;

            //Z
            Leg[1].Zt = -H;
            Leg[3].Zt = -H;
            Leg[5].Zt = -H;
         }
         else
         {
            //X
            Leg[1].Xt = Leg[1].current_x + Vx / 1000;
            Leg[3].Xt = Leg[3].current_x + Vx / 1000;
            Leg[5].Xt = Leg[5].current_x + Vx / 1000;

            //Y
            Leg[1].Yt = Leg[1].current_y + Vy / 1000;
            Leg[3].Yt = Leg[3].current_y + Vy / 1000;
            Leg[5].Yt = Leg[5].current_y + Vy / 1000;

            //Z
            Leg[1].Zt = -k * Leg[1].Yt * Leg[1].Yt - k * (Leg[1].Xt - X_OFFSET - 30) * (Leg[1].Xt - X_OFFSET - 30) - H + dH;
            Leg[3].Zt = -k * (Leg[3].Yt - Y_OFFSET) * (Leg[3].Yt - Y_OFFSET) - k * (Leg[3].Xt + X_OFFSET) * (Leg[3].Xt + X_OFFSET) - H + dH;
            Leg[5].Zt = -k * (Leg[5].Yt + Y_OFFSET) * (Leg[5].Yt + Y_OFFSET) - k * (Leg[5].Xt + X_OFFSET) * (Leg[5].Xt + X_OFFSET) - H + dH;
         }

         for (uint8_t i = 0; i < 6; i++)
         {
            moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
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

      input_roll = map(channel[0], 600, 1600, 0.524, -0.524);

      if (fabs(input_roll) < 0.02)
      {
         input_roll = 0;
      }

      //SWC switch mode
      if (channel[5] > 1300)                                //low
      {
         fullServoTest();         
      }
      else if (channel[5] < 1200 && channel[5] > 900)       //mid
      {
         //heightTest(H);
         rotateBody();
      }
      else if (channel[5] < 700)                            //high
      {
         hexapodMove();
      }   
   }
}
