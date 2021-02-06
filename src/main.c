#include "splitmind_stm32f103_lib.h"
#include "hexapod_2Gen.h"

void setup()
{	
    //SETUP SYSTICK TIMER
    SysTick_Config(SystemCoreClock / 1000000);   //1 mcs
   
   UART1_init(2000000);
   
   UART1_println_str("Setup...");
    
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
   
   UART1_println_str("Done!");
   
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

      input_roll = map(channel[0], 600, 1600, 0.35, -0.35);
      input_pitch = map(channel[1], 600, 1600, 0.35, -0.35);
      input_yaw = map(channel[3], 600, 1600, 0.35, -0.35);

      if (fabs(input_roll) < 0.02)
      {
         input_roll = 0;
      }
      
      if(servo_enable == 0)
      {
         UART1_print_str("Vx: ");
         UART1_print(Vx);
         UART1_print_str(" Vy: ");
         UART1_println(Vy);
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
