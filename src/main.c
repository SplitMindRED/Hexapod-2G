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
       
   hexapod_init();
   
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
      convert_input_data();
      
//      if(servo_enable == 0)
//      {
//         UART1_print_str("Vx: ");
//         UART1_print(Vx);
//         UART1_print_str(" Vy: ");
//         UART1_println(Vy);
//      }
      

      //SWC switch mode
      if (channel[5] > 1300)                                //low
      {
         fullServoTest();

      }
      else if (channel[5] < 1200 && channel[5] > 900)       //mid
      {
         //heightTest(H);
         //rotateBody();
         new_version();
      }
      else if (channel[5] < 700)                            //high
      {
         hexapodMove();
         //new_version();
         //square_test();
      }   
   }
}
