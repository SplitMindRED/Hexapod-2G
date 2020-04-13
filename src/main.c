#include "SML.h"

float Vx, Vy, Vz;
float InputX, InputY, InputZ;
float Last_InputX, Last_InputY, Last_InputZ;
float CoordX, CoordY, CoordZ;
float Throttle, Pitch, Roll, Yaw, Switch;
bool ServoEnable = false;

#define NumberOfServo 18
#define NumberOfLeg   6
int Movement = 0;
bool StandPosition = false;

//Arrays
unsigned long NextTime[NumberOfServo];//ориентир времени движения
//int AddTime[NumberOfServo] = { 1000, 500, 1500, 1500, 500, 1000 }; //добавиление времени к ориентиру времени движения
int AddTime[NumberOfServo] = { 0, 0, 0, 0, 0, 0 };
unsigned long TimePause[NumberOfServo]; //время-ориентир для команд регулировки скорости
//int Pause[NumberOfServo] = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}; //массив с значениями паузы для регулировки скорости вращения
int Pause[NumberOfServo] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//int Pause[NumberOfServo] = { 10, 10, 10, 10, 10, 10 };
int Step[NumberOfServo];  //массив с шагами для каждой сервы
uint8_t Leg[NumberOfLeg] = { 0, 3, 6, 9, 12, 15 }; //номер лапы и номер первой сервы в этой лапе
int CurrentAngle[NumberOfServo]; // текущий угол положения сервы
int TargetAngle[NumberOfServo]; //угол, в который нужно поместить серву
int StopCount[NumberOfServo];// = { 3, 2, 4, 3, 2, 4 }; //массив с ограничением счетчика шагов
int StateServo[NumberOfServo];//статус сервы 0-стоит, 1-получил команду, 2-движется, 3-закончил движение
int StateLeg[NumberOfLeg] = { 0, 0, 0, 0, 0, 0 }; // статус лапы
int StateHexapod = 0;
uint8_t SitAngle[NumberOfServo] = { 90, 180, 140, 90, 180, 140, 90, 180, 140, 90, 180, 140, 90, 180, 140, 90, 180, 140 };
uint8_t StandAngle[NumberOfServo] = { 90, 120, 120, 90, 120, 120, 90, 120, 120, 90, 120, 120, 90, 120, 120, 90, 120, 120 };
uint8_t AngleArrayMoveForward[NumberOfServo][4] =  //массив с углами
{ {90, 90, 120, 120},      //0-
  {150, 120, 120, 150},    //1+
  {60, 60, 60, 60},    //2

  {105, 105, 75, 75},      //3-
  {120, 150, 150, 120},    //4
  {60, 60, 60, 60},    //5

  {60, 60, 90, 90},        //6-
  {150, 120, 120, 150},    //7+
  {60, 60, 60, 60},    //8

  {90, 90, 120, 120},      //9-
  {120, 150, 150, 120},    //10
  {60, 60, 60, 60},    //11

  {105, 105, 75, 75},      //12-
  {150, 120, 120, 150},    //13+
  {60, 60, 60, 60},    //14

  {60, 60, 90, 90},        //15-
  {120, 150, 150, 120},    //16
  {60, 60, 60, 60}     //17
};

uint8_t AngleArrayMoveBackward[NumberOfServo][4] =
{
  {90, 90, 120, 120},      //0-
  {120, 150, 150, 120},    //1+
  {120, 120, 120, 120},    //2

  {105, 105, 75, 75},      //3-
  {150, 120, 120, 150},    //4
  {120, 120, 120, 120},    //5

  {60, 60, 90, 90},        //6-
  {120, 150, 150, 120},    //7+
  {120, 120, 120, 120},    //8

  {90, 90, 120, 120},      //9-
  {150, 120, 120, 150},    //10
  {120, 120, 120, 120},    //11

  {105, 105, 75, 75},      //12-
  {120, 150, 150, 120},    //13+
  {120, 120, 120, 120},    //14

  {60, 60, 90, 90},        //15-
  {150, 120, 120, 150},    //16
  {120, 120, 120, 120}     //17
};

uint8_t AngleArrayMoveLeft[NumberOfServo][4] =
{
  {120, 120, 90, 90},      //0-
  {150, 120, 120, 150},    //1
  {120, 120, 120, 120},    //2

  {75, 75, 105, 105},      //3-
  {120, 150, 150, 120},    //4
  {120, 120, 120, 120},    //5

  {90, 90, 60, 60},        //6-
  {150, 120, 120, 150},    //7
  {120, 120, 120, 120},    //8

  {90, 90, 120, 120},      //9-
  {120, 150, 150, 120},    //10
  {120, 120, 120, 120},    //11

  {105, 105, 75, 75},      //12-
  {150, 120, 120, 150},    //13
  {120, 120, 120, 120},    //14

  {60, 60, 90, 90},         //15-
  {120, 150, 150, 120},     //16
  {120, 120, 120, 120}      //17
};

uint8_t AngleArrayMoveRight[NumberOfServo][4] =
{
  {120, 120, 90, 90},      //0-
  {120, 150, 150, 120},    //1
  {120, 120, 120, 120},    //2

  {75, 75, 105, 105},      //3-
  {150, 120, 120, 150},    //4
  {120, 120, 120, 120},    //5

  {90, 90, 60, 60},        //6-
  {120, 150, 150, 120},    //7
  {120, 120, 120, 120},    //8

  {90, 90, 120, 120},      //9-
  {150, 120, 120, 150},    //10
  {120, 120, 120, 120},    //11

  {105, 105, 75, 75},      //12-
  {120, 150, 150, 120},    //13
  {120, 120, 120, 120},    //14

  {60, 60, 90, 90},         //15-
  {150, 120, 120, 150},     //16
  {120, 120, 120, 120}      //17
};



void Setup()
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

    //disable servo control
    digitalWrite(PORT_A, 12, 1); //1-off 0-on

    I2C1_init();
    PCA9685_init(PCA9685_ADDRESS_1);
    PCA9685_init(PCA9685_ADDRESS_2);
    

    //for (uint8_t ServoNum = 0; ServoNum < NumberOfServo; ServoNum++) //для всех сервоприводов подаем команды встать в начальное положение
    //{
    //    SetServoAngle(ServoNum, StandAngle[ServoNum]);//стартовое положение
    //    CurrentAngle[ServoNum] = StandAngle[ServoNum];//текущее положение серв при старте
    //    TargetAngle[ServoNum] = 0;
    //    Step[ServoNum] = 0;
    //    NextTime[ServoNum] = 0;
    //    TimePause[ServoNum] = 0;
    //    StateServo[ServoNum] = 0;
    //    StopCount[ServoNum] = 4;
    //    Pause[ServoNum] = 15;
    //}

    //StateHexapod = 1;

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
        SetServoAngle(i, (Throttle * 0.18 - 180));
        SetServoAngle(i + 1, (Pitch * 0.18 - 180));
        SetServoAngle(i + 2, (Roll * 0.18 - 180));
    }
}

//====================================================================================
void StateZero()//обнулить все статусы
{
    for (uint8_t ServoNum = 0; ServoNum < NumberOfServo; ServoNum++)
    {
        Step[ServoNum] = 0;
        StateServo[ServoNum] = 0;
    }
    for (uint8_t LegNum = 0; LegNum < NumberOfLeg; LegNum++)
    {
        StateLeg[LegNum] = 0;
    }

}

bool IsLegReady(uint8_t LegNum)//проверка готовости лапы, если все 3 сервы у лапы закончили движение (статус 3) вернуть true, иначе false
{
    if (StateServo[Leg[LegNum]] == 3 && StateServo[Leg[LegNum] + 1] == 3 && StateServo[Leg[LegNum] + 2] == 3) // если 3 сервы ждут команды
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SpeedServoAngle(uint8_t ServoNum, double angle, int pause) //регулировка скорости вращения
{
    if (TimeFromStart > TimePause[ServoNum])//проверка времени для поворотов на 1 градус
    {
        TargetAngle[ServoNum] = angle;
        if (CurrentAngle[ServoNum] == TargetAngle[ServoNum])//когда текущий угол сравнялся с заданным
        {
            NextTime[ServoNum] = TimeFromStart + AddTime[ServoNum];
            Step[ServoNum]++;               //прибавляем шаг
            if (Step[ServoNum] == StopCount[ServoNum])    //когда шаг последний - вернуться на первый
            {
                Step[ServoNum] = 0;
            }
            StateServo[ServoNum] = 3;           //статус сервы 3 - закончил движение

            if (IsLegReady(0) && IsLegReady(1) && IsLegReady(2) && IsLegReady(3) && IsLegReady(4) && IsLegReady(5))//обнуление статуса лап, обнуление или продление статуса движения гексапода
            {
                StateLeg[0] = 0;
                StateLeg[1] = 0;
                StateLeg[2] = 0;
                StateLeg[3] = 0;
                StateLeg[4] = 0;
                StateLeg[5] = 0;
                if (Movement == 0 || Movement == 1)
                {
                    StateHexapod = 0;
                }
                else
                {
                    StateHexapod = 1;
                }
                for (uint8_t ServoNum = 0; ServoNum < NumberOfServo; ServoNum++)
                {
                    StateServo[ServoNum] = 0;
                }
            }
        }
        else if (CurrentAngle[ServoNum] < TargetAngle[ServoNum])//если текущий угол меньше заданного - прибавить угол и отправить на серву
        {
            CurrentAngle[ServoNum]++;
            SetServoAngle(ServoNum, CurrentAngle[ServoNum]);
            TimePause[ServoNum] = TimeFromStart + pause;
        }
        else if (CurrentAngle[ServoNum] > TargetAngle[ServoNum])//если текущий угол больше заданного - вычесть угол и отправить на серву
        {
            CurrentAngle[ServoNum]--;
            SetServoAngle(ServoNum, CurrentAngle[ServoNum]);
            TimePause[ServoNum] = TimeFromStart + pause;
        }
    }
}

void HexapodMove()//двигаем гексапод
{
    if (StateHexapod == 1)//если приняли команду
    {
        for (uint8_t LegNum = 0; LegNum < NumberOfLeg; LegNum++)//всем лапам
        {
            if (StateLeg[LegNum] == 0)
            {
                StateLeg[LegNum] = 1;             //статус "принял команду"
            }
        }
        StateHexapod = 2;                   //статус гексапода "движется"
    }
}

void MoveLeg(uint8_t LegNum)//двигаем лапы
{
    if (StateLeg[LegNum] == 1) //если получили команду движения
    {
        if (StateServo[Leg[LegNum]] == 0 && StateServo[Leg[LegNum] + 1] == 0 && StateServo[Leg[LegNum] + 2] == 0) // если 3 сервы ждут команды
        {
            StateServo[Leg[LegNum]] = 1;
            StateServo[Leg[LegNum] + 1] = 1;
            StateServo[Leg[LegNum] + 2] = 1; //переключить три состояния движения
            StateLeg[LegNum] = 2;
        }
    }
}

void MoveServo()//двигаем сервы
{
    for (uint8_t ServoNum = 0; ServoNum < NumberOfServo; ServoNum++)
    {
        if (StateServo[ServoNum] == 1)//пришла команда 
        {
            switch (Movement)
            {
            case 0://сесть
                SpeedServoAngle(ServoNum, SitAngle[ServoNum], Pause[ServoNum]);
                //Step[ServoNum] = 3;
                break;

            case 1://встать
                SpeedServoAngle(ServoNum, StandAngle[ServoNum], Pause[ServoNum]);
                //Step[ServoNum] = 3;
                break;

            case 2://вперед
                SpeedServoAngle(ServoNum, AngleArrayMoveForward[ServoNum][Step[ServoNum]], Pause[ServoNum]);
                break;

            case 3://назад
                SpeedServoAngle(ServoNum, AngleArrayMoveBackward[ServoNum][Step[ServoNum]], Pause[ServoNum]);
                break;

            case 4://влево
                SpeedServoAngle(ServoNum, AngleArrayMoveLeft[ServoNum][Step[ServoNum]], Pause[ServoNum]);
                break;

            case 5://вправо
                SpeedServoAngle(ServoNum, AngleArrayMoveRight[ServoNum][Step[ServoNum]], Pause[ServoNum]);
                break;

//            case 6://приветствие
//                SpeedServoAngle(ServoNum, AngleArrayHello[ServoNum][Step[ServoNum]], Pause[ServoNum]);
//                break;

//            case 7://танец
//                SpeedServoAngle(ServoNum, AngleArrayDance[ServoNum][Step[ServoNum]], Pause[ServoNum]);
//                break;
            }
        }
    }
}
//====================================================================================

int main()
{
    Setup();

    //main loop
    int i = 0;
    while (1)
    {	
        GetSignals();

        //ServoTest4();

        //FullServoTest();

        /*Movement = 2;

        HexapodMove();
        MoveLeg(0);
        MoveLeg(1);
        MoveLeg(2);
        MoveLeg(3);
        MoveLeg(4);
        MoveLeg(5);
        MoveServo();*/

        for (uint8_t n = 0; n < 18; n++)
        {
            SetServoAngle(n, AngleArrayMoveForward[n][i]);            
        }
        
        delay(500);

        i++;
        if (i == 4)
        {
            i = 0;
        }
    }
}
