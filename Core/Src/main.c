/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define a 94.5
#define b 180
#define c 180
#define PI 3.1415926536

#define BUFSIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint16_t bufferUART = 1;
uint16_t rx_buff_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//int Set_Servo_Angle(uint8_t);////////////////////////////////////////////////////////////
//void ReciveToServo(uint8_t rx_buff[1]);
void UpdatePosition(int, float);
float getTheta(int, float, float, float);
void moveTOPS(float, float, float,
              float, float, float,
              float, float, float,
              float, float, float);
void inPlace();
void stand();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*I2C1------------------------------------------------------------------------*/
#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime>>8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}
/*I2C1------------------------------------------------------------------------*/


uint8_t rx_buff[1]={0};
float offset[4][3] = { { 120.0, 0, 47.449 },       //FL leg
                       { 120.0, 0, 58.851 },       //FR leg
                       { 120.0, 180, 298.149 },    //BL leg
                       { 120.0, 180, 307.388 } };  //BR leg

/////////////////////////////////// не полный буфер ///////////////////////////////////////
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		rx_buff_len = BUFSIZE - huart->RxXferCount;
		uint8_t res = HAL_UART_Transmit_IT(&huart1, (uint8_t*)rx_buff, rx_buff_len);
		if(res == HAL_ERROR) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_ERROR - rx_buff == NULL or rx_buff_len == 0\n", 48, 1000);
		else if(res == HAL_BUSY) HAL_UART_Transmit(&huart1, (uint8_t*)"HAL_BUSY\n", 9, 1000);
		HAL_UART_AbortReceive(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		//ReciveToServo((uint8_t*)rx_buff);////////////////////////////////////////////////////////
	}
}

/////////////////////////////////// полный буфер ///////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart1)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart1);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  //ReciveToServo((uint8_t*)rx_buff);////////////////////////////////////////////////////////
	  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// ErrorCallback //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		uint32_t er = HAL_UART_GetError(&huart1);
		HAL_UART_Abort_IT(&huart1);

		switch(er)
		{
			case HAL_UART_ERROR_PE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Parity error\n", 27, 1000);
				__HAL_UART_CLEAR_PEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_NE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Noise error\n", 26, 1000);
				__HAL_UART_CLEAR_NEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_FE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Frame error\n", 26, 1000);
				__HAL_UART_CLEAR_FEFLAG(&huart1);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_ORE:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - Overrun error\n", 28, 1000);
				__HAL_UART_CLEAR_OREFLAG(huart);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			case HAL_UART_ERROR_DMA:
				HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Callbck - DMA transfer error\n", 33, 1000);
				huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;

			default:
			break;
		}
	}

}

float getTheta(int joint, float X, float Y, float Z)
{
	float L = 0;
	float angle = 0;

	L = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
	//calculates abad, hip, or knee angle
	switch (joint)
	{
    case 0:  //abad
      angle = atan(X / Z) + acos(a / (sqrt(pow(X, 2) + pow(Z, 2))));
      break;
    case 1:  //hip
      angle = (PI / 2) - asin(-Y / sqrt(L - pow(a, 2))) - acos((pow(b, 2) - pow(a, 2) - pow(c, 2) + L) / (2 * b * sqrt(L - pow(a, 2))));
      break;
    case 2:  //knee
      angle = acos((pow(a, 2) + pow(b, 2) + pow(c, 2) - L) / (2 * b * c));
      break;
	}
	/*if (joint > 2)
	{
		return (180 - angle * (180 / PI));
	}
	else
	{
		return (angle * (180 / PI));
	}*/
	return (angle * (180 / PI));
    //converts angle to degrees and returns the value
}
/*void ReciveToServo(uint8_t rx_buff[1])
{
	float X = 110;
	float Y = -100;
	float Z = 300;//(float)rx_buff[0];//300

	switch(rx_buff[0])
	{
		case 49:
			Z = 300;
			break;
		case 50:
			Z = 280;
			break;
		case 51:
			Z = 240;
			break;
		case 52:
			Z = 220;
			break;
		case 53:
			Z = 200;
			break;
		case 54:
			Z = 180;
			break;
		case 55:
			Z = 160;
			break;
		case 56:
			Z = 140;
			break;
		case 57:
			Z = 100;
			break;
	}

	float L = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
	float theta_abad = atan((X / Z)) + acos((a / (sqrt(pow(X, 2) + pow(Z, 2)))));
	float theta_hip = (PI / 2) - asin(-Y / sqrt(L - pow(a, 2))) - acos((pow(b, 2) - pow(a, 2) - pow(c, 2) + L) / (2 * b * sqrt(L - pow(a, 2))));
	float theta_knee = acos((pow(a, 2) + pow(b, 2) + pow(c, 2) - L) / (2 * b * c));
	UpdatePosition(theta_abad* 180 / PI, theta_hip* 180 / PI, theta_knee* 180 / PI);
	HAL_UART_Receive_DMA(&huart1, rx_buff, 1);
}*/
//Fun. for calculation angle legs and transmit at fun. UpdatePosition
void moveTOPS(float X0, float Y0, float Z0,    //FL leg
              float X1, float Y1, float Z1,    //FR leg
              float X2, float Y2, float Z2,    //BL leg
              float X3, float Y3, float Z3)	   //BR leg
{
  float relPos = 0;                                //odrive relative position
  //X, Y, Z positions of each foot
  float pos[4][3] = { { X0, Y0, Z0 },    //FL leg
                      { X1, Y1, Z1 },    //FR leg
                      { X2, Y2, Z2 },    //BL leg
                      { X3, Y3, Z3 } };  //BR leg
  //move each actuator
  for (int i = 0; i < 4; i++)
  {                                                  //cycle through each leg (FL, FR, BL, BR)
    for (int j = 0; j < 3; j++)
    {
    	//cycle through each legs joints (abad, hip, knee)
    	relPos = getTheta(j, pos[i][0], pos[i][1], pos[i][2]);// - offset[i][j];  //calculate relative joint position and make positive
    	//relPos = fabs(relPos / 360.0 * GR) * dir[i][j];                            //converts relative joint position from degrees to counts using gear ratio
    	//odriveCAN.SetPosition(i * 3 + j, relPos);                                  //moves the actuator to its joint position (axisID, relative position)
    	//UpdatePosition(theta_abad* 180 / PI, theta_hip* 180 / PI, theta_knee* 180 / PI);
    	/*if (i > 1)
    	{
    		UpdatePosition(i * 3 + j, relPos);
    	}
    	else
    	{
    		UpdatePosition(i * 3 + j, 180 - relPos);
    	}*/
    	UpdatePosition(i * 3 + j, relPos);
    	HAL_UART_Receive_DMA(&huart1, rx_buff, 1);
    }
  }
}

//trots in place
void inPlace()
{
  float normX = 110;
  float normYF = -30;
  float normYB = -100;
  float normZ = 300;
  moveTOPS(normX, normYF, normZ - 160,   //FL
           normX, normYF, normZ,        //FR
           normX, normYB, normZ,        //BL
           normX, normYB, normZ - 160);  //BR
  HAL_Delay(150);
  moveTOPS(normX, normYF, normZ,   //FL
           normX, normYF, normZ,   //FR
           normX, normYB, normZ,   //BL
           normX, normYB, normZ);  //BR
  HAL_Delay(150);
  moveTOPS(normX, normYF, normZ,       //FL
           normX, normYF, normZ - 160,  //FR
           normX, normYB, normZ - 160,  //BL
           normX, normYB, normZ);      //BR
  HAL_Delay(150);
  moveTOPS(normX, normYF, normZ,   //FL
           normX, normYF, normZ,   //FR
           normX, normYB, normZ,   //BL
           normX, normYB, normZ);  //BR
  HAL_Delay(150);
}

void step()
{
  int normYF = -50;
  int normYB = -50;//-110
  int offset = 200;//100
  int offsetY = 300;
  int time = 160;
  int normX = 110;
  int normZ = 300;
  moveTOPS(normX, normYF, normZ - offset,   //FL
           normX, normYF, normZ,                 //FR
           normX, normYB, normZ,                 //BL
           normX, normYB, normZ - offset);  //BR
  HAL_Delay(time);
  moveTOPS(normX, normYF + offsetY, normZ - offset,                           //FL
           normX, normYF, normZ,                                                             //FR
           normX, normYB, normZ,                                                             //BL
           normX, normYB + offsetY, normZ - offset);  //BR
  HAL_Delay(time);
  moveTOPS(normX, normYF + offsetY, normZ,                           //FL
           normX, normYF, normZ,                                               //FR
           normX, normYB, normZ,                                               //BL
           normX, normYB + offsetY, normZ);  //BR
  HAL_Delay(time);
  moveTOPS(normX, normYF, normZ,                //FL
           normX, normYF, normZ - offset,  //FR
           normX, normYB, normZ - offset,  //BL
           normX, normYB, normZ);               //BR
  HAL_Delay(time);
  moveTOPS(normX, normYF, normZ,                                                            //FL
           normX, normYF + offsetY, normZ - offset,  //FR
           normX, normYB + offsetY, normZ - offset,                          //BL
           normX, normYB, normZ);                                                           //BR
  HAL_Delay(time);
  moveTOPS(normX, normYF, normZ,                                              //FL
           normX, normYF + offsetY, normZ,  //FR
           normX, normYB + offsetY, normZ,                          //BL
           normX, normYB, normZ);                                             //BR
  HAL_Delay(time);
}

void stand()
{
	int normYF = -50;
	int normYB = -50;//-110
	int normX = 110;
	int normZ = 300;
	int time = 160;
	  moveTOPS(normX, normYF, normZ,   //FL
	           normX, normYF, normZ,                 //FR
	           normX, normYB, normZ,                 //BL
	           normX, normYB, normZ);  //BR
	  HAL_Delay(time);
}

//void UpdatePosition(float theta_abad, float theta_hip, float theta_knee)
void UpdatePosition(int j, float relPos)
{
	PCA9685_SetServoAngle(j, relPos);
	/*
	 * front left
	 * 			abad плечо
	 * 			hip бедро
	 * 			knee колено
	 * front right
	 * back left
	 * back right
	 */
	//uint16_t Pulse_length = 500;
	//float theta_abad_PWM = 500;//centre 750
	//float theta_hip_PWM = 400;
	//float theta_knee_PWM = 200;
	    /*if (theta_abad >= 0  &&  theta_abad <= 180)
	    {
	    	theta_abad_PWM += (1000-500)/180 * theta_abad;
	    }
	    if (theta_hip >= 0  &&  theta_hip <= 180)
	    {
	    	theta_hip_PWM += (1300-400)/180 * theta_hip;
	    }
	    if (theta_knee >= 0  &&  theta_knee <= 180)
	    {
	    	theta_knee_PWM += (1300-200)/180 * theta_knee;
	    }*/
	    //TIM2->CCR1=Pulse_length;
	    //return 0;
//abad плечо
//hip бедро
//knee колено
	/*switch(j)
	{
	//Передняя левая
	case 0:
		TIM2 ->CCR1 = relPos + 750 - 50;//плечо
		break;
	case 1:
		TIM2 ->CCR2 = relPos + 750 - 300;//бедро
		break;
	case 2:
		TIM2 ->CCR3 = relPos + 750;//колено 1300 200
		break;
	//Предняя правая
	case 3:
		TIM2 ->CCR4 = relPos + 750 - 470;
		break;
	case 4:
		TIM3 ->CCR1 = relPos + 750 - 30;
		break;
	case 5:
		TIM3 ->CCR2 = relPos + 750 - 250;
		break;
	//Звдняя левая
	case 6:
		TIM3 ->CCR3 = relPos + 750 - 150;
		break;
	case 7:
		TIM3 ->CCR4 = relPos + 750 + 250;
		break;
	case 8:
		TIM4 ->CCR1 = relPos + 750 - 600;
		break;
	//Задняя правая
	case 9:
		TIM4 ->CCR2 = relPos + 750 - 350;
		break;
	case 10:
		TIM4 ->CCR3 = relPos + 750 - 450;
		break;
	case 11:
		TIM5 ->CCR1 = relPos + 750 + 200;
		//TIM4 ->CCR1 = relPos + 750;
		break;
	}*/
	/*switch(j)
		{
		//Передняя левая
		case 0:
			TIM2 ->CCR1 = relPos + 750-250;//плечо
			break;
		case 1:
			TIM2 ->CCR2 = relPos + 750 + 400;//бедро
			break;
		case 2:
			TIM2 ->CCR3 = relPos + 750 - 400;//колено 1300 200
			break;
		//Предняя правая
		case 3:
			TIM2 ->CCR4 = relPos + 750 - 450;
			break;
		case 4:
			TIM3 ->CCR1 = relPos + 750 + 100;
			break;
		case 5:
			TIM3 ->CCR2 = relPos + 750 - 250;
			break;
		//Звдняя левая
		case 6:
			TIM3 ->CCR3 = relPos + 750 - 150;
			break;
		case 7:
			TIM3 ->CCR4 = relPos + 750 + 350;
			break;
		case 8:
			TIM4 ->CCR1 = relPos + 750 - 600;
			break;
		//Задняя правая
		case 9:
			TIM4 ->CCR2 = relPos + 750 - 350;
			break;
		case 10:
			TIM4 ->CCR3 = relPos + 750 - 350;
			break;
		case 11:
			TIM5 ->CCR1 = relPos + 750 + 200;
			//TIM4 ->CCR1 = relPos + 750;
			break;
		}*/
/*
	TIM2 ->CCR1 = theta_abad_PWM;
	TIM2 ->CCR2 = theta_hip_PWM;
	TIM2 ->CCR3 = theta_knee_PWM;

	TIM2 ->CCR4 = theta_abad_PWM;
	TIM3 ->CCR1 = 0;
	TIM3 ->CCR2 = 0;

	TIM3 ->CCR3 = theta_abad_PWM;
	TIM3 ->CCR4 = 0;
	TIM4 ->CCR1 = 0;

	TIM4 ->CCR2 = theta_abad_PWM;
	TIM4 ->CCR3 = 0;
	TIM4 ->CCR4 = 0;*/
	    //TIM2 ->CCR1 = 900;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  /*I2C1------------------------------------------------------------------------*/
  PCA9685_Init(50); // 50Hz for servo
  /*I2C1------------------------------------------------------------------------*/







  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, rx_buff, bufferUART);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);///////////////////////////////////////////////////////////////////////////////////
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  /*int Set_Servo_Angle(uint8_t Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180)
    {
        Pulse_length += (2700-500)/180 * Angle;
    }

    TIM2->CCR1=Pulse_length;
    return 0;
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	        PCA9685_SetServoAngle(5, 0);
	        PCA9685_SetServoAngle(7, 36);
	        PCA9685_SetServoAngle(9, 72);
	        PCA9685_SetServoAngle(11, 108);
	        PCA9685_SetServoAngle(13, 144);
	        PCA9685_SetServoAngle(15, 180);
	        HAL_Delay(1000);

	        PCA9685_SetServoAngle(5, 36);
	        PCA9685_SetServoAngle(7, 72);
	        PCA9685_SetServoAngle(9, 108);
	        PCA9685_SetServoAngle(11, 144);
	        PCA9685_SetServoAngle(13, 180);
	        PCA9685_SetServoAngle(15, 0);
	        HAL_Delay(1000);

	        PCA9685_SetServoAngle(5, 72);
	        PCA9685_SetServoAngle(7, 108);
	        PCA9685_SetServoAngle(9, 144);
	        PCA9685_SetServoAngle(11, 180);
	        PCA9685_SetServoAngle(13, 0);
	        PCA9685_SetServoAngle(15, 36);
	        HAL_Delay(1000);

	        PCA9685_SetServoAngle(5, 108);
	        PCA9685_SetServoAngle(7, 144);
	        PCA9685_SetServoAngle(9, 180);
	        PCA9685_SetServoAngle(11, 0);
	        PCA9685_SetServoAngle(13, 36);
	        PCA9685_SetServoAngle(15, 72);
	        HAL_Delay(1000);

	        PCA9685_SetServoAngle(5, 144);
	        PCA9685_SetServoAngle(7, 180);
	        PCA9685_SetServoAngle(9, 0);
	        PCA9685_SetServoAngle(11, 36);
	        PCA9685_SetServoAngle(13, 72);
	        PCA9685_SetServoAngle(15, 108);
	        HAL_Delay(1000);

	        PCA9685_SetServoAngle(5, 180);
	        PCA9685_SetServoAngle(7, 0);
	        PCA9685_SetServoAngle(9, 36);
	        PCA9685_SetServoAngle(11, 72);
	        PCA9685_SetServoAngle(13, 108);
	        PCA9685_SetServoAngle(15, 144);
	        HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10100;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//////////////////////////////////////////////////////////////////
{
  HAL_UART_Receive_DMA(&huart1, rx_buff, bufferUART); //You need to toggle a breakpoint on this line!
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
