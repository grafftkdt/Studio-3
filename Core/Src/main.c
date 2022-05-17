/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//UART Zoneeeeeeeeeeeeeeeeeeeeee
uint8_t RxBuffer[64]={0};
int finishtask = 0;

void uartprotocol();
void sendAck(uint8_t ack);
void EndEffector();

//uint8_t ack=0;
int sunflower=0;
uint8_t Posdata = 0;
uint8_t PosdataPre = 0;
uint8_t stateuart = 0;
uint8_t mode = 0;
uint8_t frame = 0;
uint8_t data = 0;
uint8_t dataframe21 = 0;
uint8_t dataframe22 = 0;
uint8_t nstation[64] = {0};
uint8_t countloop = 0;
uint8_t loopcollect = 0;
uint8_t gripper = 1;
uint8_t gogoal = 0;
//uint8_t winput = 0;
//uint16_t thetainput = 0;
//uint8_t sethome = 0;
uint8_t position[4] = {0};
uint8_t station[10] = {0};
uint8_t nth = 0;
uint8_t numberofstation = 0;
uint8_t nextstation=0;
///////////////////////////////////////

uint64_t _micros = 0;
float EncoderVel = 0;
float wgu = 0.00;
float Kp = 610;
float Ki = 100;
float Kd = 80;
float inpwm = 700;
int homepwm = 2000;
float angel =0;
float pwm = 0;
float integral =0;
float error = 0;
float last =0;
float PID = 0;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;
float PID2 = 0;
int sethome = 1;
int homeset = 0;
uint16_t limit_state =0;
uint64_t Timestamp_Encoder = 0;
uint64_t Timestamp_home = 0;
int cw = 0;


uint64_t tt = 0;
uint64_t timestampmain = 0;
float winput = 0; //wdesire
float thetanow = 0;
float thetainput = 0;
float thetadiff = 0;
float testclockwiseoranticlockwise = 0;
float clockwiseoranticlockwise = 0;  //if 1 = clockwise , if 0 = anticlockwise
float t = 0;
float count = 0;
float theta = 0;
float w = 0;
float pu1 = 0;
float pe2 = 0;
float pe1 = 0;
float u = 0;
float delta_u = 0;
uint64_t x_bar = 0;

//rad,s,rad^2

//the group of variable for equation in physic
float thetainputrad = 0;
float thetanowrad = 0;
float thetadiffrad = 0;
float alpharad = 0.5;
float wmaxrad = 0;
float winputrad = 0;
float thalfrad = 0;
float timerad = 0;
float trad = 0;
float timestamploop1rad = 0;
float timeuseinloop1rad = 0;
float thetarad = 0;
float thetahalfrad = 0;
float wrad = 0;
float timestamplooprad = 0;
float timeuseinlooprad = 0;
float thetastablerad = 0;
float tstablerad = 0;
float tstableirad = 0;
float tstablefrad = 0;
float time1rad = 0;
float thetastableirad = 0;
float thetastablefrad = 0;
float wstableifrad = 0;
float wradhalf = 0;

//optional variables
float a = 0;
float b = 0;
float c = 0;
float d = 0;
float f = 0;
float g = 0;
float h = 0;
float i = 0;
float j = 0;
float k = 0;
float l = 0;
float m = 0;
float n = 0;
float bank = 0;
float start = 0;
float bank1 = 0;
//float gogoal = 0;
float ttt = 0;
float run = 0;
float stoprun = 0;
float testsethome = 0;
uint8_t goal[255] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
float EncoderVelocity_Update();
float pidvelo();
float pidposi();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //UART Zoneeeeeeeeeeeeeee
  HAL_UART_Receive_DMA(&huart2,RxBuffer, 64);
  ///////////////////////////////////////////////////////

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uartprotocol();

//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
//	 limit_state =  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
//	 HAL_GPIO_ReadPin()

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
//	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, cw);


	  angel = (((htim1.Instance->CNT)*0.08789062));

	  ttt = htim1.Instance->CNT;
	 tt = htim5.Instance->CNT;
	 t = tt;
	 trad = (t/1000000);
	 PID2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

	  station[1]= 30;
	  station[2]= 60;
	  station[3]= 220;
	  station[4]= 120;
	  station[5]= 140;
	  station[6]= 170;
	  station[7]= 75;
	  station[8]= 180;
	  station[9]= 200;

	  if(numberofstation>=1)
	  {
		  thetainput= station[goal[nextstation]];
	  }

	  if (sethome == 1)
	  {
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==1)
		  {
		  	  testsethome = 1;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, homepwm);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , 0);
		  }

		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==0)
		  {
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			htim1.Instance->CNT = 0;

			if(htim1.Instance->CNT == 0)
			{
				htim1.Instance->CNT = 0;
				sethome = 0;
				Timestamp_home = micros();
//				pwm = 0;
				homeset = 1;
				testsethome = 0;
				stoprun=1;
			}

		  }
	  }

	  if (micros() - Timestamp_home >= 2000000 && homeset ==1)
	  {
			htim1.Instance->CNT = 0;
			homeset = 0;
	  }

	 if(gogoal == 1)
	 {
		 b = thetainput;
		 d = winput;
		 count = 0;
		 thetanow = angel;
		 gogoal = 0;
		 start = 1;
		 stoprun = 0;
		 sunflower = 0;
	 }

	 testclockwiseoranticlockwise = thetainput - thetanow;

	 if(testclockwiseoranticlockwise >= 0)
	 {
		 clockwiseoranticlockwise = 1;     //clockwise
	 }
	 else if(testclockwiseoranticlockwise < 0)
	 {
		 clockwiseoranticlockwise = 0;     //anticlockwise
	 }

	 l = thetainput - thetanow;
	 thetadiff = fabs(l);
	 thetainputrad = ((thetainput*M_PI)/180);
	 thetanowrad = ((thetanow*M_PI)/180);
	 m = thetainputrad - thetanowrad;
	 thetadiffrad = fabs(m);
	 wmaxrad = sqrt(alpharad*thetadiffrad);
	 winputrad = ((winput*2*M_PI)/60);
	 n = -1;

	 if(wmaxrad < winputrad && start == 1) //case not reach to input
	 {
		 if(clockwiseoranticlockwise == 1)
	     {
			 thalfrad = wmaxrad/alpharad;
			 timerad = 2*thalfrad;

			 if(count == 0)
			 {
				 timestamploop1rad = trad;
				 run = 1;
				 count = 1;
			 }

			 timeuseinloop1rad = trad - timestamploop1rad;


			 if(timeuseinloop1rad <= thalfrad)    //first half
			 {
				 k = (0.5)*(alpharad)*(timeuseinloop1rad)*(timeuseinloop1rad);
				 thetarad = k+thetanowrad;
				 wrad = (alpharad)*(timeuseinloop1rad);
				 thetahalfrad = thetarad;
				 wradhalf = wrad;
			 }

			 else if(timeuseinloop1rad > thalfrad && timeuseinloop1rad <= timerad)
			 {
				 a = timeuseinloop1rad-thalfrad;
				 thetarad = (wradhalf*a)-(0.5*alpharad*a*a)+thetahalfrad;
				 wrad = wradhalf-(alpharad*a);
			 }

			 else if(timeuseinloop1rad > timerad && stoprun == 0)
			 {
				 run = 0;
				 stoprun = 1;

				 //theta = angel;
			 }

		 }
		 else if(clockwiseoranticlockwise == 0)
		 {
			 thalfrad = wmaxrad/alpharad;
			 timerad = 2*thalfrad;

			 if(count == 0)
			 {
				 run = 1;
				 timestamploop1rad = trad;
				 count = 1;
			 }

			 timeuseinloop1rad = trad - timestamploop1rad;

			 if(timeuseinloop1rad <= thalfrad)    //first half
			 {
				 k = (n)*(0.5)*(alpharad)*(timeuseinloop1rad)*(timeuseinloop1rad);
				 thetarad = k+thetanowrad;
				 wrad = (n)*(alpharad)*(timeuseinloop1rad);
				 thetahalfrad = thetarad;
				 wradhalf = wrad;
			 }

			 else if(timeuseinloop1rad > thalfrad && timeuseinloop1rad <= timerad)
			 {
				 a = timeuseinloop1rad-thalfrad;
				 thetarad = (wradhalf*a)+(0.5*alpharad*a*a)+thetahalfrad;
				 wrad = wradhalf+(alpharad*a);
			 }

			 else if(timeuseinloop1rad > timerad && stoprun == 0)
			 {
				 run = 0;
				 stoprun = 1;
				 //theta = angel;

			 }
		 }
	  }
	  else if(wmaxrad >= winputrad && start == 1)
	  {
		 if(clockwiseoranticlockwise == 1)
		 {
			  if(count == 0)
			  {
				  run = 1;
				  timestamplooprad = trad;
				  count = 1;
			  }

			  timeuseinlooprad = trad - timestamplooprad;
			  c = (winputrad*winputrad)/alpharad;
			  thetastablerad = thetadiffrad - c;
			  f = winputrad/alpharad;
			  g = thetastablerad/winputrad;
			  tstablerad = g;
			  tstableirad = f;
			  tstablefrad = f + g;
			  time1rad = f + g + f;

			  if(timeuseinlooprad <= tstableirad)
			  {
				   wrad = alpharad*timeuseinlooprad;
				   j = (wrad*wrad)/(2*alpharad);
				   thetarad = j+thetanowrad;
				   thetastableirad = thetarad;
				   wstableifrad = wrad;
			  }

			  else if(timeuseinlooprad > tstableirad && timeuseinlooprad <= tstablefrad)
			  {
				   h = timeuseinlooprad-tstableirad;
				   thetarad = (wstableifrad*h)+(thetastableirad);
				   wrad = wstableifrad;
				   thetastablefrad = thetarad;
			  }

			  else if(timeuseinlooprad > tstablefrad && timeuseinlooprad <= time1rad)
			  {
				   i = timeuseinlooprad-tstablefrad;
				   thetarad = (wstableifrad*i)-(0.5*alpharad*i*i)+(thetastablefrad);
				   wrad = wstableifrad-(alpharad*i);
			  }

			  else if(timeuseinlooprad > time1rad && stoprun == 0)
			  {
				   run = 0;
				   stoprun = 1;
				   //theta = angel;

			  }
	  	 }
		 else if(clockwiseoranticlockwise == 0)
		 {
			  if(count == 0)
			  {
				  run = 1;
				  timestamplooprad = trad;
				  count = 1;
			  }

			  timeuseinlooprad = trad - timestamplooprad;
			  c = (winputrad*winputrad)/alpharad;
			  thetastablerad = thetadiffrad - c;
			  f = winputrad/alpharad;
			  g = thetastablerad/winputrad;
			  tstablerad = g;
			  tstableirad = f;
			  tstablefrad = f + g;
			  time1rad = f + g + f;

			  if(timeuseinlooprad <= tstableirad)
			  {
//						   alpharad = 0.5;
				   wrad = n*alpharad*timeuseinlooprad;
				   j = (n*wrad*wrad)/(2*alpharad);
				   thetarad = j+thetanowrad;
				   thetastableirad = thetarad;
				   wstableifrad = wrad;
			  }

			  else if(timeuseinlooprad > tstableirad && timeuseinlooprad <= tstablefrad)
			  {
//						   alpha = 0;
				   h = timeuseinlooprad-tstableirad;
				   thetarad = (wstableifrad*h)+(thetastableirad);
				   wrad = wstableifrad;
				   thetastablefrad = thetarad;
			  }

			  else if(timeuseinlooprad > tstablefrad && timeuseinlooprad <= time1rad)
			  {
//						   alpha = -0.5;
				   i = timeuseinlooprad-tstablefrad;
				   thetarad = (wstableifrad*i)+(0.5*alpharad*i*i)+(thetastablefrad);
				   wrad = wstableifrad+(alpharad*i);
			  }

			  else if(timeuseinlooprad > time1rad && stoprun == 0)
			  {
				  run = 0;
				  stoprun = 1;

			  }
	  	   }
	  }
	  theta = (thetarad*180)/(M_PI);
	  w = (wrad*60)/(2*M_PI);
//

//trad = second
//theta = degree
//w = rpm rad/min

//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int) pwm);
	  ///////////////PID////////////////
//	  x_bar = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

	  if (micros() - Timestamp_Encoder >= 100)
	  {
 			Timestamp_Encoder = micros();
			EncoderVel = (EncoderVel * 99 + EncoderVelocity_Update()) / 100.0;
			wgu = EncoderVel*60 / 4095;

			if (wgu > 15 || wgu<-15)
			{
				wgu = 0;
			}

			error = (theta - angel);

			if (error<0)
			{
				error =0-error;
			}

			integral += error;
			if(stoprun!=1)
			{
				PID_p = Kp*error;
				PID_i = Ki*integral/Timestamp_Encoder;
				PID_d = Kd*(error-last);
				PID = PID_p + PID_i + PID_d;
			}

			last = error;
			if (sethome == 0)
			{
				pwm = PID  ;
			}



			if (pwm > 10000 )
			{
				 pwm = 10000;
			}

			if (angel < thetainput )
			{

			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)pwm+inpwm);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , 0);
			}

			if (angel > thetainput)
			{

			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)pwm+inpwm);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 , 1);
			}

			if(run == 0 && stoprun == 1 && sunflower == 0)
			{





				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				pwm = 0;
				PID = 0;
				PID_i  = 0;
				PID_d = 0;

				integral = 0;
				finishtask = 1;


//				EndEffector();
			}

			if (finishtask == 1)
			  {
				sendAck(2);
//				sunflower=7;
				finishtask = 0;
				sunflower = 1;
				EndEffector();
			  }
      }
//
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4096;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

enum
{
	init=0,startmode=1,collectdataframe2_1=2,collectdataframe2_2=3,loopframe3=4,collectdataframe3=5,checksum=6,checkconnect=7
};

void uartprotocol(){

	Posdata=huart2.RxXferSize-huart2.hdmarx->Instance->NDTR;
	if(Posdata!=PosdataPre )
	{
		switch(stateuart)
		{
			case 0:			//init >> check if it connect to MCU
				if (RxBuffer[PosdataPre] == 0b10010010)
				{
					mode = 2;		//connect mcu
					frame = 1;
					stateuart = checkconnect;
				}
			break;

			case 1:			//startmode >> check Start+Mode
				if (RxBuffer[PosdataPre] == 0b10010011)
				{
					mode = 3;		//disconnect mcu
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10010100)
				{
					mode = 4;		//set angular velocity
					frame = 2;
					data = 0x94;
					stateuart = collectdataframe2_1;
				}

				else if (RxBuffer[PosdataPre] == 0b10010101)
				{
					mode = 5;		//set angular position
					frame = 2;
					data = 0x95;
					stateuart = collectdataframe2_1;
				}

				else if (RxBuffer[PosdataPre] == 0b10010110)
				{
					mode = 6;		//set goal 1 station
					frame = 2;
					data = 0x96;
					stateuart = collectdataframe2_1;
				}

				else if (RxBuffer[PosdataPre] == 0b10010111)
				{
					mode = 7;		//set goal n station
					frame = 3;
					data = 0x97;
					stateuart = loopframe3;
				}

				else if (RxBuffer[PosdataPre] == 0b10011000)
				{
					mode = 8;		//go goal
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011001)
				{
					mode = 9;		//request current station
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011010)
				{
					mode = 10;		//request angular position
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011011)
				{
					mode = 11;		//request angular velocity
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011100)
				{
					mode = 12;		//enable gripper
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011101)
				{
					mode = 13;		//disable gripper
					frame = 1;
					stateuart = checksum;
				}

				else if (RxBuffer[PosdataPre] == 0b10011110)
				{
					mode = 14;		//set home
					frame = 1;
					stateuart = checksum;
				}


			break;

			case 2:			//collect data frame 2_1
				if (frame == 2)
				{
					dataframe21 = RxBuffer[PosdataPre];
					data += RxBuffer[PosdataPre];
				}
				stateuart = collectdataframe2_2;

			break;

			case 3:			//collect data frame 2_2
				if (frame == 2)
				{
					dataframe22 = RxBuffer[PosdataPre];
					data += RxBuffer[PosdataPre];
				}
				stateuart = checksum;

			break;

			case 4:			//loop frame 3
				{
				loopcollect = RxBuffer[PosdataPre];
				data += RxBuffer[PosdataPre];
				stateuart = collectdataframe3;
				}
			break;

			case 5:			//collect data frame 3
				if (countloop < loopcollect)
				{
					data += RxBuffer[PosdataPre];
					nstation[countloop] = (RxBuffer[PosdataPre] & 0b1111);
					nstation[countloop+1] = ((RxBuffer[PosdataPre] & 0b11110000) >>4 );
					countloop += 2;
					stateuart = collectdataframe3;
					sunflower = 2;
				}
				else
				{
					stateuart = checksum;
				}

			break;

			case 6:			//checksum
				data= ~(data);
				sunflower = 10;
				if (mode == 3 && RxBuffer[PosdataPre] == 0b01101100)	//disconnect
				{
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 4 && RxBuffer[PosdataPre] == data )	//set angular velocity
				{
					winput = dataframe22*10/255;			//rpm
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 5 && RxBuffer[PosdataPre] ==  data )	//set angular position
				{
					thetainput = ((dataframe21*256) + dataframe22)*180/(10000*3.14159265);  	//degree
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 6 && RxBuffer[PosdataPre] == data )	//set goal 1 station
				{
					goal[0] = dataframe22;
					numberofstation=1;
					nextstation=0;
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 7 && RxBuffer[PosdataPre] == data )	//set goal n station
				{
					sunflower =3;
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 8 && RxBuffer[PosdataPre] == 0b01100111)	//go goal
				{
					gogoal = 1;
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 9 && RxBuffer[PosdataPre] == 0b01100110)	//request current station
				{
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 10 && RxBuffer[PosdataPre] == 0b01100101)	//request angular position
				{
					sendAck(1);
					float radthousand = angel*10000*3.14/180;		//angel = present degree that read from encoder
					uint8_t p1 = (uint8_t)(radthousand/256);
					uint8_t p2 = (uint8_t)((int)(radthousand)%256);
					uint8_t checksumposition = ~((0b10011010+p1+p2) & 0xff);
					uint8_t position[32] = {0b10011010, p1,p2,checksumposition};
					HAL_UART_Transmit(&huart2, position, 4, 100);
					stateuart = startmode;
				}

				else if (mode == 11 && RxBuffer[PosdataPre] == 0b01100100)	//request angular velocity
				{
					sendAck(1);
					uint8_t omega = (uint8_t)(wgu*255/10);			// wgu = present omega that read from encoder
					uint8_t checksumvelocity = ~((0b10011011+omega) & 0xff);
					uint8_t velocity[32] = {0b10011011, 0,omega,checksumvelocity};
					HAL_UART_Transmit(&huart2, velocity, 4, 100);
					stateuart = startmode;
				}

				else if (mode == 12 && RxBuffer[PosdataPre] == 0b01100011)	//enable gripper
				{
					gripper = 1;
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 13 && RxBuffer[PosdataPre] == 0b01100010)	//disable gripper
				{
					gripper = 0;
					sendAck(1);
					stateuart = startmode;
				}

				else if (mode == 14 && RxBuffer[PosdataPre] == 0b01100001)	//set home
				{
					sethome = 1;
					sendAck(1);
					stateuart = startmode;
				}

			break;

			case 7:		//checkconnect
				if (RxBuffer[PosdataPre] == 0b01101101)
				{
					sendAck(1);
				}
				stateuart = startmode;
			break;

		}
		PosdataPre=(PosdataPre+1)%huart2.RxXferSize;
	}
}

void sendAck(uint8_t ack)
{
	static uint8_t ack1[2]={0x58,0b01110101};
	static uint8_t ack2[2]={70,110};
	HAL_Delay(20);
	if(ack == 1)
	{
		HAL_UART_Transmit_IT(&huart2, ack1, 2);
	}

	else if(ack ==2)
	{
//		return;
		HAL_UART_Transmit_IT(&huart2, ack2, 2);
//	ackackbank = 1;

//		ackcount = ackcount + 1;
	}

}

void EndEffector()
{
	if (gripper == 1)
	{
		static const uint8_t laser_Addr=(0x23<<1);
		static uint8_t pdatastart[1]={0x45};
		HAL_I2C_Master_Transmit_IT(&hi2c1, laser_Addr, pdatastart, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
	  TIM1->CNT = 0;
  }
}
#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 8192
#define  MAX_ENCODER_PERIOD 8192
float EncoderVelocity_Update() {
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW) {
		EncoderPositionDiff -= MAX_ENCODER_PERIOD;
	} else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW) {
		EncoderPositionDiff += MAX_ENCODER_PERIOD;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;
	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += 4294967295;
	}
}
uint64_t micros() {
 	return _micros + htim5.Instance->CNT;
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
