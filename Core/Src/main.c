/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "i2c.h"
#include "app_lorawan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "arm_math.h"
#include "math_helper.h"

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

/* USER CODE BEGIN PV */
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define TEST_LENGTH_SAMPLES 300
#define SNR_THRESHOLD_F32 140.0f
#define BLOCK_SIZE 32
#define NUM_TAPS 29

// Sir khurram
#define ADXL_ADR 0X53 << 1
#define REG_THRESHOLD_ACT 0X24
#define REG_THRESHOLD_INACT 0X25
#define REG_TIME_INACT 0X26
#define REG_ACT_INACT_CTRL 0X27
#define REG_DEVICE_ID 0X00   // READ ONLY
#define REG_INT_MAP 0X2F     // READ /WRITE
#define REG_INT_EN 0X2E      // READ /WRITE
#define REG_PWR_CNTRL 0X2D   // READ /WRITE
#define REG_DATA_FORMAT 0X31 // READ /WRITE
#define ADXL_DDEVICE_ID 0xE5
#define REG_INT_SOURCE 0x30

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
// Global variables to store the current orientation in degrees
float Angle_X = 0.0;
float Angle_Y = 0.0;
float Angle_Z = 0.0;
float AxAdjustedForGravity = 0;
float AyAdjustedForGravity = 0;
float AzAdjustedForGravity = 0;

float Ax = 0;
float Ay = 0;
float Az = 0;
float Gx = 0;
float Gy = 0;
float Gz = 0;
float ax = 0; // angles based on accerlerometer
float ay = 0;
double positionX = 0.0;
double positionY = 0.0;
float initialAxNoise = 0;
float initialAyNoise = 0;
float initialAzNoise = 0;
float initialAxNoiseSum = 0;
float initialAyNoiseSum = 0;
float initialAzNoiseSum = 0;

float initialGzNoise = 0;
float initialGxNoise = 0;
float initialGyNoise = 0;
float initialGxNoiseSum = 0;
float initialGyNoiseSum = 0;
float initialGzNoiseSum = 0;

float t = 0;
int i = 0;
int firstRun = 1;
const int numSamples = 300;
float accelerationsx[300];
float accelerationsy[300];
float SAMPLING_RATE = 0.001;
float distanceMovedx = 0;
float distanceMovedy = 0;
float currentPositionx = 0;
float currentPositiony = 0;
double initial_x = 0.0; // Initial estimate
double initial_P = 1.0; // Initial estimate error covariance
double Q = 0.01;        // Process noise covariance
double R = 0.25;        // Measurement noise covariance
float yaw = 0;
float roll = 0;
float pitch = 0;

float positions_x[100];
float positions_y[100];
int positions_x_counter = 0;
int positions_y_counter = 0;
int sampleCount = 0;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f; // Quaternion elements representing the estimated orientation
float pitchAdjusted = 0;
float yawAdjusted = 0;
float rollAdjusted = 0;
uint8_t check = 10;
arm_fir_instance_f32 FIR_Filter;
#define BLOCK_SIZE 1 // Number of samples to process at a time
arm_fir_instance_f32 S;

#define NUM_TAPS 29 // Number of filter coefficients
static float32_t accelerationsxThroughFilter[TEST_LENGTH_SAMPLES];
static float32_t accelerationsyThroughFilter[TEST_LENGTH_SAMPLES];

static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
static const float32_t firCoeffs32[NUM_TAPS] = {
    0.00512228f, 0.00588714f, 0.00810891f, 0.01169291f, 0.01647229f, 0.0222163f,
    0.02864195f, 0.03542865f, 0.04223479f, 0.04871565f, 0.05454144f, 0.05941469f,
    0.06308592f, 0.06536686f, 0.06614043f, 0.06536686f, 0.06308592f, 0.05941469f,
    0.05454144f, 0.04871565f, 0.04223479f, 0.03542865f, 0.02864195f, 0.0222163f,
    0.01647229f, 0.01169291f, 0.00810891f, 0.00588714f, 0.00512228f};

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES / BLOCK_SIZE;

float32_t snr;

// sir khurram
uint8_t RawData[6] = {'\0'};
uint16_t AccelValues[6] = {'\0'};
float axis_val[3] = {'\0'};
uint8_t SRC = 0;
volatile int Activity_detected = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void measure_Accel_on_all_axis(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
  double x; // Estimated state
  double P; // Estimate error covariance
  double Q; // Process noise covariance
  double R; // Measurement noise covariance
} KalmanFilter;

// Function to initialize the Kalman filter
void kalman_init(KalmanFilter *kf, double initial_x, double initial_P, double Q, double R)
{
  kf->x = initial_x;
  kf->P = initial_P;
  kf->Q = Q;
  kf->R = R;
}
KalmanFilter kf;

// Function to update the Kalman filter with a new measurement
void kalman_update(KalmanFilter *kf, double measurement)
{
  // Prediction step
  double x_hat = kf->x;
  double P_hat = kf->P + kf->Q;

  // Update step
  double K = P_hat / (P_hat + kf->R);
  kf->x = x_hat + K * (measurement - x_hat);
  kf->P = (1 - K) * P_hat;

  // Optionally, you can update process noise covariance Q based on your system dynamics
}

void MPU6050_Init(void)
{
  uint8_t Data;

  // check device ID WHO_AM_I

  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

  if (check == 104) // 0x68 will be returned by the sensor if everything goes well
  {
    // power management register 0X6B we should write all 0's to wake the sensor up
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

    // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ∞/s
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
  }
}

void MPU6050_Read_Accel(void)
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from ACCEL_XOUT_H register

  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  /*** convert the RAW values into acceleration in 'g'
       we have to divide according to the Full scale value set in FS_SEL
       I have configured FS_SEL = 0. So I am dividing by 16384.0
       for more details check ACCEL_CONFIG Register              ****/

  if (firstRun == 1)
  {
    Ax = ((Accel_X_RAW * 9.8) / 16384.0); // The noise is constant, and different each time the program is run
    Ay = -((Accel_Y_RAW * 9.8) / 16384.0);
    Az = ((Accel_Z_RAW * 9.8) / 16384.0) + 9.8;
  }
  else
  {
    Ax = ((Accel_X_RAW * 9.8) / 16384.0) - initialAxNoise; // The noise is constant, and different each time the program is run
    Ay = -((Accel_Y_RAW * 9.8) / 16384.0) - initialAyNoise;
    Az = ((Accel_Z_RAW * 9.8) / 16384.0) - initialAzNoise;
  }
}

float sinPitchLinear;
float sinPitch;
float AxCompensated;
void compensateGravity()
{

  float rollRad = (rollAdjusted * M_PI) / 180;
  float pitchRad = (pitchAdjusted * M_PI) / 180;
  float yawRad = (yawAdjusted * M_PI) / 180;

  AxAdjustedForGravity = Ax - (-9.8 * sin(pitchRad));
  AyAdjustedForGravity = Ay - (9.8 * sin(rollRad));
  AzAdjustedForGravity = Az - (-9.8 * sin(yawRad));
}

void MPU6050_Read_Gyro(void)
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from GYRO_XOUT_H register

  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

  Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  /*** convert the RAW values into dps (∞/s)
       we have to divide according to the Full scale value set in FS_SEL
       I have configured FS_SEL = 0. So I am dividing by 131.0
       for more details check GYRO_CONFIG Register              ****/

  Gx = (Gyro_X_RAW / 131.0) - initialGxNoise;
  Gy = (Gyro_Y_RAW / 131.0) - initialGyNoise;
  Gz = (Gyro_Z_RAW / 131.0) - initialGzNoise;
}
void findErrors()
{
  for (int i = 0; i < 1000; i++)
  {
    MPU6050_Read_Gyro();
    initialGxNoiseSum += Gx;
    initialGyNoiseSum += Gy;
    initialGzNoiseSum += Gz;
  }
  initialGxNoise = initialGxNoiseSum / 1000.0;
  initialGyNoise = initialGyNoiseSum / 1000.0;
  initialGzNoise = initialGzNoiseSum / 1000.0;

  for (int i = 0; i < 1000; i++)
  {
    MPU6050_Read_Accel();
    initialAxNoiseSum += Ax;
    initialAyNoiseSum += Ay;
    initialAzNoiseSum += Az;
  }
  initialAxNoise = initialAxNoiseSum / 1000.0;
  initialAyNoise = initialAyNoiseSum / 1000.0;
  initialAzNoise = initialAzNoiseSum / 1000.0;
}
void estimatePosition(float accelerationsx[], float accelerationsy[], int numSamples)
{
  //    double positionX = 0.0;
  //    double positionY = 0.0;
  double deltaVelocityx = 0.0;

  // Iterate through the array of acceleration values
  for (int i = 0; i < numSamples - 1; ++i)
  {

    // Add the contribution of the current data point to the area
    deltaVelocityx += accelerationsx[i] * SAMPLING_RATE;
  }

  // Calculate distance using the trapezoidal rule
  distanceMovedx = deltaVelocityx * (numSamples - 1) * SAMPLING_RATE;

  double deltaVelocityy = 0.0;

  // Iterate through the array of acceleration values
  for (int i = 0; i < numSamples - 1; ++i)
  {

    // Add the contribution of the current data point to the area
    deltaVelocityy += accelerationsy[i] * SAMPLING_RATE;
  }

  // Calculate distance using the trapezoidal rule
  distanceMovedy = deltaVelocityy * (numSamples - 1) * SAMPLING_RATE;

  distanceMovedx = distanceMovedx * 32; // calibration
  distanceMovedy = distanceMovedy * 32;

  currentPositiony += distanceMovedy;
  currentPositionx += distanceMovedx;
}
void compensateOrientation()
{
  float yawRad = yawAdjusted * (M_PI / 180.0);
  AxAdjustedForGravity = AxAdjustedForGravity * cos(yawRad);
  AyAdjustedForGravity = AxAdjustedForGravity * sin(yawRad);
}
//
void fir()
{
  uint32_t i;
  float32_t *inputxF32, *outputxF32;

  /* Initialize input and output buffer pointers */
  inputxF32 = &accelerationsx[0];
  outputxF32 = &accelerationsxThroughFilter[0];

  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */

  for (i = 0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputxF32 + (i * blockSize), outputxF32 + (i * blockSize), blockSize);
  }

  float32_t *inputyF32, *outputyF32;

  /* Initialize input and output buffer pointers */
  inputyF32 = &accelerationsy[0];
  outputyF32 = &accelerationsyThroughFilter[0];

  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */

  for (i = 0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputyF32 + (i * blockSize), outputyF32 + (i * blockSize), blockSize);
  }
}
void complementaryFilter()
{
  // Calculate roll and pitch from the accelerometer data
  // Assuming Ax, Ay, Az are properly updated with the latest accelerometer readings before this function is called

  // Roll: rotation around the X-axis
  // atan2 parameters: first is y (Ay), second is z (Az)
  ax = atan2(Ay, Az);

  // Pitch: rotation around the Y-axis
  // atan2 parameters: first is x (-Ax), second is the hypotenuse of the y and z components
  ay = atan2(-Ax, sqrt(pow(Ay, 2) + pow(Az, 2)));

  //    ax = 0;
  //    ay = 0;

  // Complementary filter to combine the accelerometer and previous estimate
  roll = roll * 0.9 + ax * 0.1;
  pitch = pitch * 0.98 + ay * 0.02;
}
void calibrateOrientation()
{
  pitchAdjusted = pitch * 140;
  yawAdjusted = -yaw * 11.8;
  rollAdjusted = roll * 11.8;
  if (yawAdjusted > 360)
  {
    yaw = 0;
    yawAdjusted = 0;
  }
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //  while (1)
  //  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  //  }
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

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage 
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE BEGIN Header_startLorawan */
/**
 * @brief  Function implementing the lorawan thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startLorawan */
void startLorawan(void *argument)
{
  /* init code for LoRaWAN */
  MX_LoRaWAN_Init();
  /* USER CODE BEGIN startLorawan */

  /* Infinite loop */
  for (;;)
  {
    MX_LoRaWAN_Process();

    osDelay(1);
  }
  /* USER CODE END startLorawan */
}

/* USER CODE BEGIN Header_StartDeadReckoning */
/**
 * @brief Function implementing the deadReckoning thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDeadReckoning */
void StartDeadReckoning(void *argument)
{
  /* USER CODE BEGIN StartDeadReckoning */
  /* Infinite loop */
  MPU6050_Init();
  osDelay(4000);
  findErrors();
  MPU6050_Read_Accel();
  MPU6050_Read_Gyro();
  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
  memset(firStateF32, 0, sizeof(firStateF32));

  for (;;)
  {
    firstRun = 0;

    for (int i = 0; i < numSamples - 1; ++i)
    {
      MPU6050_Read_Gyro();
      MPU6050_Read_Accel();

      yaw += Gz * (SAMPLING_RATE);
      roll += Gx * (SAMPLING_RATE);
      pitch += Gy * (SAMPLING_RATE);
      complementaryFilter();

      calibrateOrientation();
      compensateGravity();

      compensateOrientation();

      accelerationsx[i] = AxAdjustedForGravity;
      accelerationsy[i] = AyAdjustedForGravity;
      osDelay(SAMPLING_RATE * 10000);
    }
    fir();

    estimatePosition(accelerationsxThroughFilter, accelerationsyThroughFilter, numSamples);
    positions_x[positions_x_counter] = currentPositionx;
    positions_y[positions_y_counter] = currentPositiony;
    positions_x_counter += 1;
    positions_y_counter += 1;

    //      currentPositionx = 10;
    //      currentPositiony = 20;

    // osDelay(1);
  }
  /* USER CODE END StartDeadReckoning */
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

#ifdef USE_FULL_ASSERT
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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
