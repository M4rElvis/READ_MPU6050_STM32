/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : STM32F103C8T6 - MPU6050 DMP MotionApps 6.12
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "mpu6050_port.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PD */

#define RAD_TO_DEG 57.2957795f

#define UART_PRINT_INTERVAL 100U

/* USER CODE END PD */


/* USER CODE BEGIN PV */

// MPU6050 STATUS
volatile uint8_t mpu_init_status = 0xFF;
volatile uint8_t mpu_dmp_error = 0;
volatile uint8_t mpu_new_data = 0;

// ORIENTATION DATA (RADIANS)
volatile float mpu_yaw_rad = 0.0f;
volatile float mpu_pitch_rad = 0.0f;
volatile float mpu_roll_rad = 0.0f;

// ORIENTATION DATA (DEGREES)
volatile int32_t mpu_yaw_deg = 0;
volatile int32_t mpu_pitch_deg = 0;
volatile int32_t mpu_roll_deg = 0;


// TIMING
uint32_t uart_last_print_ms = 0;
uint32_t dmp_last_packet_ms = 0;


// DMP DIAGNOSTIC
volatile uint32_t dmp_packet_count = 0;
volatile uint8_t dmp_no_packet_warning = 0;

/* USER CODE END PV */


/* USER CODE BEGIN 0 */

// UART SEND STRING
void UART_SendString(const char *message)
{
    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)message,
        strlen(message),
        1000
    );
}


// PRINT INITIALIZATION STATUS
static void PrintInitStatus(void)
{
    char message[120];

    snprintf(
        message,
        sizeof(message),
        "MPU INIT STATUS: %u\r\n",
        (unsigned)mpu_init_status
    );

    UART_SendString(message);


    snprintf(
        message,
        sizeof(message),
        "DMP ERROR CODE: %u\r\n",
        (unsigned)mpu_dmp_error
    );

    UART_SendString(message);
}


// PRINT ORIENTATION
static void PrintOrientation(float yaw, float pitch, float roll)
{
    char message[150];
    int32_t yaw_deg = (int32_t)(yaw * RAD_TO_DEG);
    int32_t pitch_deg = (int32_t)(pitch * RAD_TO_DEG);
    int32_t roll_deg = (int32_t)(roll * RAD_TO_DEG);

    snprintf(
        message,
        sizeof(message),
        "YAW: %ld deg | PITCH: %ld deg | ROLL: %ld deg\r\n",
        (long)yaw_deg,
        (long)pitch_deg,
        (long)roll_deg
    );

    UART_SendString(message);
}


// PRINT INITIALIZATION ERROR
static void PrintInitError(void)
{
    switch (mpu_init_status)
    {
        case 1:
            UART_SendString(
                "ERROR: MPU6050 CONNECTION FAILED!\r\n"
            );
            UART_SendString(
                "Check WHO_AM_I / testConnection().\r\n"
            );

            break;

        case 2:
            UART_SendString(
                "ERROR: DMP INITIALIZATION FAILED!\r\n"
            );
            UART_SendString(
                "Check DMP firmware upload and I2C backend.\r\n"
            );
            break;

        case 3:
            UART_SendString(
                "ERROR: DMP ENABLE FAILED!\r\n"
            );
            break;

        default:
            UART_SendString(
                "ERROR: UNKNOWN INITIALIZATION STATUS!\r\n"
            );
            break;
    }
}

/* USER CODE END 0 */


/**
 * @brief Main program
 */
int main(void)
{
    /* Initialize HAL */
    HAL_Init();


    /* Configure System Clock */
    SystemClock_Config();


    /* Initialize peripherals */
    MX_GPIO_Init();

    MX_I2C1_Init();

    MX_USART1_UART_Init();


    /* USER CODE BEGIN 2 */

    // 1. START MESSAGE
    HAL_Delay(100);
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString(" STM32F103C8T6 MPU6050 DMP TEST\r\n");
    UART_SendString(" MotionApps 6.12\r\n");
    UART_SendString("========================================\r\n");


    // 2. INITIALIZE MPU6050 AND DMP
    UART_SendString("\r\n");
    UART_SendString("Initializing MPU6050...\r\n");
    UART_SendString("Loading DMP firmware...\r\n");


    mpu_init_status = MPU6050_Port_Init();
    mpu_dmp_error = MPU6050_Port_DMPError();


    // 3. PRINT INITIALIZATION RESULT
    UART_SendString("\r\n");
    PrintInitStatus();


    if (mpu_init_status == 0U)
    {
        UART_SendString("\r\n");
        UART_SendString("DMP INITIALIZATION SUCCESS!\r\n");
        UART_SendString("DMP ENABLED!\r\n");
        UART_SendString("FIFO PACKET SIZE: 28 BYTES\r\n");
        UART_SendString("Waiting for DMP data...\r\n");


        dmp_last_packet_ms = HAL_GetTick();
        uart_last_print_ms = HAL_GetTick();
    }
    else
    {
        UART_SendString("\r\n");
        PrintInitError();
    }


    /* USER CODE END 2 */


    /* Infinite loop */
    while (1)
    {
        /* USER CODE BEGIN WHILE */

        //CHECK DMP INITIALIZATION
        if (mpu_init_status == 0U)
        {
            float yaw = 0.0f;
            float pitch = 0.0f;
            float roll = 0.0f;

            //READ DMP FIFO PACKET
            if (MPU6050_Port_ReadYPR(
                    &yaw,
                    &pitch,
                    &roll))
            {

                //SAVE ORIENTATION DATA
                mpu_yaw_rad = yaw;
                mpu_pitch_rad = pitch;
                mpu_roll_rad = roll;

                // Convert radians to degrees
                mpu_yaw_deg =
                    (int32_t)(yaw * RAD_TO_DEG);
                mpu_pitch_deg =
                    (int32_t)(pitch * RAD_TO_DEG);
                mpu_roll_deg =
                    (int32_t)(roll * RAD_TO_DEG);

                mpu_new_data = 1U;
                dmp_packet_count++;
                dmp_last_packet_ms = HAL_GetTick();
                dmp_no_packet_warning = 0U;

                //PRINT EVERY 100 ms
                if (
                    (HAL_GetTick() - uart_last_print_ms)
                    >= UART_PRINT_INTERVAL
                )
                {
                    PrintOrientation(
                        yaw,
                        pitch,
                        roll
                    );
                    uart_last_print_ms = HAL_GetTick();
                }
            }

            //CHECK FIFO PACKET TIMEOUT
            if (
                !dmp_no_packet_warning &&
                (HAL_GetTick() - dmp_last_packet_ms) >= 3000U
            )
            {
                UART_SendString(
                    "WARNING: NO NEW DMP FIFO PACKET!\r\n"
                );

                UART_SendString(
                    "DMP enabled, but data is not received.\r\n"
                );

                dmp_no_packet_warning = 1U;
            }
        }

        // WAIT BEFORE NEXT POLL
        HAL_Delay(2);


        /* USER CODE END WHILE */
    }
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (
        HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}


/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation here */
}

#endif