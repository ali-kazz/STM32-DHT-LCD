/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c-lcd.h"
#include "dht.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;  // I2C handle for LCD
TIM_HandleTypeDef htim2;  // Timer handle for DHT sensor

float temperature, humidity;  // Sensor values
char lcd_buffer[16];  // Buffer for LCD display

/* Function Prototypes ------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void Display_Temperature_Humidity(void);

/* Main Program -------------------------------------------------------------*/
int main(void)
{
    /* Initialize the hardware */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    /* Initialize LCD */
    lcd_init();
    lcd_clear();
    lcd_puts("STM32 DHT LCD");
    HAL_Delay(2000);
    lcd_clear();

    /* Main loop */
    while (1)
    {
        /* Read DHT sensor */
        if (DHT_Read(&temperature, &humidity) == 0)
        {
            /* Display on LCD */
            Display_Temperature_Humidity();
        }
        else
        {
            /* Error Handling */
            lcd_clear();
            lcd_puts("DHT Error!");
        }

        /* Wait before next reading */
        HAL_Delay(2000);
    }
}

/* Function to Display Temperature & Humidity on LCD */
void Display_Temperature_Humidity(void)
{
    lcd_clear();

    /* Display Temperature */
    sprintf(lcd_buffer, "Temp: %.1f C", temperature);
    lcd_gotoxy(0, 0);
    lcd_puts(lcd_buffer);

    /* Display Humidity */
    sprintf(lcd_buffer, "Hum: %.1f %%", humidity);
    lcd_gotoxy(0, 1);
    lcd_puts(lcd_buffer);
}

/* I2C1 Initialization */
static void MX_I2C1_Init(void)
{
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
}

/* TIM2 Initialization (For DHT Sensor) */
static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Ports */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO for DHT Sensor */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Error Handler */
void Error_Handler(void)
{
    while (1)
    {
        /* Stay here */
    }
}
