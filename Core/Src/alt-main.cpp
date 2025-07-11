#include <alt-main.h>
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stdio.h"
#include <string.h>

#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "command.hpp"
#include "as5048a.h"
#include "servo_control.h"
#include "mode_select.hpp"
#include "motion.h"
#include "ma702.h"
#include "flash.h"

#define UART1_RX_BUFFER_SIZE 1
ModeSelector modeSelector(8);

// コマンド処理関連変数
CommandHandler cmdHandler;
volatile bool uart1_data_ready = false;
uint8_t uart1_rx_buffer[1];
uint16_t uart1_rx_len = 1;
volatile bool command_received = false;
char received_mode;
int received_value = 0;
std::vector<uint8_t> uart1_cmd_buffer;

// AS5048Aセンサーのインスタンス
AS5048A as5048a[5] = {
    AS5048A(&hspi1, SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), // SPI1, encoder1, MP1
    AS5048A(&hspi2, SPI2_SS1_GPIO_Port, SPI2_SS1_Pin),  // SPI2, encoder2, MP2
    AS5048A(&hspi3, SPI3_SS1_GPIO_Port, SPI3_SS1_Pin), // SPI3, encoder3, MP3
    AS5048A(&hspi4, SPI4_SS1_GPIO_Port, SPI4_SS1_Pin),  // SPI4, encoder4, ABD
    AS5048A(&hspi5, SPI5_SS1_GPIO_Port, SPI5_SS2_Pin)  // SPI5, encoder5, MP4
};

MA702 ma702[5] = {
    MA702(&hspi1, SPI1_SS2_GPIO_Port, SPI1_SS2_Pin), // PIP1
    MA702(&hspi2, SPI2_SS2_GPIO_Port, SPI2_SS2_Pin), // PIP2
    MA702(&hspi3, SPI3_SS2_GPIO_Port, SPI3_SS2_Pin), // PIP3
    MA702(&hspi5, SPI5_SS2_GPIO_Port, SPI5_SS2_Pin),  // CM
    MA702(&hspi5, SPI5_SS3_GPIO_Port, SPI5_SS3_Pin), // PIP4
};

float zero_position_map[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// サーボコントローラーのインスタンス
ServoController servoControllers[10] = {
    ServoController(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a[0], true), // MP1
    ServoController(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[0], false), // PIP1

    ServoController(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a[1], true), // MP2
    ServoController(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[1], true),  // PIP2

    ServoController(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a[2], false), // MP3
    ServoController(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[2], true),  // PIP3

    ServoController(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a[4], false), // MP4
    ServoController(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[3], true),  // CM

    ServoController(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[4], true), // PIP4
    ServoController(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a[3], false) // ABD
};

MotionController motionController(
    &servoControllers[0], &servoControllers[1],
    &servoControllers[2], &servoControllers[3],
    &servoControllers[4], &servoControllers[5],
    &servoControllers[6], &servoControllers[7],
    &servoControllers[8], &servoControllers[9]
);

bool isServoPidOn[10] = {false, false, false, false, false, false, false, false, false, false};

void alt_setup()
{
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_UART_Receive_IT(&huart1, uart1_rx_buffer, UART1_RX_BUFFER_SIZE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    ma702[0].init();
    ma702[1].init();
    ma702[2].init();
    ma702[3].init();
    ma702[4].init();

    Flash_ReadFloatArray(zero_position_map);

    HAL_GPIO_WritePin(SPI1_SS2_GPIO_Port, SPI1_SS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_SS2_GPIO_Port, SPI2_SS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI3_SS2_GPIO_Port, SPI3_SS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI4_SS2_GPIO_Port, SPI4_SS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI5_SS2_GPIO_Port, SPI5_SS2_Pin, GPIO_PIN_SET);


    cmdHandler.setCallback([](char mode, int value)
                           {
    // コマンド内容をグローバル変数に保存し、フラグを立てる
    received_mode = mode;
    received_value = value;
    command_received = true; });

    setbuf(stdout, NULL); // printfのバッファリングを無効化
    printf("System initialized.\n");
    printf("Zero position map: ");
    for (int i = 0; i < 10; i++)    
    {
        printf("%f ", zero_position_map[i]);
    }
    printf("\n");
}

void alt_main()
{
    // モード選択
    modeSelector.selectMode();

    // 選択されたモードに応じて処理を実行
    modeSelector.executeSelectedMode();
}

void alt_loop()
{
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

    if (command_received)
    {
        command_received = false;
        ProcessCommand(received_mode, received_value);
    }

    HAL_Delay(500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // HAL_UART_Transmit(&huart1, uart1_rx_buffer, 1, HAL_MAX_DELAY);
    if (huart->Instance == USART1)
    {
        uart1_cmd_buffer.push_back(uart1_rx_buffer[0]);
        if (uart1_rx_buffer[0] == '\n')
        {
            cmdHandler.onUartReceive(uart1_cmd_buffer.data(), uart1_cmd_buffer.size());
            uart1_cmd_buffer.clear();
        }
        HAL_UART_Receive_IT(&huart1, uart1_rx_buffer, UART1_RX_BUFFER_SIZE);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        for (int i = 0; i < 10; i++)
        {
            if (isServoPidOn[i])
            {
                servoControllers[i].control();
            }
        }
    }
}

