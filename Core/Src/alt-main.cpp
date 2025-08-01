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
uint8_t uart1_rx_buffer[1] = {0};
uint16_t uart1_rx_len = 1;
volatile bool command_received = false;
char received_mode;
int received_value1 = 0;
int received_value2 = 0;
std::vector<uint8_t> uart1_cmd_buffer;
volatile bool uart1_cmd_ready = false;

float current_angle[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

int count = 0;

// AS5048Aセンサーのインスタンス
/*AS5048A as5048a[5] = {
    AS5048A(&hspi1, SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), // SPI1, encoder1, MP1
    AS5048A(&hspi2, SPI2_SS1_GPIO_Port, SPI2_SS1_Pin),  // SPI2, encoder2, MP2
    AS5048A(&hspi3, SPI3_SS1_GPIO_Port, SPI3_SS1_Pin), // SPI3, encoder3, MP3
    AS5048A(&hspi4, SPI4_SS1_GPIO_Port, SPI4_SS1_Pin),  // SPI4, encoder4, ABD
    AS5048A(&hspi5, SPI5_SS2_GPIO_Port, SPI5_SS2_Pin)  // SPI5, encoder5, MP4
};*/
AS5048A as5048a(&hspi4, SPI4_SS1_GPIO_Port, SPI4_SS1_Pin);

MA702 ma702[9] = {
    MA702(&hspi1, SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), // SPI1, encoder1, MP1
    MA702(&hspi1, SPI1_SS2_GPIO_Port, SPI1_SS2_Pin), // PIP1
    MA702(&hspi2, SPI2_SS1_GPIO_Port, SPI2_SS1_Pin), // SPI2, encoder2, MP2
    MA702(&hspi2, SPI2_SS2_GPIO_Port, SPI2_SS2_Pin), // PIP2
    MA702(&hspi3, SPI3_SS1_GPIO_Port, SPI3_SS1_Pin), // SPI3, encoder3, MP3
    MA702(&hspi3, SPI3_SS2_GPIO_Port, SPI3_SS2_Pin), // PIP3
    MA702(&hspi5, SPI5_SS1_GPIO_Port, SPI5_SS1_Pin),  // CM
    MA702(&hspi5, SPI5_SS2_GPIO_Port, SPI5_SS2_Pin),  // SPI5, encoder5, MP4
    MA702(&hspi5, SPI5_SS3_GPIO_Port, SPI5_SS3_Pin), // PIP4
};

float zero_position_map[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; //mp1, pip1, mp2, pip2, mp3, pip3, cm, mp4, pip4, abd 下2つはk値

// サーボコントローラーのインスタンス
ServoController servoControllers[10] = {
    ServoController(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[0], false, 1), // MP1
    ServoController(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[1], true, 1/*35.0f/90.0f*/), // PIP1

    ServoController(&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[2], false, 1), // MP2
    ServoController(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[3], false, 1/*105.0f/90.0f*/),  // PIP2

    ServoController(&htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[4], true, 1), // MP3
    ServoController(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[5], false, 1/*32.0f/90.0f*/),  // PIP3

    ServoController(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[6], true, 1),  // CM
    ServoController(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[7], false, 1), // MP4

    ServoController(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &ma702[8], true, 1/*40.0f/90.0f*/), // PIP4
    ServoController(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, 30.0f, 0.0f, 0.0f, 90.0f, 1200, &as5048a, false, 1) // ABD

    //左手の場合はCM，MP4，PIP4のみtrue-falseを反転
};

MotionController motionController(
    &servoControllers[0], &servoControllers[1],
    &servoControllers[2], &servoControllers[3],
    &servoControllers[4], &servoControllers[5],
    &servoControllers[6], &servoControllers[7],
    &servoControllers[8], &servoControllers[9]
);

bool isServoPidOn[10] = {false, false, false, false, false, false, false, false, false, false}; //mp1, pip1, mp2, pip2, mp3, pip3, cm, mp4, pip4, abd

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
    ma702[5].init();
    ma702[6].init();
    ma702[7].init();
    ma702[8].init();
    as5048a.init();

    Flash_ReadFloatArray(zero_position_map);
    for (int i = 0; i < 10; i++)
    {
        servoControllers[i].setZeroPosition(zero_position_map[i]);
    }

    // k値の設定
    ma702[1].setKRatio(zero_position_map[10]);
    ma702[3].setKRatio(zero_position_map[10]);
    ma702[5].setKRatio(zero_position_map[10]);

    uart1_cmd_buffer.clear();

    cmdHandler.setCallback([](char mode, int value1, int value2) 
    {
        // コマンド内容をグローバル変数に保存し、フラグを立てる
        received_mode = mode;
        received_value1 = value1;
        received_value2 = value2;
        command_received = true;
    });

    setbuf(stdout, NULL); // printfのバッファリングを無効化
    printf("System initialized.\n");
    printf("Zero position map: ");
    for (int i = 0; i < 12; i++)    
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

    if (uart1_cmd_ready)
    {
        uart1_cmd_ready = false;
        cmdHandler.onUartReceive(uart1_cmd_buffer.data(), uart1_cmd_buffer.size());
        uart1_cmd_buffer.clear();
    }
    
    if (command_received)
    {
        printf("Command received: mode=%c, value1=%d, value2=%d\n", received_mode, received_value1, received_value2);
        command_received = false;
        ProcessCommand(received_mode, received_value1, received_value2);
    }

    for (size_t i = 0; i < 9; i++)
    {
        current_angle[i] = ma702[i].normalize(ma702[i].read2angle(ma702[i].getRawRotation()) - zero_position_map[i]);
        printf("[%d]: %f \t", i, current_angle[i]);
    }
    printf("\n");

    HAL_Delay(500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // HAL_UART_Transmit(&huart1, uart1_rx_buffer, 1, HAL_MAX_DELAY);
    if (huart->Instance == USART1)
    {
        uart1_cmd_buffer.push_back(uart1_rx_buffer[0]);
        //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        if (uart1_rx_buffer[0] == '\n')
        {
            uart1_cmd_ready = true;
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

