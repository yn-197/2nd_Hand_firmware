#include "mode_select.hpp"
#include "gpio.h"
#include "main.h"
#include "adc.h"
#include "alt-main.h"
#include "servo_control.h"
#include "as5048a.h"
#include "motion.h"
#include <stdio.h>

#define SELECT_TIMEOUT_MS 20000 // 20秒
#define CHATTERING_DELAY_MS 30

extern AS5048A as5048a[5];
extern ServoController servoControllers[10];
extern MotionController motionController;
extern bool isServoPidOn[10];

float current_angle[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

ModeSelector::ModeSelector(int mode_num)
    : mode_num_(mode_num), mode_(0), selected_mode_(-1), sw1_prev_(1), sw2_prev_(1)
{
}

uint32_t adc_val_ch0 = 0;
uint32_t adc_val_ch2 = 0;

void ModeSelector::selectMode()
{
    uint32_t start = HAL_GetTick();
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
    HAL_Delay(50);
    while (1)
    {
        // タイムアウト判定
        if (HAL_GetTick() - start > SELECT_TIMEOUT_MS)
        {
            selected_mode_ = -1; // タイムアウト時は未選択
            break;
        }

        // SW1: モード選択
        uint8_t sw1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
        if (sw1 == GPIO_PIN_RESET && sw1_prev_ == GPIO_PIN_SET)
        {
            HAL_Delay(CHATTERING_DELAY_MS); // チャタリング防止
            if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET)
            {
                mode_ = (mode_ + 1) % mode_num_;
                for (int i = 0; i < mode_ + 1; i++)
                {
                    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
                    HAL_Delay(50);
                    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
                    HAL_Delay(50);
                }
            }
        }
        sw1_prev_ = sw1;

        // SW2: 決定
        uint8_t sw2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
        if (sw2 == GPIO_PIN_RESET && sw2_prev_ == GPIO_PIN_SET)
        {
            HAL_Delay(CHATTERING_DELAY_MS); // チャタリング防止
            if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET)
            {
                selected_mode_ = mode_;
                HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
                HAL_Delay(50);
                HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
                HAL_Delay(50);
                break;
            }
        }
        sw2_prev_ = sw2;

        // 必要ならここでLED表示など
        // HAL_Delay(50);
    }
    // モード選択完了後の処理
    if (selected_mode_ != -1)
    {
        executeSelectedMode();
    }
}

int ModeSelector::getSelectedMode() const
{
    return selected_mode_;
}

// 各モードの処理をここに記述
void ModeSelector::executeSelectedMode()
{
    switch (selected_mode_)
    {
    case 0:
        isServoPidOn[0] = false;
        isServoPidOn[1] = false;
        isServoPidOn[2] = false;
        isServoPidOn[3] = false;
        isServoPidOn[4] = false;
        isServoPidOn[5] = false;
        isServoPidOn[6] = false;
        isServoPidOn[7] = false;
        isServoPidOn[8] = false;
        isServoPidOn[9] = false;
        while (1)
        {
            servoControllers[0].onOffControl(forward);
            servoControllers[1].onOffControl(forward);
            servoControllers[2].onOffControl(forward);
            servoControllers[3].onOffControl(forward);
            servoControllers[4].onOffControl(forward);
            servoControllers[5].onOffControl(forward);
            servoControllers[6].onOffControl(forward);
            servoControllers[7].onOffControl(forward);
            servoControllers[8].onOffControl(forward);
            servoControllers[9].onOffControl(forward);
            HAL_Delay(1000);
            servoControllers[0].onOffControl(reverse);
            servoControllers[1].onOffControl(reverse);
            servoControllers[2].onOffControl(reverse);
            servoControllers[3].onOffControl(reverse);
            servoControllers[4].onOffControl(reverse);
            servoControllers[5].onOffControl(reverse);
            servoControllers[6].onOffControl(reverse);
            servoControllers[7].onOffControl(reverse);
            servoControllers[8].onOffControl(reverse);
            servoControllers[9].onOffControl(reverse);
            HAL_Delay(1000);
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        break;
    case 1:
        while (1)
        {
            for (size_t i = 0; i < 5; i++)
            {
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                current_angle[i] = as5048a[i].normalize(as5048a[i].read2angle(as5048a[i].getRawRotation()));
                printf("[%d]: %f \t", i, current_angle[i]);
            }
            printf("\n");
            HAL_Delay(100);
        }
        break;
    case 2:
        while (1)
        {
            ADC_ChannelConfTypeDef sConfig = {0};
            sConfig.Channel = ADC_CHANNEL_0;
            sConfig.Rank = ADC_REGULAR_RANK_1;
            sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
            HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            adc_val_ch0 = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);

            // チャンネル2の変換
            sConfig.Channel = ADC_CHANNEL_2;
            HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            adc_val_ch2 = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);

            printf("%lu, %lu\n", adc_val_ch0, adc_val_ch2);

            HAL_Delay(100);
        }
        break;
    case 3:
        while (1)
        {
            motionController.setMotion(OPEN);
            HAL_Delay(1000);
            motionController.setMotion(CLOSE);
            HAL_Delay(1000);
        }
        break;
    case 4:
        while (1)
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            HAL_Delay(500);
        }
        break;
    case 5:
        // モード5の処理
        break;
    case 6:
        // モード6の処理
        break;
    case 7:
        // モード7の処理
        break;
    default:
        // 未選択やタイムアウト時の処理
        break;
    }
}