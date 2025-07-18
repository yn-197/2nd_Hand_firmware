#include "command.hpp"
#include <cstring>
#include <sstream>
#include <algorithm>

#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "motion.h"

extern ServoController servoControllers[10];
extern MotionController motionController;
extern bool isServoPidOn[10];

CommandHandler::CommandHandler() {}

void CommandHandler::setCallback(Callback cb) {
    callback_ = cb;
}

void CommandHandler::onUartReceive(const uint8_t* data, size_t len) {
    std::string cmd(reinterpret_cast<const char*>(data), len);

    // 末尾に\rがあれば除去（PCからの送信でCRLFの場合対応）
    if (!cmd.empty() && cmd.back() == '\r') {
        cmd.pop_back();
    }

    // 改行で終わっている場合も除去
    if (!cmd.empty() && cmd.back() == '\n') {
        cmd.pop_back();
    }

    // コマンド解析
    parseCommand(cmd);
}

void CommandHandler::parseCommand(const std::string& cmd) {
    // 例: "A,123" または "A,123,456"
    std::istringstream ss(cmd);
    std::string mode_str, value1_str, value2_str;

    if (std::getline(ss, mode_str, ',') &&
        std::getline(ss, value1_str, ',')) {

        // 3つ目の値（オプション）も取得
        std::getline(ss, value2_str, ',');

        if (mode_str.length() == 1 && !value1_str.empty()) {
            char* endptr1 = nullptr;
            long value1 = strtol(value1_str.c_str(), &endptr1, 10);

            int value2 = 0;
            if (!value2_str.empty()) {
                char* endptr2 = nullptr;
                long v2 = strtol(value2_str.c_str(), &endptr2, 10);
                if (endptr2 != value2_str.c_str() && *endptr2 == '\0') {
                    value2 = static_cast<int>(v2);
                }
            }

            if (endptr1 != value1_str.c_str() && *endptr1 == '\0' && callback_) {
                callback_(mode_str[0], static_cast<int>(value1), value2);
            }
        }
    }
}

void ProcessCommand(char mode, int value1, int value2) {
    //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    switch (mode)
    {
    case 'M':
        switch (value1)
        {
        case 1: {
            // モードM、値1の処理
            printf("start motion1:OPEN\n");
            motionController.setMotion(OPEN);
            break;
        }
        case 2: {
            // モードM、値2の処理
            printf("start motion2:CLOSE\n");
            motionController.setMotion(CLOSE);
            break;
        }
        case 3: {
            // モードM、値3の処理
            printf("start motion3:STOP\n");
            motionController.stopMotion();
            break;
        }
        case 4: {
            // モードM、値4の処理
            printf("start motion4:position control\n");
            motionController.setPosition(100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f);
            break;
        }
        default:
            printf("unknown motion\n");
            break;
        }
        break;
    case 'P':
        switch (value1)
        {
        case 1: 
            printf("start position control:MP1, %d\n", value2);
            isServoPidOn[0] = true;
            servoControllers[0].setTargetAngle(static_cast<float>(value2));
            break;
        case 2:
            printf("start position control:PIP1, %d\n", value2);
            isServoPidOn[1] = true;
            servoControllers[1].setTargetAngle(static_cast<float>(value2));
            break;
        case 3: 
            printf("start position control:MP2, %d\n", value2);
            isServoPidOn[2] = true;
            servoControllers[2].setTargetAngle(static_cast<float>(value2));
            break;
        case 4: 
            printf("start position control:PIP2, %d\n", value2);
            isServoPidOn[3] = true;
            servoControllers[3].setTargetAngle(static_cast<float>(value2));
            break;
        case 5: 
            printf("start position control:MP3, %d\n", value2);
            isServoPidOn[4] = true;
            servoControllers[4].setTargetAngle(static_cast<float>(value2));
            break;
        case 6: 
            printf("start position control:PIP3, %d\n", value2);
            isServoPidOn[5] = true;
            servoControllers[5].setTargetAngle(static_cast<float>(value2));
            break;
        case 7: 
            printf("start position control:CM, %d\n", value2);
            isServoPidOn[6] = true;
            servoControllers[6].setTargetAngle(static_cast<float>(value2));
            break;
        case 8: 
            printf("start position control:MP4, %d\n", value2);
            isServoPidOn[7] = true;
            servoControllers[7].setTargetAngle(static_cast<float>(value2));
            break;
        case 9: 
            printf("start position control:PIP4, %d\n", value2);
            isServoPidOn[8] = true;
            servoControllers[8].setTargetAngle(static_cast<float>(value2));
            break;
        case 10: 
            printf("start position control:ABD, %d\n", value2);
            isServoPidOn[9] = true;
            servoControllers[9].setTargetAngle(static_cast<float>(value2));
            break;
        default:
            printf("unknown position command\n");
            break;
        }
        break;
    default:
        printf("unknown command: %c, value1: %d, value2: %d\n", mode, value1, value2);
        break;
    }
}