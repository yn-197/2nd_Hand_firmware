#include "command.hpp"
#include <cstring>
#include <sstream>
#include <algorithm>

#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "usart.h"

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
    // 例: "A,1234"
    if (cmd.find(',') == std::string::npos) return;

    std::istringstream ss(cmd);
    std::string mode_str, value_str;
    if (std::getline(ss, mode_str, ',') &&
        std::getline(ss, value_str, ',')) {
        if (mode_str.length() == 1 && !value_str.empty()) {
            char* endptr = nullptr;
            long value = strtol(value_str.c_str(), &endptr, 10);
            if (endptr != value_str.c_str() && *endptr == '\0' && value >= 0 && value <= 4096 && callback_) {
                callback_(mode_str[0], static_cast<int>(value));
            }
        }
    }
}

void ProcessCommand(char mode, int value) {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    switch (mode)
    {
    case 'M':
        switch (value)
        {
        case 1: {
            // モードM、値1の処理
            uint8_t msg[] = "start motion1\n";
            HAL_UART_Transmit(&huart1, msg, sizeof(msg), HAL_MAX_DELAY);
            break;
        }
        case 2: {
            // モードM、値2の処理
            uint8_t msg2[] = "start motion2\n";
            HAL_UART_Transmit(&huart1, msg2, sizeof(msg2), HAL_MAX_DELAY);
            break;
        }
        default:
            uint8_t msg_default[] = "unknown motion\n";
            HAL_UART_Transmit(&huart1, msg_default, sizeof(msg_default), HAL_MAX_DELAY);
            break;
        }
        break;
    
    default:
        uint8_t msg[] = "unknown command\n";
        HAL_UART_Transmit(&huart1, msg, sizeof(msg), HAL_MAX_DELAY);
        break;
    }
}