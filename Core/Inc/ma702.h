/**
  ******************************************************************************
  * @file           : ma702.h
  * @brief          : Header for ma702.c file.
  *                   This file contains the common defines of the application.
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Raimondas Pomarnacki.
  * All rights reserved.</center></h2>
  *
  * This software component is not licensed,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MA702_H
#define __MA702_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <math.h>
#include "encoder_base.h"

#ifdef __cplusplus
}
#endif

class MA702 : public EncoderBase {

    uint8_t errorFlag = 0;
    uint16_t _cs;
    uint16_t cs;
    GPIO_TypeDef* _ps;
    SPI_HandleTypeDef* _spi;
    uint8_t dout;
    uint8_t din;
    uint8_t clk;
    uint16_t position;
    uint16_t transaction(uint16_t data);
    float k_ratio = 1.0f; //歪み量　1.0fは歪みなし

public:
    MA702(SPI_HandleTypeDef *hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs);

    void init();
    void close();
    void open();
    uint16_t read(uint8_t registerAddress);
    uint16_t write(uint8_t registerAddress, uint8_t data);

    uint16_t getRawRotation() override;
    int getRotation();
    uint16_t getState();
    uint8_t error();
    uint16_t getErrors();
    void setZeroPosition(uint16_t arg_position);
    uint16_t getZeroPosition();
    float normalize(float angle) override;
    float read2angle(uint16_t angle) override;
    float getKRatio(float angle1, float angle2);
    void setKRatio(float k_ratio) {
        this->k_ratio = k_ratio;
    }

private:

};

// 定数定義
const int MA702_ZERO_POSITION_LOW = 0x00;
const int MA702_ZERO_POSITION_HIGH = 0x01;
const int MA702_ABZ_LOW = 0x04;
const int MA702_ABZ_HIGH = 0x05;
const int MA702_MAGNETIC_THRESHOLDS = 0x06;
const int MA702_ROOTATION_DIRECTION = 0x09;
const int MA702_MAGNET_FLAG = 0x1B;

#endif /* __MA702_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/