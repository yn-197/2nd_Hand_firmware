/**
  ******************************************************************************
  * @file           : AS5048A.h
  * @brief          : Header for as5048a.c file.
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
#ifndef __AS5048A_H
#define __AS5048A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <math.h>
#include "encoder_base.h"

// --- クラス宣言はextern "C"の外で ---
class AS5048A : public EncoderBase {

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

public:
    AS5048A(SPI_HandleTypeDef *hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs);
    void init();
    void close();
    void open();
    uint16_t read(uint16_t registerAddress);
    uint16_t write(uint16_t registerAddress, uint16_t data);
    uint16_t getRawRotation() override;
    int getRotation();
    uint16_t getState();
    uint8_t error();
    uint8_t getGain();
    uint16_t getErrors();
    void setZeroPosition(uint16_t arg_position);
    uint16_t getZeroPosition();
    float normalize(float angle) override;
    float read2angle(uint16_t angle) override;

private:
    uint8_t spiCalcEvenParity(uint16_t value);
};

// --- 定数定義 ---
const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const int AS5048A_DIAG_AGC                      = 0x3FFD;
const int AS5048A_MAGNITUDE                     = 0x3FFE;
const int AS5048A_ANGLE                         = 0x3FFF;
const int AEAT9922_Angle                        = 0x0008;


#ifdef __cplusplus
}
#endif

#endif /* __AS5048A_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
