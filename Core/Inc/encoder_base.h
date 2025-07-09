#ifndef __ENCODER_BASE_H
#define __ENCODER_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "main.h"

class EncoderBase {
public:
    virtual uint16_t getRawRotation() = 0;
    virtual float normalize(float angle) = 0;
    virtual float read2angle(uint16_t angle) = 0;
    virtual ~EncoderBase() {}
};

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_BASE_H */