#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

void Flash_WriteFloatArray(float* data);
void Flash_ReadFloatArray(float* dest);


#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */