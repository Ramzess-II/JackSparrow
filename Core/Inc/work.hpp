#ifndef INC_WORK_HPP_
#define INC_WORK_HPP_

#ifdef __cplusplus
extern "C" {
#endif

//----------------------- подключим файлы ------------------------------------//
#include "stm32f0xx_hal.h"
#include <math.h>
#include "ARGB.h"
//----------------------- дефайним значения ----------------------------------//

//----------------------- объявим функции ------------------------------------//
void doWork (void);
void Work (void);
void dmaCalback (DMA_HandleTypeDef *_hdma);
void startMoveAndLed (uint8_t delay, uint32_t count);
//----------------------- объявим структуры ----------------------------------//


#ifdef __cplusplus
}
#endif

#endif /* INC_WORK_HPP_ */
