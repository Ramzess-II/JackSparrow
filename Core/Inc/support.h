#ifndef INC_SUPPORT_H_
#define INC_SUPPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

//----------------------- подключим файлы ------------------------------------//
#include "stm32f0xx_hal.h"
//----------------------- дефайним значения ----------------------------------//
#define COUNT_SCREENSAVER              120


#define LED_Pin                        GPIO_PIN_13
#define LED_GPIO_Port                  GPIOC
#define STEP_Pin                       GPIO_PIN_2
#define STEP_GPIO_Port                 GPIOB
#define DIR_Pin                        GPIO_PIN_10
#define DIR_GPIO_Port                  GPIOB
#define Inerupts_Pin                   GPIO_PIN_6

#define RAD_TO_DEG                     57.295779513082320876798154814105
#define degrees(rad)                   ((rad)*RAD_TO_DEG)
//----------------------- объявим функции ------------------------------------//
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t constrain(int32_t x, int32_t min, int32_t max);
//----------------------- объявим структуры ----------------------------------//




#ifdef __cplusplus
}
#endif



#endif /* INC_SUPPORT_H_ */
