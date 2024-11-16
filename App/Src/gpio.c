
#include <stm32h747xx.h>

#include "gpio.h"

void GPIO_Init() {

  // TEST_PIN GPIO_PF9
  GPIOF->MODER = ((uint32_t)(GPIOF->MODER) & ~GPIO_MODER_MODE9_Msk) | (1 << GPIO_MODER_MODE9_Pos);
  GPIOF->BSRR = GPIO_BSRR_BR9;
}

void TEST_PIN(int d) {

  if(d == 1) {
    GPIOF->BSRR = GPIO_BSRR_BS9;
  } else if(d == 0) {
    GPIOF->BSRR = GPIO_BSRR_BR9;
  } else {
    uint32_t odr = GPIOF->ODR;
    GPIOF->BSRR = ((odr & GPIO_ODR_OD9) << 16) | (~odr & GPIO_ODR_OD9);
  }
}
