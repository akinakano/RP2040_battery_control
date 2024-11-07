#include <stdbool.h>
#include <stm32h747xx.h>

#include "gpio.h"

int BatterySBUS_Supported = 0;

// LEDを点ける
void GPIO_DebugLedOn(uint16_t idx)
{
    if (idx & GPIO_DEBUG_LED_1)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BS12_Pos);
    }
    if (idx & GPIO_DEBUG_LED_2)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BS13_Pos);
    }
    if (idx & GPIO_DEBUG_LED_3)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BS14_Pos);
    }
    if (idx & GPIO_DEBUG_LED_4)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BS15_Pos);
    }
}

// LEDを消す
void GPIO_DebugLedOff(uint16_t idx)
{
    if (idx & GPIO_DEBUG_LED_1)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR12_Pos);
    }
    if (idx & GPIO_DEBUG_LED_2)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR13_Pos);
    }
    if (idx & GPIO_DEBUG_LED_3)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR14_Pos);
    }
    if (idx & GPIO_DEBUG_LED_3)
    {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR15_Pos);
    }
}


//SW
uint8_t GPIO_Read_SW(void)
{
    int16_t sw_state = GPIOC->IDR & GPIO_IDR_ID13;
    if(sw_state == 0){
        return 1;
    }else{
        return 0;
    }
}

//15V-SW
uint8_t GPIO_Read_15V_SW(void)
{
    int16_t sw_state = GPIOB->IDR & GPIO_IDR_ID1;
    if(sw_state == 0){
        return 1;
    }else{
        return 0;
    }
}

void GPIO_HeartBeat() {

  uint32_t odr = GPIOA->ODR;
  GPIOA->BSRR = ((odr & GPIO_ODR_OD10) << 16) | (~odr & GPIO_ODR_OD10);

  uint32_t odr2 = GPIOB->ODR;
  GPIOB->BSRR = ((odr2 & GPIO_ODR_OD14) << 16) | (~odr2 & GPIO_ODR_OD14);
}

void GPIO_15V(int sw) {

  if(sw) {
    GPIOB->BSRR = GPIO_BSRR_BS2;
  } else {
    GPIOB->BSRR = GPIO_BSRR_BS2 << 16;
  }
}

// GPIOの設定
void GPIO_Init(void)
{
    uint32_t rcc_ahb4enr = //RCC_APB4ENR_SYSCFGEN
                          (1 << RCC_AHB4ENR_GPIOGEN_Pos)
                         | (1 << RCC_AHB4ENR_GPIOEEN_Pos)
                         | (1 << RCC_AHB4ENR_GPIODEN_Pos)
                         | (1 << RCC_AHB4ENR_GPIOCEN_Pos)
                         | (1 << RCC_AHB4ENR_GPIOBEN_Pos)
                         | (1 << RCC_AHB4ENR_GPIOAEN_Pos);
    RCC->AHB4ENR |= rcc_ahb4enr;

    //LED1,2,3,4
    GPIOD->MODER = ((uint32_t)(GPIOD->MODER) & ~GPIO_MODER_MODE12_Msk) | (1 << GPIO_MODER_MODE12_Pos);
    GPIOD->MODER = ((uint32_t)(GPIOD->MODER) & ~GPIO_MODER_MODE13_Msk) | (1 << GPIO_MODER_MODE13_Pos);
    GPIOD->MODER = ((uint32_t)(GPIOD->MODER) & ~GPIO_MODER_MODE14_Msk) | (1 << GPIO_MODER_MODE14_Pos);
    GPIOD->MODER = ((uint32_t)(GPIOD->MODER) & ~GPIO_MODER_MODE15_Msk) | (1 << GPIO_MODER_MODE15_Pos);

    GPIOA->MODER = ((uint32_t)(GPIOA->MODER) & ~GPIO_MODER_MODE10_Msk) | (1 << GPIO_MODER_MODE10_Pos);
    GPIOA->BSRR = (1 << GPIO_BSRR_BS10_Pos);

    //SPI1_CS PG10
    GPIOG->MODER = ((uint32_t)(GPIOG->MODER) & ~GPIO_MODER_MODE10_Msk) | (1 << GPIO_MODER_MODE10_Pos);

    uint32_t u32;
    //for debug GPIO出力
    /*
    u32 = GPIOA -> AFR[0];
    u32 &= 0xFFFFF000;
    u32 |= 0x00000111;
    GPIOA -> AFR[0] = u32;
    */
    // PA0, PA1 をOUTPUTに設定
/*
    u32 = GPIOA -> MODER;
    u32 &= 0xABFFFFF0;
    u32 |= 0x00000005;
    GPIOA -> MODER = u32;
*/


    //SPI2_CS PB9 他はCubeMXで自動生成
    //GPIOB->MODER = ((uint32_t)(GPIOB->MODER) & ~GPIO_MODER_MODE9_Msk) | (1 << GPIO_MODER_MODE9_Pos);

    BatterySBUS_Supported = (GPIOC->IDR & GPIO_IDR_ID6) == 0;

}
