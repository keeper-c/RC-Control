#include "drv_bsp.h"

void BSP_LED(Enum_BSP_LEDState state, uint8_t LED_Location)
{
    GPIO_PinState pin_state = (GPIO_PinState)state;
    uint16_t pin = 0;

    switch(LED_Location)
    {
        case 1:
            pin = GPIO_PIN_1;
            break;
        case 2:
            pin = GPIO_PIN_2;
            break;
        case 3:
            pin = GPIO_PIN_3;
            break;
        case 4:
            pin = GPIO_PIN_4;
            break;
        case 5:
            pin = GPIO_PIN_5;
            break;
        case 6:
            pin = GPIO_PIN_6;
            break;
        case 7:
            pin = GPIO_PIN_7;
            break;
        case 8:
            pin = GPIO_PIN_8;
            break;
        default:
            // 非法编号直接退出，不操作硬件
            return;
    }

    HAL_GPIO_WritePin(GPIOG, pin, pin_state);
}    