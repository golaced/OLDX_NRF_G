#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"
#define TX_LED 0
#define RX_LED 1

void LED_GPIO_Config(void);
void LED_STATE(u8 sel,u8 on);

void KEY_GPIO_Config(void);
extern u8 key[8];
void RED_KEY(void);
extern u8 key_sel[4];
void RED_KEY_SEL(void);
void KEY_SEL_Config(void);
#endif /* __LED_H */
