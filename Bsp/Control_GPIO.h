#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1


#define LED1_PORT GPIOB
#define LED2_PORT GPIOB

#define LED1_PIN GPIO_Pin_14
#define LED2_PIN GPIO_Pin_12


#define LED1_RCC RCC_APB2Periph_GPIOB
#define LED2_RCC RCC_APB2Periph_GPIOB


#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_PORT,LED2_PIN)

      
#define	  FLIP_LED1		GPIOB->ODR ^= LED1_PIN
#define		FLIP_LED2	  GPIOB->ODR ^= LED2_PIN;

					
#define DISABLE_OVERLOAD GPIO_SetBits(GPIOA,GPIO_Pin_11)
#define ENABLE_OVERLOAD  GPIO_ResetBits(GPIOA,GPIO_Pin_11)			
					
void Contorl_GPIO_Config(void);

#endif /* __LED_H */
