#include "stm32f4xx.h"
#include "stm32f4xx_adc.h" 
#include "misc.h" 
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "joystick.h"
#include "uart.h"
#include "board.h"


void joystick_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(ADC_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ADC_CLK_PINPACK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = ADC_PIN_X | ADC_PIN_Y;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);
    
    ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
    
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_8Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);
	
	ADC_Init(ADC_PINSPACK, &ADC_InitStruct);
	
	// Enable ADC
	ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t joystick_read_raw(uint8_t channel)
{
    uint32_t timeout = 0xFFF;
	
	ADC_RegularChannelConfig(ADC_PINSPACK, channel, 1, ADC_SampleTime_15Cycles);
    
	// Start software conversion
	ADC_PINSPACK->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	
	// Wait till done
	while (!(ADC_PINSPACK->SR & ADC_SR_EOC)) {
		if (timeout-- == 0x00) {
			return 0;
		}
	}
    
	return ADC_PINSPACK->DR;
}

float joystick_read(joystick_data_t *joystick)
{
    uint32_t timeout = 0xFFF;
    
	ADC_RegularChannelConfig(ADC_PINSPACK, joystick->channel, 1, ADC_SampleTime_15Cycles);
    
	// Start software conversion
	ADC_PINSPACK->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	
	// Wait till done
	while (!(ADC_PINSPACK->SR & ADC_SR_EOC)) {
		if (timeout-- == 0x00) {
			//return 0;
            uart_printf("joystick error\n\r");
		}
	}
    
	uint16_t raw = ADC_PINSPACK->DR;
    
    if((raw >= joystick->raw_min) && (raw <= joystick->raw_max) && ((raw > joystick->middle + joystick->deadband) || (raw < joystick->middle - joystick->deadband))){
        // scale result
        joystick->value = (joystick->max - joystick->min) * ((float)(raw - joystick->middle) / (joystick->raw_max - joystick->raw_min));
    }
    else{
        joystick->value = 0.0f;
    }
    
    if((joystick->value < -joystick->threshold) && (joystick->setpoint >= joystick->min)){
        joystick->setpoint = joystick->setpoint - (0.01 - joystick->value / 200);
    }
    if((joystick->value > joystick->threshold) && (joystick->setpoint <= joystick->max)){
        joystick->setpoint = joystick->setpoint + (0.01 + joystick->value / 200);
    }
    /*if((joystick->value < -joystick->threshold_normal) && (joystick->setpoint >= joystick->min)){
        joystick->setpoint = joystick->setpoint - 0.3;
    }
    if((joystick->value > joystick->threshold_normal) && (joystick->setpoint <= joystick->max)){
        joystick->setpoint = joystick->setpoint + 0.3;
    }
    if((joystick->value < -joystick->threshold_slow) && (joystick->setpoint >= joystick->min)){
        joystick->setpoint = joystick->setpoint - 0.1;
    }
    if((joystick->value > joystick->threshold_slow) && (joystick->setpoint <= joystick->max)){
        joystick->setpoint = joystick->setpoint + 0.1;
    }*/
    return joystick->setpoint;
}