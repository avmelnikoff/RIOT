/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2016 Alexander Melnikov
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f1
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Alexander Melnikov <avmelnikoff@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "periph_conf.h"
#include "periph/adc.h"
#include "periph/gpio.h"

/* guard in case that no ADC device is defined */
#if ADC_NUMOF

typedef struct {
    int max_value;
} adc_config_t;

adc_config_t adc_config[ADC_NUMOF];

/* static function definitions */
static void _pin_config(void);

int adc_init(adc_t dev, adc_precision_t precision)
{
    ADC_TypeDef *adc = 0;

    adc_poweron(dev);

    switch (dev) {
#if ADC_0_EN
        case ADC_0:
            adc = ADC_0_DEV;
            break;
#endif
#if ADC_1_EN
        case ADC_1:
            adc = ADC_1_DEV;
            break;
#endif
        default:
            return -1;
    }

    /* reset control registers */
    adc->CR1 = 0;
    adc->CR2 = 0;
    adc->SQR1 = 0;

    /* set precision */
    switch (precision) {
        /* stm32f10x has only 12-bit resolution, see RM0008 */
        case ADC_RES_12BIT:
            adc_config[dev].max_value = 0xfff;
            break;
        case ADC_RES_6BIT:
        case ADC_RES_8BIT:
        case ADC_RES_10BIT:
        case ADC_RES_14BIT:
        case ADC_RES_16BIT:
            adc_poweroff(dev);
            return -1;
            break;
    }

    /* enable the ADC module */
    adc->CR2 |= ADC_CR2_ADON;

    /* configure pins for channels */
    _pin_config();

    return 0;
}

static void _pin_config(void)
{
#ifdef ADC_CH0_PIN
    gpio_init_af(ADC_CH0_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH1_PIN
    gpio_init_af(ADC_CH1_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH2_PIN
    gpio_init_af(ADC_CH2_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH3_PIN
    gpio_init_af(ADC_CH3_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH4_PIN
    gpio_init_af(ADC_CH4_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH5_PIN
    gpio_init_af(ADC_CH5_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH6_PIN
    gpio_init_af(ADC_CH6_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH7_PIN
    gpio_init_af(ADC_CH7_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH8_PIN
    gpio_init_af(ADC_CH8_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH9_PIN
    gpio_init_af(ADC_CH9_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH10_PIN
    gpio_init_af(ADC_CH10_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH11_PIN
    gpio_init_af(ADC_CH11_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH12_PIN
    gpio_init_af(ADC_CH12_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH13_PIN
    gpio_init_af(ADC_CH13_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH14_PIN
    gpio_init_af(ADC_CH14_PIN, GPIO_AF_IN_ANALOG);
#endif
#ifdef ADC_CH15_PIN
    gpio_init_af(ADC_CH15_PIN, GPIO_AF_IN_ANALOG);
#endif
}

int adc_sample(adc_t dev, int channel)
{
    ADC_TypeDef *adc = 0;

    switch (dev) {
#if ADC_0_EN
        case ADC_0:
            adc = ADC_0_DEV;
            break;
#endif
#if ADC_1_EN
        case ADC_1:
            adc = ADC_1_DEV;
            break;
#endif
    }

    /* switch on(temperature sensor and Vrefint */
    if (channel == 16 || channel == 17) {
        adc->CR2 |= ADC_CR2_TSVREFE;
    }

    /* set L bits to 0 to indicate that 1 conversion contain channel */
    adc->SQR1 = 0;
    /* set channel number into the 1 conversion */
    adc->SQR3 = channel & 0x1f;
    /* start single conversion */
    //adc->CR2 |= ADC_CR2_SWSTART;
    adc->CR2 |= ADC_CR2_ADON; /* starts conversion by writing 1 for a second time */
    /* wait until conversion is complete */
    while (!(adc->SR & ADC_SR_EOC));

    /* enable internal channels (temperature sensor and Vrefint) */
    if (channel == 16 || channel == 17) {
        adc->CR2 &= ~ADC_CR2_TSVREFE;
    }

    /* read and return result */
    return (int)adc->DR;
}

void adc_poweron(adc_t dev)
{
    switch (dev) {
#if ADC_0_EN
        case ADC_0:
            ADC_0_CLKEN();
            break;
#endif
#if ADC_1_EN
        case ADC_1:
            ADC_1_CLKEN();
            break;
#endif
    }
}

void adc_poweroff(adc_t dev)
{
    switch (dev) {
#if ADC_0_EN
        case ADC_0:
            ADC_0_CLKDIS();
            break;
#endif
#if ADC_1_EN
        case ADC_1:
            ADC_1_CLKDIS();
            break;
#endif
    }
}

int adc_map(adc_t dev, int value, int min, int max)
{
    return (int)adc_mapf(dev, value, (float)min, (float)max);
}

float adc_mapf(adc_t dev, int value, float min, float max)
{
    return ((max - min) / ((float)adc_config[dev].max_value)) * value;
}

#endif /* ADC_NUMOF */
