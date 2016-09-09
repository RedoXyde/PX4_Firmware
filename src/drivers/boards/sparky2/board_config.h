/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4FMUv1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4IO connection configuration */
//#define PX4IO_SERIAL_DEVICE	"/dev/ttyS1"
#define UDID_START		0x1FFF7A10

//#ifdef CONFIG_STM32_SPI2
//#  error "SPI2 is not supported on this board"
//#endif

#if defined(CONFIG_STM32_CAN1)
#  warning "CAN1 is not supported on this board"
#endif

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

// PX4FMUv1_Blue_PB15 > Sparky2_Blue_PB5
#define GPIO_LED1		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
// PX4FMUv1_Amber_PB14 > Sparly2_Amber_PB4
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)

/* External interrupts */
#define GPIO_EXTI_COMPASS	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN7)

/* SPI chip selects */
/*
#define GPIO_SPI_CS_GYRO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SPI_CS_ACCEL	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_MPU		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_SPI_CS_SDCARD	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
*/
/* Sparky2, SPI3 - Flash, RFM22B */
#define GPIO_SPI_CS_FLASH	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)
#define GPIO_SPI_CS_RADIO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)
/* Sparky2, SPI1 - MPU9250 */
#define GPIO_SPI_CS_IMU	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)

#define PX4_SPI_BUS_SENSORS	1

/*
 * Use these in place of the spi_dev_e enumeration to
 * select a specific SPI device on SPI3
 */
#define SPIDEV_FLASH	1

#define PX4_SPIDEV_RADIO	2
#define PX4_SPIDEV_MPU		3

/*
 * Optional devices on IO's external port
 */
/*
#define PX4_SPIDEV_ACCEL_MAG	2
*/

/*
 * I2C busses
 */

/* Sparky2, FlexiPort CONN1, w/ Pressure Sensor */
#define PX4_I2C_BUS_ONBOARD	1
/* Sparky2, FlexiPort CONN3 */
#define PX4_I2C_BUS_EXPANSION 2
/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_IC2_OBDEV_MS5611 0xee
// #define PX4_I2C_OBDEV_HMC5883	0x1e
// #define PX4_I2C_OBDEV_EEPROM	NOTDEFINED
// #define PX4_I2C_OBDEV_LED	0x55

// #define PX4_I2C_OBDEV_PX4IO_BL	0x18
// #define PX4_I2C_OBDEV_PX4IO	0x1a

/* User GPIOs
 *
 * GPIO0-4 are the buffered high-power GPIOs.
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)
#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)

 /*
 * GPIO2-5 are the USART2 pins.
 * GPIO6-7 are the CAN2 pins.
 */
#define GPIO_GPIO2_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN13)
#define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN2)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_GPIO_DIR		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/*
 * Tone alarm output
 * TODO FIXME
 */
#define TONE_ALARM_TIMER	4	/* timer 11 */
#define TONE_ALARM_CHANNEL	3	/* channel 3 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN14)

/*
 * PWM
 *
 *  Four buffered PWM can be configured, routed to the
 *      - Q1A, PB14, TIM12CH1
 *      - Q1B, PB15, TIM12CH2
 *      - Q2A, PC8,  TIM8CH3
 *      - Q2B, PC9,  TIM8CH4
 *
 *  Six PWM can be configured and are routed to the Servo Port
 *      - Servo_Out_1   PB0, TIM3CH3
 *      - Servo_Out_2   PB1, TIM3CH4
 *      - Servo_Out_3   PA3, TIM9CH2
 *      - Servo_Out_4   PA2, TIM2CH3
 *      - Servo_Out_5   PA1, TIM5CH2
 *      - Servo_Out_6   PA0, TIM5CH1
 *
 */
#define GPIO_TIM2_CH3OUT	GPIO_TIM2_CH3OUT_1

#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_1

#define GPIO_TIM5_CH1OUT	GPIO_TIM5_CH1OUT_1
#define GPIO_TIM5_CH2OUT	GPIO_TIM5_CH2OUT_1

#define GPIO_TIM8_CH3OUT  GPIO_TIM8_CH3OUT_1
#define GPIO_TIM8_CH4OUT  GPIO_TIM8_CH4OUT_1

#define GPIO_TIM9_CH2OUT	GPIO_TIM9_CH2OUT_1

#define GPIO_TIM12_CH1OUT GPIO_TIM12_CH1OUT_2
#define GPIO_TIM12_CH2OUT GPIO_TIM12_CH2OUT_2

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN8)

/*
 * High-resolution timer
 * FIXME
 */
#define HRT_TIMER		1	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* Assume SBUS and PPM use the same input, PC7 */
#define HRT_PPM_CHANNEL		2	/* use capture/compare channel 2 */
#define GPIO_PPM_IN		(GPIO_ALT|GPIO_AF1|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

#define BOARD_NAME "SPARKY2"

/* By Providing BOARD_ADC_USB_CONNECTED this board support the ADC
 * system_power interface, and therefore provides the true logic
 * GPIO BOARD_ADC_xxxx macros.
 */
// #define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
// #define BOARD_ADC_BRICK_VALID   (!px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
// #define BOARD_ADC_SERVO_VALID   (!px4_arch_gpioread(GPIO_VDD_SERVO_VALID))
// #define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
// #define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))
//
// #define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
       {GPIO_GPIO0_INPUT, GPIO_GPIO0_OUTPUT, 0}, \
       {GPIO_GPIO1_INPUT, GPIO_GPIO1_OUTPUT, 0}, \
       {GPIO_GPIO2_INPUT, GPIO_GPIO2_OUTPUT, GPIO_USART2_CTS_1}, \
       {GPIO_GPIO3_INPUT, GPIO_GPIO3_OUTPUT, GPIO_USART2_RTS_1}, \
       {GPIO_GPIO4_INPUT, GPIO_GPIO4_OUTPUT, GPIO_USART2_TX_1}, \
       {GPIO_GPIO5_INPUT, GPIO_GPIO5_OUTPUT, GPIO_USART2_RX_1}, \
       {GPIO_GPIO6_INPUT, GPIO_GPIO6_OUTPUT, GPIO_CAN2_TX_2}, \
       {GPIO_GPIO7_INPUT, GPIO_GPIO7_OUTPUT, GPIO_CAN2_RX_2}, }

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120
/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */

__END_DECLS
