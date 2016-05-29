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

#define UDID_START		0x1FFF7A10

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

// Sparky2_Blue_PB5
#define GPIO_LED1		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
// Sparky2_Amber_PB4
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)
// Sparky2_???_PB6
#define GPIO_LED3		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)

/* External interrupts */
/* Not used anywhere in the code
#define GPIO_EXTI_COMPASS	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN7)
*/

/* Sparky2, SPI1 - MPU9250 */
/* SPI chip selects */
/* SPI BUS for sensors: MPU9250, MS5611, L3GD20, HMC5883 LSM303D MPU6000
 * In this case, we're only interested in MPU9250, but whatever
 */
#define PX4_SPI_BUS_SENSORS	1
/* SPI Chip select for MPU9250 */
#define GPIO_SPI_CS_IMU	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)
/*
 * Use these in place of the spi_dev_e enumeration to
 * select a specific SPI device on SPI1
 */
#define PX4_SPIDEV_MPU		1

/* Sparky2, SPI3 - Flash, RFM22B */
/* SPI Bus for MTD */
#define PX4_SPI_BUS_MTD 3
/* SPI Chip Select for MTD */
#define GPIO_SPI_CS_FLASH	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)
/* SPI Bus for RADIO */
#define PX4_SPI_BUS_RADIO 3
/* SPI Bus for RADIO */
#define GPIO_SPI_CS_RADIO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/*
 * Use these in place of the spi_dev_e enumeration to
 * select a specific SPI device on SPI3
 *
 * Note: Do not prepend SPIDEV_FLASH w/ PX4_. The #define is used by NuttX code.
 */
#define PX4_SPIDEV_RADIO	1
#define SPIDEV_FLASH      2

/*
 * I2C busses
 */

/* Sparky2 I2C1, FlexiPort CONN1 (bottom side of the board), and MS5611 */
#define PX4_I2C_BUS_ONBOARD	1
/* Sparky2 I2C2, FlexiPort CONN3 */
//#define PX4_I2C_BUS_EXPANSION 2
// Actually, let's keep CONN3 as USART and re-use the I2C on CONN1 instead
#define PX4_I2C_BUS_EXPANSION 1
// No I2C Led :/
#define PX4_I2C_BUS_LED       1
/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_IC2_OBDEV_MS5611 0xee
// #define PX4_I2C_OBDEV_HMC5883	0x1e
// #define PX4_I2C_OBDEV_EEPROM	NOTDEFINED
// #define PX4_I2C_OBDEV_LED	0x55

/*
 * ADC Channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 12) | (1 << 13)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL   13 /* PC3 */
#define ADC_BATTERY_CURRENT_CHANNEL   12 /* PC2 */
#define ADC_5V_RAIL_SENSE             0  /* Not in use, PA8 is not an ADC */
#define ADC_AIRSPEED_VOLTAGE_CHANNEL  0  /* Not in use */

/* User GPIOs
 *
 * GPIO0-5 are the PWM ouputs
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO2_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)

/*
 * GPIO6-9 are the buffered high-power GPIOs.
 */
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)
#define GPIO_GPIO9_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)
#define GPIO_GPIO8_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)

/*
 * Tone alarm output
 * FIXME Check Timer and Channel, so it doesn't conflict with any other periph
 * FIXME Fix the GPIO. Currently assigned to an unrouted pin
 */
#define TONE_ALARM_TIMER	4	/* timer 11 */
#define TONE_ALARM_CHANNEL	3	/* channel 3 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN1)

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

#define DIRECT_PWM_OUTPUT_CHANNELS 6

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN8)

/*
 * High-resolution timer
 * Note: Don't use "complementary" channels (such as ch1 & ch2) for HRT and PPM
 *       it just doesn't work (see note in src/drivers/stm32/drv_hrt.c)
 * Note: Don't use TIM1 or TIM8, they're "advanced" timers. I tried with TIM1
 *       and the systicks went crazy
 *
 * FIXME: Check the Timer and Channel are not conflicting with another periph
 */
#define HRT_TIMER		3       /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL	4	/* use capture/compare channel 4 */

/*
 * PPM Input channel and pin
 * Uses the same Timer as HRT, but a different channel
 *
 * FIXME: Currently assuming SBUS and PPM can use the same input, PC7, might not
 *        be possible
 * FIXME: Check the Channel is not conflicting with another peripheral
 */
#define HRT_PPM_CHANNEL		2	/* use capture/compare channel 2 */
#define GPIO_PPM_IN       (GPIO_ALT|GPIO_AF1|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

/*
 * Serial port for RC input: USART6 => /dev/ttyS5
 *
 * Setting this option disables CPPM input in the fmu module
 */
#define RC_SERIAL_PORT		"/dev/ttyS5"

/*
 * SBUS I/O enabling SBUS inversion, PC6
 *
 * Note: The schematic also shows PC0 as SBUS Invert. Must be a mistake since
 *       it's not wired
 */
#define GPIO_SBUS_INV (GPIO_ALT|GPIO_AF1|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN6)
#define INVERT_RC_INPUT(_s)		stm32_gpiowrite(GPIO_SBUS_INV, _s);


/*
 * PWM Input settings
 *
 * Currently disabled since it's not used and there's no dedicated pin
 */
/*
#define PWMIN_TIMER		2
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_2
*/
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
