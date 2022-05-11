/*
 * Copyright (c) 2017-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ===========================================================================
 *  @file       CC1352R1_SMARTCOM.h
 *
 *  @brief      CC1352R1_SMARTCOM Board Specific header file.
 *
 *  The CC1352R1_SMARTCOM header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CC1352R1_SMARTCOM.h"
 *  @endcode
 *
 *  ===========================================================================
 */
#ifndef __CC1352R1_SMARTCOM_BOARD_H__
#define __CC1352R1_SMARTCOM_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki-conf.h"

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#define CC1352R1_LAUNCHXL

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>         <pin mapping>   <comments>
 */

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */
/* Analog Capable DIOs */
#define CC1352R1_SMARTCOM_DIO23_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO24_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO25_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO26_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO27_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO28_ANALOG          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO29_ANALOG          PIN_UNASSIGNED

/* Digital IOs */
#define CC1352R1_SMARTCOM_DIO12                 PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO15                 PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO16_TDO             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO17_TDI             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO21                 PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO22                 PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_DIO30                 PIN_UNASSIGNED

/* Discrete Inputs */
#define CC1352R1_SMARTCOM_PIN_BTN1              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PIN_BTN2              PIN_UNASSIGNED

/* GPIO */
#define CC1352R1_SMARTCOM_GPIO_LED_ON           1
#define CC1352R1_SMARTCOM_GPIO_LED_OFF          0

/* I2C */
#define CC1352R1_SMARTCOM_I2C0_SCL0             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_I2C0_SDA0             PIN_UNASSIGNED

/* I2S */
#define CC1352R1_SMARTCOM_I2S_ADO               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_I2S_ADI               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_I2S_BCLK              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_I2S_MCLK              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_I2S_WCLK              PIN_UNASSIGNED

/* LEDs */
#define CC1352R1_SMARTCOM_PIN_LED_ON            1
#define CC1352R1_SMARTCOM_PIN_LED_OFF           0
#define CC1352R1_SMARTCOM_PIN_RLED              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PIN_GLED              PIN_UNASSIGNED

/* PWM Outputs */
#define CC1352R1_SMARTCOM_PWMPIN0               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN1               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN2               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN3               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN4               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN5               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN6               PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_PWMPIN7               PIN_UNASSIGNED

/* SPI */
#define CC1352R1_SMARTCOM_SPI_FLASH_CS          PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_FLASH_CS_ON           0
#define CC1352R1_SMARTCOM_FLASH_CS_OFF          1

/* SPI Board */
#define CC1352R1_SMARTCOM_SPI0_MISO             PIN_UNASSIGNED         /* RF1.20 */
#define CC1352R1_SMARTCOM_SPI0_MOSI             PIN_UNASSIGNED         /* RF1.18 */
#define CC1352R1_SMARTCOM_SPI0_CLK              PIN_UNASSIGNED         /* RF1.16 */
#define CC1352R1_SMARTCOM_SPI0_CSN              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_SPI1_MISO             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_SPI1_MOSI             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_SPI1_CLK              PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_SPI1_CSN              PIN_UNASSIGNED

/* UART Board */
#define CC1352R1_SMARTCOM_UART0_RX              IOID_7                  /* RXD */
#define CC1352R1_SMARTCOM_UART0_TX              IOID_8                  /* TXD */
#define CC1352R1_SMARTCOM_UART0_CTS             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_UART0_RTS             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_UART1_RX              IOID_12
#define CC1352R1_SMARTCOM_UART1_TX              IOID_13
#define CC1352R1_SMARTCOM_UART1_CTS             PIN_UNASSIGNED
#define CC1352R1_SMARTCOM_UART1_RTS             PIN_UNASSIGNED
/* For backward compatibility */
#define CC1352R1_SMARTCOM_UART_RX               CC1352R1_SMARTCOM_UART0_RX
#define CC1352R1_SMARTCOM_UART_TX               CC1352R1_SMARTCOM_UART0_TX
#define CC1352R1_SMARTCOM_UART_CTS              CC1352R1_SMARTCOM_UART0_CTS
#define CC1352R1_SMARTCOM_UART_RTS              CC1352R1_SMARTCOM_UART0_RTS

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC1352R1_SMARTCOM_initGeneral(void);

/*!
 *  @brief  Shut down the external flash present on the board files
 *
 *  This function bitbangs the SPI sequence necessary to turn off
 *  the external flash on LaunchPads.
 */
void CC1352R1_SMARTCOM_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void CC1352R1_SMARTCOM_wakeUpExtFlash(void);

/*!
 *  @def    CC1352R1_SMARTCOM_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum CC1352R1_SMARTCOM_ADCBufName {
    CC1352R1_SMARTCOM_ADCBUF0 = 0,

    CC1352R1_SMARTCOM_ADCBUFCOUNT
} CC1352R1_SMARTCOM_ADCBufName;

/*!
 *  @def    CC1352R1_SMARTCOM_ADCBuf0ChannelName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC1352R1_SMARTCOM_ADCBuf0ChannelName {
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL0 = 0,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL1,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL2,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL3,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL4,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL5,
    CC1352R1_SMARTCOM_ADCBUF0CHANNEL6,
    CC1352R1_SMARTCOM_ADCBUF0CHANNELVDDS,
    CC1352R1_SMARTCOM_ADCBUF0CHANNELDCOUPL,
    CC1352R1_SMARTCOM_ADCBUF0CHANNELVSS,

    CC1352R1_SMARTCOM_ADCBUF0CHANNELCOUNT
} CC1352R1_SMARTCOM_ADCBuf0ChannelName;

/*!
 *  @def    CC1352R1_SMARTCOM_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC1352R1_SMARTCOM_ADCName {
    CC1352R1_SMARTCOM_ADC0 = 0,
    CC1352R1_SMARTCOM_ADC1,
    CC1352R1_SMARTCOM_ADC2,
    CC1352R1_SMARTCOM_ADC3,
    CC1352R1_SMARTCOM_ADC4,
    CC1352R1_SMARTCOM_ADC5,
    CC1352R1_SMARTCOM_ADC6,
    CC1352R1_SMARTCOM_ADCDCOUPL,
    CC1352R1_SMARTCOM_ADCVSS,
    CC1352R1_SMARTCOM_ADCVDDS,

    CC1352R1_SMARTCOM_ADCCOUNT
} CC1352R1_SMARTCOM_ADCName;

/*!
 *  @def    CC1352R1_SMARTCOM_ECDHName
 *  @brief  Enum of ECDH names
 */
typedef enum CC1352R1_SMARTCOM_ECDHName {
    CC1352R1_SMARTCOM_ECDH0 = 0,

    CC1352R1_SMARTCOM_ECDHCOUNT
} CC1352R1_SMARTCOM_ECDHName;

/*!
 *  @def    CC1352R1_SMARTCOM_ECDSAName
 *  @brief  Enum of ECDSA names
 */
typedef enum CC1352R1_SMARTCOM_ECDSAName {
    CC1352R1_SMARTCOM_ECDSA0 = 0,

    CC1352R1_SMARTCOM_ECDSACOUNT
} CC1352R1_SMARTCOM_ECDSAName;

/*!
 *  @def    CC1352R1_SMARTCOM_ECJPAKEName
 *  @brief  Enum of ECJPAKE names
 */
typedef enum CC1352R1_SMARTCOM_ECJPAKEName {
    CC1352R1_SMARTCOM_ECJPAKE0 = 0,

    CC1352R1_SMARTCOM_ECJPAKECOUNT
} CC1352R1_SMARTCOM_ECJPAKEName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC1352R1_SMARTCOM_AESCCMName {
    CC1352R1_SMARTCOM_AESCCM0 = 0,

    CC1352R1_SMARTCOM_AESCCMCOUNT
} CC1352R1_SMARTCOM_AESCCMName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC1352R1_SMARTCOM_AESGCMName {
    CC1352R1_SMARTCOM_AESGCM0 = 0,

    CC1352R1_SMARTCOM_AESGCMCOUNT
} CC1352R1_SMARTCOM_AESGCMName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC1352R1_SMARTCOM_AESCBCName {
    CC1352R1_SMARTCOM_AESCBC0 = 0,

    CC1352R1_SMARTCOM_AESCBCCOUNT
} CC1352R1_SMARTCOM_AESCBCName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC1352R1_SMARTCOM_AESCTRName {
    CC1352R1_SMARTCOM_AESCTR0 = 0,

    CC1352R1_SMARTCOM_AESCTRCOUNT
} CC1352R1_SMARTCOM_AESCTRName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC1352R1_SMARTCOM_AESECBName {
    CC1352R1_SMARTCOM_AESECB0 = 0,

    CC1352R1_SMARTCOM_AESECBCOUNT
} CC1352R1_SMARTCOM_AESECBName;

/*!
 *  @def    CC1352R1_SMARTCOM_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC1352R1_SMARTCOM_AESCTRDRBGName {
    CC1352R1_SMARTCOM_AESCTRDRBG0 = 0,

    CC1352R1_SMARTCOM_AESCTRDRBGCOUNT
} CC1352R1_SMARTCOM_AESCTRDRBGName;

/*!
 *  @def    CC1352R1_SMARTCOM_SHA2Name
 *  @brief  Enum of SHA2 names
 */
typedef enum CC1352R1_SMARTCOM_SHA2Name {
    CC1352R1_SMARTCOM_SHA20 = 0,

    CC1352R1_SMARTCOM_SHA2COUNT
} CC1352R1_SMARTCOM_SHA2Name;

/*!
 *  @def    CC1352R1_SMARTCOM_TRNGName
 *  @brief  Enum of TRNG names
 */
typedef enum CC1352R1_SMARTCOM_TRNGName {
    CC1352R1_SMARTCOM_TRNG0 = 0,

    CC1352R1_SMARTCOM_TRNGCOUNT
} CC1352R1_SMARTCOM_TRNGName;

/*!
 *  @def    CC1352R1_SMARTCOM_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC1352R1_SMARTCOM_GPIOName {
    CC1352R1_SMARTCOM_GPIO_S1 = 0,
    CC1352R1_SMARTCOM_GPIO_S2,
    CC1352R1_SMARTCOM_SPI_MASTER_READY,
    CC1352R1_SMARTCOM_SPI_SLAVE_READY,
    CC1352R1_SMARTCOM_GPIO_SPI_FLASH_CS,
    CC1352R1_SMARTCOM_SDSPI_CS,
    CC1352R1_SMARTCOM_GPIOCOUNT
} CC1352R1_SMARTCOM_GPIOName;

/*!
 *  @def    CC1352R1_SMARTCOM_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC1352R1_SMARTCOM_GPTimerName {
    CC1352R1_SMARTCOM_GPTIMER0A = 0,
    CC1352R1_SMARTCOM_GPTIMER0B,
    CC1352R1_SMARTCOM_GPTIMER1A,
    CC1352R1_SMARTCOM_GPTIMER1B,
    CC1352R1_SMARTCOM_GPTIMER2A,
    CC1352R1_SMARTCOM_GPTIMER2B,
    CC1352R1_SMARTCOM_GPTIMER3A,
    CC1352R1_SMARTCOM_GPTIMER3B,

    CC1352R1_SMARTCOM_GPTIMERPARTSCOUNT
} CC1352R1_SMARTCOM_GPTimerName;

/*!
 *  @def    CC1352R1_SMARTCOM_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC1352R1_SMARTCOM_GPTimers {
    CC1352R1_SMARTCOM_GPTIMER0 = 0,
    CC1352R1_SMARTCOM_GPTIMER1,
    CC1352R1_SMARTCOM_GPTIMER2,
    CC1352R1_SMARTCOM_GPTIMER3,

    CC1352R1_SMARTCOM_GPTIMERCOUNT
} CC1352R1_SMARTCOM_GPTimers;

/*!
 *  @def    CC1352R1_SMARTCOM_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC1352R1_SMARTCOM_I2CName {
#if TI_I2C_CONF_I2C0_ENABLE
    CC1352R1_SMARTCOM_I2C0 = 0,
#endif

    CC1352R1_SMARTCOM_I2CCOUNT
} CC1352R1_SMARTCOM_I2CName;

/*!
 *  @def    CC1352R1_SMARTCOM_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum CC1352R1_SMARTCOM_I2SName {
    CC1352R1_SMARTCOM_I2S0 = 0,

    CC1352R1_SMARTCOM_I2SCOUNT
} CC1352R1_SMARTCOM_I2SName;

/*!
 *  @def    CC1352R1_SMARTCOM_PDMName
 *  @brief  Enum of I2S names
 */
typedef enum CC1352R1_SMARTCOM_PDMCOUNT {
    CC1352R1_SMARTCOM_PDM0 = 0,

    CC1352R1_SMARTCOM_PDMCOUNT
} CC1352R1_SMARTCOM_PDMName;

/*!
 *  @def    CC1352R1_SMARTCOM_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC1352R1_SMARTCOM_NVSName {
#if TI_NVS_CONF_NVS_INTERNAL_ENABLE
    CC1352R1_SMARTCOM_NVSCC26XX0 = 0,
#endif
#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE
    CC1352R1_SMARTCOM_NVSSPI25X0,
#endif

    CC1352R1_SMARTCOM_NVSCOUNT
} CC1352R1_SMARTCOM_NVSName;

/*!
 *  @def    CC1352R1_SMARTCOM_PWMName
 *  @brief  Enum of PWM outputs
 */
typedef enum CC1352R1_SMARTCOM_PWMName {
    CC1352R1_SMARTCOM_PWM0 = 0,
    CC1352R1_SMARTCOM_PWM1,
    CC1352R1_SMARTCOM_PWM2,
    CC1352R1_SMARTCOM_PWM3,
    CC1352R1_SMARTCOM_PWM4,
    CC1352R1_SMARTCOM_PWM5,
    CC1352R1_SMARTCOM_PWM6,
    CC1352R1_SMARTCOM_PWM7,

    CC1352R1_SMARTCOM_PWMCOUNT
} CC1352R1_SMARTCOM_PWMName;

/*!
 *  @def    CC1352R1_SMARTCOM_SDName
 *  @brief  Enum of SD names
 */
typedef enum CC1352R1_SMARTCOM_SDName {
    CC1352R1_SMARTCOM_SDSPI0 = 0,

    CC1352R1_SMARTCOM_SDCOUNT
} CC1352R1_SMARTCOM_SDName;

/*!
 *  @def    CC1352R1_SMARTCOM_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC1352R1_SMARTCOM_SPIName {
#if TI_SPI_CONF_SPI0_ENABLE
    CC1352R1_SMARTCOM_SPI0 = 0,
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    CC1352R1_SMARTCOM_SPI1,
#endif

    CC1352R1_SMARTCOM_SPICOUNT
} CC1352R1_SMARTCOM_SPIName;

/*!
 *  @def    CC1352R1_SMARTCOM_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC1352R1_SMARTCOM_UARTName {
#if TI_UART_CONF_UART0_ENABLE
    CC1352R1_SMARTCOM_UART0 = 0,
#endif
#if TI_UART_CONF_UART1_ENABLE
    CC1352R1_SMARTCOM_UART1,
#endif

    CC1352R1_SMARTCOM_UARTCOUNT
} CC1352R1_SMARTCOM_UARTName;

/*!
 *  @def    CC1352R1_SMARTCOM_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1352R1_SMARTCOM_UDMAName {
    CC1352R1_SMARTCOM_UDMA0 = 0,

    CC1352R1_SMARTCOM_UDMACOUNT
} CC1352R1_SMARTCOM_UDMAName;

/*!
 *  @def    CC1352R1_SMARTCOM_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC1352R1_SMARTCOM_WatchdogName {
    CC1352R1_SMARTCOM_WATCHDOG0 = 0,

    CC1352R1_SMARTCOM_WATCHDOGCOUNT
} CC1352R1_SMARTCOM_WatchdogName;


#ifdef __cplusplus
}
#endif

#endif /* __CC1352R1_SMARTCOM_BOARD_H__ */
