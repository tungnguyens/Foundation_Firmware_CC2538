//*****************************************************************************
//! @file       bsp.h
//! @brief      Board support package header file for CC2538 on SmartRF06BB.
//!
//! Revised     $Date: 2013-04-11 20:13:31 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9715 $
//
//  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef __BSP_H__
#define __BSP_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include "hw_types.h"
#include "hw_memmap.h"


/******************************************************************************
* DEFINES
*/
// Clock speed defines
//! Default system clock speed
#define BSP_SYS_CLK_SPD         32000000UL
//! Default SPI clock speed. 8 MHz is supported by all SmartRF06EB peripherals.
#define BSP_SPI_CLK_SPD         8000000UL

// SPI defines (Common for LCD, SD reader and accelerometer)
#define BSP_SPI_SSI_BASE        SSI0_BASE
//! Bitmask to enable SSI module.
#define BSP_SPI_SSI_ENABLE_BM   SYS_CTRL_PERIPH_SSI0
#define BSP_SPI_BUS_BASE        GPIO_A_BASE
#define BSP_SPI_SCK             GPIO_PIN_2      //!< PA2
#define BSP_SPI_MOSI            GPIO_PIN_4      //!< PA4
#define BSP_SPI_MISO            GPIO_PIN_5      //!< PA5

// Board LED defines
#define BSP_LED_BASE            GPIO_C_BASE
#define BSP_LED_1               GPIO_PIN_0      //!< PC0
#define BSP_LED_2               GPIO_PIN_1      //!< PC1
#define BSP_LED_3               GPIO_PIN_2      //!< PC2
#define BSP_LED_4               GPIO_PIN_3      //!< PC3
#define BSP_LED_ALL             (BSP_LED_1 | \
                                 BSP_LED_2 | \
                                 BSP_LED_3 | \
                                 BSP_LED_4)     //!< Bitmask of all LEDs

// Board key defines
#define BSP_KEY_DIR_BASE        GPIO_C_BASE     //!< Base for left/right/up/down
#define BSP_KEY_SEL_BASE        GPIO_A_BASE     //!< Base for Select
#define BSP_KEY_1               GPIO_PIN_4      //!< PC4
#define BSP_KEY_2               GPIO_PIN_5      //!< PC5
#define BSP_KEY_3               GPIO_PIN_6      //!< PC6
#define BSP_KEY_4               GPIO_PIN_7      //!< PC7
#define BSP_KEY_5               GPIO_PIN_3      //!< PA3
#define BSP_KEY_ALL             (BSP_KEY_1| \
                                 BSP_KEY_2| \
                                 BSP_KEY_3| \
                                 BSP_KEY_4| \
                                 BSP_KEY_5)     //!< Bitmask of all keys
#define BSP_KEY_LEFT            BSP_KEY_1
#define BSP_KEY_RIGHT           BSP_KEY_2
#define BSP_KEY_UP              BSP_KEY_3
#define BSP_KEY_DOWN            BSP_KEY_4
#define BSP_KEY_SELECT          BSP_KEY_5
#define BSP_KEY_DIR_ALL         (BSP_KEY_LEFT|  \
                                 BSP_KEY_RIGHT| \
                                 BSP_KEY_UP|    \
                                 BSP_KEY_DOWN)  //!< Bitmask of all dir. keys


/******************************************************************************
* FUNCTION PROTOTYPES
*/
void bspInit(uint32_t ui32SysClockSpeed);
void bspSpiInit(uint32_t ui32ClockSpeed);
uint32_t bspSpiClockSpeedGet(void);
void bspSpiClockSpeedSet(uint32_t ui32ClockSpeed);
void bspAssert(void);


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_H__ */
