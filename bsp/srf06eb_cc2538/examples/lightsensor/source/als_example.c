//*****************************************************************************
//! @file       als_example.c
//! @brief      Example of board support package for the ambient light sensor.
//!
//! Revised     $Date: 2013-04-11 20:13:13 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9714 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "bsp_led.h"
#include "als_sfh5711.h"


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTION PROTOTYPES
*/


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    volatile uint32_t ui32Loop;
    uint16_t ui16AlsValue = 0;


    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize light sensor
    //
    alsInit();

    //
    // Infinite loop
    //
    while(1)
    {
        //
        // Read data
        //
        ui16AlsValue = alsRead();

        //
        // Indicate value using LEDs
        //
        bspLedClear(BSP_LED_ALL);
        if(ui16AlsValue > 300)
        {
            bspLedSet(BSP_LED_1);
        }
        if(ui16AlsValue > 550)
        {
            bspLedSet(BSP_LED_2);
        }
        if(ui16AlsValue > 800)
        {
            bspLedSet(BSP_LED_3);
        }
        if(ui16AlsValue > 1050)
        {
            bspLedSet(BSP_LED_4);
        }

        //
        // Simple wait
        //
        for(ui32Loop = 0; ui32Loop < 100000; ui32Loop++)
        {
        }
    }
}
