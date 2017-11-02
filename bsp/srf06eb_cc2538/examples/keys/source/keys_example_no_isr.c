//*****************************************************************************
//! @file       keys_example_no_isr.c
//! @brief      Example of board support package key functionality.
//!             This example shows how the bsp key package should be used.
//!             In this example, the keys are handled using polling and active
//!             wait software debounce instead of interrupt and watchdog timer
//!             controller debounce.
//!
//!             When bspKeyInit() is called with argument BSP_KEY_MODE_POLL,
//!             the \b bspKey functions do not need interrupts to work. The
//!             drawback is that the CPU is kept busy during key debounce.
//!
//!             See example \e keys_example for using the bsp key functionality
//!             with interrupt debounce.
//!
//!             See bsp_key.c for more information about the BSP key package.
//!
//! Revised     $Date: 2013-03-14 11:55:47 +0100 (on, 14 mar 2013) $
//! Revision    $Revision: 7035 $
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
#include "bsp_key.h"
#include "bsp_led.h"


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
    volatile uint8_t ui8KeysPressed;

    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize key handler to use polling
    //
    bspKeyInit(BSP_KEY_MODE_POLL);

    //
    // Infinite loop
    //
    while(1)
    {
        //
        // Read out bitmask (argument ignored when key handler uses polling)
        //
        ui8KeysPressed = bspKeyPushed(BSP_KEY_ALL);

        //
        // Toggle LEDs based on key presses
        //
        if(ui8KeysPressed & BSP_KEY_LEFT)
        {
            bspLedToggle(BSP_LED_1);
        }
        if(ui8KeysPressed & BSP_KEY_RIGHT)
        {
            bspLedToggle(BSP_LED_2);
        }
        if(ui8KeysPressed & BSP_KEY_UP)
        {
            bspLedToggle(BSP_LED_3);
        }
        if(ui8KeysPressed & BSP_KEY_DOWN)
        {
            bspLedToggle(BSP_LED_4);
        }
        if(ui8KeysPressed & BSP_KEY_SELECT)
        {
            bspLedToggle(BSP_LED_ALL);
        }
    }
}
