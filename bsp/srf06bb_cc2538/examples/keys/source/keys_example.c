//*****************************************************************************
//! @file       keys_example.c
//! @brief      Example of board support package key functionality.
//!             This example shows how the bsp key package should be used.
//!             Interrupt service routines are assigned to the desired keys.
//!             When pressing the different keys, one or more LEDs are toggled.
//!
//!             See example \e keys_example_no_isr for using the bsp key
//!             functionality without interrupts.
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
#include "interrupt.h"          // Access to IntMasterEnable function


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTION PROTOTYPES
*/
//
// ISR prototypes
//
static void dirKeyIsr(void);
static void selectKeyIsr(void);


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    Main function of example.
******************************************************************************/
void main(void)
{
    //
    // Initialize board
    //
    bspInit(BSP_SYS_CLK_SPD);

    //
    // Initialize key handler to use interrupts
    //
    bspKeyInit(BSP_KEY_MODE_ISR);

    //
    // Map function dirKeyIsr to UP, LEFT, RIGHT and DOWN keys
    //
    bspKeyIntRegister((BSP_KEY_UP|BSP_KEY_LEFT|BSP_KEY_RIGHT|BSP_KEY_DOWN),
                      &dirKeyIsr);

    //
    // Map function selectKeyIsr to SELECT key
    //
    bspKeyIntRegister(BSP_KEY_SELECT, &selectKeyIsr);

    //
    // Enable interrupts on all keys
    //
    bspKeyIntEnable(BSP_KEY_ALL);

    //
    // Enable global interrupts
    //
    IntMasterEnable();

    //
    // Infinite loop
    //
    while(1)
    {
        //
        // Do nothing.
        // Actions on keypresses are handled in interrupts.
        //
    }
}


/******************************************************************************
* LOCAL FUNCTIONS
*/

/**************************************************************************//**
* @brief    Interrupt service routine. This function will be called when an
*           interrupt on key LEFT, RIGHT, UP or DOWN is triggered.
*           The key pushed bitmask is cleared by this function. Interrupt flags
*           are handled by the BSP key package.
******************************************************************************/
static void dirKeyIsr(void)
{
    uint8_t ui8KeysPressed;
    uint_fast8_t ui8LedBm = 0;

    //
    // Get bitmask of buttons pushed (clear directional keys' bitmask)
    //
    ui8KeysPressed = bspKeyPushed(BSP_KEY_DIR_ALL);

    //
    // Determine which LEDs to toggle
    //
    if(ui8KeysPressed & BSP_KEY_LEFT)
    {
        ui8LedBm |= BSP_LED_1;
    }
    if(ui8KeysPressed & BSP_KEY_RIGHT)
    {
        ui8LedBm |= BSP_LED_2;
    }
    if(ui8KeysPressed & BSP_KEY_UP)
    {
        ui8LedBm |= BSP_LED_3;
    }
    if(ui8KeysPressed & BSP_KEY_DOWN)
    {
        ui8LedBm |= BSP_LED_4;
    }

    //
    // Toggle LED(s)
    //
    bspLedToggle(ui8LedBm);
}


/**************************************************************************//**
* @brief    Interrupt service routine. This function will be called when an
*           interrupt on the SELECT key is triggered.
*           The key pushed bitmask is cleared by this function. Interrupt flags
*           are handled by the BSP key package.
******************************************************************************/
static void selectKeyIsr(void)
{
    //
    // Call bspKeyPushed() to clear BSP_KEY_SELECT's bitmask
    //
    bspKeyPushed(BSP_KEY_SELECT);

    //
    // Toggle all LEDs
    //
    bspLedToggle(BSP_LED_ALL);
}
