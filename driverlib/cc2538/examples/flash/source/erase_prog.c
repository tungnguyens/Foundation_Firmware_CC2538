/******************************************************************************
*  Filename:       erase_prog.c
*  Revised:        $Date: 2013-04-10 11:18:41 +0200 (on, 10 apr 2013) $
*  Revision:       $Revision: 9702 $
*
*  Description:    Example demonstrating how to erase and program the flash. 
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "sys_ctrl.h"
#include "flash.h"
#include "debug.h"

//
// NOTE:
// This example has been run on a SmartRF06 evaluation board.  
//
//*****************************************************************************
//
//! \addtogroup flash_examples_list
//! <h1>Flash Erase and Program (erase_prog)</h1>
//!
//! This example shows how to erase a flash page and program it.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - None
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you can add these interrupt handlers to your
//! vector table.
//! - None
//!
//*****************************************************************************
#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))


//*****************************************************************************
//
// Configure the device, erase an page and the program the page.
//
//*****************************************************************************
int
main(void)
{
    int32_t i32Res;
    uint32_t i;
    char pcStrInRam[16] = "Hello ... world!";    

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    //
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
       
    //
    // Erase a page
    //
    i32Res = FlashMainPageErase(PAGE_TO_ERASE_START_ADDR);
    ASSERT(i32Res==0);
    
    //
    // Program a page in chunks
    //
    for(i=0; i<PAGE_SIZE; i+=sizeof(pcStrInRam))
    {
        i32Res = FlashMainPageProgram((uint32_t*) pcStrInRam, 
                                      PAGE_TO_ERASE_START_ADDR+i,
                                      sizeof(pcStrInRam));
        ASSERT(i32Res==0);
    }
    
    //
    // Loop forever
    //
    while(1)
    {
    }
}
