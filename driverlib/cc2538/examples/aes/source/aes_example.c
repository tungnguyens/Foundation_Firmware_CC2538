/******************************************************************************
*  Filename:       aes_example.c
*  Revised:        $Date: 2013-05-02 14:32:18 +0200 (Thu, 02 May 2013) $
*  Revision:       $Revision: 9961 $
*
*  Description:    AES ECB access example for CC2538 on SmartRF06EB. 
*                  This example shows how AES ECB should be used. 
*                  The example also verifies the CCM functionality.
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

#include "aes.h"
#include <string.h>
#include "sys_ctrl.h"



//
// NOTES:
// This example has been run on a SmartRF06 evaluation board.
//
//*****************************************************************************
//
//! \addtogroup aes_examples_list
//! <h1>AES ECB (aes_example)</h1>
//!
//! This example shows how to configure the AES ECB with interrupt handler 
//! and polling.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you can add these interrupt handlers to your
//! vector table.
//! - AESIntHandler
//!
//! \note The example registers the handler in dynamic fashion. This consumes
//!       RAM space as the vector table is allocated and copied to RAM.
//
//*****************************************************************************

//*****************************************************************************
//
// Variables to syncronize with interrupt handler
//
//*****************************************************************************
uint8_t ui8AESECBIntHandler = 0;

//*****************************************************************************
//
// AES ECB Example structure
//
//*****************************************************************************
typedef struct
{
    uint8_t ui8AESKey[16];            // stores the Aes Key
    uint8_t ui8AESKeyLocation;        // location in Key RAM
    uint8_t ui8AESBuf[16];            // input buffer
    uint8_t ui8AESExpectedOutput[16]; // expected results
    uint8_t ui8IntEnable;             // set to true to enable interrupts
}tAESExample;

tAESExample sAESexample[] =
{
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        0,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        1,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        2,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        3,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        4,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        5,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        6,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        7,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        false
    },
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        2,
        { 0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f },
        { 0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1 },
        true
    }
};

//*****************************************************************************
//
// Compare buffers
//
// param   ui8Src1       pointer to input buffer
// param   ui8Src2       pointer to buffer to compare with
// param   ui32Len       length in bytes
//
// return  true if buffers match and false if they don't
//
//*****************************************************************************
bool AESMemCmp(const uint8_t *pui8Src1, const uint8_t *pui8Src2, 
               uint32_t ui32Len )
{
    const uint8_t *pui8Src1Temp;
    const uint8_t *pui8Src2Temp;
    
    pui8Src1Temp = pui8Src1;
    pui8Src2Temp = pui8Src2;
    
    while ( ui32Len-- )
    {
        if( *pui8Src1Temp++ != *pui8Src2Temp++ )
            return (false);
    }
    return (true);
}

//*****************************************************************************
//
// Interrupt handler for AES
//
// param   None
//
// return  None
//
//*****************************************************************************
void AESIntHandler(void)
{
    switch (g_ui8CurrentAESOp)
    {
    case AES_ECB:
        ui8AESECBIntHandler = 1;
        //
        // clear interrupts
        //
        HWREG(AES_CTRL_INT_CLR) = 0x00000003;
        break;
        
    case AES_NONE:
        break;
        
    case AES_CCM:
        break;
        
    case AES_SHA256:
        break;
        
    case AES_KEYL0AD:
        break;
    }
}

//*****************************************************************************
//
// AES ECB example
//
// param   pui8Key            Pointer to buffer containing the key
// param   ui8KeyLocation     location of Key in the Key RAM. Must be one of
//                            the following:
//                            KEY_AREA_0
//                            KEY_AREA_1
//                            KEY_AREA_2
//                            KEY_AREA_3
//                            KEY_AREA_4
//                            KEY_AREA_5
//                            KEY_AREA_6
//                            KEY_AREA_7
// param   pui8Buf            pointer to input Buffer
// param   pui8ExpectedOutput pointer to buffer with expected results
// param   ui8IntEnable       set to true to enable interrupts and false
//                            to disable
//
// return  AES_SUCCESS if successful
//
//*****************************************************************************
uint8_t AesEcbExample(uint8_t *pui8Key,
                      uint8_t ui8KeyLocation,
                      uint8_t *pui8Buf,
                      uint8_t *pui8ExpectedOutput,
                      uint8_t ui8IntEnable)
{
    if(ui8IntEnable)
    {
        //
        // example using Interrupt service routine
        //
        AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation);
        AESECBStart(pui8Buf, pui8Buf, ui8KeyLocation, true, true);
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(ui8AESECBIntHandler == 0);
        
        ui8AESECBIntHandler = 0;
        
        AESECBGetResult();
        
        // 
        // Verify AES ECB output
        //
        if (AESMemCmp(pui8Buf, pui8ExpectedOutput, 16) == false)
        {
            return (AES_ECB_TEST_ERROR);
        }
    }
    else
    {
        //
        // example using polling
        //
        AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation);
        AESECBStart(pui8Buf, pui8Buf, ui8KeyLocation, true, false);
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(!(AESECBCheckResult()));
        
        AESECBGetResult();
        
        //
        // Verify AES ECB output
        //
        if (AESMemCmp(pui8Buf, pui8ExpectedOutput, 16) == false)
        {
            return AES_ECB_TEST_ERROR;
        }
    }
    return (AES_SUCCESS);
}

//*****************************************************************************
//
// Main function of example.
//
//*****************************************************************************
void  main(void)
{
    uint8_t ui8Status;
    
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
    // Enable AES peripheral
    //
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_AES);
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_AES);
    
    //
    // Register interrupt handler
    //
    IntRegister(INT_AES, AESIntHandler);
    
    //
    // Enable global interrupts
    //
    IntAltMapEnable();
    IntMasterEnable();
    
    for(uint8_t i = 0; i < sizeof(sAESexample)/sizeof(sAESexample[0]); i++)
    {
        //
        // Run and Verify AES ECB operation
        //
        ui8Status = AesEcbExample(sAESexample[i].ui8AESKey,
                                  sAESexample[i].ui8AESKeyLocation,
                                  sAESexample[i].ui8AESBuf,
                                  sAESexample[i].ui8AESExpectedOutput,
                                  sAESexample[i].ui8IntEnable);
        if(ui8Status != AES_SUCCESS)
        {
            while(1)
            {
                //
                // AES ECB failed
                //
            }
        }
    }
    
    // ECB was successful 
    while(1)
    {
    }
}

