/******************************************************************************
*  Filename:       sha256_example.c
*  Revised:        $Date: 2013-05-02 14:31:55 +0200 (Thu, 02 May 2013) $
*  Revision:       $Revision: 9960 $
*
*  Description:    SHA 256 access example for CC2538 on SmartRF06EB. This 
*                  example shows how SHA256 should be used. The example also 
*                  verifies the SHA256 functionality.
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
#include "sys_ctrl.h"
#include "sha256.h"
#include <string.h>



// NOTE:
// This example has been run on a SmartRF06 evaluation board.
//
//*****************************************************************************
//
//! \addtogroup sha256_examples_list
//! <h1>SHA256 (sha256_example)</h1>
//!
//! This example shows how to configure the SHA256. 
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//
//*****************************************************************************
typedef struct
{
    uint8_t *pui8Msg[3];    // input message
    uint8_t ui8Hash[32];    // Hash
    uint8_t ui8NumBuffer;   // number of buffers
} tSHA256Example;

tSHA256Example sSHA256Example[]=
{
    // Example case:0 Simple
    { 
        "abc",
        NULL,
        NULL,
        { 
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
            0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
            0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
            0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
        },
        1 
    },
    // Example case:1 Simple
    { 
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
        NULL,
        NULL,
        { 
            0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8,
            0xe5, 0xc0, 0x26, 0x93, 0x0c, 0x3e, 0x60, 0x39,
            0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff, 0x21, 0x67,
            0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1
        },
        1 
    },
    // Example case:2 with message of length 1026
    {
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklabcd"
        "efghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmn",
        NULL,
        NULL,
        {
            0x15, 0xd2, 0x3e, 0xea, 0x57, 0xb3, 0xd4, 0x61,
            0xbf, 0x38, 0x91, 0x12, 0xab, 0x4c, 0x43, 0xce,
            0x85, 0xe1, 0x68, 0x23, 0x8a, 0xaa, 0x54, 0x8e,
            0xc8, 0x6f, 0x0c, 0x9d, 0x65, 0xf9, 0xb9, 0x23
        },
        1 
    },
    // Example case:3 with message of length 1024
    {
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklabcd"
        "efghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl",
        NULL,
        NULL,
        {
            0xf8, 0xa3, 0xf2, 0x26, 0xfc, 0x42, 0x10, 0xe9,
            0x0d, 0x13, 0x0c, 0x7f, 0x41, 0xf2, 0xbe, 0x66,
            0x45, 0x53, 0x85, 0xd2, 0x92, 0x0a, 0xda, 0x78,
            0x15, 0xf8, 0xf7, 0x95, 0xd9, 0x44, 0x90, 0x5f
        },
        1 
    },
    // Example case:4 with message of length 512 bytes
    { 
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl",
        NULL,
        NULL,
        {
            0x2f, 0xcd, 0x5a, 0x0d, 0x60, 0xe4, 0xc9, 0x41,
            0x38, 0x1f, 0xcc, 0x4e, 0x00, 0xa4, 0xbf, 0x8b,
            0xe4, 0x22, 0xc3, 0xdd, 0xfa, 0xfb, 0x93, 0xc8,
            0x09, 0xe8, 0xd1, 0xe2, 0xbf, 0xff, 0xae, 0x8e
        },
        1 
    },
    // Example case:5 with message of length 514 bytes
    { 
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmn",
        NULL,
        NULL,
        { 
            0x92, 0x90, 0x1c, 0x85, 0x82, 0xe3, 0x1c, 0x05,
            0x69, 0xb5, 0x36, 0x26, 0x9c, 0xe2, 0x2c, 0xc8,
            0x30, 0x8b, 0xa4, 0x17, 0xab, 0x36, 0xc1, 0xbb,
            0xaf, 0x08, 0x4f, 0xf5, 0x8b, 0x18, 0xdc, 0x6a
        },
        1 
    },
    // Example case:6
    { 
        "abcdbcdecdefde",
        "fgefghfghighijhijkijkljklmklmnlmnomnopnopq",
        NULL,
        { 
            0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8,
            0xe5, 0xc0, 0x26, 0x93, 0x0c, 0x3e, 0x60, 0x39,
            0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff, 0x21, 0x67,
            0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1
        },
        2 
    },
    { 
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl", 
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl",
        NULL,
        { 
            0xf8, 0xa3, 0xf2, 0x26, 0xfc, 0x42, 0x10, 0xe9,
            0x0d, 0x13, 0x0c, 0x7f, 0x41, 0xf2, 0xbe, 0x66,
            0x45, 0x53, 0x85, 0xd2, 0x92, 0x0a, 0xda, 0x78,
            0x15, 0xf8, 0xf7, 0x95, 0xd9, 0x44, 0x90, 0x5f
        },
        2
    },
    { 
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefgh",
        "ijkl",
        "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijkl",
        { 
            0xf8, 0xa3, 0xf2, 0x26, 0xfc, 0x42, 0x10, 0xe9,
            0x0d, 0x13, 0x0c, 0x7f, 0x41, 0xf2, 0xbe, 0x66,
            0x45, 0x53, 0x85, 0xd2, 0x92, 0x0a, 0xda, 0x78,
            0x15, 0xf8, 0xf7, 0x95, 0xd9, 0x44, 0x90, 0x5f
        },
        3
    }
};

//*****************************************************************************
//
// Compare buffers
//
// param   pui8Src1       pointer to input buffer
// param   pui8Src2       pointer to buffer to compare with
// param   ui32Len        length in bytes
//
// return  true if buffers match and false if they don't
//
//*****************************************************************************
bool SHA256MemCmp(  const uint8_t *pui8Src1, const uint8_t *pui8Src2, 
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
// sha256 example
//
// return  SHA256_SUCCESS if successful
//
//*****************************************************************************
uint8_t SHA256Examples(void)
{
    uint8_t ui8Tmp[32];
    uint8_t ui8I, ui8J;
    tSHA256State sMd;
    for (ui8I = 0; ui8I < (int)(sizeof(sSHA256Example) / 
                                sizeof(sSHA256Example[0])); ui8I++)
    {
        SHA256Init(&sMd);
        for (ui8J = 0; ui8J < sSHA256Example[ui8I].ui8NumBuffer ; ui8J++)
        {
            SHA256Process(&sMd, (uint8_t*)sSHA256Example[ui8I].pui8Msg[ui8J], 
                          (uint32_t)strlen((char const *)sSHA256Example[ui8I].
                          pui8Msg[ui8J]));
        }
        SHA256Done(&sMd, ui8Tmp);
        
        //
        // Verify SHA-256 Output
        //
        if (SHA256MemCmp(ui8Tmp, sSHA256Example[ui8I].ui8Hash, 32) == false)
        {
            return SHA256_TEST_ERROR;
        }
    }
    return (SHA256_SUCCESS);
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
    // Enable global interrupts
    //
    IntAltMapEnable();
    IntMasterEnable();
    
    //
    // Run and Verify SHA-256 operation.
    //
    ui8Status = SHA256Examples();
    
    
    if(ui8Status != SHA256_SUCCESS)
    {
        while(1)
        {
            //
            // SHA256 failed
            //
        }
    }
    
    // SHA256 was successful
    while(1)
    {
    }
}

