/******************************************************************************
*  Filename:       aes_ccm_example.c
*  Revised:        $Date: 2013-05-02 14:32:32 +0200 (Thu, 02 May 2013) $
*  Revision:       $Revision: 9962 $
*
*  Description:    AES CCM access example for CC2538 on SmartRF06EB. This 
*                  example shows how CCM should be used. The example also 
*                  verifies the CCM functionality.
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
#include "ccm.h"
#include <string.h>
#include "sys_ctrl.h"

//
// NOTES:
// This example has been run on a SmartRF06 evaluation board.
//
//*****************************************************************************
//
//! \addtogroup aes_examples_list
//! <h1>AES CCM (aes_ccm_example)</h1>
//!
//! This example shows how to configure the AES CCM with interrupt handler 
//! and polling.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - NONE
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you can add these interrupt handlers to your
//! vector table.
//! - CCMIntHandler
//!
//! \note The example registers the handler in dynamic fashion.  This consumes
//!       RAM space as the vector table is allocated and copied to RAM.
//
//*****************************************************************************

//
// ccm encrypt example structure
//
typedef struct
{
    uint8_t ui8CCMEncrypt;
    uint8_t ui8CCMkey[16];             // array to hold the Key
    uint8_t ui8CCMMval;                // length of authentication
    uint8_t ui8CCMN[13];               // Nonce
    uint8_t ui8CCMA[15];               // Additional data
    uint8_t ui8CCMM[20];               // input message
    uint16_t ui16CCMLenM;              // length of message
    uint16_t ui16CCMLenA;              // length of additional data
    uint8_t ui8CCMKeyLocation;         // location in Key RAM
    uint8_t ui8CCMCstate[8];           // authentication Tag
    uint8_t ui8CCMCCMLVal;             //  Lval for ccm
    uint8_t ui8CCMIntEnable;           // set to true to enable interrupts
    // and false to disable
    uint8_t ui8CCMExpectedOutput[24];  // reference output
} tCCMEncryptExample;

//
// ccm decrypt example structure
//
typedef struct
{
    uint8_t ui8CCMDecrypt;
    uint8_t ui8CCMKey[16];             // array to hold the Key
    uint8_t ui8CCMMval;                // length of authentication
    uint8_t ui8CCMN[13];               // Nonce
    uint8_t ui8CCMA[15];               // Additional data
    uint8_t ui8CCMC[24];               // input encrypted message
    uint16_t ui16CCMLenC;              // length of encrypted message
    uint16_t ui16CCMLenA;              // length of additional data
    uint8_t ui8CCMKeyLocation;         // location in Key RAM
    uint8_t ui8CCMCstate[8];           // authentication Tag
    uint8_t ui8CCMLVal;                //  Lval for ccm
    uint8_t ui8CCMIntEnable;           // set to true to enable interrupts
    // and false to disable
    uint8_t ui8CCMExpectedOutput[20];  // reference output
} tCCMDecryptExample;

//
// ccm No encryption authentication only example structure
//
typedef struct
{
    uint8_t ui8CCMEncrypt;
    uint8_t ui8CCMkey[16];            // array to hold the Key
    uint8_t ui8CCMMval;               // length of authentication
    uint8_t ui8CCMN[13];              // Nonce
    uint8_t ui8CCMA[26];              // Additional data
    uint8_t ui8CCMM[8];               // input message
    uint16_t ui16CCMLenM;             // length of message
    uint16_t ui16CCMLenA;             // length of additional data
    uint8_t ui8CCMKeyLocation;        // location in Key RAM
    uint8_t ui8CCMCstate[8];          // authentication Tag
    uint8_t ui8CCMCCMLVal;            //  Lval for ccm
    uint8_t ui8CCMIntEnable;          // set to true to enable interrupts
    // and false to disable
    uint8_t ui8CCMExpectedOutput[8];  // reference output
} tCCMNoencryptExample;

//
// ccm No decryption authentication only example structure
//
typedef struct
{
    uint8_t ui8CCMDecrypt;
    uint8_t ui8CCMKey[16];             // array to hold the Key
    uint8_t ui8CCMMval;                // length of authentication
    uint8_t ui8CCMN[13];               // Nonce
    uint8_t ui8CCMA[26];               // Additional data
    uint8_t ui8CCMC[8];                // input encrypted message
    uint16_t ui16CCMLenC;              // length of encrypted message
    uint16_t ui16CCMLenA;              // length of additional data
    uint8_t ui8CCMKeyLocation;         // location in Key RAM
    uint8_t ui8CCMCstate[8];           // authentication Tag
    uint8_t ui8CCMLVal;                // Lval for ccm
    uint8_t ui8CCMIntEnable;           // set to true to enable interrupts
    // and false to disable
    uint8_t ui8CCMExpectedOutput[20];  // reference output
} tCCMNodecryptExample;


tCCMDecryptExample sCCMDecryptExample[] =
{
    { // example case len_a and Mval = 0
        // decrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x0,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // C encrypted message
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6, 0x2c,
        0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09},
        // len_c
        20,
        // len_a
        0x0,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f }
    },
    {
        // decrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x4,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // C encrypted message
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6, 0x2c,
        0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09},
        // len_c
        24,
        // len_a
        0xF,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f }
    },
    {
        // decrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x4,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // C encrypted message
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6, 0x2c,
        0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09},
        // len_c
        24,
        // len_a
        0xF,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        true,
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f }
    },
};

tCCMNodecryptExample sCCMNodecryptExample[] =
{
    { // example case No decrypt
        // encrypt
        false,
        // key
        {0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF},
        // Mval
        0x8,
        // N Nonce
        {0xAC, 0xDE, 0x48, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x05, 0x02},
        // Header
        { 0x08, 0xD0, 0x84, 0x21, 0x43, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x48, 0xDE, 0xAC, 0x02, 0x05, 0x00,
        0x00, 0x00, 0x55, 0xCF, 0x00, 0x00, 0x51, 0x52,
        0x53, 0x54},
        // Payload
        {0x22, 0x3B, 0xC1, 0xEC, 0x84, 0x1A, 0xB5, 0x53},
        // len_c
        8,
        // len_a
        26,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        {0x22, 0x3B, 0xC1, 0xEC, 0x84, 0x1A, 0xB5, 0x53 }
    }
};

tCCMNoencryptExample sCCMNoencryptExample[] =
{
    { // example case No encrypt
        // encrypt
        false,
        // key
        {0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF},
        // Mval
        0x8,
        // N Nonce
        {0xAC, 0xDE, 0x48, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x05, 0x02},
        // Header
        { 0x08, 0xD0, 0x84, 0x21, 0x43, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x48, 0xDE, 0xAC, 0x02, 0x05, 0x00,
        0x00, 0x00, 0x55, 0xCF, 0x00, 0x00, 0x51, 0x52,
        0x53, 0x54},
        // Payload
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // len_m
        0,
        // len_a
        26,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        { 0x22, 0x3B, 0xC1, 0xEC, 0x84, 0x1A, 0xB5, 0x53}
    }
};

tCCMEncryptExample sCCMEncryptExample[] =
{
    { // example case len_a and Mval = 0
        // encrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x0,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // M message
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f },
        // len_m
        0x14,
        // len_a
        0x0,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6, 0x2c,
        0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09}
    },
    {
        // encrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x4,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // M message
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f },
        // len_m
        0x14,
        // len_a
        0xF,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        false,
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6, 0x2c,
        0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09}
    },
    {
        // encrypt
        true,
        // key
        { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // Mval
        0x4,
        // N Nonce
        { 0x00, 0x00, 0xf0, 0xe0, 0xd0, 0xc0, 0xb0, 0xa0,
        0x00, 0x00, 0x00, 0x00, 0x05 },
        // A
        { 0x69, 0x98, 0x03, 0x33, 0x63, 0xbb, 0xaa, 0x01,
        0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x03},
        // M message
        { 0x14, 0xaa, 0xbb, 0x00, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
        0x0c, 0x0d, 0x0e, 0x0f },
        // len_m
        0x14,
        // len_a
        0xF,
        // key location
        0,
        // mic
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // Lval
        2,
        true,
        { 0x92, 0xe8, 0xad, 0xca, 0x53, 0x81, 0xbf, 0xd0,
        0x5b, 0xdd, 0xf3, 0x61, 0x09, 0x09, 0x82, 0xe6,
        0x2c, 0x61, 0x01, 0x4e, 0x7b, 0x34, 0x4f, 0x09}
    }
};

//*****************************************************************************
//
// Variables to syncronize with interrupt handler
//
//*****************************************************************************
static uint8_t volatile ui8AESECBIntHandler = 0;
static uint8_t volatile ui8CCMIntHandler = 0;

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
bool CCMMemCmp(const uint8_t *pui8Src1, const uint8_t *pui8Src2, 
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
// Interrupt handler for CCM
//
// param   None
//
// return  None
//
//*****************************************************************************
void CCMIntHandler(void)
{
    switch (g_ui8CurrentAESOp)
    {
    case AES_ECB:
        ui8AESECBIntHandler = 1;
        // clear interrupts
        HWREG(AES_CTRL_INT_CLR) = 0x00000003;
        break;
        
    case AES_NONE:
        break;
        
    case AES_CCM:
        ui8CCMIntHandler = 1;
        // clear interrupts
        HWREG(AES_CTRL_INT_CLR) = 0x00000003;
        break;
        
    case AES_SHA256:
        break;
        
    case AES_KEYL0AD:
        break;
    }
}

//*****************************************************************************
//
// AES-CCM encrypt example
//
// param   bEncrypt           if set to true run encryption  
// param   pui8Key            pointer to Key
// param   ui8Mval            length of authentication
// param   pui8N              Nonce
// param   pui8M              input message
// param   ui16LenM           length of message
// param   pui8A              Additional data
// param   ui16LenA           length of additional data
// param   ui8KeyLocation     location in Key RAM. Must be one of
//                            the following:
//                            KEY_AREA_0
//                            KEY_AREA_1
//                            KEY_AREA_2
//                            KEY_AREA_3
//                            KEY_AREA_4
//                            KEY_AREA_5
//                            KEY_AREA_6
//                            KEY_AREA_7
// param   ui8CCMLVal         Lval for ccm
// param   ui8CCMIntEnable    set to true to enable interrupts and false to disable
// param   pui8ExpectedOutput pointer to expected output
// param   pui8Cstate         authentication tag
// param   ui8CCMIntEnable    set true/false to enable/disable interrupts
// param   pui8ExpectedOutput pointer to Expected Output
//
// return  AES_SUCCESS if successful
//
//*****************************************************************************
uint8_t CCMEncryptExample(bool bEncrypt,
                          uint8_t* pui8Key,
                          uint8_t ui8Mval,
                          uint8_t *pui8N,
                          uint8_t *pui8M,
                          uint16_t ui16LenM,
                          uint8_t *pui8A,
                          uint16_t ui16LenA,
                          uint8_t ui8KeyLocation,
                          uint8_t *pui8Cstate,
                          uint8_t ui8CCMLVal,
                          uint8_t ui8CCMIntEnable,
                          uint8_t *pui8ExpectedOutput)
{
    uint8_t status;
    if(ui8CCMIntEnable)
    {
        //
        // Register AES interrupt
        //
        IntRegister(INT_AES, CCMIntHandler);
        
        //
        // example using interrupt service routine
        //
        if((status = AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation)) != 
           AES_SUCCESS)
        {
            return status;
        }
        
        if((status = CCMAuthEncryptStart (bEncrypt, ui8Mval, pui8N, pui8M, 
                                          ui16LenM, pui8A, ui16LenA,
                                          ui8KeyLocation, pui8Cstate, 
                                          ui8CCMLVal, ui8CCMIntEnable)) 
           != AES_SUCCESS)
        {
            return status;
        }
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(ui8CCMIntHandler == 0 ); 
        
        ui8CCMIntHandler = 0;
        
        if((status = CCMAuthEncryptGetResult(ui8Mval, ui16LenM, pui8Cstate)) 
           != AES_SUCCESS)
        {
            return status;
        }
    }
    else
    {
        //
        // Unregister AES interrupt
        //
        IntUnregister(INT_AES);
        
        // 
        // example using polling
        //
        if((status = AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation)) 
           != AES_SUCCESS)
        {
            return status;
        }
        if((status = CCMAuthEncryptStart(bEncrypt, ui8Mval, pui8N, pui8M, 
                                          ui16LenM, pui8A, ui16LenA,
                                          ui8KeyLocation, pui8Cstate, 
                                          ui8CCMLVal, ui8CCMIntEnable)) 
           != AES_SUCCESS)
        {
            return status;
        }
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(!(CCMAuthEncryptCheckResult()));
        
        
        if((status = CCMAuthEncryptGetResult(ui8Mval, ui16LenM, pui8Cstate)) != 
           AES_SUCCESS)
        {
            return status;
        }
    }
    
    if (CCMMemCmp(pui8M,  (uint8_t const *)pui8ExpectedOutput, ui16LenM) == 
        false)
    {
        return AES_CCM_TEST_ERROR;
    }
    
    //
    // Verify CCM output
    //
    if(bEncrypt)
    {
        if (CCMMemCmp(pui8Cstate, (uint8_t const *)pui8ExpectedOutput + 
                      ui16LenM, ui8Mval) == false)
        {
            return AES_CCM_TEST_ERROR;
        }
    }
    else
    {
        if (CCMMemCmp(pui8Cstate, (uint8_t const *)pui8ExpectedOutput,
                      ui8Mval) == false)
        {
            return AES_CCM_TEST_ERROR;
        }
    }
    
    return AES_SUCCESS;
}

//*****************************************************************************
//
// brief   AES-CCM decrypt example
//
// param   bDecrypt           if set to true run decryption
// param   pui8Key            pointer to Key
// param   ui8Mval            length of authentication
// param   pui8N              Nonce
// param   C                  input encrypted message
// param   ui16LenC           length of message
// param   pui8A              Additional data
// param   ui16LenA           length of additional data
// param   ui8KeyLocation     location in Key RAM. Must be one of
//                            the following:
//                            KEY_AREA_0
//                            KEY_AREA_1
//                            KEY_AREA_2
//                            KEY_AREA_3
//                            KEY_AREA_4
//                            KEY_AREA_5
//                            KEY_AREA_6
//                            KEY_AREA_7
// param   ui8CCMLVal         Lval for ccm
// param   ui8CCMIntEnable    set to true to enable interrupts and false to 
//                            disable
// param   pui8ExpectedOutput pointer to expected output
// param   pui8Cstate         authentication Tag
// param   ui8CCMIntEnable    set true/false to enable/disable interrupts
// param   pui8ExpectedOutput pointer to Expected Output
//
//return  AES_SUCCESS if successful
//
//*****************************************************************************
uint8_t CCMDecryptExamples(bool bDecrypt,
                              uint8_t* pui8Key,
                              uint8_t ui8Mval,
                              uint8_t *pui8N,
                              uint8_t *pui8C,
                              uint16_t ui16LenC,
                              uint8_t *pui8A,
                              uint16_t ui16LenA,
                              uint8_t ui8KeyLocation,
                              uint8_t *pui8Cstate,
                              uint8_t ui8CCMLVal,
                              uint8_t ui8CCMIntEnable,
                              uint8_t *pui8ExpectedOutput)
{
    uint8_t status;
    if(ui8CCMIntEnable)
    {    
        //
        // example using interrupt service routine
        //
        if((status = AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation)) != 
           AES_SUCCESS)
        {
            return status;
        }
        
        if((status = CCMInvAuthDecryptStart(bDecrypt, ui8Mval, pui8N, pui8C, 
                                            ui16LenC, pui8A, ui16LenA,
                                            ui8KeyLocation, pui8Cstate,
                                            ui8CCMLVal, ui8CCMIntEnable)) != 
           AES_SUCCESS)
        {
            return status;
        }
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(ui8CCMIntHandler == 0);
        
        ui8CCMIntHandler = 0;
        
        if((status = CCMInvAuthDecryptGetResult(ui8Mval, pui8C, ui16LenC, 
                                                pui8Cstate)) != AES_SUCCESS)
        {
            return status;
        }
    } 
    else
    {
        //
        // example using polling
        //
        if((status = AESLoadKey((uint8_t*)pui8Key, 
                                ui8KeyLocation)) != AES_SUCCESS)
        {
            return status;
        }
        if((status = CCMInvAuthDecryptStart(bDecrypt, ui8Mval, pui8N, pui8C, 
                                            ui16LenC, pui8A, ui16LenA, 
                                            ui8KeyLocation, pui8Cstate, 
                                            ui8CCMLVal, 
                                            ui8CCMIntEnable))!= AES_SUCCESS )
        {
            return status;
        }
        
        //
        // wait for completion of the operation
        //
        do
        {
            ASM_NOP;
        }while(!(CCMInvAuthDecryptCheckResult()));
        
        if((status = CCMInvAuthDecryptGetResult(ui8Mval, pui8C, ui16LenC, 
                                                pui8Cstate)) != AES_SUCCESS)
        {
            return status;
        }
    }
    
    //
    // Verify CCM output
    //
    if (CCMMemCmp(pui8C,  (uint8_t const *)pui8ExpectedOutput, 
                  (ui16LenC-ui8Mval)) == false)
    {
        return AES_CCM_TEST_ERROR;
    }
    
    return AES_SUCCESS;
}

//*****************************************************************************
//
// Main function of example.
//
//*****************************************************************************
void  main(void)
{
    uint8_t status;
    
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
    // Register AES interrupt
    //
    IntRegister(INT_AES, CCMIntHandler);
    
    //
    // Enable global interrupts
    //
    IntAltMapEnable();
    IntMasterEnable();
    
    for(uint8_t i = 0; i < sizeof(sCCMNodecryptExample)/
        sizeof(sCCMNodecryptExample[0] ); i++)
    {
        //
        // Run and Verify CCM Inverse Authentication only
        //
        status = CCMDecryptExamples(sCCMNodecryptExample[i].ui8CCMDecrypt,
                                    sCCMNodecryptExample[i].ui8CCMKey,
                                    sCCMNodecryptExample[i].ui8CCMMval,
                                    sCCMNodecryptExample[i].ui8CCMN,
                                    sCCMNodecryptExample[i].ui8CCMC,
                                    sCCMNodecryptExample[i].ui16CCMLenC,
                                    sCCMNodecryptExample[i].ui8CCMA,
                                    sCCMNodecryptExample[i].ui16CCMLenA,
                                    sCCMNodecryptExample[i].ui8CCMKeyLocation,
                                    sCCMNodecryptExample[i].ui8CCMCstate,
                                    sCCMNodecryptExample[i].ui8CCMLVal,
                                    sCCMNodecryptExample[i].ui8CCMIntEnable,
                                    sCCMNodecryptExample[i].
                                        ui8CCMExpectedOutput);
        if(status != AES_SUCCESS)
        {
            while(1)
            {
                //
                // AES CCM failed
                //
            }
        }
    }
    
    for(uint8_t i = 0; i < sizeof(sCCMNoencryptExample)/
        sizeof(sCCMNoencryptExample[0] ); i++)
    {
        //
        // Run and Verify CCM Authentication only
        //
        status = CCMEncryptExample(sCCMNoencryptExample[i].ui8CCMEncrypt,
                                      sCCMNoencryptExample[i].ui8CCMkey,
                                      sCCMNoencryptExample[i].ui8CCMMval,
                                      sCCMNoencryptExample[i].ui8CCMN,
                                      sCCMNoencryptExample[i].ui8CCMM,
                                      sCCMNoencryptExample[i].ui16CCMLenM,
                                      sCCMNoencryptExample[i].ui8CCMA,
                                      sCCMNoencryptExample[i].ui16CCMLenA,
                                      sCCMNoencryptExample[i].ui8CCMKeyLocation,
                                      sCCMNoencryptExample[i].ui8CCMCstate,
                                      sCCMNoencryptExample[i].ui8CCMCCMLVal,
                                      sCCMNoencryptExample[i].ui8CCMIntEnable,
                                      sCCMNoencryptExample[i].
                                          ui8CCMExpectedOutput);
        if(status != AES_SUCCESS)
        {
            while(1)
            {
                //
                // AES CCM failed
                //
            }
        }
    }
    
    for(uint8_t i = 0; i < sizeof(sCCMEncryptExample)/
        sizeof(sCCMEncryptExample[0] ); i++)
    {
        //
        // Run and Verify CCM Authentication + Encryption
        //
        status = CCMEncryptExample(sCCMEncryptExample[i].ui8CCMEncrypt,
                                      sCCMEncryptExample[i].ui8CCMkey,
                                      sCCMEncryptExample[i].ui8CCMMval,
                                      sCCMEncryptExample[i].ui8CCMN,
                                      sCCMEncryptExample[i].ui8CCMM,
                                      sCCMEncryptExample[i].ui16CCMLenM,
                                      sCCMEncryptExample[i].ui8CCMA,
                                      sCCMEncryptExample[i].ui16CCMLenA,
                                      sCCMEncryptExample[i].ui8CCMKeyLocation,
                                      sCCMEncryptExample[i].ui8CCMCstate,
                                      sCCMEncryptExample[i].ui8CCMCCMLVal,
                                      sCCMEncryptExample[i].ui8CCMIntEnable,
                                      sCCMEncryptExample[i].
                                          ui8CCMExpectedOutput);
        if(status != AES_SUCCESS)
        {
            while(1)
            {
                //
                // AES CCM failed
                //
            }
        }
    }
    
    for(uint8_t i = 0; i < sizeof(sCCMDecryptExample)/
        sizeof(sCCMDecryptExample[0] ); i++)
    {
        //
        // Run and Verify CCM Inverse Authentication + Decryption
        //
        status = CCMDecryptExamples(sCCMDecryptExample[i].ui8CCMDecrypt,
                                    sCCMDecryptExample[i].ui8CCMKey,
                                    sCCMDecryptExample[i].ui8CCMMval,
                                    sCCMDecryptExample[i].ui8CCMN,
                                    sCCMDecryptExample[i].ui8CCMC,
                                    sCCMDecryptExample[i].ui16CCMLenC,
                                    sCCMDecryptExample[i].ui8CCMA,
                                    sCCMDecryptExample[i].ui16CCMLenA,
                                    sCCMDecryptExample[i].ui8CCMKeyLocation,
                                    sCCMDecryptExample[i].ui8CCMCstate,
                                    sCCMDecryptExample[i].ui8CCMLVal,
                                    sCCMDecryptExample[i].ui8CCMIntEnable,
                                    sCCMDecryptExample[i].
                                        ui8CCMExpectedOutput);
        if(status != AES_SUCCESS)
        {
            while(1)
            {
                //
                // AES CCM failed
                //
            }
        }
    }
    
    // CCM was successful
    while(1)
    {
    }
}

