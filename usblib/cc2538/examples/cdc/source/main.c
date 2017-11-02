//*****************************************************************************
//! @file       main.c
//! @brief      Main file for CC2538 USB CDC example. This example configures
//!             the CC2538 as an USB CDC device that echos data received
//!             over USB.
//!
//! Revised     $Date: 2013-03-26 09:47:02 +0100 (Tue, 26 Mar 2013) $
//! Revision    $Revision: 9535 $
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

//*****************************************************************************
//
// Include driver libraries
//
//*****************************************************************************
#include "bsp.h"
#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"

//*****************************************************************************
//
// Local variablse
//
//*****************************************************************************
USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
static uint8_t pInBuffer[128];
static uint8_t pOutBuffer[128];
static uint8_t pAppBuffer[128];


//*****************************************************************************
//
// Implementations of function that are required by usb framework.
//
//*****************************************************************************
void usbsuspHookEnteringSuspend(bool remoteWakeupAllowed) {
    if (remoteWakeupAllowed) {
    }
}


void usbsuspHookExitingSuspend(void) {
}


//
// Application entry point
//
int main(void)
{

    //
    // Initialize board and system clock
    //
    bspInit(SYS_CTRL_32MHZ);

    //
    // Initialize buffers
    //
    memset(&usbCdcInBufferData, 0x00, sizeof(USB_EPIN_RINGBUFFER_DATA));
    usbCdcInBufferData.pBuffer = pInBuffer;
    usbCdcInBufferData.size = sizeof(pInBuffer);
    usbCdcInBufferData.endpointReg = USB_F4;
    usbCdcInBufferData.endpointIndex = 4;
    usbCdcInBufferData.endpointSize = 64;
    memset(&usbCdcOutBufferData, 0x00, sizeof(USB_EPOUT_RINGBUFFER_DATA));
    usbCdcOutBufferData.pBuffer = pOutBuffer;
    usbCdcOutBufferData.size = sizeof(pOutBuffer);
    usbCdcOutBufferData.endpointReg = USB_F4;
    usbCdcOutBufferData.endpointIndex = 4;

    //
    // Enable the USB interface
    //
    usbCdcInit(76800);

    //
    // Main loop
    //
    while (1) {

        //
        // Process USB events
        //
        usbCdcProcessEvents();

        //
        // Implement COM-port loopback
        //
        uint16_t count = usbibufGetMaxPushCount(&usbCdcInBufferData);
        uint16_t maxPopCount = usbobufGetMaxPopCount(&usbCdcOutBufferData);
        if (count > maxPopCount)
        {
            count = maxPopCount;
        }
        if (count)
        {
            usbobufPop(&usbCdcOutBufferData, pAppBuffer, count);
            usbibufPush(&usbCdcInBufferData, pAppBuffer, count);
        }
    }
}
