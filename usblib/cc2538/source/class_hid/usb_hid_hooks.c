//*****************************************************************************
//! @file       usb_hid_hooks.c
//! @brief      Contains the necessary hook functions for various USB request
//!             processing that is featured from the USB firmware library. Some
//!             functions are empty.
//!
//! Revised     $Date: 2013-04-08 13:14:23 +0200 (Mon, 08 Apr 2013) $
//! Revision    $Revision: 9658 $
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

#include "usb_firmware_library_headers.h"
#include "usb_class_requests.h"


/******************************************************************************
* GLOBAL DATA
*/

// ****************************************************************************
// Hooks and functions required by the USB library.
// ****************************************************************************

/** \brief This function processes USB class requests with OUT data phase.
 *
 */
void usbcrHookProcessOut(void)
{
    //
    // Process USB class requests with OUT data phase, or stall endpoint 0 when
    // unsupported.
    //
    switch(usbSetupHeader.request)
    {
    case SET_REPORT:
        usbcrSetReport();
        break;
    case SET_PROTOCOL:
        usbcrSetProtocol();
        break;
    case SET_IDLE:
        usbcrSetIdle();
        break;
    default:
        usbfwData.ep0Status = EP_STALL;
        break;
    }
}


/** \brief This function processes USB class requests with IN data phase.
 *
 */
void usbcrHookProcessIn(void)
{
    //
    // Process USB class requests with IN data phase, or stall endpoint 0 when
    // unsupported.
    //
    switch(usbSetupHeader.request)
    {
    case GET_REPORT:
        usbcrGetReport();
        break;
    case GET_PROTOCOL:
        usbcrGetProtocol();
        break;
    case GET_IDLE:
        usbcrGetIdle();
        break;
    default:
        usbfwData.ep0Status = EP_STALL;
        break;
    }
}


// ****************************************************************************
// Unsupported USB hooks
// ****************************************************************************

void usbvrHookProcessOut(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbvrHookProcessIn(void)
{
    usbfwData.ep0Status = EP_STALL;
}


// ****************************************************************************
// Unsupported/unhandled standard requests
// ****************************************************************************

void usbsrHookSetDescriptor(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookSynchFrame(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookClearFeature(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookSetFeature(void)
{
    usbfwData.ep0Status = EP_STALL;
}
uint8_t usbsrHookModifyGetStatus(USB_SETUP_HEADER* pSetupHeader, uint8_t ep0Status, uint16_t* pStatus)
{
    return ep0Status;
}


/** \brief This function handles USB standard request event processing.
 *
 * \param[in]       event
 *     The event to be processed.
 * \param[in]       index
 *     The index.
 */
void usbsrHookProcessEvent(uint8_t event, uint8_t index)
{
    //
    // Process relevant events, one at a time.
    //
    switch(event)
    {
    case USBSR_EVENT_CONFIGURATION_CHANGING:
        //
        //(the device configuration is about to change)
        //
        break;
    case USBSR_EVENT_CONFIGURATION_CHANGED:
        //
        // (the device configuration has changed)
        //
        break;
    case USBSR_EVENT_INTERFACE_CHANGING:
        //
        //(the alternate setting of the given interface is about to change)
        //
        break;
    case USBSR_EVENT_INTERFACE_CHANGED:
        //
        //(the alternate setting of the given interface has changed)
        //
        break;
    case USBSR_EVENT_REMOTE_WAKEUP_ENABLED:
        //
        //(remote wakeup has been enabled by the host)
        //
        break;
    case USBSR_EVENT_REMOTE_WAKEUP_DISABLED:
        //
        //(remote wakeup has been disabled by the host)
        //
        break;
    case USBSR_EVENT_EPIN_STALL_CLEARED:
        //
        //(the given IN endpoint's stall condition has been cleared the host)
        //
        break;
    case USBSR_EVENT_EPIN_STALL_SET:
        //
        //(the given IN endpoint has been stalled by the host)
        //
        break;
    case USBSR_EVENT_EPOUT_STALL_CLEARED:
        //
        //(the given OUT endpoint's stall condition has been cleared the host)
        //
        break;
    case USBSR_EVENT_EPOUT_STALL_SET:
        //
        //(the given OUT endpoint has been stalled by the PC)
        //
        break;
    }
}


/** \brief This function handles USB interrupt event processing.
 *
 */
void usbirqHookProcessEvents(void)
{
    //
    // Handle events that require immediate processing here
    //
}
