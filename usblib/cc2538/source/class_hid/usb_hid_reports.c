//*****************************************************************************
//! @file       usb_hid_reports.c
//! @brief      Implementation of HID reports.
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

#include "usb_firmware_library_headers.h"
#include "usb_class_requests.h"
#include "usb_hid_reports.h"


HID_DATA hidData =
{
    .keyboardOutReport.ledStatus = 0,
    .keyboardInReport           = {0},
    .mouseInReport              = {0},
    .keyboardProtocol           = HID_PROTOCOL_REPORT,  // Default, as suggested by HID spec
    .mouseProtocol              = HID_PROTOCOL_REPORT,
    .keyboardIdleRate           = HID_IDLE_NOT_SET,     // Use this as init value, until it is set by USB host
    .mouseIdleRate              = HID_IDLE_NOT_SET,
};


uint8_t hidSendKeyboardInReport(void)
{
    uint8_t result = false;
    bool intDisabled = IntMasterDisable();

    USBFW_SELECT_ENDPOINT(1);
    if(!(HWREG(USB_CSIL) & USB_CSIL_INPKTRDY_M))
    {
        //
        // Send the report
        //
        usbfwWriteFifo(USB_F1, sizeof(KEYBOARD_IN_REPORT), &hidData.keyboardInReport);
        HWREG(USB_CSIL) |= USB_CSIL_INPKTRDY_M;
        result = true;
    }
    if(!intDisabled)
    {
        IntMasterEnable();
    }

    return result;
}


uint8_t hidSendMouseInReport(void)
{
    uint8_t result = false;
    bool intDisabled = IntMasterDisable();

    USBFW_SELECT_ENDPOINT(2);
    if(!(HWREG(USB_CSIL) & USB_CSIL_INPKTRDY_M))
    {
        //
        // Send the report
        //
        usbfwWriteFifo(USB_F2, sizeof(MOUSE_IN_REPORT), &hidData.mouseInReport);
        HWREG(USB_CSIL) |= USB_CSIL_INPKTRDY_M;
        result = true;
    }
    if(!intDisabled)
    {
        IntMasterEnable();
    }

    return result;
}




void hidUpdateKeyboardInReport(KEYBOARD_IN_REPORT *pNewReport)
{
    bool intDisabled = IntMasterDisable();
    hidData.keyboardInReport.modifiers = pNewReport->modifiers;
    hidData.keyboardInReport.pKeyCodes[0] = pNewReport->pKeyCodes[0];
    hidData.keyboardInReport.pKeyCodes[1] = pNewReport->pKeyCodes[1];
    hidData.keyboardInReport.pKeyCodes[2] = pNewReport->pKeyCodes[2];
    hidData.keyboardInReport.pKeyCodes[3] = pNewReport->pKeyCodes[3];
    hidData.keyboardInReport.pKeyCodes[4] = pNewReport->pKeyCodes[4];
    hidData.keyboardInReport.pKeyCodes[5] = pNewReport->pKeyCodes[5];
    if(!intDisabled)
    {
        IntMasterEnable();
    }
}




void hidUpdateMouseInReport(MOUSE_IN_REPORT *pNewReport)
{
    bool intDisabled = IntMasterDisable();

    //
    // Buttons are updated
    //
    hidData.mouseInReport.buttons = pNewReport->buttons;

    //
    // Movements are updated
    //
    hidData.mouseInReport.dX = pNewReport->dX;
    hidData.mouseInReport.dY = pNewReport->dY;
    hidData.mouseInReport.dZ = pNewReport->dZ;
    if(!intDisabled)
    {
        IntMasterEnable();
    }
}
