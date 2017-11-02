//*****************************************************************************
//! @file       app_usb_descriptor.c
//! @brief      USB descriptor for HID class.
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

#include "app_usb_descriptor.h"
#include <stdio.h>


//
/// Keyboard report descriptor (using format for Boot interface descriptor)
//
static const uint8_t pHid0ReportDesc[] = 
{
    0x05, 0x01,    // Usage Pg (Generic Desktop)
    0x09, 0x06,    // Usage (Keyboard)
    0xA1, 0x01,    // Collection: (Application)
                   //     8 bits: Modifier keys
    0x95, 0x08,    //     Report Count (8)
    0x75, 0x01,    //     Report Size (1)
    0x05, 0x07,    //     Usage Pg (Key Codes)
    0x19, 0xE0,    //     Usage Min (224)
    0x29, 0xE7,    //     Usage Max (231)
    0x15, 0x00,    //     Log Min (0)
    0x25, 0x01,    //     Log Max (1)
    0x81, 0x02,    //     Input: (Data, Variable, Absolute)
                   //     1 byte: Reserved
    0x95, 0x01,    //     Report Count (1)
    0x75, 0x08,    //     Report Size (8)
    0x81, 0x01,    //     Input: (Constant)
                   //     5 bits: LED report (NumLock, CapsLock, ScrollLock, Compose, Kana)
    0x95, 0x05,    //     Report Count (5)
    0x75, 0x01,    //     Report Size (1)
    0x05, 0x08,    //     Usage Pg (LEDs)
    0x19, 0x01,    //     Usage Min (1)
    0x29, 0x05,    //     Usage Max (5)
    0x91, 0x02,    //     Output: (Data, Variable, Absolute)
                   //     3 bits: LED report padding
    0x95, 0x01,    //     Report Count (1)
    0x75, 0x03,    //     Report Size (3)
    0x91, 0x01,    //     Output: (Constant)
                   //     6 bytes: Keycode1 - Keycode6
    0x95, 0x06,    //     Report Count (6)
    0x75, 0x08,    //     Report Size (8)
    0x05, 0x07,    //     Usage Pg (Key Codes)
    0x19, 0x00,    //     Usage Min (0)
    0x29, 0x65,    //     Usage Max (101)
    0x15, 0x00,    //     Log Min (0)
    0x25, 0x65,    //     Log Max (101)
    0x81, 0x00,    //     Input: (Data, Array)
                   //
    0xC0           // End Collection
};


//
/// Mouse report descriptor
//
static const uint8_t pHid1ReportDesc[] = 
{
    0x05, 0x01,    // Usage Pg (Generic Desktop)
    0x09, 0x02,    // Usage (Mouse)
    0xA1, 0x01,    // Collection: (Application)
    0x09, 0x01,    //     Usage Pg (Pointer)
    0xA1, 0x00,    //     Collection (Physical)
                   //         5 bits: Mouse buttons 1-5
    0x95, 0x05,    //         Report Count (5)
    0x75, 0x01,    //         Report Size (1)
    0x05, 0x09,    //         Usage (Button)
    0x19, 0x01,    //         Usage Min (1)
    0x29, 0x05,    //         Usage Max (5)
    0x15, 0x00,    //         Log Min (0)
    0x25, 0x01,    //         Log Max (1)
    0x81, 0x02,    //         Input: (Data, Variable, Absolute)
                   //         3 bits: Mouse button report padding
    0x95, 0x01,    //         Report Count (1)
    0x75, 0x03,    //         Report Size (3)
    0x81, 0x01,    //         Input: (Constant)
                   //         3 bytes: X, Y, Wheel
    0x95, 0x03,    //         Report Count (3)
    0x75, 0x08,    //         Report Size (8)
    0x05, 0x01,    //         Usage Pg (Generic Desktop)
    0x09, 0x30,    //         Usage (X)
    0x09, 0x31,    //         Usage (Y)
    0x09, 0x38,    //         Usage (Wheel)
    0x15, 0x81,    //         Log Min (-127)
    0x25, 0x7F,    //         Log Max (127)
    0x81, 0x06,    //         Input: (Data, Variable, Relative)
    0xC0,          //     End Collection
    0xC0           // End Collection
};


const USB_DESCRIPTOR usbDescriptor = 
{
    { // device
        sizeof(USB_DEVICE_DESCRIPTOR),
        USB_DESC_TYPE_DEVICE,           // bDescriptorType
        0x0200,                         // bcdUSB (USB 2.0)
        0x00,                           // bDeviceClass (given by interface)
        0x00,                           // bDeviceSubClass
        0x00,                           // bDeviceProtocol
        USB_EP0_PACKET_SIZE,            // bMaxPacketSize0
        0x0451,                         // idVendor (Texas Instruments)
        0x16C9,                         // idProduct (CC2538 HID)
        0x0100,                         // bcdDevice (v1.0)
        0x01,                           // iManufacturer
        0x02,                           // iProduct
        0x00,                           // iSerialNumber
        0x01,                           // bNumConfigurations
    },
        { // configuration0
            sizeof(USB_CONFIGURATION_DESCRIPTOR),
            USB_DESC_TYPE_CONFIG,           // bDescriptorType
            SIZEOF_CONFIGURATION0_DESC,     // wTotalLength
            0x02,                           // bNumInterfaces
            0x01,                           // bConfigurationValue
            0x00,                           // iConfiguration
            0xA0,                           // bmAttributes (7,4-0: res, 6: self-powered, 5: remote wakeup)
            25                              // bMaxPower (max 2 * 25 = 50 mA)
        },
            { // interface0
                sizeof(USB_INTERFACE_DESCRIPTOR),
                USB_DESC_TYPE_INTERFACE,        // bDescriptorType
                0x00,                           // bInterfaceNumber
                0x00,                           // bAlternateSetting (none)
                0x01,                           // bNumEndpoints
                0x03,                           // bInterfaceClass (HID)
                0x01,                           // bInterfaceSubClass (Boot)
                0x01,                           // bInterfaceProcotol (Keyboard)
                0x00                            // iInterface
            },
                { // hid0
                    sizeof(USBHID_DESCRIPTOR),
                    USB_DESC_TYPE_HID,              // bDescriptorType
                    0x0111,                         // bcdHID (HID v1.11)
                    0x00,                           // bCountryCode (not localized)
                    0x01,                           // bNumDescriptors
                    USB_DESC_TYPE_HIDREPORT,        // bRDescriptorType
                    sizeof(pHid0ReportDesc)         // wDescriptorLength
                },
                { // endpoint0
                    sizeof(USB_ENDPOINT_DESCRIPTOR),
                    USB_DESC_TYPE_ENDPOINT,         // bDescriptorType
                    0x81,                           // bEndpointAddress
                    USB_EP_ATTR_INT,                // bmAttributes (INT)
                    0x0008,                         // wMaxPacketSize
                    0x0A                            // bInterval (10 full-speed frames = 10 ms)
                },
            { // interface1
                sizeof(USB_INTERFACE_DESCRIPTOR),
                USB_DESC_TYPE_INTERFACE,        // bDescriptorType
                0x01,                           // bInterfaceNumber
                0x00,                           // bAlternateSetting (none)
                0x01,                           // bNumEndpoints
                0x03,                           // bInterfaceClass (HID)
                0x01,                           // bInterfaceSubClass (Boot)
                0x02,                           // bInterfaceProcotol (Mouse)
                0x00                            // iInterface
            },
                { // hid1
                    sizeof(USBHID_DESCRIPTOR),
                    USB_DESC_TYPE_HID,              // bDescriptorType
                    0x0111,                         // bcdHID (HID v1.11)
                    0x00,                           // bCountryCode (not localized)
                    0x01,                           // bNumDescriptors
                    USB_DESC_TYPE_HIDREPORT,        // bRDescriptorType
                    sizeof(pHid1ReportDesc)         // wDescriptorLength
                },
                { // endpoint1
                    sizeof(USB_ENDPOINT_DESCRIPTOR),
                    USB_DESC_TYPE_ENDPOINT,         // bDescriptorType
                    0x82,                           // bEndpointAddress
                    USB_EP_ATTR_INT,                // bmAttributes (INT)
                    0x0008,                         // wMaxPacketSize
                    0x0A                            // bInterval (10 full-speed frames = 10 ms)
                },
    { // strings
        { // langIds
            sizeof(USB_STRING_0_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            0x0409 // English US
        },
        { // manufacturer
            sizeof(USB_STRING_1_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            'T', 'e', 'x', 'a', 's', ' ', 'I', 'n', 's', 't', 'r', 'u', 'm', 'e', 'n', 't', 's'
        },
        { // product
            sizeof(USB_STRING_2_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            'C', 'C', '2', '5', '3', '8', ' ', 'U', 'S', 'B', ' ', 'H', 'I', 'D'
        }
    }
};

//
// Serial number (application-initialized, depends on chip serial number, IEEE address etc.)
//
USB_STRING_3_DESCRIPTOR usbSerialNumberStringDesc;


//
// Look-up table for descriptors other than device and configuration (table is NULL-terminated)
//
const USB_DESCRIPTOR_LUT pUsbDescriptorLut[] = 
{
//    value   index   length                           pDesc
    { 0x0300, 0x0000, sizeof(USB_STRING_0_DESCRIPTOR), &usbDescriptor.strings.langIds },
    { 0x0301, 0x0409, sizeof(USB_STRING_1_DESCRIPTOR), &usbDescriptor.strings.manufacturer },
    { 0x0302, 0x0409, sizeof(USB_STRING_2_DESCRIPTOR), &usbDescriptor.strings.product },
    { 0x0303, 0x0409, sizeof(USB_STRING_3_DESCRIPTOR), &usbSerialNumberStringDesc },
    { 0x2100, 0x0000, sizeof(USBHID_DESCRIPTOR),       &usbDescriptor.hid0 },
    { 0x2100, 0x0001, sizeof(USBHID_DESCRIPTOR),       &usbDescriptor.hid1 },
    { 0x2200, 0x0000, sizeof(pHid0ReportDesc),         &pHid0ReportDesc },
    { 0x2200, 0x0001, sizeof(pHid1ReportDesc),         &pHid1ReportDesc },
    { 0x0000, 0x0000, 0,                               NULL }
};


//
// Look-up table entry specifying double-buffering for each interface descriptor
//
const USB_INTERFACE_EP_DBLBUF_LUT pUsbInterfaceEpDblbufLut[] = 
{
//    pInterface                 inMask  outMask
    { &usbDescriptor.interface0, 0x0000, 0x0000 },
    { &usbDescriptor.interface1, 0x0000, 0x0000 }
};
