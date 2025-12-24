/*
 * usd_hid_standard.h
 *
 *  Created on: Dec 24, 2025
 *      Author: tudo
 */

#ifndef INC_USBT_INC_USD_HID_STANDARD_H_
#define INC_USBT_INC_USD_HID_STANDARD_H_

#define USB_DESCRIPTOR_TYPE_HID                 0x21
#define USB_DESCRIPTOR_TYPE_HID_REPORT          0x22
#define USB_HID_COUNTRY_CODE_NONE              0x00

/** HID descriptor types */
typedef struct
{
    uint8_t  bLength;                   // Size of this descriptor in bytes
    uint8_t  bDescriptorType;           // HID descriptor type
    uint16_t bcdHID;                    // HID Class Specification release number
    uint8_t  bCountryCode;              // Hardware target country
    uint8_t  bNumDescriptors;           // Number of HID class descriptors
    uint8_t  bReportDescriptorType;     // Report descriptor type
    uint16_t wReportDescriptorLength; // Total length of Report descriptor
} UsbHidDescriptor;

#endif /* INC_USBT_INC_USD_HID_STANDARD_H_ */
