/*
 * usb_standards.h
 *
 *  Created on: Oct 20, 2025
 *      Author: dongo
 */

#ifndef INC_USBT_INC_USBD_STANDARDS_H_
#define INC_USBT_INC_USBD_STANDARDS_H_

// @ref: 47.15.46 OTG device IN endpoint x control register (OTG_DIEPCTLx), EPTYP[1:0]
typedef enum UsbEndpointType
{
	USB_ENDPOINT_TYPE_CONTROL,
	USB_ENDPOINT_TYPE_ISOCHRONOUS,
	USB_ENDPOINT_TYPE_BULK,
	USB_ENDPOINT_TYPE_INTERRUPT
} UsbEndpointType;

#endif /* INC_USBT_INC_USBD_STANDARDS_H_ */
