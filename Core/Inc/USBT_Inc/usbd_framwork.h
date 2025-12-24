/*
 * usbd_framework.h
 *
 *  Created on: Sep 11, 2025
 *      Author: dongo
 */

#ifndef USBD_FRAMEWORK_H_
#define USBD_FRAMEWORK_H_
#include "usbd_driver.h"
#include "USBT_Inc/usb_device.h"


void usbd_initialize(UsbDevice * usb_device);
void usbd_poll();

#endif /* USBD_FRAMEWORK_H_ */
