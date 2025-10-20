/*
 * usbd_driver.h
 *
 *  Created on: Aug 27, 2025
 *      Author: dongo
 */

#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_
#include "stm32l4xx.h"

// @Ref: Table 2. STM32L49x/L4Ax devices memory map and peripheral register boundary addresses(1)
#define USB_OTG_FS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_PCGCCTL ((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) //OTG power and clock gating control

void initialize_usb_pin();
void initialize_core();
void connect();


#endif /* USBD_DRIVER_H_ */
