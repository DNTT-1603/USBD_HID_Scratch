/*
 * usbd_driver.h
 *
 *  Created on: Aug 27, 2025
 *      Author: dongo
 */

#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_
#include "stm32l476xx.h"
#include "stm32l4xx.h"

#include "USBT_Inc/usbd_standards.h"

// @Ref: Table 2. STM32L49x/L4Ax devices memory map and peripheral register boundary addresses(1)
#define USB_OTG_FS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_PCGCCTL ((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) //OTG power and clock gating control

/// \brief Totol count of IN or OUT enpont
#define ENDPOINT_COUNT 6 	// (1 bin-direction enpoint 0 and 5 other IN/OUT endpoints)




/**
 * @brief  Returns the USB_OTG_INEndpointTypeDef structure pointer
 *         for the specified IN endpoint.
 * @param  enp_num: Endpoint number.
 * @retval Pointer to USB_OTG_INEndpointTypeDef structure.
 */
inline static USB_OTG_INEndpointTypeDef* IN_ENDPOINT(uint8_t enp_num)
{
    return (USB_OTG_INEndpointTypeDef*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (enp_num * USB_OTG_EP_REG_SIZE));
}

/**
 * @brief  Returns the USB_OTG_OUTEndpointTypeDef structure pointer
 *         for the specified OUT endpoint.
 * @param  enp_num: Endpoint number.
 * @retval Pointer to USB_OTG_OUTEndpointTypeDef structure.
 */
inline static USB_OTG_OUTEndpointTypeDef* OUT_ENDPOINT(uint8_t enp_num)
{
    return (USB_OTG_OUTEndpointTypeDef*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (enp_num * USB_OTG_EP_REG_SIZE));
}

inline static __IO uint32_t* FIFO(uint8_t enp_num)
{
    return (__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (enp_num * USB_OTG_FIFO_SIZE));
}

void initialize_usb_pin();
void initialize_core();
void initialize_usb_core_external();

void connect();

/// \brief USB driver functions exposed to USB framework.
typedef struct
{
	void (*initialize_core)();
    void (*initialize_usb_core_external)();
	void (*initialize_gpio_pins)();
	void (*set_device_address)(uint8_t address);
	void (*connect)();
	void (*disconnect)();
	void (*flush_rxfifo)();
	void (*flush_txfifo)(uint8_t endpoint_number);
	void (*configure_in_endpoint)(const uint8_t enp_number, UsbEndpointType enp_type, uint16_t enp_size);
	void (*read_packet)(void const *buffer, uint16_t size);
	void (*write_packet)(uint8_t endpoint_number, void const *buffer, uint16_t size);
	void (*poll)();
	// ToDO Add pointers to the other driver functions.
} UsbDriver;

extern const UsbDriver usbd_driver;
extern UsbEvents usb_events;

#endif /* USBD_DRIVER_H_ */
