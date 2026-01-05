/*
 * usb_device.h
 *
 *  Created on: Oct 31, 2025
 *      Author: tudo
 */

#ifndef INC_USBT_INC_USB_DEVICE_H_
#define INC_USBT_INC_USB_DEVICE_H_

#include "USBT_Inc/usbd_standards.h"

typedef struct {
  /// \brief The current USB device state
  UsbDeviceState device_state;
  /// \brief The current control transfer stage (For endpoint 0)
  UsbControlTransferStage control_transfer_stage;

    /// \brief The selected USB configuration value
  uint8_t configuration_value;

  /// \brief UsbDeviceOutInBufferPointers
  void const *prt_out_buffer;
  uint32_t out_data_size;
  void *prt_in_buffer;
  uint32_t in_data_size;


}UsbDevice;

#endif /* INC_USBT_INC_USB_DEVICE_H_ */
