/*
 * usbd_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: dongo
 */

#include "USBT_Inc/usbd_driver.h"

void initialize_usb_pin(){
	// Enable clock for GPIOA (DM: PA11, DP: PA12)
	SET_BIT(RCC->AHB2ENR, 1 << RCC_AHB2ENR_GPIOAEN_Pos);

	//Configure (DM: PA11, DP: PA12) as AF
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE11_Msk, 0b10 <<GPIO_MODER_MODE11_Pos );
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE12_Msk, 0b10 <<GPIO_MODER_MODE12_Pos );

	// Set AF10 for PA11 and PA12
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL11_Msk, 0b1010 << GPIO_AFRH_AFSEL11_Pos);
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL12_Msk, 0b1010 << GPIO_AFRH_AFSEL12_Pos);

	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED11_Msk, 0b11 << GPIO_OSPEEDR_OSPEED11_Pos);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED12_Msk, 0b11 << GPIO_OSPEEDR_OSPEED12_Pos);
	MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD11_Msk, 0b00 << GPIO_PUPDR_PUPD11_Pos);
	MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD12_Msk, 0b00 << GPIO_PUPDR_PUPD12_Pos);

}

void initialize_core() {
// CORE
	// Enable PWR clock and VDDUSB supply
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
	SET_BIT(PWR->CR2, PWR_CR2_USV);
	// Enable clock for usb core.
	SET_BIT(RCC->AHB2ENR, 1 << RCC_AHB2ENR_OTGFSEN_Pos);

	// Configure global reg USB_OTG_FS_GLOBAL same as USB_OTG_FS
	SET_BIT(USB_OTG_FS->GUSBCFG,1 << USB_OTG_GUSBCFG_FDMOD_Pos);			// Configures the USB core to run on device mode
	SET_BIT(USB_OTG_FS->GUSBCFG, 1 << USB_OTG_GUSBCFG_PHYSEL_Pos);			// Full Speed serial transceiver select
	MODIFY_REG(USB_OTG_FS->GUSBCFG,USB_OTG_GUSBCFG_TRDT_Msk, 0x6 << USB_OTG_GUSBCFG_TRDT_Pos); // Config TRDT[3:0]: USB turnaround time @ref Table 326. TRDT values (FS): >=32Mhz: 0x6.

	// Core reset and FIFO flush
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) { }
	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_CSRST);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) { }
	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) { }
	WRITE_REG(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH | (0x10U << USB_OTG_GRSTCTL_TXFNUM_Pos));
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) { }


// DEVICE:
	// 1. Program the following fields in the OTG_DCFG register
	//Configure device reg:
	MODIFY_REG(USB_OTG_FS_DEVICE->DCFG, USB_OTG_DCFG_DSPD_Msk, 0b11 << USB_OTG_DCFG_DSPD_Pos); //11: Full speed (USB 1.1 transceiver clock is 48 MHz)

	//Enable VSENSE
	//SET_BIT(USB_OTG_FS -> GCCFG, 1 << USB_OTG_GCCFG_VBDEN_Pos);

	// 3. Program the OTG_GINTMSK register to unmask the following interrupts:  USB reset, Enumeration done , Early suspend, USB suspend, SOF
	SET_BIT(USB_OTG_FS -> GINTMSK, USB_OTG_GINTMSK_USBRST | \
			USB_OTG_GINTMSK_ENUMDNEM | \
			USB_OTG_GINTMSK_ESUSPM | \
			USB_OTG_GINTMSK_USBSUSPM | \
			USB_OTG_GINTMSK_SOFM | \
			USB_OTG_GINTMSK_RXFLVLM | \
			USB_OTG_GINTMSK_WUIM | \
			USB_OTG_GINTMSK_IEPINT);

	// Clears all pending cores interrupt
	// WRITE_REG(USB_OTG_FS->GINTSTS, )
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF;

	// Un-mask global USB interrupt
	SET_BIT(USB_OTG_FS ->GAHBCFG, 1 << USB_OTG_GAHBCFG_GINT_Pos);

}

void connect()
{
	// Powers the transceivers on
	SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

	// Connects the device to the bus
	CLEAR_BIT(USB_OTG_FS_DEVICE->DCTL,USB_OTG_DCTL_SDIS);

}

void disconnect()
{
	// software disconnect
	SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);

	// Powers the transceivers off
	CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

}

void GINTSTS_handler() {
	volatile uint32_t gintsts = USB_OTG_FS_GLOBAL -> GINTSTS;

	if(gintsts & USB_OTG_GINTSTS_USBRST_Msk) {

	}
	else if (gintsts & USB_OTG_GINTSTS_USBRST_Msk) {

	}
	else if (gintsts & USB_OTG_GINTSTS_USBRST_Msk) {

	}
	else if (gintsts & USB_OTG_GINTSTS_USBRST_Msk) {

	}
	else if (gintsts & USB_OTG_GINTSTS_USBRST_Msk) {

	}

}
