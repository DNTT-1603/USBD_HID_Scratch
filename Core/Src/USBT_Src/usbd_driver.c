/*
 * usbd_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: dongo
 */

#include "USBT_Inc/usbd_driver.h"
#include "USBT_Inc/usbd_standards.h"
#include "logger.h"
#include "string.h"

// ----------------------------------------------------------------------
// USB Driver declaration functions
// ----------------------------------------------------------------------
static void configure_endpoint0(const uint16_t endpoint_size);
static void configure_in_enpoint(const uint8_t enp_number, UsbEndpointType enp_type, uint16_t enp_size );
static void deconfigure_inOut_enpoint(const uint8_t enp_number);
static void configure_rxfifo_size(const uint16_t size);
static void configure_txfifo_size(const uint8_t enp_number, const uint16_t size);
static void refresh_fifo_start_address();
static void flush_rxfifo();
static void flush_txfifo(const uint8_t enp_number);

static void usbrts_handle();
static void enumeration_done_handle();
static void rxflvl_handle();
static void read_packet(void const *buffer, uint16_t size);
static void write_packet(uint8_t enp_number, const void *buffer, uint16_t size);

// ----------------------------------------------------------------------
// USB Driver definition functions
// ----------------------------------------------------------------------

/** 
 * @brief  Initialize USB core external function, open additional enable mask INT here
 * Just call after Initlize all other USB core settings.
 */
void initialize_usb_core_external()
{
	SET_BIT(USB_OTG_FS_DEVICE -> DOEPMSK, USB_OTG_DOEPMSK_XFRCM);	// OUT transfer completed interrupt mask
	SET_BIT(USB_OTG_FS_DEVICE -> DIEPMSK, USB_OTG_DIEPMSK_XFRCM);	// IN transfer completed interrupt mask

}

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
	CLEAR_BIT(USB_OTG_FS_GLOBAL->GCCFG, USB_OTG_GCCFG_PWRDWN);

}

static void iepint_handler() {
	//Find which IN endpoint generated the interrupt 
	uint32_t daint = ffs(USB_OTG_FS_DEVICE -> DAINT) -1; //It finds the position of the first (least significant) bit that is set to 1

	// complete IN transfer
	if(IN_ENDPOINT(daint)->DIEPINT & USB_OTG_DIEPINT_XFRC) {
		log_debug("IN endpoint %d transfer completed", daint);

		usb_events.on_in_transfer_completed(daint);

		// clear interrupt
		SET_BIT(IN_ENDPOINT(daint)->DIEPINT, USB_OTG_DIEPINT_XFRC);
	}
}


static void oepint_handler() {
	//Find which OUT endpoint generated the interrupt 
	uint32_t daint = ffs((USB_OTG_FS_DEVICE -> DAINT)>>16) -1; //It finds the position of the first (least significant) bit that is set to 1
	
	// complete OUT transfer
	if(OUT_ENDPOINT(daint)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
		log_debug("OUT endpoint %d transfer completed", daint);

		usb_events.on_out_transfer_completed(daint);

		// clear interrupt
		SET_BIT(OUT_ENDPOINT(daint)->DOEPINT, USB_OTG_DOEPINT_XFRC);
	}
}


void GINTSTS_handler() {
	// NOTE: after finish handerling intr, it have to be cleared.
	// Otherwise, It will always be set.

	volatile uint32_t gintsts = USB_OTG_FS_GLOBAL -> GINTSTS;

	if(gintsts & USB_OTG_GINTSTS_USBRST) {
		usbrts_handle();

		//clear Interrupt
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_USBRST_Msk);
	}
	else if (gintsts & USB_OTG_GINTSTS_ENUMDNE) {
		enumeration_done_handle();

		//clear Interrupt
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_ENUMDNE_Msk);

	}
	else if (gintsts & USB_OTG_GINTSTS_RXFLVL) {
		// Indicates that there is at least one packet pending
		// to be read from the Rx FIFO.
		rxflvl_handle();
		//clear Interrupt
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_RXFLVL_Msk);

	}
	else if (gintsts & USB_OTG_GINTSTS_IEPINT) { 
		// there are in interrupt need to handle hear, 
		iepint_handler();
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_IEPINT_Msk);

	}
	else if (gintsts & USB_OTG_GINTSTS_OEPINT) {
		// there are out interrupt need to handle hear, 
		oepint_handler();
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS,USB_OTG_GINTSTS_OEPINT_Msk);
	}
	usb_events.on_usb_polled();

}

// TODO: something wrong with endpoint 0 configuration here
// Datasheet has seperated IN/OUT endpoint0 configuration, it only set 2bits in 
// MPSIZ[1:0]: Maximum packet size
// So it should be passed these value
// 00: 64 bytes -> 0 (uint)
// 01: 32 bytes -> 1 (uint))
// 10: 16 bytes -> 2 (uint)
// 11: 8 bytes  -> 3 (uint)
// Other endpoints use 11bits MPSIZ[10:0] (enp 1~5)
static void configure_endpoint0(const uint16_t endpoint_size) {
	// unmask control 0  IN/OUT endpoint
	SET_BIT(USB_OTG_FS_DEVICE ->DAINTMSK, \
			(1 << USB_OTG_DAINTMSK_IEPM_Pos) || (1 << USB_OTG_DAINTMSK_OEPM_Pos) );

	// Setup the data FIFO RAM for FIFOS enp0
	MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
			USB_OTG_DIEPCTL_MPSIZ_Msk,
			USB_OTG_DIEPCTL_USBAEP |  							// Enable endpoint
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ,endpoint_size) | 	// Max packet size
			USB_OTG_DIEPCTL_SNAK
			); 								// Set NAK

	// Clear NAK, and enable endpoint data out transfer
	SET_BIT(OUT_ENDPOINT(0)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

	// Note 64 bytes is the max packet size for full speed USB devices
	configure_rxfifo_size(64);
	configure_txfifo_size(0,endpoint_size);

}

static void configure_in_enpoint(const uint8_t enp_number, UsbEndpointType enp_type, uint16_t enp_size )
{
	// unmask IN endpoint
	SET_BIT(USB_OTG_FS_DEVICE ->DAINTMSK, (1 << enp_number));

	// Actives the IN endpoint
	SET_BIT(IN_ENDPOINT(enp_number)->DIEPCTL,USB_OTG_DIEPCTL_USBAEP);

	// Setup enpoind size
	MODIFY_REG(IN_ENDPOINT(enp_number)->DIEPCTL,
			USB_OTG_DIEPCTL_MPSIZ_Msk,
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ,enp_size));

	// Accociate TXFNUM to endpoint number (Ep0-> TXF0, Ep1-> TXF1, ...)
	MODIFY_REG(IN_ENDPOINT(enp_number)->DIEPCTL,
			USB_OTG_DIEPCTL_TXFNUM_Msk,
			_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM,enp_number));

	// Set endpoint type
	MODIFY_REG(IN_ENDPOINT(enp_number)->DIEPCTL,
			USB_OTG_DIEPCTL_EPTYP_Msk,
			_VAL2FLD(USB_OTG_DIEPCTL_EPTYP,enp_type));
	// enable endpoint NAK
	SET_BIT(IN_ENDPOINT(enp_number)->DIEPCTL, USB_OTG_DIEPCTL_SNAK);

	// Enable SD0PID/SEVNFRM
	SET_BIT(IN_ENDPOINT(enp_number)->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);

	configure_txfifo_size(enp_number,enp_size);
}

static void deconfigure_inOut_enpoint(const uint8_t enp_number)
{
	USB_OTG_INEndpointTypeDef* in_enp = IN_ENDPOINT(enp_number);
	USB_OTG_OUTEndpointTypeDef* out_enp = OUT_ENDPOINT(enp_number);

	// Mask all interrupts of endpoint
	CLEAR_BIT(USB_OTG_FS_DEVICE ->DAINTMSK, (1 << enp_number) | (1 << (enp_number + 16)));

	// clear all interrupts of endpoint
	SET_BIT(in_enp->DIEPINT,0x28FB);
	SET_BIT(out_enp->DOEPINT,0x303B);

	// Disable endpoint if possible
	if (in_enp->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
		SET_BIT(in_enp->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
	}
	// Deactivate the OUT endpoint
	CLEAR_BIT(in_enp->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);

	// NOTE: Endpoint out 0 cannot be deconfigured, it is always active
	if (enp_number != 0)
	{
		// Disable endpoint if possible
		if (out_enp->DOEPCTL & USB_OTG_DOEPCTL_EPENA ) {
			SET_BIT(out_enp->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
		}
		// Deactivate the OUT endpoint
		CLEAR_BIT(out_enp->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
	}

	flush_rxfifo();
	flush_txfifo(enp_number);

}

// @ref to 47.11.3 FIFO RAM allocation/Device mode
/**
 * @brief  Configures the size of the Rx FIFO
 * @param  size: Size of the largest OUT endpoint in bytes
 */
static void configure_rxfifo_size(const uint16_t size)
{
	// consider the space reuired for status information and get the size in terms of 32-bit words
	uint16_t true_size = 10 + (2 * ((size / 4) + 1));

	// Configure the size of the Rx FIFO
	MODIFY_REG(USB_OTG_FS_GLOBAL->GRXFSIZ,
			USB_OTG_GRXFSIZ_RXFD_Msk,
			_VAL2FLD(USB_OTG_GRXFSIZ_RXFD,true_size));
	refresh_fifo_start_address();
}

static void configure_txfifo_size(const uint8_t enp_number, const uint16_t size)
{
	uint16_t true_size =(size + 3) / 4; // get the size in terms of 32-bit words

	if (enp_number == 0)
	{
		MODIFY_REG(USB_OTG_FS_GLOBAL->DIEPTXF0_HNPTXFSIZ,
				USB_OTG_TX0FD_Msk,
				_VAL2FLD(USB_OTG_TX0FD,true_size)
				);
	}
	else {
		// Configure the size of the Tx FIFO
		// TODO: in the video it use USB_OTG_NPTXFD reg??
		MODIFY_REG(USB_OTG_FS_GLOBAL->DIEPTXF[enp_number -1],
				USB_OTG_NPTXFD,
				_VAL2FLD(USB_OTG_NPTXFD,true_size)
				);
	}
	refresh_fifo_start_address();
}

static void refresh_fifo_start_address()
{
	uint16_t start_addr = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD, USB_OTG_FS_GLOBAL->GRXFSIZ) * 4; // in bytes

	// update the start address of  the TXFIFO
	MODIFY_REG(USB_OTG_FS_GLOBAL->DIEPTXF0_HNPTXFSIZ,
			USB_OTG_TX0FSA_Msk,
			_VAL2FLD(USB_OTG_TX0FSA,start_addr)
			);

	// The next start address after where the last Tx FIFO ends
	start_addr += _FLD2VAL(USB_OTG_TX0FD, USB_OTG_FS_GLOBAL->DIEPTXF0_HNPTXFSIZ) * 4;
	 // in bytes
	for (uint8_t i = 0; i < ENDPOINT_COUNT - 1; i++) {
		MODIFY_REG(USB_OTG_FS_GLOBAL->DIEPTXF[i],
				USB_OTG_NPTXFSA,
				_VAL2FLD(USB_OTG_NPTXFSA,start_addr)
				);
		start_addr += _FLD2VAL(USB_OTG_NPTXFD, USB_OTG_FS_GLOBAL->DIEPTXF[i]) * 4; // in bytes
	}

}

static void usbrts_handle() {
	log_info("USB reset detected");
 
	for(uint8_t i=0; i <= ENDPOINT_COUNT; i++) {
		deconfigure_inOut_enpoint(i);
	}
	usb_events.on_usb_reset_received();
}

static void enumeration_done_handle() {
	log_info("USB enumeration done detected");
	configure_endpoint0(USB_MAX_PACKET_SIZE_ENP0); // TODO: pass correct endpoint size here

}

static void flush_rxfifo()
{
	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);

}

static void flush_txfifo(const uint8_t enp_number){
	MODIFY_REG(USB_OTG_FS->GRSTCTL,
			USB_OTG_GRSTCTL_TXFNUM_Msk,
			_VAL2FLD(USB_OTG_GRSTCTL_TXFNUM,enp_number)
			);
	SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);

}

static void rxflvl_handle() {
	// each time reading this register, read a word from the Rx FIFO
	uint32_t receive_status = USB_OTG_FS_GLOBAL->GRXSTSP;
	uint8_t enp_number = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, receive_status);
	uint16_t byte_count = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, receive_status);
	uint16_t packet_status = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, receive_status);

	log_debug("RXFLVL handler: enp_number=%d, byte_count=%d, packet_status=0x%02X",
			enp_number, byte_count, packet_status);
	switch (packet_status)
	{
	case 0x06 : // SETUP packet
		usb_events.on_setup_data_received(enp_number,byte_count);
		break;

	case 0x02 : // OUT data packet
		/* code */
		break;
	// When the setup stage is completed, the core will disable the endpoint. So we have to re-enable it here
	case 0x04: // SETUP stage completed
		// Re-enable endpoint 0 OUT
		SET_BIT(OUT_ENDPOINT(enp_number)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

		break;
	case 0x03: // OUT transfer completed
		// When the setup stage is completed, the core will disable the endpoint. So we have to re-enable it here
		SET_BIT(OUT_ENDPOINT(enp_number)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
		break;

	default:
		break;
	}
}

/** \brief Pops data from the RxFIFO and stores it in the buffer.
 * \param buffer Pointer to the buffer, in which the popped data will be stored.
 * \param size Count of bytes to be popped from the dedicated RxFIFO memory.
 */
static void read_packet(void const *buffer, uint16_t size)
{
	// Note: There is only one RxFIFO.
	volatile uint32_t *fifo = FIFO(0);

	for (; size >= 4; size -=4, buffer += 4)
	{
		// Pops one 32-bit word of data (until there is less than one word remaining).
		uint32_t data = *fifo;
		// Stores the data in the buffer.
		*((uint32_t*)buffer) = data;
	}

	if (size > 0)
	{
		// Pops the last remaining bytes (which are less than one word).
		uint32_t data = *fifo;

		for(; size > 0; size--, buffer++, data >>= 8)
		{
			// Stores the data in the buffer with the correct alignment.
			*((uint8_t*)buffer) = 0xFF & data;
		}
	}
}
/** \brief Writes data from the buffer into the TxFIFO of the specified IN endpoint.
 * \param enp_number IN endpoint number, whose TxFIFO will be written to.
 * \param buffer Pointer to the buffer, from which the data will be written.
 * \param size Count of bytes to be written into the dedicated TxFIFO memory.
 */
static void write_packet(uint8_t enp_number, const void *buffer, uint16_t size)
{
	volatile uint32_t *fifo = FIFO(enp_number);
	USB_OTG_INEndpointTypeDef* in_enp = IN_ENDPOINT(enp_number);

	// configure the transfer size and packet count
	in_enp->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ_Msk); // clear previous size
	in_enp->DIEPTSIZ |= (size << USB_OTG_DIEPTSIZ_XFRSIZ_Pos); // set new size

	// send only one packet
	in_enp->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT_Msk); // clear previous pkt count
	in_enp->DIEPTSIZ |= (1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos); // set pkt count to 1

	// Enable transmission after clearing both STALL and NAK of enpoint
	in_enp->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL; // clear STALL
	in_enp->DIEPCTL |= USB_OTG_DIEPCTL_CNAK; // clear NAK
	in_enp->DIEPCTL |= USB_OTG_DIEPCTL_EPENA; // enable endpoint
	// Gets the size in term of 32-bit words (to avoid integer overflow in the loop).

	size = (size + 3) / 4;
	for (; size > 0; size--, buffer += 4)
	{
		// Pushes the data to the TxFIFO.
		*fifo = *((uint32_t *)buffer);
	}
}

static void set_device_address(uint8_t address)
{
  USB_OTG_FS_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD_Msk);

  USB_OTG_FS_DEVICE->DCFG|= (uint32_t) ((address <<USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD_Msk);

}

const UsbDriver usbd_driver =
{
	.initialize_core = initialize_core,
	.initialize_usb_core_external = initialize_usb_core_external,
	.initialize_gpio_pins = initialize_usb_pin,
	.set_device_address = set_device_address,
	.connect = connect,
	.disconnect = disconnect,
	.flush_rxfifo = flush_rxfifo,
	.flush_txfifo = flush_txfifo,
	.configure_in_endpoint = configure_in_enpoint,
	.read_packet = read_packet,
	.write_packet = write_packet,
	.poll = GINTSTS_handler
	// ToDO Add pointers to the other driver functions.
};
