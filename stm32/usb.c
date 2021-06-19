/* Copyright (C) 2021 Sam Bazley
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "usb.h"
#include "common.h"
#include "oven.h"
#include <stm32f0xx.h>
#include <string.h>

#define uart_send_str(...)
#define uart_send_int(...)

struct buf_desc_entry {
	volatile uint16_t addr, count;
} __attribute__((packed));

struct buf_desc {
	volatile struct buf_desc_entry tx, rx;
} __attribute__((packed));

#define btable ((volatile struct buf_desc *) USB_PMAADDR)

#define PMA_TX 0
#define PMA_RX 1

#define USB_EP(x) (&USB->EP0R + x * 2)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static struct endpoint {
	uint16_t rx_size, tx_size;
	uint16_t type;
	enum {DIR_BIDIR, DIR_IN, DIR_OUT} dir;
} endpoints [] = {
	{64, 64, USB_EP_CONTROL, DIR_BIDIR},
	{8, 0, USB_EP_BULK, DIR_OUT},
	{0, 8, USB_EP_BULK, DIR_IN},
};

#define ENDPOINT_COUNT (sizeof(endpoints) / sizeof(struct endpoint))

static uint16_t *pma_addr(uint8_t ep_addr, uint8_t isrx)
{
	uint16_t addr = ENDPOINT_COUNT * sizeof(struct buf_desc);

	for (int i = 0; i < ep_addr; i++) {
		addr += endpoints[i].tx_size;
		addr += endpoints[i].rx_size;
	}

	if (isrx) {
		addr += endpoints[ep_addr].tx_size;
	}

	return (uint16_t *) (uint32_t) (USB_PMAADDR + addr);
}

static void usb_set_irq()
{
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM |
		USB_CNTR_RESETM;
}

static void set_ep_rx_count(volatile struct buf_desc_entry *e, int length)
{
	if (length > 62) {
		e->count = 0x8000 | (((length >> 5) - 1) << 10);
	} else {
		e->count = (length >> 1) << 10;
	}
}

static void set_ep_tx_status(volatile uint16_t *epreg, uint16_t status)
{
	*epreg = (*epreg & USB_EPTX_DTOGMASK) ^ status;
}

static void set_ep_rx_status(volatile uint16_t *epreg, uint16_t status)
{
	*epreg = (*epreg & USB_EPRX_DTOGMASK) ^ status;
}

static volatile struct usb_transmit_data {
	uint16_t length, wLength;
	uint16_t sent;
	const uint8_t *data;
} usb_tx_data [3];

static void usb_continue_send_data(int ep)
{
	volatile struct usb_transmit_data *tx = &usb_tx_data[ep];
	uint16_t *dest = pma_addr(ep, PMA_TX);
	uint8_t tx_length = tx->length - tx->sent;

	if (tx->wLength != 0 && tx_length > tx->wLength) {
		tx_length = tx->wLength;
	} else if (tx_length > endpoints[ep].tx_size) {
		tx_length = endpoints[ep].tx_size;
	}

	for (int i = 0; i < tx_length; i += 2) {
		*dest++ = tx->data[i] | ((uint16_t) tx->data[i + 1] << 8);
	}

	tx->data += tx_length;
	tx->sent += tx_length;

	btable[ep].tx.count = tx_length;

	set_ep_tx_status(USB_EP(ep), USB_EP_TX_VALID);
}

static void usb_send_data(uint8_t endpoint,
		const uint8_t *data,
		uint16_t length,
		uint16_t wLength)
{
	usb_tx_data[endpoint].data = data;
	usb_tx_data[endpoint].length = length;
	usb_tx_data[endpoint].wLength = wLength;
	usb_tx_data[endpoint].sent = 0;

	usb_continue_send_data(endpoint);
}

struct usb_setup_packet {
	volatile uint8_t bmRequestType;
	volatile uint8_t bRequest;
	volatile uint16_t wValue;
	volatile uint16_t wIndex;
	volatile uint16_t wLength;
} __attribute__((packed));

static const uint8_t device_descriptor [] = {
	18,         // bLength (constant)
	1,          // bDescriptorType (constant)
	0x10, 0x01, // bcdUSB - USB 1.1
	0,          // bDeviceClass
	0,          // bDeviceSubClass
	0,          // bDeviceProtocol
	64,         // bMaxPacketSize
	0x76, 0x98, // idVendor
	0x89, 0x67, // idProduct
	0x01, 0x01, // bcdDevice - device version number
	1,          // iManufacturer
	2,          // iProduct
	3,          // iSerialNumber
	1,          // bNumConfigurations
};

#define MSB(x) ((x) >> 8)
#define LSB(x) ((x) & 0xFF)

#define CFG_SIZE (9 + 9 + 7 + 7)

static const uint8_t config1_descriptor [] = {
	9,                            // bLength
	2,                            // bDescriptorType
	LSB(CFG_SIZE), MSB(CFG_SIZE), // wTotalLength
	1,                            // bNumInterfaces
	1,                            // bConfigurationValue
	0,                            // iConfiguration
	0x80,                         // bmAttributes
	25,                           // bMaxPower

	// interface 0
	9,                            // bLength
	4,                            // bDescriptorType
	0,                            // bInterfaceNumber
	0,                            // bAlternateSetting
	2,                            // bNumendpoints
	0xff,                         // bInterfaceClass
	0x00,                         // bInterfaceSubClass
	0,                            // bInterfaceProtocol
	0,                            // iInterface

	// OUT endpoint
	7,                            // bLength
	0x05,                         // bDescriptorType
	0x01,                         // bendpointAddress
	0x02,                         // bmAttributes (bulk)
	8, 0,                         // wMaxPacketSize (8)
	0,                            // bInterval

	// IN endpoint
	7,                            // bLength
	0x05,                         // bDescriptorType
	0x82,                         // bendpointAddress
	0x02,                         // bmAttributes (bulk)
	8, 0,                         // wMaxPacketSize (8)
	10,                           // bInterval
};

static const uint8_t lang_str [] = {
	6, 3,
	0x09, 0x04, 0x00, 0x00
};

static const uint8_t vendor_str [] = {
	22, 3,
	'S', 0,
	'a', 0,
	'm', 0,
	' ', 0,
	'B', 0,
	'a', 0,
	'z', 0,
	'l', 0,
	'e', 0,
	'y', 0
};

static const uint8_t product_str [] = {
	10, 3,
	'O', 0,
	'v', 0,
	'e', 0,
	'n', 0,
};

static const uint8_t serial_no_str [] = {
	4, 3, '1', 0
};

enum {
	DESC_DEVICE = 1,
	DESC_CONFIG,
	DESC_STRING
};

#define DESCRIPTOR(type, index, wIndex, addr) \
{(type << 8) | index, wIndex, addr, sizeof(addr)}

static const struct descriptor_entry {
	const uint16_t wValue;
	const uint16_t wIndex;
	const uint8_t *addr;
	const uint8_t length;
} descriptor_table [] = {
	DESCRIPTOR(DESC_DEVICE, 0, 0x0000, device_descriptor),
	DESCRIPTOR(DESC_CONFIG, 0, 0x0000, config1_descriptor),
	DESCRIPTOR(DESC_STRING, 0, 0x0000, lang_str),
	DESCRIPTOR(DESC_STRING, 1, 0x0409, vendor_str),
	DESCRIPTOR(DESC_STRING, 2, 0x0409, product_str),
	DESCRIPTOR(DESC_STRING, 3, 0x0409, serial_no_str),
};

static void find_descriptor(const uint16_t wValue,
		const uint16_t wIndex,
		const uint8_t **addr,
		uint8_t *length)
{
	int ndesc = sizeof(descriptor_table) / sizeof(struct descriptor_entry);

	for (int i = 0; i < ndesc; i++) {
		const struct descriptor_entry *entry = descriptor_table + i;
		if (entry->wValue == wValue && entry->wIndex == wIndex) {
			*addr = entry->addr;
			*length = entry->length;
			return;
		}
	}

	uart_send_str("No descriptor found, wValue: ");
	uart_send_int(wValue);
	uart_send_str(", wIndex: ");
	uart_send_int(wIndex);
	uart_send_str("\n");

	*addr = 0;
	*length = 0;
}

enum {
	REQ_CLEAR_FEATURE = 1,
	REQ_SET_ADDRESS = 5,
	REQ_GET_DESCRIPTOR = 6,
	REQ_SET_CONFIGURATION = 9,
	REQ_SET_INTERFACE = 11,
	REQ_AUDIO_GET_CUR = 129,
	REQ_AUDIO_GET_MIN = 130,
	REQ_AUDIO_GET_MAX = 131,
	REQ_AUDIO_GET_RES = 132,
};

enum {
	RECIP_DEVICE,
	RECIP_INTERFACE,
	RECIP_ENDPOINT,
	RECIP_OTHER
};

struct interface {
	uint16_t index;
	uint16_t alternate;
	void (*on_ctrl)(struct interface *, volatile struct usb_setup_packet *);
};

volatile uint8_t usb_selected_config;

static volatile int address = 0;

static void ack(int ep)
{
	btable[ep].tx.count = 0;
	set_ep_tx_status(USB_EP(ep), USB_EP_TX_VALID);
}

static void on_control_out_interface0(struct interface *iface,
		volatile struct usb_setup_packet *sp)
{
	(void) iface;

	switch (sp->bRequest) {
	default:
		uart_send_str("== UNHANDLED INTERFACE 0 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		ack(0);
	}
}

static void on_control_out_interface1(struct interface *iface,
		volatile struct usb_setup_packet *sp)
{
	switch (sp->bRequest) {
	case REQ_SET_INTERFACE:
		uart_send_str("SET_IFACE ");
		uart_send_int(sp->wValue);
		uart_send_str("\n");
		iface->alternate = sp->wValue;
		volatile uint16_t *epr = USB_EP(1);

		if (iface->alternate) {
			if (*epr & USB_EP_DTOG_TX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
			}

			if (*epr & USB_EP_DTOG_RX) {
				*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
			}

			set_ep_rx_status(epr, USB_EP_RX_VALID);
			set_ep_tx_status(epr, USB_EP_TX_VALID);
		} else {
			set_ep_rx_status(epr, USB_EP_RX_DIS);
			set_ep_tx_status(epr, USB_EP_TX_DIS);
		}

		ack(0);

		break;
	default:
		uart_send_str("== UNHANDLED INTERFACE 1 REQUEST ");
		uart_send_int(sp->bRequest);
		uart_send_str(" ==\n");
		ack(0);
	}
}

static struct interface interfaces [] = {
	{0, 0, on_control_out_interface0},
	{1, 0, on_control_out_interface1},
};

static void on_control_out_interface(volatile struct usb_setup_packet *sp)
{
	int niface = sizeof(interfaces) / sizeof(struct interface);
	for (int i = 0; i < niface; i++) {
		struct interface *iface = interfaces + i;
		if ((sp->wIndex & 0xFF) == iface->index) {
			iface->on_ctrl(iface, sp);
			return;
		}
	}

	uart_send_str("Interface ");
	uart_send_int(sp->wIndex);
	uart_send_str(" not found\n");
}

static void on_control_out_device(volatile struct usb_setup_packet *sp)
{
	switch (sp->bRequest) {
		case REQ_GET_DESCRIPTOR:
		{
			const uint8_t *addr;
			uint8_t length;

			find_descriptor(sp->wValue, sp->wIndex, &addr, &length);

			if (!addr) {
				uart_send_str("Failed to find descriptor\n");
				set_ep_tx_status(&USB->EP0R, USB_EP_TX_STALL);
				break;
			}

			usb_send_data(0, addr, length, sp->wLength);
			break;
		}
		case REQ_SET_ADDRESS:
			address = sp->wValue & 0x7F;
			ack(0);
			break;
		case REQ_SET_CONFIGURATION:
			uart_send_str("SET_CONF");
			uart_send_int(sp->wValue);
			uart_send_str("\n");
			usb_selected_config = sp->wValue;

			ack(0);

			break;
		default:
			uart_send_str("== UNHANDLED REQUEST ");
			uart_send_int(sp->bRequest);
			uart_send_str(" ==\n");
			ack(0);
	}
}

static void on_control_out()
{
	if (USB->EP0R & USB_EP_SETUP) {
		volatile struct usb_setup_packet *sp =
			(volatile struct usb_setup_packet *) pma_addr(0, PMA_RX);

		uint8_t recipient = sp->bmRequestType & 0x1f;

		switch (recipient) {
		case RECIP_DEVICE:
			on_control_out_device(sp);
			break;
		case RECIP_INTERFACE:
			on_control_out_interface(sp);
			return;
		default:
			uart_send_str("Recipient ");
			uart_send_int(recipient);
			uart_send_str(" not implemented for control out\n");
			ack(0);
			return;
		}
	}
}

static void on_control_in()
{
	set_ep_rx_status(&USB->EP0R,USB_EP_RX_VALID);

	if (address) {
		USB->DADDR = address | USB_DADDR_EF;
		address = 0;
	}
}

static int ackd = 1;

static void on_bulk_out()
{
	static union usb_packet_out packet;
	static uint8_t total_received = 0;

	volatile uint8_t received = btable[1].rx.count & 0x3ff;
	uint8_t *data = (uint8_t *) pma_addr(1, PMA_RX);

	ackd = 1;

	if (received == 0) {
		total_received = 0;
		return;
	}

	if (total_received && total_received + received > packet.any.length) {
		total_received = 0;
		return;
	}

	memcpy(((char *) &packet) + total_received, data, received);

	total_received += received;

	if (total_received < packet.any.length) {
		return;
	}

	switch (packet.any.type) {
	case USB_PACKET_START:
		oven_enable(1);
		break;
	case USB_PACKET_QUERY_TEMP:
		oven_query_temp();
		break;
	}

	total_received = 0;
}

void usb_send_fault(enum thermo_fault fault)
{
	static struct usb_packet_fault data = {
		sizeof(data),
		USB_PACKET_FAULT,
		0
	};

	data.fault = fault;

	usb_send_data(2, (uint8_t *) &data, sizeof(data), 0);
}

void usb_send_temp(float temp, float target)
{
	if (!ackd) {
		oven_enable(0);
		return;
	}

	static struct usb_packet_temp data = {
		sizeof(data),
		USB_PACKET_TEMP,
		0.f,
		0.f
	};

	data.temp = temp;
	data.target = target;

	usb_send_data(2, (uint8_t *) &data, sizeof(data), 0);

	ackd = 0;
}

static void on_correct_transfer()
{
	uint8_t ep = USB->ISTR & USB_ISTR_EP_ID;
	if (USB->ISTR & USB_ISTR_DIR) {
		if (ep == 0) {
			on_control_out();
		} else if (ep == 1) {
			on_bulk_out();
			set_ep_rx_status(&USB->EP1R, USB_EP_RX_VALID);
		}

		*USB_EP(ep) = *USB_EP(ep) & ~USB_EP_CTR_RX & USB_EPREG_MASK;
	} else {
		uint16_t remaining = usb_tx_data[ep].length - usb_tx_data[ep].sent;
		if (remaining) {
			usb_continue_send_data(ep);
		} else if ((usb_tx_data[ep].length % usb_tx_data[ep].wLength) == 0) {
			ack(ep);
		}

		if (ep == 0) {
			on_control_in();
		}

		*USB_EP(ep) = *USB_EP(ep) & ~USB_EP_CTR_TX & USB_EPREG_MASK;
	}
}

static void open_endpoints()
{
	for (unsigned int i = 0; i < ENDPOINT_COUNT; i++) {
		volatile uint16_t *epr = USB_EP(i);
		struct endpoint *ep = &endpoints[i];

		set_ep_rx_status(epr, USB_EP_RX_DIS);
		set_ep_tx_status(epr, USB_EP_TX_DIS);

		*epr = (*epr & USB_EPREG_MASK) | i;
		*epr = (*epr & USB_EP_T_MASK) | ep->type;

		if (*epr & USB_EP_DTOG_TX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
		}

		if (*epr & USB_EP_DTOG_RX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
		}

		switch (ep->type) {
		case USB_EP_CONTROL:
			set_ep_rx_count(&btable[i].rx, ep->rx_size);
			set_ep_rx_status(epr,USB_EP_RX_VALID);
			set_ep_tx_status(epr,USB_EP_TX_NAK);
			break;
		case USB_EP_ISOCHRONOUS:
			if (ep->dir == DIR_OUT) {
				set_ep_rx_count(&btable[i].rx, ep->rx_size);
				set_ep_rx_count(&btable[i].tx, ep->tx_size);
				set_ep_rx_status(epr, USB_EP_RX_VALID);
			}
			break;
		case USB_EP_INTERRUPT:
		case USB_EP_BULK:
			if (ep->dir == DIR_OUT) {
				set_ep_rx_count(&btable[i].rx, ep->rx_size);
				set_ep_rx_status(epr, USB_EP_RX_VALID);
			} else {
				set_ep_tx_status(epr,USB_EP_TX_NAK);
			}
			break;
		default:
			uart_send_str("endpoint type ");
			uart_send_int(ep->type);
			uart_send_str(" not implemented\n");
		}
	}
}

void usb_irq()
{
	while (USB->ISTR & USB_ISTR_CTR) {
		on_correct_transfer();
	}

	if (USB->ISTR & USB_ISTR_WKUP) {
		USB->CNTR &= ~USB_CNTR_FSUSP;
		USB->CNTR &= ~USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_WKUP;
	}

	if (USB->ISTR & USB_ISTR_SUSP) {
		USB->CNTR |= USB_CNTR_FSUSP;
		USB->CNTR |= USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_SUSP;
	}

	if (USB->ISTR & USB_ISTR_RESET) {
		usb_selected_config = 0;

		USB->DADDR = USB_DADDR_EF;
		open_endpoints();
		USB->ISTR &= ~USB_ISTR_RESET;
	}
}

void usb_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;

	USB->BTABLE = 0;

	USB->BCDR |= USB_BCDR_DPPU;

	for (unsigned int i = 0; i < ENDPOINT_COUNT; i++) {
		uint32_t rx = (uint32_t) pma_addr(i, PMA_RX);
		uint32_t tx = (uint32_t) pma_addr(i, PMA_TX);
		btable[i].rx.addr = rx - USB_PMAADDR;
		btable[i].tx.addr = tx - USB_PMAADDR;
	}

	usb_selected_config = 0;

	USB->ISTR = 0;
	NVIC_EnableIRQ(USB_IRQn);
	usb_set_irq();
}
