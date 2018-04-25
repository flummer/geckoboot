/*
 * This file is part of geckoboot.
 * Copyright 2017-2018 Emil Renner Berthing <esmil@esmil.dk>
 *
 * geckoboot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * geckoboot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with geckoboot. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "lib/clock.c"
#include "lib/gpio.c"
#include "lib/leuart0.c"
#include "lib/flash.c"
#include "lib/usb.c"

#ifdef NDEBUG
#define debug(...)
#else
#define debug(...) printf(__VA_ARGS__)
#endif

#define USB_FIFO_RXSIZE  (256/sizeof(uint32_t))
#define USB_FIFO_TX0SIZE ( 64/sizeof(uint32_t))
#define USB_FIFO_TX1SIZE (256/sizeof(uint32_t))
#define USB_FIFO_TX2SIZE (256/sizeof(uint32_t))
#define USB_FIFO_TX3SIZE ( 64/sizeof(uint32_t))

#define USB_EP1_PACKETSIZE 64

#define USB_WORD(x) ((x) & 0xFF),((x) >> 8)

#define BE_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

#define B2W(a,b,c,d) (((d)<<24)|((c)<<16)|((b)<<8)|(a))

#define FLASH_TOTAL_PAGES  64U
#ifdef NDEBUG
#  define FLASH_PAGE_OFFSET   4U
#else
#  define FLASH_PAGE_OFFSET  32U
#endif
#define FLASH_ADDRESS      ((void *)FLASH_BASE)

//#define USBMS_BLOCKSIZE 512U
#define USBMS_BLOCKSIZE 1024U

#define PART_FIRST_BLOCK (1024U*1024U/USBMS_BLOCKSIZE)
#define PART_BLOCKS ((2*USBMS_BLOCKSIZE/3) - 32)

#define USBMS_BLOCKS (PART_FIRST_BLOCK+PART_BLOCKS)

//#define USBMS_MODE_SENSE_CACHE_DATA
#define USBMS_READ_FORMAT_CAPACITY
//#define USBMS_REPORT_LUNS

//#define DFU_UPLOAD
//#define DFU_INTERFACE_NAME

struct usb_packet_setup {
	union {
		struct {
			uint8_t bmRequestType;
			uint8_t bRequest;
		};
		uint16_t request;
	};
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

struct usb_descriptor_device {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
};

struct usb_descriptor_configuration {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
	uint8_t rest[];
};

struct usb_descriptor_string {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wCodepoint[];
};

struct command_block_wrapper {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

struct command_status_wrapper {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

struct inquiry_data {
	uint8_t peripheral;
	uint8_t rmb;
	uint8_t version;
	uint8_t response_data_format;
	uint8_t additional_length;
	uint8_t flags[3];
	uint8_t vendor_id[8];
	uint8_t product_id[16];
	uint8_t product_revision[4];
};

struct sense_data {
	uint8_t response_code;
	uint8_t obsolete;
	uint8_t sense_key;
	uint8_t information[4];
	uint8_t additional_sense_length;
	uint8_t command_specific[4];
	uint8_t additional_sense_code;
	uint8_t additional_sense_code_qualifier;
	uint8_t field_replacable_unit_code;
	uint8_t sense_key_specific[3];
	uint8_t rest[];
};

struct mode_sense_data6 {
	uint8_t mode_data_length;
	uint8_t medium_type;
	uint8_t device_specific;
	uint8_t block_descriptor_length;
	uint8_t rest[];
};

struct format_capacity_data {
	/* capacity list header */
	uint8_t reserved[3];
	uint8_t capacity_list_length;
	/* current/maximum capacity descriptor */
	uint32_t number_of_blocks;
	uint8_t descriptor_type;
	uint8_t block_length[3];
};

struct fat12_directory_entry {
	uint8_t name[8];
	uint8_t ext[3];
	uint8_t attrs;
	uint8_t ext_attrs;
	uint8_t ctime_ms;
	uint16_t ctime;
	uint16_t cdate;
	uint16_t adate;
	uint16_t rights;
	uint16_t mtime;
	uint16_t mdate;
	uint16_t start;
	uint32_t size;
};

static const __align(4) struct usb_descriptor_device usb_descriptor_device = {
	.bLength            = 18,
	.bDescriptorType    = 1,
	.bcdUSB             = 0x0200,
	.bDeviceClass       = 0x00, /* 0x00 = per interface */
	.bDeviceSubClass    = 0x00,
	.bDeviceProtocol    = 0x00,
	.bMaxPacketSize0    = 64,
	.idVendor           = 0x0483,
	.idProduct          = 0xdf11,
	.bcdDevice          = 0x0200,
	.iManufacturer      = 1,
	.iProduct           = 2,
	.iSerialNumber      = 3,
	.bNumConfigurations = 1,
};

static const __align(4) struct usb_descriptor_configuration usb_descriptor_configuration1 = {
	.bLength              = 9,
	.bDescriptorType      = 2,
	.wTotalLength         = 50,
	.bNumInterfaces       = 2,
	.bConfigurationValue  = 1,
	.iConfiguration       = 0,
	.bmAttributes         = 0x80,
	.bMaxPower            = 250,
	.rest = {
	/* Interface */
	/* .bLength            */ 9,
	/* .bDescriptorType    */ 4,
	/* .bInterfaceNumber   */ 0,
	/* .bAlternateSetting  */ 0,
	/* .bNumEndpoints      */ 2,
	/* .bInterfaceClass    */ 0x08, /* 0x08 = Mass-Storage Class */
	/* .bInterfaceSubClass */ 0x06, /* 0x06 = SCSI transparent command set */
	/* .bInterfaceProtocol */ 0x50, /* 0x50 = Bulk-Only Transport (BBB) */
	/* .iInterface         */ 0,
	/* Endpoint */
	/* .bLength            */ 7,
	/* .bDescriptorType    */ 5,
	/* .bEndpointAddress   */ 0x81, /* IN, endpoint 1 */
	/* .bmAttributes       */ 0x02, /* bulk, data endpoint */
	/* .wMaxPacketSize     */ USB_WORD(USB_EP1_PACKETSIZE),
	/* .bInterval          */ 0,    /* unused */
	/* Endpoint */
	/* .bLength            */ 7,
	/* .bDescriptorType    */ 5,
	/* .bEndpointAddress   */ 0x01, /* OUT, endpoint 1 */
	/* .bmAttributes       */ 0x02, /* bulk, data endpoint */
	/* .wMaxPacketSize     */ USB_WORD(USB_EP1_PACKETSIZE),
	/* .bInterval          */ 0,    /* unused */
	/* Interface */
	/* .bLength            */ 9,
	/* .bDescriptorType    */ 0x04, /* interface */
	/* .bInterfaceNumber   */ 1,
	/* .bAlternateSetting  */ 0,
	/* .bNumEndpoints      */ 0,    /* only the control pipe is used */
	/* .bInterfaceClass    */ 0xFE, /* application specific */
	/* .bInterfaceSubClass */ 0x01, /* device firmware upgrade */
	/* .bInterfaceProtocol */ 0x02, /* DFU mode protocol */
#ifdef DFU_INTERFACE_NAME
	/* .iInterface         */ 4,
#else
	/* .iInterface         */ 0,
#endif
	/* Functional */
	/* .bLength            */ 9,
	/* .bDescriptorType    */ 0x21, /* DFU functional descriptor */
#ifdef DFU_UPLOAD
	/* .bmAttributes       */ 0x07,
#else
	/* .bmAttributes       */ 0x05,
#endif
	/* .wDetachTimeOut     */ USB_WORD(500), /* 500ms */
	/* .wTransferSize      */ USB_WORD(FLASH_PAGE_SIZE),
	/* .bcdDFUVersion      */ USB_WORD(0x0101), /* DFU v1.1 */
	}
};

static const struct usb_descriptor_configuration *const usb_descriptor_configuration[] = {
	&usb_descriptor_configuration1,
};

static const __align(4) struct usb_descriptor_string usb_descriptor_string0 = {
	.bLength         = 4,
	.bDescriptorType = 3,
	.wCodepoint = {
		0x0409, /* English (US) */
	},
};

static const __align(4) struct usb_descriptor_string usb_descriptor_vendor = {
	.bLength         = 16,
	.bDescriptorType = 3,
	.wCodepoint = {
		'L','a','b','i','t','a','t',
	},
};

static const __align(4) struct usb_descriptor_string usb_descriptor_product = {
	.bLength         = 22,
	.bDescriptorType = 3,
	.wCodepoint = {
		'G','e','c','k','o','n','a','t','o','r',
	},
};

/* must be at least 12 characters long and consist of only '0'-'9','A'-'B'
 * at least according to the mass-storage bulk-only document */
static const __align(4) struct usb_descriptor_string usb_descriptor_serial = {
	.bLength         = 26,
	.bDescriptorType = 3,
	.wCodepoint = {
		'0','0','0','0','0','0','0','0','0','0','0','1',
	},
};

#ifdef DFU_INTERFACE_NAME
static const __align(4) struct usb_descriptor_string usb_descriptor_interface0 = {
	.bLength         = 20,
	.bDescriptorType = 3,
	.wCodepoint = {
		'G','e','c','k','o','B','o','o','t',
	},
};
#endif

static const struct usb_descriptor_string *const usb_descriptor_string[] = {
	&usb_descriptor_string0,
	&usb_descriptor_vendor,
	&usb_descriptor_product,
	&usb_descriptor_serial,
#ifdef DFU_INTERFACE_NAME
	&usb_descriptor_interface0,
#endif
};

static const __align(4) struct inquiry_data inquiry_data = {
	.peripheral           = 0x00, /* peripheral device connected, SBC-2 (magnetic disk) */
	.rmb                  = 0x80, /* removable medium bit set */
	/* 0x05 = SPC-3, but FreeBSD then needs REPORT LUNS (0xa0) and SERVICE ACTION IN (0x9e) */
	.version              = 0x04,
	.response_data_format = 0x02, /* 0x02 = SPC-3 */
	.additional_length    = 31,
	.flags                = { 0x00, 0x00, 0x00 },
	.vendor_id            = { 'L','a','b','i','t','a','t',' ' },
	.product_id           = { 'G','e','c','k','o','n','a','t','o','r','B','o','o','t',' ',' ' },
	.product_revision     = { '0','0','0','1' },
};

static const __align(4) struct sense_data sense_data = {
	.response_code                   = 0x70, /* current errors, fixed format */
	.obsolete                        = 0x00,
	.sense_key                       = 0x05, /* illegal request */
	.information                     = { 0x00, 0x00, 0x00, 0x00 },
	.additional_sense_length         = 10,
	.command_specific                = { 0x00, 0x00, 0x00, 0x00 },
	.additional_sense_code           = 0x24,
	.additional_sense_code_qualifier = 0x00, /* invalid field in CDB */
	.field_replacable_unit_code      = 0x00,
	.sense_key_specific              = { 0x00, 0x00, 0x00 },
	/* .rest = { } */
};

static const __align(4) struct mode_sense_data6 mode_sense_data6 = {
#ifdef USBMS_MODE_SENSE_CACHE_DATA
	.mode_data_length        = 35,
	.medium_type             = 0x00,
	.device_specific         = 0x00,
	.block_descriptor_length = 0x00,
	.rest = {
	/* Caching Page */
	/* .page_code             = */ 0x08,
	/* .page_length           = */ 18,
	/* .flags1                = */ 0x00,
	/* .retention_priority    = */ 0x00,
	/* .disable_prefetch      = */ 0x00, 0x00,
	/* .minimum_prefetch      = */ 0x00, 0x00,
	/* .maximum_prefetch      = */ 0x00, 0x00,
	/* .maximum_ceiling       = */ 0x00, 0x00,
	/* .flags2                = */ 0x00,
	/* .cache_segments        = */ 0x00,
	/* .cache_segment_size    = */ 0x00, 0x00,
	/* .reserved              = */ 0x00,
	/* .noncache_segment_size = */ 0x00, 0x00, 0x00,
	/* Information Exceptions Control Mode Page */
	/* .page_code             = */ 0x1C,
	/* .page_length           = */ 10,
	/* .flags1                = */ 0x00,
	/* .mrie                  = */ 0x00,
	/* .interval_timer        = */ 0x00, 0x00, 0x00, 0x00,
	/* .report_count          = */ 0x00, 0x00, 0x00, 0x00,
	},
#else
	.mode_data_length        = 3,
	.medium_type             = 0x00,
	.device_specific         = 0x00,
	.block_descriptor_length = 0x00,
#endif
};

#ifdef USBMS_READ_FORMAT_CAPACITY
static const __align(4) struct format_capacity_data format_capacity_data = {
	.reserved             = { 0x00, 0x00, 0x00 },
	.capacity_list_length = 8,
	.number_of_blocks     = BE_UINT32(USBMS_BLOCKS),
	.descriptor_type      = 0x02, /* formatted media */
	.block_length         = { USBMS_BLOCKSIZE >> 16, (USBMS_BLOCKSIZE >> 8) & 0xFF, USBMS_BLOCKSIZE & 0xFF }
};
#endif

enum usb_ep0_state {
	USB_EP0_IDLE,
	USB_EP0_SETUP,
	USB_EP0_OUT,
	USB_EP0_OUT_ACK,
	USB_EP0_IN,
	USB_EP0_IN_ACK,
};

static struct {
	enum usb_ep0_state state;
	uint32_t bytes;
	uint32_t packetsize;
} ep0state;

static __uninitialized uint32_t ep0buf[(4*sizeof(struct usb_packet_setup) + FLASH_PAGE_SIZE)/sizeof(uint32_t)];

enum usbms_state {
	USBMS_STATE_CBW,
	USBMS_STATE_DATA_OUT,
	USBMS_STATE_DATA_IN,
	USBMS_STATE_CSW,
	USBMS_STATE_ERROR,
};

static struct {
	enum usbms_state state;
	union {
		const void *data;
		void *buf;
	};
	uint32_t bytes;
	uint32_t chunk;
	uint32_t lba;
	unsigned int page;
} usbms;

static __uninitialized union {
	struct command_block_wrapper cbw;
	struct command_status_wrapper csw;
	uint8_t byte[USB_EP1_PACKETSIZE];
	uint32_t word[USB_EP1_PACKETSIZE/sizeof(uint32_t)];
} usbms_buf;

static unsigned int reboot_flag;
static bool reboot_on_reset;

#ifndef NDEBUG
static struct {
	volatile uint32_t first;
	volatile uint32_t last;
	uint8_t buf[1024];
	//uint8_t buf[2048];
} leuart0_output;

static void
leuart0_write(const uint8_t *ptr, size_t len)
{
	uint32_t last = leuart0_output.last;

	for (; len > 0; len--) {
		leuart0_output.buf[last++] = *ptr++;
		last %= ARRAY_SIZE(leuart0_output.buf);
	}
	leuart0_output.last = last;
	leuart0_flag_tx_buffer_level_enable();
}

void
LEUART0_IRQHandler(void)
{
	uint32_t first = leuart0_output.first;

	if (first != leuart0_output.last) {
		leuart0_txdata(leuart0_output.buf[first++]);
		leuart0_output.first = first % ARRAY_SIZE(leuart0_output.buf);
	} else
		leuart0_flag_tx_buffer_level_disable();
}

ssize_t
_write(int fd, const uint8_t *ptr, size_t len)
{
	leuart0_write(ptr, len);

	return len;
}

static const char *const usb_ep0_state_string[] = {
	[USB_EP0_IDLE]    = "idle",
	[USB_EP0_SETUP]   = "setup",
	[USB_EP0_OUT]     = "out",
	[USB_EP0_OUT_ACK] = "out-ack",
	[USB_EP0_IN]      = "in",
	[USB_EP0_IN_ACK]  = "in-ack",
};
#endif

static __noreturn void
reboot(void)
{
	usb_disconnect();
	NVIC_SystemReset();
	__builtin_unreachable();
}

static int
flash_write_page(const uint32_t *buf, uint32_t page)
{
	unsigned int i;

	flash_write_enable();
	flash_address_set(page*FLASH_PAGE_SIZE);
	flash_address_load();
	if (flash_address_invalid())
		return 1;

	flash_erase_page();
	while (flash_busy())
		/* wait */;

	gpio_clear(GPIO_PA8);
	for (i = 0; i < FLASH_PAGE_SIZE/sizeof(uint32_t); i++) {
		while (!flash_wdata_ready())
			/* wait */;
		flash_wdata(*buf++);
		flash_write_once();
	}
	gpio_set(GPIO_PA8);

	return 0;
}

static void
dumpsetup(const struct usb_packet_setup *p)
{
	debug("{\r\n"
	      "  bmRequestType 0x%02x\r\n"
	      "  bRequest      0x%02x\r\n"
	      "  wValue        0x%04x\r\n"
	      "  wIndex        0x%04x\r\n"
	      "  wLength       0x%04x\r\n"
	      "}\r\n",
		p->bmRequestType,
		p->bRequest,
		p->wValue,
		p->wIndex,
		p->wLength);
}

static void
dumpcbw(const struct command_block_wrapper *cbw)
{
	debug("CBW {\r\n"
	      "  dCBWSignature          0x%08lx\r\n"
	      "  dCBWTag                %lu\r\n"
	      "  dCBWDataTransferLength %lu\r\n"
	      "  bmCBWFlags             0x%02hx\r\n"
	      "  bCBWLUN                %hu\r\n"
	      "  bCBWCBLength           %hu\r\n"
	      "  CBWCB { opcode = 0x%02hx }\r\n"
	      "}\r\n",
			cbw->dCBWSignature,
			cbw->dCBWTag,
			cbw->dCBWDataTransferLength,
			cbw->bmCBWFlags,
			cbw->bCBWLUN,
			cbw->bCBWCBLength,
			cbw->CBWCB[0]);
}

static void
usb_ep0out_prepare_setup(void *buf)
{
	usb_ep0out_dma_address_set(buf);
	usb_ep0out_transfer_size(0, 0);
	usb_ep0out_enable_setup();
}

static void
usb_suspend(void)
{
	usb_phy_stop();
	/* TODO: power down more */
	usb_flags_clear(~0U);
	usb_flags_enable(USB_GINTMSK_WKUPINTMSK |
	                 USB_GINTMSK_RESETDETMSK);
}

static void
usb_unsuspend(void)
{
	/* disable interrupt */
	NVIC_DisableIRQ(USB_IRQn);

	usb_phy_start();

	usb_disconnect();
	usb_core_reset();
	while (usb_core_resetting())
		/* wait */;

	/* SiliconLabs code delays around 1us here */

	/* Wait for AHB master IDLE state. */
	while (!usb_ahb_idle())
		/* wait */;

	/* initialise USB core */
	usb_ahb_config(USB_AHB_CONFIG_DMA_ENABLE |
	               USB_AHB_CONFIG_BURST_INCR |
	               USB_AHB_CONFIG_INTERRUPTS_ENABLE);
	usb_flags_clear(~0U);
	usb_flags_enable(USB_GINTMSK_ENUMDONEMSK |
	                 USB_GINTMSK_USBRSTMSK |
	                 USB_GINTMSK_USBSUSPMSK |
	                 USB_GINTMSK_ERLYSUSPMSK |
	                 USB_GINTMSK_OEPINTMSK |
	                 USB_GINTMSK_IEPINTMSK);

	/* initialize device */
	USB->DCFG = USB_DCFG_RESVALID_DEFAULT
	          | USB_DCFG_PERFRINT_80PCNT
	          /* | USB_DCFG_ENA32KHZSUSP */
	          | USB_DCFG_NZSTSOUTHSHK
	          | USB_DCFG_DEVSPD_FS;

	/* ignore frame numbers on ISO transfers. */
	USB->DCTL = USB_DCTL_IGNRFRMNUM
	          | USB_DCTL_PWRONPRGDONE
	          | USB_DCTL_SFTDISCON;

	/* setup fifo allocation */
	USB->GRXFSIZ = USB_FIFO_RXSIZE;
	USB->GNPTXFSIZ = (USB_FIFO_TX0SIZE << 16)
	               | (USB_FIFO_RXSIZE);
	USB->DIEPTXF1  = (USB_FIFO_TX1SIZE << 16)
	               | (USB_FIFO_RXSIZE+USB_FIFO_TX0SIZE);
	USB->DIEPTXF2  = (USB_FIFO_TX2SIZE << 16)
	               | (USB_FIFO_RXSIZE+USB_FIFO_TX0SIZE+USB_FIFO_TX1SIZE);
	USB->DIEPTXF3  = (USB_FIFO_TX3SIZE << 16)
	               | (USB_FIFO_RXSIZE+USB_FIFO_TX0SIZE+USB_FIFO_TX1SIZE+USB_FIFO_TX2SIZE);

	/* stop all endpoints */
	USB->DOEP0CTL = USB_DOEP0CTL_SNAK
	              | USB_DOEP0CTL_STALL
	              | USB_DOEP0CTL_EPTYPE_DEFAULT
	              | USB_DOEP0CTL_USBACTEP
	              | USB_DOEP0CTL_MPS_64B;
	USB->DOEP[0].CTL = USB_DOEP_CTL_SNAK;
	USB->DOEP[1].CTL = USB_DOEP_CTL_SNAK;
	USB->DOEP[2].CTL = USB_DOEP_CTL_SNAK;

	USB->DIEP0CTL = USB_DIEP0CTL_SNAK
	              | USB_DIEP0CTL_STALL
	              | (0 << _USB_DIEP0CTL_TXFNUM_SHIFT)
	              | USB_DIEP0CTL_EPTYPE_DEFAULT
	              | USB_DIEP0CTL_USBACTEP
	              | USB_DIEP0CTL_MPS_64B;
	USB->DIEP[0].CTL = USB_DIEP_CTL_SNAK;
	USB->DIEP[1].CTL = USB_DIEP_CTL_SNAK;
	USB->DIEP[2].CTL = USB_DIEP_CTL_SNAK;

	usb_connect();

	/* enable interrupt */
	NVIC_EnableIRQ(USB_IRQn);
}

static void
usb_reset(void)
{
	if (reboot_on_reset)
		reboot();

	/* configure DMA for endpoint 0 */
	usb_ep0out_prepare_setup(&ep0buf);

	/* stop all endpoints */
	USB->DOEP[0].CTL = USB_DOEP_CTL_SNAK;
	USB->DOEP[1].CTL = USB_DOEP_CTL_SNAK;
	USB->DOEP[2].CTL = USB_DOEP_CTL_SNAK;

	usb_ep0in_stall();
	USB->DIEP[0].CTL = USB_DIEP_CTL_SNAK;
	USB->DIEP[1].CTL = USB_DIEP_CTL_SNAK;
	USB->DIEP[2].CTL = USB_DIEP_CTL_SNAK;

	/* enable interrupts for endpoint 0 only */
	USB->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
	USB->DOEPMSK = USB_DOEPMSK_SETUPMSK
	             | USB_DOEPMSK_XFERCOMPLMSK;
	USB->DIEPMSK = USB_DIEPMSK_XFERCOMPLMSK
	             | USB_DIEPMSK_TIMEOUTMSK;

	ep0state.state = USB_EP0_SETUP;
	ep0state.bytes = 0;

#if 0
	/* flush fifos */
	usb_fifo_flush();
	while (usb_fifo_flushing())
		/* wait */;
#endif
}

static void
usb_enumdone(void)
{
	switch (usb_enumerated_speed()) {
	case 0:
	case 1:
		debug("\r\nERROR!\r\n");
		return;
	case 2:
		debug("low speed.. ");
#if 0
		USB->DOEP0CTL = USB_DOEP0CTL_EPENA
		              | USB_DOEP0CTL_SNAK
		              | USB_DOEP0CTL_USBACTEP
		              | USB_DOEP0CTL_MPS_8B;
		USB->DIEP0CTL = USB_DIEP0CTL_SNAK
		              | USB_DIEP0CTL_STALL
		              | (0 << _USB_DIEP0CTL_TXFNUM_SHIFT)
		              | USB_DIEP0CTL_EPTYPE_DEFAULT
		              | USB_DIEP0CTL_USBACTEP
		              | USB_DIEP0CTL_MPS_8B;

		ep0state.state = USB_EP0_SETUP;
#endif
		ep0state.packetsize = 8;
		break;
	case 3:
		debug("full speed.. ");
#if 0
		USB->DOEP0CTL = USB_DOEP0CTL_EPENA
		              | USB_DOEP0CTL_SNAK
		              | USB_DOEP0CTL_USBACTEP
		              | USB_DOEP0CTL_MPS_64B;
		USB->DIEP0CTL = USB_DIEP0CTL_SNAK
		              | USB_DIEP0CTL_STALL
		              | (0 << _USB_DIEP0CTL_TXFNUM_SHIFT)
		              | USB_DIEP0CTL_EPTYPE_DEFAULT
		              | USB_DIEP0CTL_USBACTEP
		              | USB_DIEP0CTL_MPS_64B;

		ep0state.state = USB_EP0_SETUP;
#endif
		ep0state.packetsize = 64;
		break;
	}
}

static int
usb_handle_set_address(const struct usb_packet_setup *p, const void **data)
{
	debug("SET_ADDRESS: wValue = %hu\r\n", p->wValue);
	reboot_on_reset = true;
	usb_set_address(p->wValue);
	return 0;
}

static int
usb_handle_get_descriptor_device(const void **data, uint8_t index)
{
	debug("GET_DESCRIPTOR: device\r\n");
	if (index != 0) {
		debug("GET_DESCRIPTOR: type = 0x01, but index = 0x%02x\r\n", index);
		return -1;
	}
	*data = &usb_descriptor_device;
	return sizeof(usb_descriptor_device);
}

static int
usb_handle_get_descriptor_configuration(const void **data, uint8_t index)
{
	const struct usb_descriptor_configuration *desc;

	debug("GET_DESCRIPTOR: configuration %hu\r\n", index);
	if (index >= ARRAY_SIZE(usb_descriptor_configuration)) {
		debug("GET_DESCRIPTOR: unknown configuration %hu\r\n", index);
		return -1;
	}
	desc = usb_descriptor_configuration[index];
	*data = desc;
	return desc->wTotalLength;
}

static int
usb_handle_get_descriptor_string(const void **data, uint8_t index)
{
	const struct usb_descriptor_string *desc;

	debug("GET_DESCRIPTOR: string %hu\r\n", index);
	if (index >= ARRAY_SIZE(usb_descriptor_string)) {
		debug("GET_DESCRIPTOR: unknown string %hu\r\n", index);
		return -1;
	}
	desc = usb_descriptor_string[index];
	*data = desc;
	return desc->bLength;
}

static int
usb_handle_get_descriptor(const struct usb_packet_setup *p, const void **data)
{
	uint8_t type = p->wValue >> 8;
	uint8_t index = p->wValue & 0xFFU;

	switch (type) {
	case 0x01:
		return usb_handle_get_descriptor_device(data, index);
	case 0x02:
		return usb_handle_get_descriptor_configuration(data, index);
	case 0x03:
		return usb_handle_get_descriptor_string(data, index);
	case 0x06: /* DEVICE QUALIFIER (for high-speed) */
		debug("DEVICE_QUALIFIER\r\n");
		break;
	default:
		debug("GET_DESCRIPTOR: unknown type 0x%02x\r\n", type);
		break;
	}
	return -1;
}

static int
usb_handle_get_configuration(const struct usb_packet_setup *p, const void **data)
{
	debug("GET_CONFIGURATION\r\n");
	*data = &usb_descriptor_configuration[0]->bConfigurationValue;
	return 1;
}

static int
usb_handle_set_configuration(const struct usb_packet_setup *p, const void **data)
{
	debug("SET_CONFIGURATION: wValue = %hu\r\n", p->wValue);

	USB->DAINTMSK |= USB_DAINTMSK_INEPMSK1 | USB_DAINTMSK_OUTEPMSK1;
	/* configure endpoint 1 */
	USB->DIEP[0].CTL = USB_DIEP_CTL_SNAK
	                 | (1 << _USB_DIEP_CTL_TXFNUM_SHIFT)
	                 | USB_DIEP_CTL_EPTYPE_BULK
	                 | USB_DIEP_CTL_USBACTEP
	                 | USB_EP1_PACKETSIZE;
	usb_ep_out_dma_address_set(0, &usbms_buf);
	usb_ep_out_transfer_size(0, 1, USB_EP1_PACKETSIZE);
	USB->DOEP[0].CTL = USB_DOEP_CTL_EPENA
	                 | USB_DOEP_CTL_CNAK
	                 | USB_DOEP_CTL_EPTYPE_BULK
	                 | USB_DOEP_CTL_USBACTEP
	                 | USB_EP1_PACKETSIZE;
	return 0;
}

static int
usb_handle_clear_features(const struct usb_packet_setup *p, const void **data)
{
	debug("CLEAR_FEATURES\r\n");
	return 0;
}

static int
usb_handle_mass_storage_reset(const struct usb_packet_setup *p, const void **data)
{
	debug("MASS_STORAGE_RESET\r\n");
	return 0;
}

static int
usb_handle_get_max_lun(const struct usb_packet_setup *p, const void **data)
{
	static const uint16_t max_lun = 0;

	debug("GET_MAX_LUN\r\n");
	*data = &max_lun;
	return sizeof(max_lun);
}

enum dfu_status {
	DFU_OK,
	DFU_errTARGET,
	DFU_errFILE,
	DFU_errWRITE,
	DFU_errERASE,
	DFU_errCHECK_ERASED,
	DFU_errPROG,
	DFU_errVERIFY,
	DFU_errADDRESS,
	DFU_errNOTDONE,
	DFU_errFIRMWARE,
	DFU_errVENDOR,
	DFU_errUSBR,
	DFU_errPOR,
	DFU_errUNKNOWN,
	DFU_errSTALLEDPKT,
};

enum dfu_state {
	DFU_appIDLE,
	DFU_appDETACH,
	DFU_dfuIDLE,
	DFU_dfuDNLOAD_SYNC,
	DFU_dfuDNBUSY,
	DFU_dfuDNLOAD_IDLE,
	DFU_dfuMANIFEST_SYNC,
	DFU_dfuMANIFEST,
	DFU_dfuMANIFEST_WAIT_RESET,
	DFU_dfuUPLOAD_IDLE,
	DFU_dfuERROR,
};

static struct {
	uint8_t bStatus;
	uint8_t bwPollTimeout[3];
	uint8_t bState;
	uint8_t iString;
} dfu_status;

/*
static int
usb_handle_dfu_detach(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_DETACH: wValue = %hu, wIndex = %hu, wLength = %hu\r\n",
			p->wValue, p->wIndex, p->wLength);
	return 0;
}
*/

static int
usb_handle_dfu_dnload(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_DNLOAD: wValue = %hu, wIndex = %hu, wLength = %hu\r\n",
			p->wValue, p->wIndex, p->wLength);

	if (p->wLength == 0) {
		dfu_status.bState = DFU_dfuIDLE;
		return 0;
	}

	if (p->wValue >= (FLASH_TOTAL_PAGES - FLASH_PAGE_OFFSET)) {
		dfu_status.bStatus = DFU_errADDRESS;
		dfu_status.bState = DFU_dfuERROR;
		return 0;
	}

	debug("flashing page %u\r\n", FLASH_PAGE_OFFSET + p->wValue);
	if (flash_write_page(*data, FLASH_PAGE_OFFSET + p->wValue)) {
		dfu_status.bStatus = DFU_errWRITE;
		dfu_status.bState = DFU_dfuERROR;
		return 0;
	}

	dfu_status.bState = DFU_dfuDNLOAD_IDLE;
	return 0;
}

#ifdef DFU_UPLOAD
static int
usb_handle_dfu_upload(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_UPLOAD: wValue = %hu, wIndex = %hu, wLength = %hu\r\n",
			p->wValue, p->wIndex, p->wLength);

	if (p->wValue < (FLASH_TOTAL_PAGES - FLASH_PAGE_OFFSET)) {
		dfu_status.bState = DFU_dfuUPLOAD_IDLE;
		*data = FLASH_ADDRESS + (FLASH_PAGE_OFFSET + p->wValue) * FLASH_PAGE_SIZE;
		return FLASH_PAGE_SIZE;
	}

	dfu_status.bState = DFU_dfuIDLE;
	return 0;
}
#endif

static int
usb_handle_dfu_getstatus(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_GETSTATUS\r\n");
	*data = &dfu_status;
	return sizeof(dfu_status);
}

static int
usb_handle_dfu_clrstatus(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_CLRSTATUS\r\n");
	dfu_status.bStatus = DFU_OK;
	dfu_status.bState = DFU_dfuIDLE;
	return 0;
}

static int
usb_handle_dfu_getstate(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_GETSTATE\r\n");
	*data = &dfu_status.bState;
	return 1;
}

static int
usb_handle_dfu_abort(const struct usb_packet_setup *p, const void **data)
{
	debug("DFU_ABORT\r\n");
	return 0;
}

struct usb_setup_handler {
	uint16_t req;
	int (*fn)(const struct usb_packet_setup *p, const void **data);
};

static const struct usb_setup_handler usb_setup_handlers[] = {
	{ .req = 0x0500, .fn = usb_handle_set_address },
	{ .req = 0x0680, .fn = usb_handle_get_descriptor },
	{ .req = 0x0880, .fn = usb_handle_get_configuration },
	{ .req = 0x0900, .fn = usb_handle_set_configuration },
	{ .req = 0x0102, .fn = usb_handle_clear_features },
	{ .req = 0xFF21, .fn = usb_handle_mass_storage_reset },
	{ .req = 0xFEA1, .fn = usb_handle_get_max_lun, },
	/* { .req = 0x0021, .fn = usb_handle_dfu_detach }, */
	{ .req = 0x0121, .fn = usb_handle_dfu_dnload },
#ifdef DFU_UPLOAD
	{ .req = 0x02a1, .fn = usb_handle_dfu_upload },
#endif
	{ .req = 0x03a1, .fn = usb_handle_dfu_getstatus },
	{ .req = 0x0421, .fn = usb_handle_dfu_clrstatus },
	{ .req = 0x05a1, .fn = usb_handle_dfu_getstate },
	{ .req = 0x0621, .fn = usb_handle_dfu_abort },
};

static int
usb_setup_handler_run(const struct usb_packet_setup *p, const void **data)
{
	const struct usb_setup_handler *h = usb_setup_handlers;
	const struct usb_setup_handler *end = h + ARRAY_SIZE(usb_setup_handlers);

	for (; h < end; h++) {
		if (h->req == p->request)
			return h->fn(p, data);
	}

	debug("unknown request 0x%04x\r\n", p->request);
	dumpsetup(p);
	return -1;
}

static void
usb_handle_setup(void)
{
	const struct usb_packet_setup *p;

	p = (struct usb_packet_setup *)
	    (usb_ep0out_dma_address() - sizeof(struct usb_packet_setup));

	if (p->bmRequestType & 0x80U) {
		const void *data;
		int ret = usb_setup_handler_run(p, &data);

		if (ret >= 0) {
			/* send IN data */
			if (ret > p->wLength)
				ret = p->wLength;
			ep0state.bytes = ret;
			if ((uint32_t)ret > ep0state.packetsize)
				ret = ep0state.packetsize;
			usb_ep0in_dma_address_set(data);
			usb_ep0in_transfer_size(1, ret);
			usb_ep0in_enable();
			/* prepare for IN ack */
			usb_ep0out_dma_address_set(&ep0buf);
			usb_ep0out_transfer_size(1, ep0state.packetsize);
			usb_ep0out_enable();
			ep0state.state = USB_EP0_IN;
			return;
		}
	} else if (p->wLength == 0) {
		const void *data;
		int ret = usb_setup_handler_run(p, &data);

		if (ret >= 0) {
			/* prepare for next SETUP package */
			usb_ep0out_prepare_setup(&ep0buf);
			/* send empty ack package */
			usb_ep0in_transfer_size(1, 0);
			usb_ep0in_enable();
			ep0state.state = USB_EP0_OUT_ACK;
			return;
		}
	} else if (p->wLength <= sizeof(ep0buf) - 4*sizeof(struct usb_packet_setup)) {
		uint32_t *rp = (uint32_t *)p;

		if (rp != &ep0buf[0]) {
			ep0buf[0] = rp[0];
			ep0buf[1] = rp[1];
		}

		/* receive OUT data */
		usb_ep0out_dma_address_set(&ep0buf[2]);
		usb_ep0out_transfer_size(1, ep0state.packetsize);
		usb_ep0out_enable();
		ep0state.bytes = p->wLength;
		ep0state.state = USB_EP0_OUT;
		return;
	}

	/* stall IN endpoint */
	usb_ep0in_stall();
	/* prepare for next SETUP package */
	usb_ep0out_prepare_setup(&ep0buf);
	ep0state.state = USB_EP0_SETUP;
}

static void
usb_handle_ep0_in(void)
{
	uint32_t flags = usb_ep0in_flags();
	uint32_t bytes;

#if 0
	debug("EP0IN  %7s 0x%04lx OUT:0x%08lx IN:0x%08lx\r\n",
			usb_ep0_state_string[ep0state.state], flags,
			USB->DOEP0CTL, USB->DIEP0CTL);
#endif

	usb_ep0in_flags_clear(flags);

	switch (ep0state.state) {
	case USB_EP0_OUT_ACK:
		ep0state.state = USB_EP0_SETUP;
		break;
	case USB_EP0_IN:
		bytes = ep0state.bytes;
		if (bytes > ep0state.packetsize) {
			bytes -= ep0state.packetsize;
			ep0state.bytes = bytes;
			/* send next package */
			if (bytes > ep0state.packetsize)
				bytes = ep0state.packetsize;
			usb_ep0in_transfer_size(1, bytes);
			usb_ep0in_enable();
		} else
			ep0state.state = USB_EP0_IN_ACK;
		break;
	default:
		debug("EP0IN: Invalid state %s\r\n",
				usb_ep0_state_string[ep0state.state]);
		usb_flags_enable(0);
	}
}

static void
usb_handle_ep0_out(void)
{
	uint32_t flags = usb_ep0out_flags();
	uint32_t bytes;

#if 0
	debug("EP0OUT %7s 0x%04lx OUT:0x%08lx IN:0x%08lx\r\n",
			usb_ep0_state_string[ep0state.state], flags,
			USB->DOEP0CTL, USB->DIEP0CTL);
#endif

	usb_ep0out_flags_clear(flags);

	if (usb_ep0out_flag_setup(flags)) {
		usb_handle_setup();
		return;
	}

	if (!usb_ep0out_flag_complete(flags))
		return;

	switch (ep0state.state) {
	case USB_EP0_SETUP:
	case USB_EP0_OUT_ACK:
		/* SETUP flag not set yet */
		break;
	case USB_EP0_IN_ACK:
		ep0state.state = USB_EP0_SETUP;
		break;
	case USB_EP0_OUT:
		bytes = ep0state.packetsize - usb_ep0out_bytes_left();
		ep0state.bytes -= bytes;
		if (ep0state.bytes > 0) {
			/* prepare for more OUT data */
			usb_ep0out_transfer_size(1, ep0state.packetsize);
			usb_ep0out_enable();
		} else {
			const void *data = &ep0buf[2];

			if (usb_setup_handler_run((const void *)&ep0buf, &data) >= 0) {
				/* send empty ack package */
				usb_ep0in_transfer_size(1, 0);
				usb_ep0in_enable();
				ep0state.state = USB_EP0_OUT_ACK;
			} else {
				/* stall IN endpoint */
				usb_ep0in_stall();
				ep0state.state = USB_EP0_SETUP;
			}
		}
		break;
	default:
		debug("EP0OUT: Invalid state %s\r\n",
				usb_ep0_state_string[ep0state.state]);
		usb_flags_enable(0);
		break;
	}
}

static __uninitialized union {
	uint32_t word[FLASH_PAGE_SIZE/sizeof(uint32_t)];
	uint8_t byte[FLASH_PAGE_SIZE];
} pagebuf;
static union {
	uint32_t word[USBMS_BLOCKSIZE/sizeof(uint32_t)];
	uint8_t byte[USBMS_BLOCKSIZE];
} fat12;
static union {
	uint32_t word[USBMS_BLOCKSIZE/sizeof(uint32_t)];
	struct fat12_directory_entry entry[USBMS_BLOCKSIZE/32];
} rootentries;

static unsigned int fat12_codebin_entry;

static unsigned int
fat12_get(unsigned int cn)
{
	unsigned int idx = (3*cn) >> 1;
	unsigned int a = fat12.byte[idx];
	unsigned int b = fat12.byte[idx+1];

	if (cn & 1)
		return (a >> 4) | (b << 4);

	return  a | ((b & 0x0F) << 8);
}

#if 0
static void
fat12_put(unsigned int cn, unsigned int next)
{
	unsigned int idx = (3*cn) >> 1;

	if (cn & 1) {
		fat12.byte[idx] = (fat12.byte[idx] & 0x0F) | ((next & 0x0F) << 4);
		fat12.byte[idx+1] = next >> 4;
	} else {
		fat12.byte[idx++] = next & 0xFF;
		fat12.byte[idx] = (fat12.byte[idx] & 0xF0) | (next >> 8);
	}
}
#endif

static int
fat12_getpage(unsigned int first, unsigned int cn)
{
	int page = FLASH_PAGE_OFFSET;
	unsigned int i;

	for (i = first; i < PART_BLOCKS; i = fat12_get(i)) {
		if (i == cn)
			return page;
		page++;
	}

	return -1;
}

static int
usbms_handle_test_unit_ready(const struct command_block_wrapper *cbw)
{
	debug("SCSI: test unit ready, reboot_flag=%u\r\n", reboot_flag);
	if (reboot_flag > 0) {
		reboot_flag++;
		return 1;
	}
	return 0;
}

static int
usbms_handle_request_sense(const struct command_block_wrapper *cbw)
{
	debug("SCSI: request sense\r\n");
	usbms.data = &sense_data;
	usbms.bytes = usbms.chunk = sense_data.additional_sense_length + 8;
	return 0;
}

static int
usbms_handle_inquiry(const struct command_block_wrapper *cbw)
{
	debug("SCSI: inquiry\r\n");
	usbms.data = &inquiry_data;
	usbms.bytes = usbms.chunk = inquiry_data.additional_length + 5;
	return 0;
}

static int
usbms_handle_mode_sense6(const struct command_block_wrapper *cbw)
{
	debug("SCSI: mode sense(6) len=%lu\r\n", cbw->dCBWDataTransferLength);
	usbms.data = &mode_sense_data6;
	usbms.bytes = usbms.chunk = mode_sense_data6.mode_data_length + 1;
	return 0;
}

static int
usbms_handle_start_stop_unit(const struct command_block_wrapper *cbw)
{
	debug("SCSI: start stop unit CBWCB[4]=0x%02x CBWCB[5]=0x%02x\r\n",
			cbw->CBWCB[4], cbw->CBWCB[5]);
	if (cbw->CBWCB[4] == 0x02) {
		reboot_flag = 1;
		return 0;
	}
	return 1;
}

static int
usbms_handle_prevent_allow_medium_removal(const struct command_block_wrapper *cbw)
{
	debug("SCSI: prevent allow medium removal\r\n");
	return 1;
}

#ifdef USBMS_READ_FORMAT_CAPACITY
/* this is required Windows */
static int
usbms_handle_read_format_capacity(const struct command_block_wrapper *cbw)
{
	debug("SCSI: read format capacity\r\n");
	usbms.data = &format_capacity_data;
	usbms.bytes = usbms.chunk = format_capacity_data.capacity_list_length + 4;
	return 0;
}
#endif

static int
usbms_handle_read_capacity(const struct command_block_wrapper *cbw)
{
	static const struct {
		uint32_t lba;
		uint32_t blocksize;
	} capacity = {
		.lba = BE_UINT32(USBMS_BLOCKS - 1U),
		.blocksize = BE_UINT32(USBMS_BLOCKSIZE),
	};

	debug("SCSI: read capacity\r\n");
	usbms.data = &capacity;
	usbms.bytes = usbms.chunk = sizeof(capacity);
	if (fat12_codebin_entry > 0)
		reboot_flag = 1;
	return 0;
}

static void
usbms_read(void)
{
	unsigned int i;

	if (usbms.lba == PART_FIRST_BLOCK+1) {
		usbms.data = &fat12;
		usbms.chunk = USBMS_BLOCKSIZE;
		return;
	}
	if (usbms.lba == PART_FIRST_BLOCK+2) {
		usbms.data = &rootentries;
		usbms.chunk = USBMS_BLOCKSIZE;
		return;
	}

	for (i = 0; i < ARRAY_SIZE(pagebuf.word); i++)
		pagebuf.word[i] = 0;

	switch (usbms.lba) {
	case 0:
		pagebuf.word[440/4] = 0xCAFEBABE;
		pagebuf.word[444/4] = B2W(0x00, 0x00, 0x80, 0xFE);
		pagebuf.word[448/4] = B2W(0xFF, 0xFF, 0x01, 0xFE);
		pagebuf.word[452/4] = B2W(0xFF, 0xFF, PART_FIRST_BLOCK & 0xFF, (PART_FIRST_BLOCK>>8) & 0xFF);
		pagebuf.word[456/4] = B2W((PART_FIRST_BLOCK>>16) & 0xFF, PART_FIRST_BLOCK>>24,
						PART_BLOCKS & 0xFF, (PART_BLOCKS>>8) & 0xFF);
		pagebuf.word[460/4] = B2W((PART_BLOCKS>>16) & 0xFF, PART_BLOCKS>>24, 0x00, 0x00);

		pagebuf.word[508/4] = B2W(0x00, 0x00, 0x55, 0xAA);
		break;
	case PART_FIRST_BLOCK:
		pagebuf.word[  0/4] = B2W(0xEB, 0x00, 0x90, 0x00);
		/* sector size, sectors pr. cluster, reserved sectors */
		pagebuf.word[  8/4] = B2W(0, 0, 0, USBMS_BLOCKSIZE & 0xFF);
		pagebuf.word[ 12/4] = B2W(USBMS_BLOCKSIZE >> 8, 1, 1, 0);
		/* number of FATs, root directory entries, total sectors */
		pagebuf.word[ 16/4] = B2W(1, 32, 0, PART_BLOCKS & 0xFF);
		/* total sectors, media descriptor, sectors pr. FAT */
		pagebuf.word[ 20/4] = B2W(PART_BLOCKS >> 8, 0xF8, 1, 0);
		/* extended boot signature, volume id, volume label, filesystem type */
		pagebuf.word[ 36/4] = B2W(0, 0, 0x29, 0xBE);
		pagebuf.word[ 40/4] = B2W(0xBA, 0xFE, 0xCA, 'G');
		pagebuf.word[ 44/4] = B2W('E', 'C', 'K', 'O');
		pagebuf.word[ 48/4] = B2W('B', 'O', 'O', 'T');
		pagebuf.word[ 52/4] = B2W(' ', ' ', 'F', 'A');
		pagebuf.word[ 56/4] = B2W('T', '1', '2', ' ');
		pagebuf.word[ 60/4] = B2W(' ', ' ', 0, 0);
		/* boot signature */
		pagebuf.word[508/4] = B2W(0, 0, 0x55, 0xAA);

		if (fat12_codebin_entry > 0)
			reboot_flag = 1;
		break;
	}

	usbms.data = &pagebuf;
	usbms.chunk = USBMS_BLOCKSIZE;
}

static int
usbms_handle_read10(const struct command_block_wrapper *cbw)
{
	uint32_t lba = (((uint32_t)cbw->CBWCB[2]) << 24)
	             | (((uint32_t)cbw->CBWCB[3]) << 16)
	             | (((uint32_t)cbw->CBWCB[4]) << 8)
	             |  ((uint32_t)cbw->CBWCB[5]);
	uint32_t len = (((uint32_t)cbw->CBWCB[7]) << 8) | ((uint32_t)cbw->CBWCB[8]);

	debug("SCSI: read(10) lba=%lu, len=%lu\r\n", lba, len);
	usbms.bytes = len * USBMS_BLOCKSIZE;
	usbms.lba = lba;
	usbms_read();
	return 0;
}

static void
usbms_write_receive(void)
{
	switch (usbms.lba) {
	case PART_FIRST_BLOCK+1:
		usbms.buf = &fat12;
		usbms.chunk = USBMS_BLOCKSIZE;
		break;
	case PART_FIRST_BLOCK+2:
		usbms.buf = &rootentries;
		usbms.chunk = USBMS_BLOCKSIZE;
		break;
	default:
		usbms.buf = &pagebuf;
		if (usbms.bytes < sizeof(pagebuf))
			usbms.chunk = usbms.bytes;
		else
			usbms.chunk = sizeof(pagebuf);
	}
}

#ifndef NDEBUG
static void
find_file(unsigned int cn)
{
	struct fat12_directory_entry *entry;

	for (entry = &rootentries.entry[0]; entry->name[0]; entry++) {
		if (entry->start > 0 && entry->start < PART_BLOCKS) {
			if (fat12_getpage(entry->start, cn) > 0) {
				printf("  that's %.8s.%.3s\r\n",
						entry->name, entry->ext);
				return;
			}
		}
	}

	printf("  not found\r\n");
}
#else
static inline void find_file(unsigned int cn) {}
#endif

static void
usbms_write_received(void)
{
#ifndef NDEBUG
	const char *extra;

	if (usbms.lba == PART_FIRST_BLOCK)
		extra = " (partbuf0)";
	else if (usbms.lba == PART_FIRST_BLOCK+1)
		extra = " (fat12)";
	else if (usbms.lba == PART_FIRST_BLOCK+2)
		extra = " (rootentries0)";
	else
		extra = "";

	debug("received lba %lu%s\r\n", usbms.lba, extra);
#endif
	if (usbms.lba == PART_FIRST_BLOCK) {
		if ((pagebuf.byte[0x25] & 0x1) == 0)
			reboot_flag = 1;
		return;
	}

	if (usbms.lba == PART_FIRST_BLOCK+2) {
		struct fat12_directory_entry *entry;

		for (entry = &rootentries.entry[0]; entry->name[0]; entry++) {
#ifndef NDEBUG
			if (entry->name[0] != 0x05 &&
			    entry->name[0] != 0xE5 &&
			    entry->attrs != 0x0F)
				printf("  %.8s.%.3s start:%hu size:%lu\r\n",
						entry->name, entry->ext,
						entry->start, entry->size);
#endif
			if (entry->ext[0] == 'B' &&
					entry->ext[1] == 'I' &&
					entry->ext[2] == 'N') {
				fat12_codebin_entry = entry->start;
				return;
			}
		}
		fat12_codebin_entry = 0;
		return;
	}

	if (usbms.lba > PART_FIRST_BLOCK+2) {
		unsigned int cn = usbms.lba - PART_FIRST_BLOCK - 1;
		int page;

		find_file(cn);

		if (fat12_codebin_entry == 0) {
			debug("flashing page %u\r\n", usbms.page);
			flash_write_page(pagebuf.word, usbms.page);
			usbms.page++;
			return;
		}

		page = fat12_getpage(fat12_codebin_entry, cn);
		debug("fat12_getpage(%u) = %d\r\n", cn, page);
		if (page >= 0) {
			debug("flashing page %u\r\n", page);
			flash_write_page(pagebuf.word, page);
			return;
		}
	}
}

static int
usbms_handle_write10(const struct command_block_wrapper *cbw)
{
	uint32_t lba = (((uint32_t)cbw->CBWCB[2]) << 24)
	             | (((uint32_t)cbw->CBWCB[3]) << 16)
	             | (((uint32_t)cbw->CBWCB[4]) << 8)
	             |  ((uint32_t)cbw->CBWCB[5]);
	uint32_t len = (((uint32_t)cbw->CBWCB[7]) << 8) | ((uint32_t)cbw->CBWCB[8]);

	debug("SCSI: write(10) lba=%lu, len=%lu\r\n", lba, len);
	usbms.lba = lba;
	usbms.bytes = len * USBMS_BLOCKSIZE;
	usbms.page = FLASH_PAGE_OFFSET;
	usbms_write_receive();
	return 0;
}

#ifdef USBMS_REPORT_LUNS
/* this is only used by FreeBSD with inquiry_data.version == 0x05 */
static int
usbms_handle_report_luns(const struct command_block_wrapper *cbw)
{
	static const struct {
		uint32_t list_length;
		uint8_t reserved[4];
		uint8_t lun_list[1][8];
	} lun_list = {
		.list_length = BE_UINT32(8),
		.reserved    = { 0x00, 0x00, 0x00, 0x00 },
		.lun_list    = {
			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		}
	};

	debug("SCSI: report luns\r\n");
	usbms.data = &lun_list;
	usbms.bytes = usbms.chunk = sizeof(lun_list);
	return 0;
}
#endif

struct usbms_cbw_handler {
	uint8_t opcode;
	uint8_t type;
	int (*fn)(const struct command_block_wrapper *cbw);
};

static const struct usbms_cbw_handler usbms_cbw_handlers[] = {
	{ 0x00, 0, usbms_handle_test_unit_ready },
	{ 0x03, 1, usbms_handle_request_sense },
	{ 0x12, 1, usbms_handle_inquiry },
	{ 0x1A, 1, usbms_handle_mode_sense6 },
	{ 0x1B, 0, usbms_handle_start_stop_unit },
	{ 0x1E, 0, usbms_handle_prevent_allow_medium_removal },
#ifdef USBMS_READ_FORMAT_CAPACITY
	{ 0x23, 1, usbms_handle_read_format_capacity },
#endif
	{ 0x25, 1, usbms_handle_read_capacity },
	{ 0x28, 1, usbms_handle_read10 },
	{ 0x2A, 2, usbms_handle_write10 },
#ifdef USBMS_REPORT_LUNS
	{ 0xA0, 1, usbms_handle_report_luns },
#endif
};

static enum usbms_state
usbms_handle_cbw(void)
{
	const struct usbms_cbw_handler *h = usbms_cbw_handlers;
	const struct usbms_cbw_handler *end = h + ARRAY_SIZE(usbms_cbw_handlers);
	uint8_t type = (usbms_buf.cbw.dCBWDataTransferLength == 0) ? 0
	             : (usbms_buf.cbw.bmCBWFlags & 0x80U) ? 1
	             : 2;
	uint8_t ret;

	for (; h < end; h++) {
		if (h->opcode == usbms_buf.cbw.CBWCB[0] && h->type == type) {
			ret = h->fn(&usbms_buf.cbw);
			goto found;
		}
	}
	debug("unknown request %hu type %u\r\n", usbms_buf.cbw.CBWCB[0], type);
	dumpcbw(&usbms_buf.cbw);
	ret = 0x01;

found:
	usbms_buf.csw.dCSWSignature = 0x53425355;
	usbms_buf.csw.bCSWStatus = ret;

	switch (type) {
	case 0:
		return USBMS_STATE_CSW;
	case 1:
		if (usbms.bytes > usbms_buf.csw.dCSWDataResidue)
			usbms.bytes = usbms_buf.csw.dCSWDataResidue;
		if (usbms.chunk > usbms.bytes)
			usbms.chunk = usbms.bytes;
		return USBMS_STATE_DATA_IN;
	case 2:
		usbms.bytes = usbms_buf.csw.dCSWDataResidue;
		return USBMS_STATE_DATA_OUT;
	default:
		__unreachable();
	}
}

static void
usb_handle_ep1_in(void)
{
	uint32_t flags = usb_ep_in_flags(0);
	uint32_t packets;

	usb_ep_in_flags_clear(0, flags);

	usbms.bytes -= usbms.chunk;
	usbms_buf.csw.dCSWDataResidue -= usbms.chunk;

	switch (usbms.state) {
	case USBMS_STATE_DATA_IN:
		if (usbms.bytes == 0) {
			usbms.state = USBMS_STATE_CSW;
			usbms.data = &usbms_buf.csw;
			usbms.bytes = usbms.chunk = 13;
		} else {
			usbms.lba++;
			usbms_read();
		}
		/* fallthrough */
	case USBMS_STATE_CSW:
		if (usbms.bytes == 0) {
			if (reboot_flag > 2)
				reboot();
			usbms.state = USBMS_STATE_CBW;
			usb_ep_out_dma_address_set(0, &usbms_buf);
			usb_ep_out_transfer_size(0, 1, USB_EP1_PACKETSIZE);
			usb_ep_out_enable(0);
			break;
		}
		packets = (usbms.chunk + USB_EP1_PACKETSIZE - 1) / USB_EP1_PACKETSIZE;
		usb_ep_in_dma_address_set(0, usbms.data);
		usb_ep_in_transfer_size(0, packets, usbms.chunk);
		usb_ep_in_enable(0);
		break;
	default:
		debug("TX: error %hu\r\n", usbms.state);
		usbms.state = USBMS_STATE_ERROR;
		usb_ep_in_stall(0);
	}
}

static void
usb_handle_ep1_out(void)
{
	uint32_t flags = usb_ep_out_flags(0);
	uint32_t len;
	uint32_t packets;

	usb_ep_out_flags_clear(0, flags);

	switch (usbms.state) {
	case USBMS_STATE_CBW:
		len = USB_EP1_PACKETSIZE - (USB->DOEP[0].TSIZ & _USB_DOEP_TSIZ_XFERSIZE_MASK);
		if (len == 31 && usbms_buf.word[0] == 0x43425355)
			usbms.state = usbms_handle_cbw();
		else
			usbms.state = USBMS_STATE_ERROR;
		break;
	case USBMS_STATE_DATA_OUT:
		if (USB->DOEP[0].TSIZ) {
			usbms.state = USBMS_STATE_ERROR;
			break;
		}
		usbms.bytes -= usbms.chunk;
		usbms_buf.csw.dCSWDataResidue -= usbms.chunk;
		usbms_write_received();
		usbms.lba++;

		if (usbms.bytes == 0) {
			usbms.state = USBMS_STATE_CSW;
			usbms.data = &usbms_buf.csw;
			usbms.bytes = usbms.chunk = 13;
		} else
			usbms_write_receive();
		break;
	default:
		usbms.state = USBMS_STATE_ERROR;
	}

	switch (usbms.state) {
	case USBMS_STATE_DATA_OUT:
		packets = (usbms.chunk + USB_EP1_PACKETSIZE - 1) / USB_EP1_PACKETSIZE;
		usb_ep_out_dma_address_set(0, usbms.buf);
		usb_ep_out_transfer_size(0, packets, usbms.chunk);
		usb_ep_out_enable(0);
		break;
	case USBMS_STATE_CSW:
		usbms.data = &usbms_buf.csw;
		usbms.bytes = usbms.chunk = 13;
		/* fallthrough */
	case USBMS_STATE_DATA_IN:
		packets = (usbms.chunk + USB_EP1_PACKETSIZE - 1) / USB_EP1_PACKETSIZE;
		usb_ep_in_dma_address_set(0, usbms.data);
		usb_ep_in_transfer_size(0, packets, usbms.chunk);
		usb_ep_in_enable(0);
		break;
	default:
		debug("RX: error %hu\r\n", usbms.state);
		usbms.state = USBMS_STATE_ERROR;
		usb_ep_in_stall(0);
		break;
	}
}

static void
usb_handle_transfer(void)
{
	uint32_t flags = usb_ep_flags();

	if (flags & USB_DAINT_INEPINT0)
		usb_handle_ep0_in();
	if (flags & USB_DAINT_OUTEPINT0)
		usb_handle_ep0_out();
	if (flags & USB_DAINT_INEPINT1)
		usb_handle_ep1_in();
	if (flags & USB_DAINT_OUTEPINT1)
		usb_handle_ep1_out();
}

void
USB_IRQHandler(void)
{
	uint32_t flags = usb_flags();

	usb_flags_clear(flags);

	gpio_clear(GPIO_PA9);

	if (usb_flag_ep(flags)) {
		usb_handle_transfer();
	}
	if (flags & (USB_GINTSTS_RESETDET | USB_GINTSTS_WKUPINT)) {
		debug("WAKEUP.. ");
		usb_unsuspend();
		debug("done\r\n");
		goto out;
	}
	if (usb_flag_suspend(flags)) {
		//if ((flags & USB_GINTSTS_USBSUSP) || (USB->DSTS & USB_DSTS_ERRTICERR)) {
		debug("SUSPEND.. ");
		usb_suspend();
		debug("done\r\n");
		goto out;
	}
	if (usb_flag_reset(flags)) {
		debug("RESET.. ");
		usb_reset();
		debug("done\r\n");
	}
	if (usb_flag_enumdone(flags)) {
		debug("ENUMDONE.. ");
		usb_enumdone();
		debug("done\r\n");
	}
out:
	gpio_set(GPIO_PA9);
}

static void
part_init(void)
{
	fat12.word[0] = B2W(0xF8, 0xFF, 0xFF, 0x00);

#if 0
	rootentries.entry[0].name[0] = 'C';
	rootentries.entry[0].name[1] = 'O';
	rootentries.entry[0].name[2] = 'D';
	rootentries.entry[0].name[3] = 'E';
	rootentries.entry[0].name[4] = ' ';
	rootentries.entry[0].name[5] = ' ';
	rootentries.entry[0].name[6] = ' ';
	rootentries.entry[0].name[7] = ' ';
	rootentries.entry[0].ext[0] = 'B';
	rootentries.entry[0].ext[1] = 'I';
	rootentries.entry[0].ext[2] = 'N';
	rootentries.entry[0].attrs = 0x24;
	rootentries.entry[0].ctime = 0x6CA0;
	rootentries.entry[0].cdate = 0x4AD4;
	rootentries.entry[0].adate = 0x4AD4;
	rootentries.entry[0].mtime = 0x6CA0;
	rootentries.entry[0].mdate = 0x4AD4;
	rootentries.entry[0].start = 3;
	rootentries.entry[0].size = (PART_BLOCKS-3)*USBMS_BLOCKSIZE;
#endif
};

#ifndef NDEBUG
static void
block_dump(uint8_t *page)
{
	unsigned int i;

	for (i = 0; i < USBMS_BLOCKSIZE; i++) {
		if (i % 16 == 0)
			printf("\r\n%5x ", i);
		else if (i % 4 == 0)
			printf(" ");
		printf("%02x", page[i]);
	}
	printf("\r\n");
}
#endif

static void
usb_init(void)
{
	/* NVIC_SetPriority(USB_IRQn, 0); */

	/* enable USB clock */
	clock_usb_enable();
#if 1
	USB->CTRL = 0;
#else
	USB->CTRL = USB_CTRL_LEMOSCCTRL_GATE
	          | USB_CTRL_LEMIDLEEN
	          | USB_CTRL_LEMPHYCTRL;
#endif
	/* enable USB PHY pins */
	usb_pins_enable();
	/* enable USB and USB core clock */
	clock_usbc_enable();
	/* make sure oscillator is ready and selected */
	clock_usbc_select_ushfrco();
	while (!clock_usbc_ushfrco_selected())
		/* wait */;
	/* enable clock recovery */
	clock_ushfrco_recovery_enable();
	/* wait for core to come out of reset */
	while (!usb_ahb_idle())
		/* wait */;

	usb_unsuspend();
}

static inline uint8_t
system_getprodrev(void)
{
	return (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK)
		>> _DEVINFO_PART_PROD_REV_SHIFT;
}

void __noreturn
main(void)
{
#if 0
	if (system_getprodrev() <= 129) {
		/* This fixes a mistaken internal connection between PC0 and PC4 */
		/* This disables an internal pulldown on PC4 */
		*(volatile uint32_t*)(0x400C6018) = (1 << 26) | (5 << 0);
		/* This disables an internal LDO test signal driving PC4 */
		*(volatile uint32_t*)(0x400C80E4) &= ~(1 << 24);
	}
#endif

	/* switch to 48MHz / 2 ushfrco as core clock */
	clock_ushfrco_48mhz_div2();
	clock_ushfrco_enable();
	while (!clock_ushfrco_ready())
		/* wait */;
	clock_hfclk_select_ushfrco();
	while (!clock_ushfrco_selected())
		/* wait */;
	clock_lfrco_enable();
	clock_hfrco_disable();
	clock_auxhfrco_enable();

#ifdef NDEBUG
	clock_lf_config(CLOCK_LFA_DISABLED | CLOCK_LFB_DISABLED | CLOCK_LFC_LFRCO);
	clock_usble_enable();
#else
	/* route 24MHz core clock / 2 / 8 to LEUART0 */
	//clock_le_div2();
	clock_le_enable();
	clock_lf_config(CLOCK_LFA_DISABLED | CLOCK_LFB_CORECLK | CLOCK_LFC_LFRCO);
	clock_leuart0_div8();
	clock_leuart0_enable();
	clock_usble_enable();
#endif
	while (clock_lf_syncbusy())
		/* wait */;

	/* enable and configure GPIOs */
	clock_gpio_enable();
	gpio_set(GPIO_PA8);
	gpio_set(GPIO_PA9);
	//gpio_set(GPIO_PA10);
	gpio_mode(GPIO_PA8,  GPIO_MODE_WIREDAND); /* red   LED */
	gpio_mode(GPIO_PA9,  GPIO_MODE_WIREDAND); /* green LED */
	//gpio_mode(GPIO_PA10, GPIO_MODE_WIREDAND); /* blue  LED */
#ifndef NDEBUG
	gpio_mode(GPIO_PD4, GPIO_MODE_PUSHPULL); /* LEUART0 TX */
	gpio_mode(GPIO_PD5, GPIO_MODE_INPUT);    /* LEUART0 RX */

	leuart0_freeze();
	leuart0_config(LEUART_CONFIG_8N1);
	leuart0_clock_div(3077); /* 256*(f/115200 - 1), f = 24MHz / 2 / 8 */
	leuart0_pins(LEUART_PINS_RXTX0); /* RX -> PD5, TX -> PD4 */
	leuart0_rxtx_enable();
	leuart0_update();
	while (leuart0_syncbusy())
		/* wait */;

	/* enable interrupt */
	//NVIC_SetPriority(LEUART0_IRQn, 0);
	NVIC_EnableIRQ(LEUART0_IRQn);
#endif

	part_init();
	dfu_status.bState = DFU_dfuIDLE;

	gpio_clear(GPIO_PA9);
	usb_init();
	gpio_set(GPIO_PA9);

#ifndef NDEBUG
	while (1) {
		while (!leuart0_rxdata_valid())
			/* wait */;
		if (leuart0_rxdata() == 'p')
			block_dump(fat12.byte);
	}
#else
	while (1) {
		__WFI();
	}
#endif
}
