#include <stdlib.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/*
 * Table B-1: MIDI Adapter Device Descriptor
 */
static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0110,
	.bDeviceClass = 0,	/* device defined at interface level */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x6666,	/* Prototype product vendor ID */
	.idProduct = 0x4649,	/* 夜露死苦 */
	.bcdDevice = 0x0100,
	.iManufacturer = 1,	/* index to string desc */
	.iProduct = 2,		/* index to string desc */
	.iSerialNumber = 3,	/* index to string desc */
	.bNumConfigurations = 1,
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
	/*
	 * Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
}, {
	/*
	 * Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x03,
	},
}};

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
	/* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 0x40,
	.bInterval = 0x00,

	.extra = &midi_bulk_endp[0],
	.extralen = sizeof(midi_bulk_endp[0])
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 0x40,
	.bInterval = 0x00,

	.extra = &midi_bulk_endp[1],
	.extralen = sizeof(midi_bulk_endp[1])
}};

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
};

/*
 * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 */
static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &audio_control_functional_descriptors,
	.extralen = sizeof(audio_control_functional_descriptors)
}};

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_in_jack_descriptor in_embedded;
	struct usb_midi_in_jack_descriptor in_external;
	struct usb_midi_out_jack_descriptor out_embedded;
	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
	/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
	.header = {
		.bLength = sizeof(struct usb_midi_header_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
		.bcdMSC = 0x0100,
		.wTotalLength = sizeof(midi_streaming_functional_descriptors),
	},
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x01,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x02,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x03,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x02,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x04,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x01,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		},
	},
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = bulk_endp,

	.extra = &midi_streaming_functional_descriptors,
	.extralen = sizeof(midi_streaming_functional_descriptors)
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = audio_control_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_streaming_iface,
}};

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,	/* can be anything, it is updated automatically
				   when the usb code prepares the descriptor */
	.bNumInterfaces = 2,	/* control and data */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,	/* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char *usb_strings[] = {
	"libopencm3.org",
	"STM32 USB-MIDI",
	usb_serial_number
};

enum
{
	MIDI_CIN_MISC					= 0,
	MIDI_CIN_CABLE_EVENT				= 1,
	MIDI_CIN_SYSCOM_2BYTE				= 2,
	MIDI_CIN_SYSCOM_3BYTE				= 3,
	MIDI_CIN_SYSEX_START				= 4,
	MIDI_CIN_SYSEX_END_1BYTE			= 5,
	MIDI_CIN_SYSEX_END_2BYTE			= 6,
	MIDI_CIN_SYSEX_END_3BYTE			= 7,
	MIDI_CIN_NOTE_OFF				= 8,
	MIDI_CIN_NOTE_ON				= 9,
	MIDI_CIN_POLY_KEYPRESS				= 10,
	MIDI_CIN_CONTROL_CHANGE				= 11,
	MIDI_CIN_PROGRAM_CHANGE				= 12,
	MIDI_CIN_CHANNEL_PRESSURE			= 13,
	MIDI_CIN_PITCH_BEND_CHANGE			= 14,
	MIDI_CIN_1BYTE_DATA				= 15,
};

enum
{
	MIDI_STATUS_SYSEX_START				= 0xF0,
	MIDI_STATUS_SYSEX_END				= 0xF7,
	MIDI_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME	= 0xF1,
	MIDI_STATUS_SYSCOM_SONG_POSITION_POINTER	= 0xF2,
	MIDI_STATUS_SYSCOM_SONG_SELECT			= 0xF3,
	MIDI_STATUS_SYSCOM_TUNE_REQUEST			= 0xF6,
	MIDI_STATUS_SYSREAL_TIMING_CLOCK		= 0xF8,
	MIDI_STATUS_SYSREAL_START			= 0xFA,
	MIDI_STATUS_SYSREAL_CONTINUE			= 0xFB,
	MIDI_STATUS_SYSREAL_STOP			= 0xFC,
	MIDI_STATUS_SYSREAL_ACTIVE_SENSING		= 0xFE,
	MIDI_STATUS_SYSREAL_SYSTEM_RESET		= 0xFF,
};

static volatile uint8_t buf_rx[256] = { 0 };
static volatile uint8_t buf_rx_head = 0;
static volatile uint8_t buf_rx_tail = 0;
static volatile uint8_t buf_tx[256] = { 0 };
static volatile uint8_t buf_tx_head = 0;
static volatile uint8_t buf_tx_tail = 0;

static volatile uint8_t leds = 0;
#if 1 /* For bluepill board. */
#define LED_RX(x)	do { (x) ? (leds |= 0b01) : (leds &= ~0b01); leds ? gpio_clear(GPIOC, GPIO13) : gpio_set(GPIOC, GPIO13); } while (0)
#define LED_TX(x)	do { (x) ? (leds |= 0b10) : (leds &= ~0b10); leds ? gpio_clear(GPIOC, GPIO13) : gpio_set(GPIOC, GPIO13); } while (0)
#else /* For strange CKS32 board. */
#define LED_RX(x)	do { (x) ? (leds |= 0b01) : (leds &= ~0b01); leds ? gpio_set(GPIOC, GPIO13) : gpio_clear(GPIOC, GPIO13); } while (0)
#define LED_TX(x)	do { (x) ? (leds |= 0b10) : (leds &= ~0b10); leds ? gpio_set(GPIOC, GPIO13) : gpio_clear(GPIOC, GPIO13); } while (0)
#endif

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep __attribute__((unused)))
{
	uint8_t buf[64];
	uint16_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	static uint8_t status = 0;

	for (uint16_t i = 0 ; i < len ; i += 4) {
		switch (buf[i + 0] & 0x0f) {
			case MIDI_CIN_SYSCOM_3BYTE:
			case MIDI_CIN_SYSEX_START:
			case MIDI_CIN_SYSEX_END_3BYTE:
				buf_tx[buf_tx_head] = buf[i + 1];
				buf_tx_head++;
				buf_tx[buf_tx_head] = buf[i + 2];
				buf_tx_head++;
				buf_tx[buf_tx_head] = buf[i + 3];
				buf_tx_head++;
				LED_TX(1);
				USART_CR1(USART1) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_NOTE_OFF:
			case MIDI_CIN_NOTE_ON:
			case MIDI_CIN_POLY_KEYPRESS:
			case MIDI_CIN_CONTROL_CHANGE:
			case MIDI_CIN_PITCH_BEND_CHANGE:
				if (buf[i + 1] != status) { /* Running Status? */
					buf_tx[buf_tx_head] = buf[i + 1];
					buf_tx_head++;
				}
				buf_tx[buf_tx_head] = buf[i + 2];
				buf_tx_head++;
				buf_tx[buf_tx_head] = buf[i + 3];
				buf_tx_head++;
				LED_TX(1);
				USART_CR1(USART1) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_SYSCOM_2BYTE:
			case MIDI_CIN_SYSEX_END_2BYTE:
				buf_tx[buf_tx_head] = buf[i + 1];
				buf_tx_head++;
				buf_tx[buf_tx_head] = buf[i + 2];
				buf_tx_head++;
				LED_TX(1);
				USART_CR1(USART1) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_PROGRAM_CHANGE:
			case MIDI_CIN_CHANNEL_PRESSURE:
				if (buf[i + 1] != status) { /* Running Status? */
					buf_tx[buf_tx_head] = buf[i + 1];
					buf_tx_head++;
				}
				buf_tx[buf_tx_head] = buf[i + 2];
				buf_tx_head++;
				LED_TX(1);
				USART_CR1(USART1) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_SYSEX_END_1BYTE:
			case MIDI_CIN_1BYTE_DATA:
				buf_tx[buf_tx_head] = buf[i + 1];
				buf_tx_head++;
				LED_TX(1);
				USART_CR1(USART1) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_MISC:
			case MIDI_CIN_CABLE_EVENT:
			default:
				break;
		}
		if (buf[i + 1] < MIDI_STATUS_SYSREAL_TIMING_CLOCK) {
			status = buf[i + 1];
		}
	}
}

static void usbmidi_data_tx(usbd_device *usbd_dev, uint8_t data)
{
	static uint8_t buf[4] = { 0 };
	static uint8_t index = 0, total = 0;

	if (data >= MIDI_STATUS_SYSREAL_TIMING_CLOCK) { /* real-time messages need to be sent right away */
		uint8_t rtbuf[4];
		rtbuf[0] = MIDI_CIN_1BYTE_DATA;
		rtbuf[1] = data;
		rtbuf[2] = 0;
		rtbuf[3] = 0;
		while (!usbd_ep_write_packet(usbd_dev, 0x81, rtbuf, sizeof(rtbuf)));
		LED_RX(0);
	} else if (index == 0) { /* New event packet */
		uint8_t const msg = data >> 4;
		uint8_t const _msg = buf[0] & 0x0F;
		index = 2;
		total = 4;

		/* Check to see if we're still in a SysEx transmit. */
		if (_msg == MIDI_CIN_SYSEX_START) {
			if (data == MIDI_STATUS_SYSEX_END) {
				buf[0] = MIDI_CIN_SYSEX_END_1BYTE;
				total = 2;
			}
			buf[1] = data;
		} else if ((msg < 0x8) && (_msg >= 0x8) && (_msg < 0xF)) { /* Running Status? */
			buf[2] = data;
			if ((_msg < 0xC) || (_msg == 0xE)) {
				index = 3;
			} else {
				index = 3;
				total = 3;
			}
		} else if (((msg >= 0x8) && (msg <= 0xB)) || (msg == 0xE)) { /* Channel Voice Messages */
			buf[0] = msg;
			buf[1] = data;
		} else if ((msg == 0xC) || (msg == 0xD)) { /* Channel Voice Messages, two-byte variants (Program Change and Channel Pressure) */
			buf[0] = msg;
			buf[1] = data;
			total = 3;
		} else if (msg == 0xf) { /* System message */
			if (data == MIDI_STATUS_SYSEX_START) {
				buf[0] = MIDI_CIN_SYSEX_START;
			} else if (data == MIDI_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME || data == MIDI_STATUS_SYSCOM_SONG_SELECT) {
				buf[0] = MIDI_CIN_SYSCOM_2BYTE;
				total = 3;
			} else if (data == MIDI_STATUS_SYSCOM_SONG_POSITION_POINTER) {
				buf[0] = MIDI_CIN_SYSCOM_3BYTE;
			} else { /* for example, MIDI_STATUS_SYSCOM_TUNE_REQUEST */
				buf[0] = MIDI_CIN_1BYTE_DATA;
				total = 2;
			}
			buf[1] = data;
		} else { /* Pack individual bytes if we don't support packing them into words. */
			buf[0] = MIDI_CIN_1BYTE_DATA;
			buf[1] = data;
			index = 2;
			total = 2;
		}
	} else { /* On-going (buffering) packet */
		buf[index++] = data;
		/* See if this byte ends a SysEx. */
		if ((buf[0] == MIDI_CIN_SYSEX_START) && (data == MIDI_STATUS_SYSEX_END)) {
			buf[0] = MIDI_CIN_SYSEX_START + (index - 1); /* END +1/+2/+3 Bytes */
			total = index;
		}
	}

	/* Send out packet */
	if ((index >= 2) && (index >= total)) {
		/* zeroes unused bytes */
		for (uint8_t idx = total ; idx < 4 ; idx++) {
			buf[idx] = 0;
		}
		while (!usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)));
		LED_RX(0);
		index = 0;
	}
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue __attribute__((unused)))
{
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbmidi_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
}

void usart1_isr(void)
{
	/* Check if we were called because of RXNE. */
	if ((USART_CR1(USART1) & USART_CR1_RXNEIE) && (USART_SR(USART1) & USART_SR_RXNE)) {
		buf_rx[buf_rx_head] = usart_recv(USART1);
		buf_rx_head++;
		LED_RX(1);
	}

	/* Check if we were called because of TXE. */
	if ((USART_CR1(USART1) & USART_CR1_TXEIE) && (USART_SR(USART1) & USART_SR_TXE)) {
		if (buf_tx_head != buf_tx_tail) {
			usart_send(USART1, buf_tx[buf_tx_tail]);
			buf_tx_tail++;
		}
		if (buf_tx_head == buf_tx_tail) {
			USART_CR1(USART1) &= ~USART_CR1_TXEIE; /* Disable the TXE interrupt as we don't need it anymore. */
			LED_TX(0);
		}
	}
}

int main(void)
{
	usbd_device *usbd_dev;
	uint8_t usbd_control_buffer[128]; /* Buffer to be used for control requests. */

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* UART */
#if 0	/* PA9=OUT, PA10=IN */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RX);
	gpio_set(GPIOA, GPIO_USART1_RX); /* Pullup */
#else	/* PB6=OUT, PB7=IN */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	AFIO_MAPR |= AFIO_MAPR_USART1_REMAP; /* Enable USART1 pin software remapping. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RE_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RE_RX);
	gpio_set(GPIOB, GPIO_USART1_RE_RX); /* Pullup */
#endif
	rcc_periph_clock_enable(RCC_USART1);
	usart_set_baudrate(USART1, 31250);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_enable(USART1);

	nvic_enable_irq(NVIC_USART1_IRQ);
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* LED */
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	LED_RX(0);
	LED_TX(0);

	/* USB */
	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

	while(1) {
		usbd_poll(usbd_dev);
		while (buf_rx_head != buf_rx_tail) {
			usbmidi_data_tx(usbd_dev, buf_rx[buf_rx_tail]);
			buf_rx_tail++;
		}
	}
}
