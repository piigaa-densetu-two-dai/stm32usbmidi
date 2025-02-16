#include <stddef.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

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

struct usb_midi3_endpoint_descriptor {
	struct usb_midi_endpoint_descriptor_head head;
	struct usb_midi_endpoint_descriptor_body jack[3];
} __attribute__((packed));

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi3_endpoint_descriptor midi_bulk_endp[] = {{
	/*
	 * Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi3_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 3,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
	.jack[1] = {
		.baAssocJackID = 0x05,
	},
	.jack[2] = {
		.baAssocJackID = 0x09,
	},
}, {
	/*
	 * Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi3_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 3,
	},
	.jack[0] = {
		.baAssocJackID = 0x03,
	},
	.jack[1] = {
		.baAssocJackID = 0x07,
	},
	.jack[2] = {
		.baAssocJackID = 0x0b,
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
	struct usb_midi_in_jack_descriptor in_embedded1;
	struct usb_midi_in_jack_descriptor in_external1;
	struct usb_midi_out_jack_descriptor out_embedded1;
	struct usb_midi_out_jack_descriptor out_external1;
	struct usb_midi_in_jack_descriptor in_embedded2;
	struct usb_midi_in_jack_descriptor in_external2;
	struct usb_midi_out_jack_descriptor out_embedded2;
	struct usb_midi_out_jack_descriptor out_external2;
	struct usb_midi_in_jack_descriptor in_embedded3;
	struct usb_midi_in_jack_descriptor in_external3;
	struct usb_midi_out_jack_descriptor out_embedded3;
	struct usb_midi_out_jack_descriptor out_external3;
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
	.in_embedded1 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x01,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external1 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x02,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded1 = {
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
	.out_external1 = {
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
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded2 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x05,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external2 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x06,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded2 = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x07,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x06,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external2 = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x08,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x05,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		},
	},
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded3 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x09,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external3 = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x0a,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded3 = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x0b,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x0a,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external3 = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x0c,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x09,
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
	MIDI_CIN_MISC,
	MIDI_CIN_CABLE_EVENT,
	MIDI_CIN_SYSCOM_2BYTE,
	MIDI_CIN_SYSCOM_3BYTE,
	MIDI_CIN_SYSEX_START,
	MIDI_CIN_SYSEX_END_1BYTE,
	MIDI_CIN_SYSEX_END_2BYTE,
	MIDI_CIN_SYSEX_END_3BYTE,
	MIDI_CIN_NOTE_OFF,
	MIDI_CIN_NOTE_ON,
	MIDI_CIN_POLY_KEYPRESS,
	MIDI_CIN_CONTROL_CHANGE,
	MIDI_CIN_PROGRAM_CHANGE,
	MIDI_CIN_CHANNEL_PRESSURE,
	MIDI_CIN_PITCH_BEND_CHANGE,
	MIDI_CIN_1BYTE_DATA,
};

enum
{
	MIDI_STATUS_SYSEX_START				= 0xf0,
	MIDI_STATUS_SYSEX_END				= 0xf7,
	MIDI_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME	= 0xf1,
	MIDI_STATUS_SYSCOM_SONG_POSITION_POINTER	= 0xf2,
	MIDI_STATUS_SYSCOM_SONG_SELECT			= 0xf3,
	MIDI_STATUS_SYSCOM_TUNE_REQUEST			= 0xf6,
	MIDI_STATUS_SYSREAL_TIMING_CLOCK		= 0xf8,
	MIDI_STATUS_SYSREAL_START			= 0xfa,
	MIDI_STATUS_SYSREAL_CONTINUE			= 0xfb,
	MIDI_STATUS_SYSREAL_STOP			= 0xfc,
	MIDI_STATUS_SYSREAL_ACTIVE_SENSING		= 0xfe,
	MIDI_STATUS_SYSREAL_SYSTEM_RESET		= 0xff,
};

struct rbuf {
	uint32_t usart;
	uint8_t buf[1024]; /* Size must be a power of 2. */
	uint32_t head;
	uint32_t tail;
};

__inline__
static void rbuf_put(volatile struct rbuf *rbuf, const uint8_t data)
{
	rbuf->buf[rbuf->head] = data;
	rbuf->head = (rbuf->head + 1) & (sizeof(rbuf->buf) - 1);
}

__inline__
static uint8_t rbuf_get(volatile struct rbuf *rbuf)
{
	uint8_t data;

	data = rbuf->buf[rbuf->tail];
	rbuf->tail = (rbuf->tail + 1) & (sizeof(rbuf->buf) - 1);

	return data;
}

__inline__
static uint8_t rbuf_empty(volatile struct rbuf *rbuf)
{
	return (rbuf->head == rbuf->tail);
}

__inline__
static uint32_t rbuf_space(volatile struct rbuf *rbuf) /* Return free space */
{
	return ((rbuf->tail - rbuf->head - 1) & (sizeof(rbuf->buf) - 1));
}

static volatile struct rbuf rbuf_rx[3] = { 0 };
static volatile struct rbuf rbuf_tx[3] = { 0 };

static volatile uint8_t leds = 0;
#if 1 /* For bluepill board. */
#define LED_ON		(GPIO_BSRR(GPIOC) = (GPIO13 << 16))
#define LED_OFF		(GPIO_BSRR(GPIOC) = GPIO13)
#else /* For strange CKS32 board. */
#define LED_ON		(GPIO_BSRR(GPIOC) = GPIO13)
#define LED_OFF		(GPIO_BSRR(GPIOC) = (GPIO13 << 16))
#endif
#define LED_RX_ON(x)	do { leds |= (1 << ((x) * 2)); leds ? LED_ON : LED_OFF; } while (0)
#define LED_RX_OFF(x)	do { leds &= ~(1 << ((x) * 2)); leds ? LED_ON : LED_OFF; } while (0)
#define LED_TX_ON(x)	do { leds |= (1 << ((x) * 2 + 1)); leds ? LED_ON : LED_OFF; } while (0)
#define LED_TX_OFF(x)	do { leds &= ~(1 << ((x) * 2 + 1)); leds ? LED_ON : LED_OFF; } while (0)

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep __attribute__((unused)))
{
	uint8_t buf[64];
	uint16_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	static uint8_t status[3] = { 0 };

	for (uint16_t i = 0 ; i < len ; i += 4) {
		const uint8_t cn = buf[i + 0] >> 4;
		if (3 <= cn) {
			continue;
		}
		volatile struct rbuf *p = &rbuf_tx[cn];
		while (rbuf_space(p) < 3);
		switch (buf[i + 0] & 0x0f) {
			case MIDI_CIN_SYSCOM_3BYTE:
			case MIDI_CIN_SYSEX_START:
			case MIDI_CIN_SYSEX_END_3BYTE:
				rbuf_put(p, buf[i + 1]);
				rbuf_put(p, buf[i + 2]);
				rbuf_put(p, buf[i + 3]);
				LED_TX_ON(cn);
				USART_CR1(p->usart) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_NOTE_OFF:
			case MIDI_CIN_NOTE_ON:
			case MIDI_CIN_POLY_KEYPRESS:
			case MIDI_CIN_CONTROL_CHANGE:
			case MIDI_CIN_PITCH_BEND_CHANGE:
				if (buf[i + 1] != status[cn]) { /* Running Status? */
					rbuf_put(p, buf[i + 1]);
				}
				rbuf_put(p, buf[i + 2]);
				rbuf_put(p, buf[i + 3]);
				LED_TX_ON(cn);
				USART_CR1(p->usart) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_SYSCOM_2BYTE:
			case MIDI_CIN_SYSEX_END_2BYTE:
				rbuf_put(p, buf[i + 1]);
				rbuf_put(p, buf[i + 2]);
				LED_TX_ON(cn);
				USART_CR1(p->usart) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_PROGRAM_CHANGE:
			case MIDI_CIN_CHANNEL_PRESSURE:
				if (buf[i + 1] != status[cn]) { /* Running Status? */
					rbuf_put(p, buf[i + 1]);
				}
				rbuf_put(p, buf[i + 2]);
				LED_TX_ON(cn);
				USART_CR1(p->usart) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_SYSEX_END_1BYTE:
			case MIDI_CIN_1BYTE_DATA:
				rbuf_put(p, buf[i + 1]);
				LED_TX_ON(cn);
				USART_CR1(p->usart) |= USART_CR1_TXEIE;
				break;
			case MIDI_CIN_MISC:
			case MIDI_CIN_CABLE_EVENT:
			default:
				continue;
		}
		if (buf[i + 1] < MIDI_STATUS_SYSREAL_TIMING_CLOCK) {
			status[cn] = buf[i + 1];
		}
	}
}

static void usbmidi_data_tx(usbd_device *usbd_dev, const uint8_t cn, const uint8_t data)
{
	struct packet {
		uint8_t buf[4];
		uint8_t index;
		uint8_t total;
	};
	static struct packet packet[3] = { 0 };
	struct packet *p = &packet[cn];
	const uint8_t _msg = p->buf[0] & 0x0f;

	if (MIDI_STATUS_SYSREAL_TIMING_CLOCK <= data) { /* real-time messages need to be sent right away */
		const uint8_t rtbuf[4] = { (cn << 4) | MIDI_CIN_1BYTE_DATA, data, 0, 0 };
		while (!usbd_ep_write_packet(usbd_dev, 0x81, rtbuf, sizeof(rtbuf))) {
			usbd_poll(usbd_dev);
		}
		LED_RX_OFF(cn);
		return;
	}

	if (p->index == 0) { /* New event packet */
		const uint8_t msg = data >> 4;
		p->index = 2;
		p->total = 4;

		/* Check to see if we're still in a SysEx transmit. */
		if (_msg == MIDI_CIN_SYSEX_START) {
			if (data == MIDI_STATUS_SYSEX_END) {
				p->buf[0] = MIDI_CIN_SYSEX_END_1BYTE;
				p->total = 2;
			}
			p->buf[1] = data;
		} else if ((msg < 0x8) && (0x8 <= _msg) && (_msg < 0xf)) { /* Running Status? */
			p->buf[2] = data;
			if ((_msg < 0xc) || (_msg == 0xe)) {
				p->index = 3;
			} else {
				p->index = 3;
				p->total = 3;
			}
		} else if (((0x8 <= msg) && (msg <= 0xb)) || (msg == 0xe)) { /* Channel Voice Messages */
			p->buf[0] = msg;
			p->buf[1] = data;
		} else if ((msg == 0xc) || (msg == 0xd)) { /* Channel Voice Messages, two-byte variants (Program Change and Channel Pressure) */
			p->buf[0] = msg;
			p->buf[1] = data;
			p->total = 3;
		} else if (msg == 0xf) { /* System message */
			if (data == MIDI_STATUS_SYSEX_START) {
				p->buf[0] = MIDI_CIN_SYSEX_START;
			} else if (data == MIDI_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME || data == MIDI_STATUS_SYSCOM_SONG_SELECT) {
				p->buf[0] = MIDI_CIN_SYSCOM_2BYTE;
				p->total = 3;
			} else if (data == MIDI_STATUS_SYSCOM_SONG_POSITION_POINTER) {
				p->buf[0] = MIDI_CIN_SYSCOM_3BYTE;
			} else { /* for example, MIDI_STATUS_SYSCOM_TUNE_REQUEST */
				p->buf[0] = MIDI_CIN_1BYTE_DATA;
				p->total = 2;
			}
			p->buf[1] = data;
		} else { /* Pack individual bytes if we don't support packing them into words. */
			p->buf[0] = MIDI_CIN_1BYTE_DATA;
			p->buf[1] = data;
			p->total = 2;
		}
	} else { /* On-going (buffering) packet */
		p->buf[p->index++] = data;
		/* See if this byte ends a SysEx. */
		if ((_msg == MIDI_CIN_SYSEX_START) && (data == MIDI_STATUS_SYSEX_END)) {
			p->buf[0] = MIDI_CIN_SYSEX_START + (p->index - 1); /* END +1/+2/+3 Bytes */
			p->total = p->index;
		}
	}

	/* Send out packet */
	if ((2 <= p->index) && (p->total <= p->index)) {
		/* zeroes unused bytes */
		for (uint8_t i = p->total ; i < 4 ; i++) {
			p->buf[i] = 0;
		}
		p->buf[0] |= (cn << 4);
		while (!usbd_ep_write_packet(usbd_dev, 0x81, p->buf, sizeof(p->buf))) {
			usbd_poll(usbd_dev);
		}
		LED_RX_OFF(cn);
		p->index = 0;
	}
}

static uint8_t initialized = 0;

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue __attribute__((unused)))
{
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbmidi_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	initialized = 1;
}

__inline__
static void usart_isr(const uint8_t cn)
{
	/* Check if we were called because of RXNE. */
	if ((USART_CR1(rbuf_rx[cn].usart) & USART_CR1_RXNEIE) && (USART_SR(rbuf_rx[cn].usart) & USART_SR_RXNE)) {
		if (rbuf_space(&rbuf_rx[cn])) {
			rbuf_put(&rbuf_rx[cn], USART_DR(rbuf_rx[cn].usart));
			LED_RX_ON(cn);
		} else {
			(void)USART_DR(rbuf_rx[cn].usart);
		}
	}

	/* Check if we were called because of TXE. */
	if ((USART_CR1(rbuf_tx[cn].usart) & USART_CR1_TXEIE) && (USART_SR(rbuf_tx[cn].usart) & USART_SR_TXE)) {
		if (!rbuf_empty(&rbuf_tx[cn])) {
			USART_DR(rbuf_tx[cn].usart) = rbuf_get(&rbuf_tx[cn]);
		}
		if (rbuf_empty(&rbuf_tx[cn])) {
			USART_CR1(rbuf_tx[cn].usart) &= ~USART_CR1_TXEIE; /* Disable the TXE interrupt as we don't need it anymore. */
			LED_TX_OFF(cn);
		}
	}
}

void usart1_isr(void)
{
	usart_isr(0);
}

void usart2_isr(void)
{
	usart_isr(1);
}

void usart3_isr(void)
{
	usart_isr(2);
}

int main(void)
{
	usbd_device *usbd_dev;
	static uint8_t usbd_control_buffer[256]; /* Buffer to be used for control requests. */

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* USART1 */
	rbuf_rx[0].usart = rbuf_tx[0].usart = USART1;
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

	/* USART2 PA2=OUT PA3=IN */
	rbuf_rx[1].usart = rbuf_tx[1].usart = USART2;
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART2_RX);
	gpio_set(GPIOA, GPIO_USART2_RX); /* Pullup */

	rcc_periph_clock_enable(RCC_USART2);
	usart_set_baudrate(USART2, 31250);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_enable(USART2);

	nvic_enable_irq(NVIC_USART2_IRQ);
	USART_CR1(USART2) |= USART_CR1_RXNEIE;

	/* USART3 PB10=OUT PB11=IN */
	rbuf_rx[2].usart = rbuf_tx[2].usart = USART3;
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART3_RX);
	gpio_set(GPIOB, GPIO_USART3_RX); /* Pullup */

	rcc_periph_clock_enable(RCC_USART3);
	usart_set_baudrate(USART3, 31250);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_enable(USART3);

	nvic_enable_irq(NVIC_USART3_IRQ);
	USART_CR1(USART3) |= USART_CR1_RXNEIE;

	/* LED */
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	LED_OFF;

	/* USB */
	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

	while(1) {
		usbd_poll(usbd_dev);
		if (!initialized) {
			continue;
		}
		for (uint8_t i = 0 ; i < 3 ; i++) {
			while (!rbuf_empty(&rbuf_rx[i])) {
				usbmidi_data_tx(usbd_dev, i, rbuf_get(&rbuf_rx[i]));
			}
		}
	}
}

__attribute__ ((section(".romtext")))
void reset_handler(void)
{
	volatile unsigned *src, *dest;
	__attribute__ ((section(".ram_vectors"))) static vector_table_t ram_vector_table;

	for (src = &_data_loadaddr, dest = &_data ; dest < &_edata ; src++, dest++) {
		*dest = *src;
	}

	while (dest < &_ebss) {
		*dest++ = 0;
	}

	ram_vector_table = vector_table;
	SCB_VTOR = (uint32_t)&ram_vector_table;

	/* Ensure 8-byte alignment of stack pointer on interrupts */
	/* Enabled by default on most Cortex-M parts, but not M3 r1 */
	SCB_CCR |= SCB_CCR_STKALIGN;

	/* Call the application's entry point. */
	main();
}
