/*
 * usb_msc_desc.c
 *
 *  Created on: 9 июн. 2021 г.
 *      Author: sa100 (sergio.rudenko@gmail.com)
 */

#include "usb_conf.h"

/* ---[ private macros ]--------------------------------------------------- */
#define LOBYTE(x)  ((uint8_t)(((x) & 0x00FFU)))
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))


/* ---[ private definitions ]---------------------------------------------- */
#define USBD_VID				0x0483U	/* ST Microelectronics */
#define USBD_PID				0x5720U	/* Mass Storage Device */

#define USB_MAX_EP0_SIZE            64U
#define USB_MAX_PACKET_SIZE			64U

#define MSC_IN_EP				  0x81U	/* EP1 */
#define MSC_OUT_EP				  0x02U /* EP2 */


/* ---[ USB MSC Device Descriptor ]---------------------------------------- */
const uint8_t USB_DeviceDescriptor[] =
  {
    0x12,                       	/* bLength */
    USB_DESC_TYPE_DEVICE,       	/* bDescriptorType */
    0x00,                       	/* bcdUSB: 2.00 */
    0x02,
    0x00,                       	/* bDeviceClass */
    0x00,                       	/* bDeviceSubClass */
    0x00,                       	/* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           	/* bMaxPacketSize */
    LOBYTE(USBD_VID),           	/* idVendor */
    HIBYTE(USBD_VID),           	/* idVendor */
    LOBYTE(USBD_PID),        		/* idProduct */
    HIBYTE(USBD_PID),        		/* idProduct */
    0x00,                       	/* bcdDevice rel. 2.00 */
    0x02,
    USB_DESC_VENDOR_INDEX,			/* Index of manufacturer string */
    USB_DESC_PRODUCT_INDEX,			/* Index of product string */
    USB_DESC_SERIAL_NUMBER_INDEX,	/* Index of serial number string */
    1,  							/* bNumConfigurations */
  };
const uint16_t USB_DeviceDescriptorSize = sizeof(USB_DeviceDescriptor);


/* ---[ USB MSC device Configuration Descriptor ]-------------------------- */
const uint8_t USB_ConfigDescriptor[] =
  {
    0x09,   						/* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,	/* bDescriptorType: Configuration */
    32,								/* Configuration Descriptor size: 32 */

    0x00,
    0x01,   						/* bNumInterfaces: 1 interface */
    0x01,   						/* bConfigurationValue: Configuration value */
    0x00,   						/* iConfiguration: Index of string descriptor describing the configuration */
    USB_DEVICE_POWER_SOURCE, 		/* bmAttributes: Bus||Self powered */
    0x32,   						/* MaxPower 100 mA */

    /******************** Descriptor of Mass Storage interface ********************/

    /* [09] */
    0x09,   						/* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,		/* bDescriptorType: Interface descriptor type */
    0x00,   						/* bInterfaceNumber: Number of Interface */
    0x00,   						/* bAlternateSetting: Alternate setting */
    0x02,   						/* bNumEndpoints*/
    0x08,   						/* bInterfaceClass: MASS STORAGE Class */
    0x06,   						/* bInterfaceSubClass : SCSI transparent */
    0x50,   						/* nInterfaceProtocol */
    USB_DESC_INTERFACE_INDEX, 		/* Index of Interface string */

    /* [18] Endpoint IN Descriptor */
    0x07,   						/* Endpoint descriptor length = 7 */
    USB_DESC_TYPE_ENDPOINT,   		/* Endpoint descriptor type */
    MSC_IN_EP, 						/* Endpoint address (IN, address 1) */
    0x02,   						/* Bulk endpoint type */
    LOBYTE(USB_MAX_PACKET_SIZE),	/* Maximum packet size (64 bytes) */
    HIBYTE(USB_MAX_PACKET_SIZE),
    0,   							/* Polling interval in milliseconds */

    /* [25] Endpoint OUT Descriptor */
    0x07,   						/* Endpoint descriptor length = 7 */
    USB_DESC_TYPE_ENDPOINT,   		/* Endpoint descriptor type */
    MSC_OUT_EP,   					/* Endpoint address (OUT, address 2) */
    0x02,   						/* Bulk endpoint type */
    LOBYTE(USB_MAX_PACKET_SIZE),	/* Maximum packet size (64 bytes) */
    HIBYTE(USB_MAX_PACKET_SIZE),
    0,     							/* Polling interval in milliseconds */
    /* [32] */
  };
const uint16_t USB_ConfigDescriptorSize = sizeof(USB_ConfigDescriptor);


/* ---[ USB String Descriptors ]------------------------------------------- */
const uint8_t USB_LangIdDescriptor[] =
  {
    0x09, 0x04, /* LangID = 0x0409: U.S. English */
  };
const uint16_t USB_LangIdDescriptorSize = sizeof(USB_LangIdDescriptor);


const uint8_t USB_VendorDescriptor[] =
  {
    /* Manufacturer: */
    'S', 0,
	'T', 0,
	'M', 0,
	'i', 0,
	'c', 0,
	'r', 0,
	'o', 0,
	'e', 0,
	'l', 0,
	'e', 0,
	'c', 0,
	't', 0,
	'r', 0,
	'o', 0,
	'n', 0,
	'i', 0,
	'c', 0,
	's', 0,
  };
const uint16_t USB_VendorDescriptorSize = sizeof(USB_VendorDescriptor);


const uint8_t USB_ProductDescriptor[] =
  {
    /* Product name: */
    'S', 0,
	'T', 0,
	'M', 0,
	'3', 0,
	'2', 0,
	' ', 0,
    'M', 0,
	'a', 0,
	's', 0,
	's', 0,
	' ', 0,
    'S', 0,
	't', 0,
	'o', 0,
	'r', 0,
	'a', 0,
	'g', 0,
	'e', 0,
  };
const uint16_t USB_ProductDescriptorSize = sizeof(USB_ProductDescriptor);


const uint8_t USB_InterfaceDescriptor[] =
  {
    /* Interface 0: */
    'S', 0,
	'T', 0,
	' ', 0,
	'M', 0,
	'a', 0,
	's', 0,
	's', 0,
  };
const uint16_t USB_InterfaceDescriptorSize = sizeof(USB_InterfaceDescriptor);
