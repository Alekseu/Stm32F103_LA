/*
 * Descriptors.h
 *
 *  Created on: 16 РјР°СЂС‚Р° 2017 Рі.
 *      Author: hudienko_a
 */

#ifndef DRIVER_DESCRIPTORS_H_
#define DRIVER_DESCRIPTORS_H_

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define VIRTUAL_COM_PORT_DATA_SIZE              64
#define VIRTUAL_COM_PORT_INT_SIZE               8

#define VIRTUAL_COM_PORT_SIZ_DEVICE_DESC        18
#define VIRTUAL_COM_PORT_SIZ_CONFIG_DESC        67
#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
#define VIRTUAL_COM_PORT_SIZ_STRING_VENDOR      38
#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     50
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

uint8_t _Virtual_Com_Port_Device_Descriptor[] =  {
		    0x12,   /* bLength */
		    USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
		    0x00,
		    0x02,   /* bcdUSB = 2.00 */
		    0x02,   /* bDeviceClass: CDC */
		    0x00,   /* bDeviceSubClass */
		    0x00,   /* bDeviceProtocol */
		    0x40,   /* bMaxPacketSize0 */
		    0x83,
		    0x04,   /* idVendor = 0x0483 */
		    0x40,
		    0x57,   /* idProduct = 0x7540 */
		    0x01,
		    0x00,   /* bcdDevice = 1.00 */
		    0x00,              /* Index of string descriptor describing manufacturer */
		    0x01,              /* Index of string descriptor describing product */
		    0x03,              /* Index of string descriptor describing the device's serial number */
		    0x01    /* bNumConfigurations */
		  };

	uint8_t _Virtual_Com_Port_ConfigDescriptor[] =
	  {
	    /*Configuration Descriptor*/
	    0x09,   /* bLength: Configuration Descriptor size */
	    USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
	    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
	    0x00,
	    0x02,   /* bNumInterfaces: 2 interface */
	    0x01,   /* bConfigurationValue: Configuration value */
	    0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
	    0xC0,   /* bmAttributes: self powered */
	    0x32,   /* MaxPower 100 mA */
	    /*Interface Descriptor*/
	    0x09,   /* bLength: Interface Descriptor size */
	    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
	    /* Interface descriptor type */
	    0x00,   /* bInterfaceNumber: Number of Interface */
	    0x00,   /* bAlternateSetting: Alternate setting */
	    0x01,   /* bNumEndpoints: One endpoints used */
	    0x02,   /* bInterfaceClass: Communication Interface Class */
	    0x02,   /* bInterfaceSubClass: Abstract Control Model */
	    0x01,   /* bInterfaceProtocol: Common AT commands */
	    0x00,   /* iInterface: */
	    /*Header Functional Descriptor*/
	    0x05,   /* bLength: Endpoint Descriptor size */
	    0x24,   /* bDescriptorType: CS_INTERFACE */
	    0x00,   /* bDescriptorSubtype: Header Func Desc */
	    0x10,   /* bcdCDC: spec release number */
	    0x01,
	    /*Call Management Functional Descriptor*/
	    0x05,   /* bFunctionLength */
	    0x24,   /* bDescriptorType: CS_INTERFACE */
	    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
	    0x00,   /* bmCapabilities: D0+D1 */
	    0x01,   /* bDataInterface: 1 */
	    /*ACM Functional Descriptor*/
	    0x04,   /* bFunctionLength */
	    0x24,   /* bDescriptorType: CS_INTERFACE */
	    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
	    0x02,   /* bmCapabilities */
	    /*Union Functional Descriptor*/
	    0x05,   /* bFunctionLength */
	    0x24,   /* bDescriptorType: CS_INTERFACE */
	    0x06,   /* bDescriptorSubtype: Union func desc */
	    0x00,   /* bMasterInterface: Communication class interface */
	    0x01,   /* bSlaveInterface0: Data Class Interface */


	    /*Endpoint 2 Descriptor*/
	    0x07,   /* bLength: Endpoint Descriptor size */
	    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	    0x82,   /* bEndpointAddress: (IN2) */
	    0x03,   /* bmAttributes: Interrupt */
	    VIRTUAL_COM_PORT_INT_SIZE,      /* wMaxPacketSize: */
	    0x00,
	    0xFF,   /* bInterval: */

	    /*Data class interface descriptor*/
	    0x09,   /* bLength: Endpoint Descriptor size */
	    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
	    0x01,   /* bInterfaceNumber: Number of Interface */
	    0x00,   /* bAlternateSetting: Alternate setting */
	    0x02,   /* bNumEndpoints: Two endpoints used */
	    0x0A,   /* bInterfaceClass: CDC */
	    0x00,   /* bInterfaceSubClass: */
	    0x00,   /* bInterfaceProtocol: */
	    0x00,   /* iInterface: */

	    /*Endpoint 3 Descriptor*/
	    0x07,   /* bLength: Endpoint Descriptor size */
	    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	    0x03,   /* bEndpointAddress: (OUT3) */
	    0x02,   /* bmAttributes: Bulk */
	    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
	    0x00,
	    0x00,   /* bInterval: ignore for Bulk transfer */

	    /*Endpoint 1 Descriptor*/
	    0x07,   /* bLength: Endpoint Descriptor size */
	    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	    0x81,   /* bEndpointAddress: (IN1) */
	    0x02,   /* bmAttributes: Bulk */
	    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
	    0x00,
	    0x00    /* bInterval */
	  };

	const uint8_t Virtual_Com_Port_StringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID] =
	  {
	    VIRTUAL_COM_PORT_SIZ_STRING_LANGID,
	    USB_STRING_DESCRIPTOR_TYPE,
	    0x09,
	    0x04 /* LangID = 0x0409: U.S. English */
	  };

	const uint8_t Virtual_Com_Port_StringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR] =
	  {
	    VIRTUAL_COM_PORT_SIZ_STRING_VENDOR,     /* Size of Vendor string */
	    USB_STRING_DESCRIPTOR_TYPE,             /* bDescriptorType*/
	    /* Manufacturer: "STMicroelectronics" */
	    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
	    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
	    'c', 0, 's', 0
	  };

	const uint8_t Virtual_Com_Port_StringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT] =
	  {
	    VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT,          /* bLength */
	    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	    /* Product name: "STM32 Virtual COM Port" */
	    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'V', 0, 'i', 0,
	    'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0,
	    'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0, ' ', 0, ' ', 0
	  };

	uint8_t Virtual_Com_Port_StringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL] =
	  {
	    VIRTUAL_COM_PORT_SIZ_STRING_SERIAL,           /* bLength */
	    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
	    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
	  };


//#define SomeDev_SIZ_DEVICE_DESC         	     18
//#define SomeDev_SIZ_CONFIG_DESC          	     32
//
//#define SomeDev_SIZ_STRING_LANGID       	     4
//#define SomeDev_SIZ_STRING_VENDOR       	     22
//#define SomeDev_SIZ_STRING_PRODUCT       		 38
//
//	const uint8_t SomeDev_DeviceDescriptor[SomeDev_SIZ_DEVICE_DESC] =
//	  {
//	    0x12,   /* bLength  */
//	    0x01,   /* bDescriptorType */
//	    0x00,   /* bcdUSB, version 2.00 */
//	    0x02,
//	    0xff,   /* bDeviceClass : each interface define the device class */
//	    0xff,   /* bDeviceSubClass */
//	    0xff,   /* bDeviceProtocol */
//	    0x40,   /* bMaxPacketSize0 0x40 = 64 */
//
//		// $USBCONFIG - СЃСЋРґР° РІСЃС‚Р°РІРёС‚СЊ vendor ID (VID). в‰€РіРѕ РјРѕР¶РЅРѕ РєСѓРїРёС‚СЊ Сѓ usb.org Р·Р° РЅРµСЃРєРѕР»СЊРєРѕ С‚С‹СЃВ¤С‡ РґРѕР»Р»Р°СЂРѕРІ :)
//		// В¬ РґР°РЅРЅРѕРј РїСЂРёРјРµСЂРµ РёСЃРїРѕР»СЊР·СѓРµС‚СЃВ¤ VID РѕС‚ ST Microelectrinics
//		// вЂќ РЅРёС… РЅР° РєР°РєРёС…-С‚Рѕ СѓСЃР»РѕРІРёВ¤СЉ РјРѕР¶РЅРѕ РїРѕР»СѓС‡РёС‚СЊ PID РґР»В¤ СЃРІРѕРµРіРѕ СѓСЃС‚СЂРѕР№СЃС‚РІР°
//		0x83,   /* idVendor     (0483) */
//	    0x04,
//		// $USBCONFIG - СЃСЋРґР° РІСЃС‚Р°РІРёС‚СЊ product ID (PID)
//		// В¬ РґР°РЅРЅРѕРј РїСЂРёРјРµСЂРµ _вЂњСњР‹в„–В Сњ вЂ” Г·в‰€Р‹в„–С‘ СњВ«РЊС�В СњС›Р‹в‰€РЊВ»СЏ В» вЂњв‰€вЂ”вЂњВ»вЂ“СњВ¬С�РЊВ»СЏ_ РёСЃРїРѕР»СЊР·СѓРµС‚СЃВ¤ PID "РѕС‚ Р±Р°Р»РґС‹".
//	    0xF0,   /* idProduct	(FFF0) */
//	    0xFF,
//	    0x00,   /* bcdDevice 2.00*/
//	    0x02,
//	    1,              /* index of string Manufacturer  */
//	    /**/
//	    2,              /* index of string descriptor of product*/
//	    /* */
//	    3,              /* */
//	    /* */
//	    /* */
//	    0x01    /*bNumConfigurations */
//	  };
//
//
//
//
//	const uint8_t SomeDev_ConfigDescriptor[SomeDev_SIZ_CONFIG_DESC] =
//	  {
//
//	    0x09,   /* bLength: Configuation Descriptor size */
//	    0x02,   /* bDescriptorType: Configuration */
//	    SomeDev_SIZ_CONFIG_DESC,
//
//	    0x00,
//	    0x01,   /* bNumInterfaces: 1 interface */
//	    0x01,   /* bConfigurationValue: */
//	    /*      Configuration value */
//	    0x00,   /* iConfiguration: */
//	    /*      Index of string descriptor */
//	    /*      describing the configuration */
//	    0xC0,   /* bmAttributes: */
//	    /*      bus powered */
//	    0x32,   /* MaxPower 100 mA */
//
//	    /******************** Descriptor of Mass Storage interface ********************/
//	    /* 09 */
//	    0x09,   /* bLength: Interface Descriptor size */
//	    0x04,   /* bDescriptorType: */
//	    /*      Interface descriptor type */
//	    0x00,   /* bInterfaceNumber: Number of Interface */
//	    0x00,   /* bAlternateSetting: Alternate setting */
//	    0x02,   /* bNumEndpoints*/
//	    0xff,   /* bInterfaceClass */
//	    0xff,   /* bInterfaceSubClass */
//	    0xff,   /* nInterfaceProtocol */
//	    4,          /* iInterface: */
//	    /* 18 */
//	    0x07,   /*Endpoint descriptor length = 7*/
//	    0x05,   /*Endpoint descriptor type */
//	    0x81,   /*Endpoint address (IN, address 1) */
//	    0x02,   /*Bulk endpoint type */
//	    0x40,   /*Maximum packet size (64 bytes) */
//	    0x00,
//	    0x00,   /*Polling interval in milliseconds */
//	    /* 25 */
//	    0x07,   /*Endpoint descriptor length = 7 */
//	    0x05,   /*Endpoint descriptor type */
//	    0x02,   /*Endpoint address (OUT, address 2) */
//	    0x02,   /*Bulk endpoint type */
//	    0x40,   /*Maximum packet size (64 bytes) */
//	    0x00,
//	    0x00,     /*Polling interval in milliseconds*/
//	    /*32*/
//		// */
//	  };
//
//	// вЂ”С‚СЂРѕРєРѕРІС‹Рµ РёРґРµРЅС‚РёС„РёРєР°С‚РѕСЂС‹
//
//	#define SomeDev_SIZ_STRING_SERIAL            16
//
//#define SomeDev_SIZ_STRING_INTERFACE        	 20
//	// СЏР·С‹Рє
//	const uint8_t SomeDev_StringLangID[SomeDev_SIZ_STRING_LANGID] =
//	  {
//	    SomeDev_SIZ_STRING_LANGID,
//	    0x03,
//	    0x09,
//	    0x04
//	  };      /* LangID = 0x0409: U.S. English */
//
//	// Vendor
//	const uint8_t SomeDev_StringVendor[SomeDev_SIZ_STRING_VENDOR] =
//	  {
//	    SomeDev_SIZ_STRING_VENDOR, /* Size of manufaturer string */
//	    0x03,           /* bDescriptorType = String descriptor */
//	    /* Manufacturer */
//	    'O', 0,
//		'A', 0,
//		'O', 0,
//		' ', 0,
//		'V', 0,
//		'E', 0,
//		'N', 0,
//		'D', 0,
//		'O', 0,
//		'R', 0
//	  };
//
//	// С•СЂРѕРґСѓРєС‚
//	const uint8_t SomeDev_StringProduct[SomeDev_SIZ_STRING_PRODUCT] =
//	  {
//	    SomeDev_SIZ_STRING_PRODUCT,
//	    0x03,
//	    /* Product name */
//	    'S', 0,
//		'a', 0,
//		'm', 0,
//		'p', 0,
//		'l', 0,
//		'e', 0,
//		' ', 0,
//		's', 0,
//	    'o', 0,
//		'm', 0,
//		'e', 0,
//		' ', 0,
//		'd', 0,
//		'e', 0,
//		'v', 0,
//		'i', 0,
//		'c', 0,
//		'e', 0
//
//	  };
//
//	// $BOARDSPECIFIC - вЂ”РµСЂРёР№РЅС‹Р№ РЅРѕРјРµСЂ
//	uint8_t SomeDev_StringSerial[SomeDev_SIZ_STRING_SERIAL] =
//	  {
//	    SomeDev_SIZ_STRING_SERIAL,
//	    0x03,
//	    /* Serial number */
//
//	    'D', 0,
//		'E', 0,
//		'F', 0,
//		'T', 0,
//		'0', 0,
//		'0', 0,
//		'2', 0
//
//	  };
//
//	// В»РЅС‚РµСЂС„РµР№СЃ
//	const uint8_t SomeDev_StringInterface[SomeDev_SIZ_STRING_INTERFACE] =
//	  {
//	    SomeDev_SIZ_STRING_INTERFACE,
//	    0x03,
//	    /* Interface 0 */
//	    'I', 0,
//		'n', 0,
//		't', 0,
//		'e', 0,
//		'r', 0,
//		'f', 0,
//		'a', 0,
//		'c', 0,
//		'e', 0
//	  };

#endif /* DRIVER_DESCRIPTORS_H_ */
