/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for USBtoSerial.c.
 */

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/interrupt.h>
		#include <avr/power.h>

		#include "Descriptors.h"
		#include "Lib/SCSI.h"
		#include "Lib/SDManager.h"

		#include <LUFA/Version.h>
		#include <LUFA/Drivers/Peripheral/Serial.h>
		#include <LUFA/Drivers/Misc/RingBuffer.h>
		#include <LUFA/Drivers/USB/USB.h>

	/* Macros: */
		/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      LEDS_LED1

		/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

		/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

		/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

		#define ADAPTER_MAC_ADDRESS      {0x06, 0x15, 0x8d, 0x0b, 0x67, 0xc5}
		
		/** Indicates if the disk is write protected or not. */
		#define DISK_READ_ONLY            false
		
		typedef struct
		{
			uint8_t  Data[ETHERNET_FRAME_SIZE_MAX]; /**< Ethernet frame contents. */
			uint16_t Length; /**< Length in bytes of the Ethernet frame stored in the buffer. */
		} Ethernet_Frame_t;
		
	/* Function Prototypes: */
		void SetupHardware(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);

		void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
		void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
		
		bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo);
		
		/** change serial configuration between atmega and jennic */
		void Serial_Config(uint32_t Baudrate, uint8_t DataBits, uint8_t StopBits, uint8_t Parity);
		/** restart jennic and set to programming or normal mode */
		void Jennic_Set_Mode(bool programming);
		
		/** handle ethernet packets from USB */
		void Ethernet_In_Task(void);
		/** handle incomming serial data from jennic */
		void Jennic_In_Task(void);
		

#endif

