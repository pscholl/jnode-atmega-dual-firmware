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
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"
#include <avr/sleep.h>
#include <util/delay.h>
#include "Lib/ringbuf.c"
#include "clock.h"

/* missing on atmega32u4 */
#define UCSR1D _SFR_MEM8(0xCB)
#define RTSEN 0
#define CTSEN 1

#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

#define SLIP_STATE_TWOPACKETS 0
#define SLIP_STATE_OK         1
#define SLIP_STATE_ESC        2
#define SLIP_STATE_RUBBISH    3

#define JEN_RESETN  _BV(7) /* PC7 */
#define JEN_SPIMISO _BV(3) /* PB3 */
#define JEN_SPIMOSI _BV(2) /* PB2 */
#define JEN_CLOCK   _BV(1) /* PB1 */
#define JEN_CTS     _BV(0) /* PF0 */
#define JEN_SPISS   _BV(6) /* PE6 */

#define PWR_BUCKON  _BV(1) /* PF1, on by default */
#define PWR_STBY    _BV(4) /* PF4, pull low! */
#define PWR_BUTTON  _BV(4) /* PB4, input pin */
#define PWR_SENSORS _BV(6) /* PC6, on by default, toggle for reset */

static struct ringbuf USARTtoUSB_Buffer, USBtoUSART_Buffer;
static uint8_t usbtouart[128], uarttousb[128]; // need size of power two!

/** Stores a complete ethernet frame from jennic */
static Ethernet_Frame_t FrameOUT;

/** used to toggle the reset lines of the Jennic module, entering programming
 * mode and resetting */
static volatile bool jennic_reset_event = false;

/** actual jennic mode */
static bool jennic_in_programming_mode = false;

/** usb config switch */
static bool rndis = true;

/** change serial configuration between atmega and jennic */
void Serial_Config(uint32_t Baudrate, uint8_t DataBits, uint8_t StopBits, uint8_t Parity);
/** restart jennic and set to programming or normal mode */
void Jennic_Set_Mode(bool programming);

/** handle ethernet packets from USB */
bool Ethernet_In_Task(void);
/** handle incoming serial data from jennic */
void Jennic_In_Task(void);
/** handle incoming data on the CDC tty */
void CDC_In_Task(void);

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
  .Config =
  {
    .ControlInterfaceNumber         = 0,
    .DataINEndpoint                 =
    {
      .Address                = CDC1_TX_EPADDR,
      .Size                   = CDC1_TXRX_EPSIZE,
      .Banks                  = 1,
    },
    .DataOUTEndpoint                =
    {
      .Address                = CDC1_RX_EPADDR,
      .Size                   = CDC1_TXRX_EPSIZE,
      .Banks                  = 1,
    },
    .NotificationEndpoint           =
    {
      .Address                = CDC1_NOTIFICATION_EPADDR,
      .Size                   = CDC_NOTIFICATION_EPSIZE,
      .Banks                  = 1,
    },
  },
};
  
 /** LUFA RNDIS Class driver interface configuration and state information. This structure is
 *  passed to all RNDIS Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_RNDIS_Device_t Ethernet_RNDIS_Interface =
{
  .Config =
  {
//    .ControlInterfaceNumber         = 2,
    .ControlInterfaceNumber         = 2,
    .DataINEndpoint                 =
    {
      .Address                = CDC2_TX_EPADDR,
      .Size                   = CDC2_TXRX_EPSIZE,
      .Banks                  = 1,
    },
    .DataOUTEndpoint                =
    {
      .Address                = CDC2_RX_EPADDR,
      .Size                   = CDC2_TXRX_EPSIZE,
      .Banks                  = 1,
    },
    .NotificationEndpoint           =
    {
      .Address                = CDC2_NOTIFICATION_EPADDR,
      .Size                   = CDC_NOTIFICATION_EPSIZE,
      .Banks                  = 1,
    },
    .AdapterVendorDescription       = "LUFA RNDIS Adapter",
    .AdapterMACAddress              = {{0x02, 0x00, 0x02, 0x00, 0x02, 0x00}},
  },
};

/** LUFA Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MS_Device_t Disk_MS_Interface =
{
  .Config =
  {
    .InterfaceNumber                = 2,
    .DataINEndpoint                 =
    {
      .Address                = MASS_STORAGE_IN_EPADDR,
      .Size                   = MASS_STORAGE_IO_EPSIZE,
      .Banks                  = 1,
    },
    .DataOUTEndpoint                =
    {
      .Address                = MASS_STORAGE_OUT_EPADDR,
      .Size                   = MASS_STORAGE_IO_EPSIZE,
      .Banks                  = 1,
    },
    .TotalLUNs                      = 1,
  },
};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  // TODO hold jennic in reset during Mass Storage operation
  static bool rndis_lock = false; /* if a rndis packet is in transmission, lock out tty packets */

  /* activate specific usb config */
  SetupHardware();
  USB_Init();

  ringbuf_init(&USARTtoUSB_Buffer, uarttousb, sizeof(uarttousb));
  ringbuf_init(&USBtoUSART_Buffer, usbtouart, sizeof(uarttousb));

  sei();

  /* restart jennic and set to normal mode. XXX needs serial line ops */
  Jennic_Set_Mode(false);

  for (;;)
  {
    /* change USB config if button was pressed */
    if ( !(PINB & PWR_BUTTON) )
    {
      USB_Detach();
      rndis = !rndis;
      _delay_ms(5000);
      /* restart jennic, otherwise jennic can crash (to be fixed) */
      Jennic_Set_Mode(false);
      USB_Attach();
    }

    /* Jennic Mode and Baudrate */
    if (jennic_reset_event)
    {
      if (!jennic_in_programming_mode)
      {
        jennic_in_programming_mode = true;
        Serial_Config(38400, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);
        Jennic_Set_Mode(true); /* pull jennic into programming mode */
      }
      else
      {
        jennic_in_programming_mode = false;
        Serial_Config(1000000, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);
        Jennic_Set_Mode(false); /* pull jennic into normal mode */

        /* reset USB connection */
        //USB_Detach();
        //_delay_ms(5000);
        //USB_Attach();
      }
      jennic_reset_event = false;
    }

    /* do house-keeping */
    USB_USBTask();
    if (jennic_in_programming_mode) {
      Jennic_In_Task();
      CDC_In_Task();
      CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    } else if (rndis) {                    // rndis+CDC mode
      Jennic_In_Task();
      if (!rndis_lock) // lock CDC tx if RNDIS packet in transmission
        CDC_In_Task();
      CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
      rndis_lock = Ethernet_In_Task();
      RNDIS_Device_USBTask(&Ethernet_RNDIS_Interface);
    } else {                             // mass storage mode
      MS_Device_USBTask(&Disk_MS_Interface);
    }

    while ( (ringbuf_elements(&USBtoUSART_Buffer) > 0) && (UCSR1A & (1 << UDRE1)) )
      UDR1 = ringbuf_get(&USBtoUSART_Buffer);
  }
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* enable clock division, run on 16MHz */
  clock_prescale_set(clock_div_1);
  clock_init();

  /* disable JTAG since we use the pins for I/O, e.g. the STBY pin */
  JTAG_DISABLE();

  /* pull STBY low, so enough power can be provided */
  DDRF  |=  (PWR_STBY);
  PORTF &= ~(PWR_STBY);

  /* reset sensors */
  DDRC  |= PWR_SENSORS;
  PORTC &= ~(PWR_SENSORS);

  /* enable pushbutton */
  DDRB |= ~(PWR_BUTTON);

  /* set default uart configuration */
  Serial_Config(1000000, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);

  /* get off the spi-bus */
  DDRB &= ~JEN_SPIMISO;
  DDRB &= ~JEN_SPIMOSI;
  DDRB &= ~JEN_CLOCK;
  DDRE &= ~JEN_SPISS;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  sleep_disable();
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  /* go to deep-sleep */
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_mode();
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  /* configure specific interfaces */
  ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
  if (rndis)
    ConfigSuccess &= RNDIS_Device_ConfigureEndpoints(&Ethernet_RNDIS_Interface);
  else
    ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);

  if (!rndis)
    SDManager_Init();

  /* if successful tell jennic */
  //if (ConfigSuccess)
  //{
  //  if (rndis)
  //  {
  //    /* activate RNDIS on jennic */
  //    Serial_SendByte(SLIP_END);
  //    Serial_SendByte('R');
  //    Serial_SendByte('N');
  //    Serial_SendByte('D');
  //    Serial_SendByte('I');
  //    Serial_SendByte('S');
  //    Serial_SendByte(SLIP_END);
  //  }
  //  else
  //  {
  //    /* stop SD access on jennic */
  //    Serial_SendByte(SLIP_END);
  //    Serial_SendByte('S');
  //    Serial_SendByte('D');
  //    Serial_SendByte(SLIP_END);

  //    SDManager_Init();
  //  }
  //}
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
  if (rndis)
    RNDIS_Device_ProcessControlRequest(&Ethernet_RNDIS_Interface);
  else
    MS_Device_ProcessControlRequest(&Disk_MS_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
  uint8_t ReceivedByte;

  if (ringbuf_elements(&USARTtoUSB_Buffer) >= ringbuf_size(&USARTtoUSB_Buffer) )
    return;

  ReceivedByte = UDR1;

  if (USB_DeviceState == DEVICE_STATE_Configured) {
    ringbuf_put(&USARTtoUSB_Buffer, ReceivedByte);
  }
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
  /* only allowed in programming mode through programmer (jenprog) */
  if (jennic_in_programming_mode)
  {
    Serial_Config(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS, 
            CDCInterfaceInfo->State.LineEncoding.DataBits, 
            CDCInterfaceInfo->State.LineEncoding.CharFormat, 
            CDCInterfaceInfo->State.LineEncoding.ParityType);
  }
}

#define SWITCH_TIMEOUT CLOCK_SECOND/5 /* in ms */
/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
  /* fire a reset jennic event iff there is a high-low-high transition within
  * SWITCH_TIMEOUT ms on the dtr line */
  static clock_time_t previous_change = 0;
  static bool currentDTRState = false;

  /* dtr line changed? */
  if ((CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != currentDTRState)
  {
    clock_time_t delay = clock_time()-previous_change;
    currentDTRState    = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
    jennic_reset_event = currentDTRState && (delay < SWITCH_TIMEOUT);
    previous_change    = clock_time(); /* returns the usb frame clock in ms */
  }
}

/** Mass Storage class driver callback function the reception of SCSI commands from the host, which must be processed.
 *
 *  \param[in] MSInterfaceInfo  Pointer to the Mass Storage class interface configuration structure being referenced
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo)
{
  bool CommandSuccess;

  /* SD request -> SCSI.c -> SDManager.c */
  CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);

  return CommandSuccess;
}

/** change serial configuration between atmega and jennic */
void Serial_Config(uint32_t Baudrate, uint8_t DataBits, uint8_t StopBits, uint8_t Parity)
{
  uint8_t ConfigMask = 0;

  switch (Parity)
  {
    case CDC_PARITY_Odd:
      ConfigMask = ((1 << UPM11) | (1 << UPM10));
      break;
    case CDC_PARITY_Even:
      ConfigMask = (1 << UPM11);
      break;
  }

  if (StopBits == CDC_LINEENCODING_TwoStopBits)
    ConfigMask |= (1 << USBS1);

  switch (DataBits)
  {
    case 6:
      ConfigMask |= (1 << UCSZ10);
      break;
    case 7:
      ConfigMask |= (1 << UCSZ11);
      break;
    case 8:
      ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
      break;
  }

  /* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
  UCSR1B = 0;
  UCSR1A = 0;
  UCSR1C = 0;

  /* Set the new baud rate before configuring the USART */
  UBRR1  = SERIAL_2X_UBBRVAL(Baudrate);
  UCSR1A = (1 << U2X1);

  UCSR1C = ConfigMask;
  UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

  /* enable RTS/CTS flow control */
  UCSR1D = (1<<RTSEN)|(1<<CTSEN);
}

/** restart jennic and set to programming or normal mode */
void Jennic_Set_Mode(bool programming)
{
  bool gotmac = false;
  char mackey[] = "mac: ", *ptr=mackey, c, macaddr[8], i=0;
  uint32_t j=0;

  SPI_Disable();
  PORTB &= ~(JEN_SPIMOSI);
  PORTC &= ~(JEN_RESETN);
   DDRC |= JEN_RESETN;
   DDRB |= JEN_SPIMISO;
  _delay_ms(5);

  if (programming)
  {
    DDRC &= ~JEN_RESETN;
    _delay_ms(10);
    DDRB &= ~JEN_SPIMISO;
  }
  else
  {
    DDRB &= ~JEN_SPIMISO;
    _delay_ms(10);
    DDRC &= ~JEN_RESETN;
  }

  /* search for mac, if found read mac-address */
  //while (!gotmac) {
  //  while (j<64000 && RingBuffer_IsEmpty(&USARTtoUSB_Buffer))
  //    j++;

  //  if (j==64000)
  //    return;

  //  c = RingBuffer_Remove(&USARTtoUSB_Buffer);
  //  if (ptr==mackey+sizeof(mackey)) {
  //    gotmac = i==7; /* here we break */
  //    macaddr[i++] = c;
  //  }
  //  else if (c==*ptr) ptr++;
  //  else         ptr=mackey;
  //}


  return;

  /* make ethernet mac addr from ieee802.15.4 macaddr */
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[0] = 0x00;
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[1] = macaddr[1];
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[2] = macaddr[2];
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[3] = macaddr[5];
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[4] = macaddr[6];
  Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[5] = macaddr[7];
}

void CDC_In_Task()
{
  /* Read bytes from the USB OUT endpoint and transmit to jennic if programming mode */
  if ( ringbuf_elements(&USBtoUSART_Buffer) < ringbuf_size(&USBtoUSART_Buffer)-2 ) {
    int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    if ( !(ReceivedByte < 0) )
      ringbuf_put(&USBtoUSART_Buffer, ReceivedByte);
  }
}

/** handle ethernet packets from USB */
void Jennic_In_Task()
{
  size_t i;

  Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);
  if ( !Endpoint_IsINReady() )
    return;

  /* Read bytes from the USART receive buffer and create ethernet frame or send with CDC */
  for( i=0; i < VirtualSerial_CDC_Interface.Config.DataINEndpoint.Size &&
            ringbuf_elements(&USARTtoUSB_Buffer) > 0; i++)
  {
    static uint8_t slip_state = SLIP_STATE_RUBBISH;
    uint8_t byte = ringbuf_get(&USARTtoUSB_Buffer);

    /* pass through in programming mode */
    if (jennic_in_programming_mode)
    {
      if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface, byte) != ENDPOINT_READYWAIT_NoError)
        return;
    }
    /* read SLIP otherwise */
    else if (rndis)
    {
      switch (slip_state)
      {
        case SLIP_STATE_RUBBISH:
          if (byte == SLIP_END)
          {
            /* new frame */
            FrameOUT.Length = 0;
            slip_state = SLIP_STATE_OK;
          }
          else
            /* no SLIP message -> printf from jennic -> pass through */
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, byte);
          break;

        case SLIP_STATE_ESC:
          slip_state = SLIP_STATE_OK;
          if (byte == SLIP_ESC_END)
            FrameOUT.Data[FrameOUT.Length++] = SLIP_END;
          else if (byte == SLIP_ESC_ESC)
            FrameOUT.Data[FrameOUT.Length++] = SLIP_ESC;
          else 
            slip_state = SLIP_STATE_RUBBISH;
          break;

        case SLIP_STATE_OK:
          if (byte == SLIP_ESC) 
            slip_state = SLIP_STATE_ESC;
          else if (byte == SLIP_END)
          {
            /* frame finished */
            slip_state = SLIP_STATE_RUBBISH;
            RNDIS_Device_SendPacket(&Ethernet_RNDIS_Interface, &FrameOUT.Data, FrameOUT.Length);
          }
          else
            FrameOUT.Data[FrameOUT.Length++] = byte;
          break;

        default:
          break;
      }
    }
  }
}

/** handle incomming serial data from jennic */
bool Ethernet_In_Task()
{
  static uint8_t buffer[CDC2_TXRX_EPSIZE];
  static uint8_t buffer_pointer = 0;
  static uint16_t packet_length = 0;

  if (packet_length == 0)
  {
    if (ringbuf_elements(&USBtoUSART_Buffer) >= ringbuf_size(&USBtoUSART_Buffer)-2)
      return false; /* make sure there is space in the tx buffer */

    /* no active packet -> check for new */
    if (RNDIS_Device_IsPacketReceived(&Ethernet_RNDIS_Interface))
    {
      /* recieve packet header */
      Endpoint_SelectEndpoint(Ethernet_RNDIS_Interface.Config.DataOUTEndpoint.Address);
      RNDIS_Packet_Message_t RNDISPacketHeader;
      Endpoint_Read_Stream_LE(&RNDISPacketHeader, sizeof(RNDIS_Packet_Message_t), NULL);

      if (le32_to_cpu(RNDISPacketHeader.DataLength) > ETHERNET_FRAME_SIZE_MAX)
      {
        Endpoint_StallTransaction();
        return false;
      }

      /* "CDC2_TXRX_EPSIZE - sizeof(RNDIS_Packet_Message_t)" are still to be read. They're read in the end of buffer so that "if (buffer_pointer == CDC2_TXRX_EPSIZE)" will work */
      Endpoint_Read_Stream_LE(&buffer[sizeof(RNDIS_Packet_Message_t)], CDC2_TXRX_EPSIZE - sizeof(RNDIS_Packet_Message_t), NULL);
      buffer_pointer = sizeof(RNDIS_Packet_Message_t);
      packet_length = (uint16_t)le32_to_cpu(RNDISPacketHeader.DataLength);
      /* start slip package */
      ringbuf_put(&USBtoUSART_Buffer, SLIP_END);
      return true;
    }
    return false;
  }
  else
  {
    if (ringbuf_elements(&USBtoUSART_Buffer) >= ringbuf_size(&USBtoUSART_Buffer)-2)
      return true; /* make sure there is space in the tx buffer */

    if (buffer_pointer == CDC2_TXRX_EPSIZE)
    {
      /* buffer empty read more bytes */
      Endpoint_SelectEndpoint(Ethernet_RNDIS_Interface.Config.DataOUTEndpoint.Address);
      if (packet_length < CDC2_TXRX_EPSIZE)
        Endpoint_Read_Stream_LE(buffer, packet_length, NULL);
      else
        Endpoint_Read_Stream_LE(buffer, CDC2_TXRX_EPSIZE, NULL);
      buffer_pointer = 0;
    }

    /* send bytes and handle special bytes */
    if (buffer[buffer_pointer] == SLIP_END)
    {
      ringbuf_put(&USBtoUSART_Buffer, SLIP_ESC);
      ringbuf_put(&USBtoUSART_Buffer, SLIP_ESC_END);
    }
    else if (buffer[buffer_pointer] == SLIP_ESC)
    {
      ringbuf_put(&USBtoUSART_Buffer, SLIP_ESC);
      ringbuf_put(&USBtoUSART_Buffer, SLIP_ESC_ESC);
    }
    else
      ringbuf_put(&USBtoUSART_Buffer, buffer[buffer_pointer]);

    buffer_pointer++;
    packet_length--;

    if (packet_length == 0)
    {
      /* packet finished */
      ringbuf_put(&USBtoUSART_Buffer, SLIP_END);
      buffer_pointer = 0;
      Endpoint_ClearOUT();
      return false;
    }
    else
      return true;
  }
}

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
  const uint8_t  DescriptorType   = (wValue >> 8);
  const uint8_t  DescriptorNumber = (wValue & 0xFF);

  const void* Address = NULL;
  uint16_t    Size    = NO_DESCRIPTOR;

  switch (DescriptorType)
  {
    case DTYPE_Device:
      Address = &DeviceDescriptor;
      Size    = sizeof(USB_Descriptor_Device_t);
      break;
    case DTYPE_Configuration:
      if (rndis)
      {
        Address = &ConfigurationDescriptorRNDIS;
        Size    = sizeof(USB_Descriptor_Configuration_RNDIS_t);
      }
      else // mass storage per default
      {
        Address = &ConfigurationDescriptorMassStorage;
        Size    = sizeof(USB_Descriptor_Configuration_MassStorage_t);
      }
      break;
    case DTYPE_String:
      switch (DescriptorNumber)
      {
        case 0x00:
          Address = &LanguageString;
          Size    = pgm_read_byte(&LanguageString.Header.Size);
          break;
        case 0x01:
          Address = &ManufacturerString;
          Size    = pgm_read_byte(&ManufacturerString.Header.Size);
          break;
        case 0x02:
          Address = &ProductString;
          Size    = pgm_read_byte(&ProductString.Header.Size);
          break;
      }

      break;
  }

  *DescriptorAddress = Address;
  return Size;
}
