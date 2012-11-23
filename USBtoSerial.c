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
#include "LUFA/Scheduler/Scheduler.h"
#include <avr/sleep.h>
#include <util/delay.h>
#include "clock.h"

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
#define PBSTAT      _BV(1) /* PF1 or PB4(v3) */

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[256];

/** Stores a complete ethernet frame from jennic */
static Ethernet_Frame_t FrameOUT;

/** used to toggle the reset lines of the Jennic module, entering programming
 * mode and resetting */
static volatile bool jennic_reset_event = false;

/** actual jennic mode */
static bool jennic_in_programming_mode = false;

/** usb config switch */
static bool rndis = false;


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
  .Config =
  {
    .ControlInterfaceNumber         = 0,

    .DataINEndpointNumber           = CDC1_TX_EPNUM,
    .DataINEndpointSize             = CDC1_TXRX_EPSIZE,
    .DataINEndpointDoubleBank       = false,

    .DataOUTEndpointNumber          = CDC1_RX_EPNUM,
    .DataOUTEndpointSize            = CDC1_TXRX_EPSIZE,
    .DataOUTEndpointDoubleBank      = false,

    .NotificationEndpointNumber     = CDC1_NOTIFICATION_EPNUM,
    .NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
    .NotificationEndpointDoubleBank = false,
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
    .ControlInterfaceNumber         = 2,

    .DataINEndpointNumber           = CDC2_TX_EPNUM,
    .DataINEndpointSize             = CDC2_TXRX_EPSIZE,
    .DataINEndpointDoubleBank       = false,

    .DataOUTEndpointNumber          = CDC2_RX_EPNUM,
    .DataOUTEndpointSize            = CDC2_TXRX_EPSIZE,
    .DataOUTEndpointDoubleBank      = false,

    .NotificationEndpointNumber     = CDC2_NOTIFICATION_EPNUM,
    .NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
    .NotificationEndpointDoubleBank = false,
        
    .AdapterMACAddress              = {ADAPTER_MAC_ADDRESS},
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

    .DataINEndpointNumber           = MASS_STORAGE_IN_EPNUM,
    .DataINEndpointSize             = MASS_STORAGE_IO_EPSIZE,
    .DataINEndpointDoubleBank       = false,

    .DataOUTEndpointNumber          = MASS_STORAGE_OUT_EPNUM,
    .DataOUTEndpointSize            = MASS_STORAGE_IO_EPSIZE,
    .DataOUTEndpointDoubleBank      = false,

    .TotalLUNs                      = 1,
  },
};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  /* activate specific usb config */
  SetupHardware();
  
  /* restart jennic and set to normal mode */
  Jennic_Set_Mode(false);

  /* get off the spi-bus */
  DDRB &= ~JEN_SPIMISO;
  DDRB &= ~JEN_SPIMOSI;
  DDRB &= ~JEN_CLOCK;
  DDRE &= ~JEN_SPISS;
  DDRF &= ~PBSTAT;

  _delay_ms(2000); // wait for jennic to pull STBY line
 
  RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
 
  /* SLIP Baudrate 38400 */
  Serial_Config(38400, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);
  sei();

  for (;;)
  {
    /* Jennic Mode and Baudrate */
    if (jennic_reset_event)
    {
      if (!jennic_in_programming_mode)
      {
        jennic_in_programming_mode = true;
        /* jennic programming baudrate */
        Serial_Config(38400, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);
        /* pull jennic into programming mode */
        Jennic_Set_Mode(true);
      }
      else
      {
        jennic_in_programming_mode = false;
        /* reset baud to 1M (SLIP Baud) */
        Serial_Config(1000000, 8, CDC_LINEENCODING_OneStopBit, CDC_PARITY_None);
        /* pull jennic into normal mode */
        Jennic_Set_Mode(false);
        /* reset USB connection */
        USB_Detach();
        _delay_ms(5000);
        USB_Attach();
      }
      jennic_reset_event = false;
    }
  

    /* Read bytes from the USB OUT endpoint and transmit to jennic if programming mode */
    int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    if (jennic_in_programming_mode && !(ReceivedByte < 0))
      Serial_SendByte(ReceivedByte);
    /* change USB config if button was pressed */
    else if (!(PINF&PBSTAT))
    {
      USB_Detach();
      rndis = !rndis;
      _delay_ms(5000);
      /* restart jennic, otherwise jennic can crash (to be fixed) */
      Jennic_Set_Mode(false);
      USB_Attach();
    }

    /* check for new ethernet packet or continue sending one if rndis mode */
    if (rndis && !jennic_in_programming_mode)
      Ethernet_In_Task();

    /* handle jennic communication */
    Jennic_In_Task();

    /* lufa tasks chosen by USB config */
    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    if (rndis)
      RNDIS_Device_USBTask(&Ethernet_RNDIS_Interface);
    else if(!jennic_in_programming_mode)
      MS_Device_USBTask(&Disk_MS_Interface);
    USB_USBTask();
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
  
  /* Hardware Initialization */
  USB_Init();
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
  
  /* if successful tell jennic */
  if (ConfigSuccess)
  {
    if (rndis)
    {
      /* activate RNDIS on jennic */
      Serial_SendByte(SLIP_END);
      Serial_SendByte('R');
      Serial_SendByte('N');
      Serial_SendByte('D');
      Serial_SendByte('I');
      Serial_SendByte('S');
      Serial_SendByte(SLIP_END);
    }
    else
    {
      /* stop SD access on jennic */
      Serial_SendByte(SLIP_END);
      Serial_SendByte('S');
      Serial_SendByte('D');
      Serial_SendByte(SLIP_END);
      
      SDManager_Init();
    }
  }   
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
  uint8_t ReceivedByte = UDR1;

  if (USB_DeviceState == DEVICE_STATE_Configured)
    RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);

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
}

/** restart jennic and set to programming or normal mode */
void Jennic_Set_Mode(bool programming)
{
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
}

/** handle ethernet packets from USB */
void Jennic_In_Task()
{
  uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
  if (BufferCount)
  {
    /* Read bytes from the USART receive buffer and create ethernet frame or send with CDC */
    while (BufferCount--)
    {
      static uint8_t slip_state = SLIP_STATE_RUBBISH;
      uint8_t byte = RingBuffer_Peek(&USARTtoUSB_Buffer);
      /* pass through in programming mode */
      if (jennic_in_programming_mode)
      {
        if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface, byte) != ENDPOINT_READYWAIT_NoError)
          break;
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
      RingBuffer_Remove(&USARTtoUSB_Buffer);
    }
  }
}

/** handle incomming serial data from jennic */
void Ethernet_In_Task()
{
  static uint8_t buffer[CDC2_TXRX_EPSIZE];
  static uint8_t buffer_pointer = 0;
  static uint16_t packet_length = 0;
          
  
  if (packet_length == 0)
  {
    /* no active packet -> check for new */
    if (RNDIS_Device_IsPacketReceived(&Ethernet_RNDIS_Interface))
    {
      /* recieve packet header */
      Endpoint_SelectEndpoint(Ethernet_RNDIS_Interface.Config.DataOUTEndpointNumber);
      RNDIS_Packet_Message_t RNDISPacketHeader;
      Endpoint_Read_Stream_LE(&RNDISPacketHeader, sizeof(RNDIS_Packet_Message_t), NULL);

      if (le32_to_cpu(RNDISPacketHeader.DataLength) > ETHERNET_FRAME_SIZE_MAX)
      {
        Endpoint_StallTransaction();
        return;
      }
      
      /* "CDC2_TXRX_EPSIZE - sizeof(RNDIS_Packet_Message_t)" are still to be read. They're read in the end of buffer so that "if (buffer_pointer == CDC2_TXRX_EPSIZE)" will work */
      Endpoint_Read_Stream_LE(&buffer[sizeof(RNDIS_Packet_Message_t)], CDC2_TXRX_EPSIZE - sizeof(RNDIS_Packet_Message_t), NULL);      
      buffer_pointer = sizeof(RNDIS_Packet_Message_t);
      packet_length = (uint16_t)le32_to_cpu(RNDISPacketHeader.DataLength);
      /* start slip package */
      Serial_SendByte(SLIP_END);
    }
  }
  else
  {
    if (buffer_pointer == CDC2_TXRX_EPSIZE)
    {
      /* buffer empty read more bytes */
      Endpoint_SelectEndpoint(Ethernet_RNDIS_Interface.Config.DataOUTEndpointNumber);
      if (packet_length < CDC2_TXRX_EPSIZE)
        Endpoint_Read_Stream_LE(buffer, packet_length, NULL);
      else
        Endpoint_Read_Stream_LE(buffer, CDC2_TXRX_EPSIZE, NULL);
      buffer_pointer = 0;
    }
    
    /* send bytes and handle special bytes */
    if (buffer[buffer_pointer] == SLIP_END)
    {
      Serial_SendByte(SLIP_ESC);
      Serial_SendByte(SLIP_ESC_END);
    }
    else if (buffer[buffer_pointer] == SLIP_ESC)
    {
      Serial_SendByte(SLIP_ESC);
      Serial_SendByte(SLIP_ESC_ESC);
    } 
    else
      Serial_SendByte(buffer[buffer_pointer]);
        
    buffer_pointer++;
    packet_length--;

    if (packet_length == 0)
    {
      /* packet finished */
      Serial_SendByte(SLIP_END);
      buffer_pointer = 0;
      Endpoint_ClearOUT();
    }
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
