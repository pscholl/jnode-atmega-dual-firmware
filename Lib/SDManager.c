
#include "SDManager.h"
#include <util/delay.h>

/* SD commands */
#define GO_IDLE_STATE     0
#define SEND_OP_COND      1
#define SWITCH_FUNC       6
#define SEND_IF_COND      8
#define SEND_CSD          9
#define SEND_CID          10
#define STOP_TRANSMISSION 12
#define SEND_STATUS       13
#define SET_BLOCKLEN      16
#define READ_SINGLE_BLOCK 17
#define WRITE_BLOCK       24
#define APP_SEND_OP_COND  41
#define APP_CMD           55
#define READ_OCR          58
#define SPI_IDLE          0xff

/* SD response lengths. */
#define R1 1
#define R2 2
#define R3 5
#define R7 5

#define START_BLOCK_TOKEN 0xfe

/* Status codes returned after writing a block. */
#define DATA_ACCEPTED 2

#define SD_TRANSACTION_ATTEMPTS   1024
#define SD_READ_RESPONSE_ATTEMPTS 8
#define SD_READ_BLOCK_ATTEMPTS    2

#define LOWER_CS() PORTE &= ~(1 << PORTE6)
#define RAISE_CS() PORTE |= (1 << PORTE6)

/* byte or block address multiplicator */
static uint16_t address_mult;
/* capacity */
static uint32_t block_count = 0;
/* SD init */
static bool inited = false;

/* send one SD command */
static void command(uint8_t cmd, uint32_t argument)
{
  uint8_t req[6], i;

  req[0] = 0x40 | cmd;
  req[1] = argument >> 24;
  req[2] = argument >> 16;
  req[3] = argument >> 8;
  req[4] = argument;
  if (cmd == GO_IDLE_STATE)
    req[5] = 0x95;
  else if (cmd == SEND_IF_COND)
    req[5] = 0x87;
    else
  req[5] = 0xff;

  for (i=0; i<sizeof(req); i++) 
    SPI_SendByte(req[i]);
}

/* SD transaction */
static void transaction(uint8_t cmd, uint32_t argument, uint8_t response_type, uint8_t * response)
{
  uint8_t i;

  LOWER_CS();
  
  /* send command */
  command(cmd, argument);

  /* wait for response */
  for (i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++)
  {
    response[0] = SPI_TransferByte(SPI_IDLE);
    if (!(response[0] & 0x80)) //check for response
      break;
  }
  /* recv response */
  for (i = 1; i < response_type; i++)
    response[i] = SPI_TransferByte(SPI_IDLE);


  /* one pause cycle */
  //SPI_SendByte(SPI_IDLE);
  RAISE_CS();
}

void calc_capacity(uint8_t sdhc)
{
  uint8_t  reg[27],tmp, multi;
  uint16_t block_len;
  uint32_t c_size;

  transaction(SEND_CSD, 0x00, sizeof(reg), reg);

  if (sdhc) {
    block_count = (reg[8]|reg[9]|reg[10])+1;
    block_count = block_count<<13;
  }
  else {
    block_len = reg[5]&0x0f;

    tmp = (reg[6] & 0x03) << 6;
    tmp |= reg[7] >> 2;
    c_size = tmp << 4;
    tmp = (reg[7]<<2) | ((reg[8] & 0xc0)>>6);
    c_size |= tmp;

    multi = ((reg[9] & 0x03) << 1);
    multi |= ((reg[10] & 0x80) >> 7);

    /* this is untested !! */
    block_count = ((c_size + 1)<<(multi + block_len - 7));
  }
}

void SDManager_Init()
{
   _delay_ms(100);
   
  uint8_t response[R7];
  bool sdhc = 0;
  bool mmc = 0;
  uint16_t i;
  
  SPI_Init(SPI_SPEED_FCPU_DIV_128 | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING | SPI_MODE_MASTER);
  /* config CS */
  DDRE |= (1 << DDE6);
  
  /* init cycles */
  for (i = 0; i < 100; i ++)
    SPI_SendByte(SPI_IDLE);
    
  /* software reset */
  transaction(GO_IDLE_STATE, 0, R1, response);
  if (!(response[0] & 0x01))
    return;
  
  /* SDHC test */
  transaction(SEND_IF_COND, 0x000001aa, R7, response);
  if (!(response[0] & 0x04) && response[3]  == 0x01 && response[4] == 0xaa)
    sdhc = true;

  /* wake up */
  for (i = 0; i < SD_TRANSACTION_ATTEMPTS; i++)
  {
    if (mmc)
      transaction(SEND_OP_COND, 0, R1, response);
    else
    {
      transaction(APP_CMD, 0, R1, response);
      transaction(APP_SEND_OP_COND, 0x40000000 * sdhc, R1, response);
      if (response[0] & 0x04)
        mmc = true;
    }
    if (!(response[0] & 0x01))
      break;
  }
  if (response[0] & 0x01)
    return;

  /* read OCR */
  transaction(READ_OCR, 0, R3, response);
  if (!(response[1] & 0x80))
    return;

  /* byte or block address */
  if (response[1] & 0x40)
    address_mult = 1;
  else
  {
    address_mult = SD_DEFAULT_BLOCK_SIZE;
    transaction(SET_BLOCKLEN, SD_DEFAULT_BLOCK_SIZE, R1, response);
  }    

  /* SD startup sucessful finished */
  inited = true;
  calc_capacity(sdhc);
  SPI_Init(SPI_USE_DOUBLESPEED | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING | SPI_MODE_MASTER);
}

uint32_t SDManager_BlockCount()
{
  return block_count;
}


bool SDManager_CheckDataflashOperation()
{
  return inited;
}

void SDManager_WriteBlocks(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo, const uint32_t BlockAddress, uint16_t TotalBlocks)
{   
  if(!inited)
    return;
  
  /* Wait until endpoint is ready before continuing */
  if (Endpoint_WaitUntilReady())
    return;
    
  uint8_t temp = 0;
  uint8_t buffer[16];
  uint16_t i, n, j;
  
  for (n = 0; n < TotalBlocks && temp == 0; n++)
  {
    uint32_t argument = (BlockAddress + n) * address_mult;

    LOWER_CS();

    command(WRITE_BLOCK, argument);
  
    /* wait for response */
    for (i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++)
    {
      temp = SPI_TransferByte(SPI_IDLE);
      if (!temp)
        break;
    }
    if (temp)
    {
      RAISE_CS();
      return;
    }
      
    /* send some idles before start token (required?) */
    for (i = 0; i < 8; i++)
      SPI_SendByte(SPI_IDLE);
      
    /* send start token */
    SPI_SendByte(START_BLOCK_TOKEN);
     
    /* send bytes */
    for (i = 0; i < SD_DEFAULT_BLOCK_SIZE/16; i++)
    {
      /* Wait until the host has sent another packet */
      if (Endpoint_WaitUntilReady())
      {
        RAISE_CS();
        return;
      }
    
      for (j = 0; j < 16; j++)
        buffer[j] = Endpoint_Read_8();
    
      /* Check if the endpoint is currently empty */
      if (!(Endpoint_IsReadWriteAllowed()))
        Endpoint_ClearOUT();
      
      for (j = 0; j < 16; j++)
        SPI_SendByte(buffer[j]);
    }
    
    /* wait for data response token */
    for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) 
    {
      temp = SPI_TransferByte(SPI_IDLE);
      if ((temp & 0x11) == 1) 
      {
        /* Data response token received. */
        temp = (temp >> 1) & 0x7;
        if(temp == DATA_ACCEPTED) 
        {
          temp = 0;
          break;
        } 
        else 
        {
          temp = 1;
          break;
        }
      }
    }      

    /* wait until sd card is finished */
    while (SPI_TransferByte(SPI_IDLE) != SPI_IDLE);

    RAISE_CS();
  }
  
  if (!(Endpoint_IsReadWriteAllowed()))
    Endpoint_ClearOUT();
}

void SDManager_ReadBlocks(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo, const uint32_t BlockAddress, uint16_t TotalBlocks)
{ 
  if(!inited)
    return;
  
  /* Wait until endpoint is ready before continuing */
  if (Endpoint_WaitUntilReady())
    return;

  uint8_t temp;
  uint8_t buffer[16];
  uint16_t i, n, j;
  
  for (n = 0; n < TotalBlocks; n++)
  {
    uint32_t argument = (BlockAddress + n) * address_mult;

    LOWER_CS();

    command(READ_SINGLE_BLOCK, argument);
  
    /* wait for response */
    for (i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++) 
    {
      temp = SPI_TransferByte(SPI_IDLE);
      if (!temp)
        break;
    }
    if (temp) 
    {
      RAISE_CS();
      return;
    }
  
    /* wait for start token */
    for (i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) 
    { 
      temp = SPI_TransferByte(SPI_IDLE);
      if(temp == START_BLOCK_TOKEN || (temp > 0 && temp <= 8))
        break;
    }

    if (temp == START_BLOCK_TOKEN)
    {
      /* read requested bytes */
      for (i = 0; i < SD_DEFAULT_BLOCK_SIZE/16; i++) 
      {
        if (!(Endpoint_IsReadWriteAllowed()))
          Endpoint_ClearIN();
        
        for (j = 0; j < 16; j++)
          buffer[j] = SPI_TransferByte(SPI_IDLE);
                
        if (Endpoint_WaitUntilReady())
        {
          RAISE_CS();
          return; 
        }
      
        for (j = 0; j < 16; j++)
          Endpoint_Write_8(buffer[j]);
      
        if (MSInterfaceInfo->State.IsMassStoreReset)
        {
          RAISE_CS();
          return;
        }
      }

      /* crc bytes (todo: check crc) */
      SPI_TransferByte(SPI_IDLE);
      SPI_TransferByte(SPI_IDLE);

      RAISE_CS();

      /* one pause cycle */
      SPI_SendByte(SPI_IDLE); 
      
    } 
    else 
    {
      RAISE_CS();
      return;
    }
  }
  
  if (!(Endpoint_IsReadWriteAllowed()))
    Endpoint_ClearIN();
}
