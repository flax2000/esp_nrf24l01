
#pragma once
// BYTE type definition
#ifndef API_H
#define API_H
//****************************************************
// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register
//***************************************************
#define RX_DR    0x40
#define TX_DS    0x20
#define MAX_RT   0x10
//***************************************************
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

//************************************************
#endif


//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void) {
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);   // chip enable
  digitalWrite(CSN, 1);  // Spi disable
}

/************************************************************************
**   * Function: SPI_RW();
 * 
 * Description:
 * Writes one uint8_t to nRF24L01, and return the uint8_t read
 * from nRF24L01 during write, according to SPI protocol
************************************************************************/
uint8_t RECEIVE_ATTR SPI_RW(uint8_t Byte) {
  return SPI.transfer(Byte);
}

/**************************************************

**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
**************************************************/
void RECEIVE_ATTR SPI_RW_Reg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSN, 0);  // CSN low, init SPI transaction
  SPI_RW(reg);           // select register
  SPI_RW(value);         // ..and write value to it..
  SPI.endTransaction();
  digitalWrite(CSN, 1);  // Set CSN high again  
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one uint8_t from nRF24L01 register, 'reg'
**************************************************/
uint8_t RECEIVE_ATTR SPI_Read(uint8_t reg) {
  uint8_t reg_val;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSN, 0);  // CSN low, initialize SPI communication...
  SPI_RW(reg);           // Select register to read from..
  reg_val = SPI_RW(0);   // ..then read register value
  SPI.endTransaction();
   digitalWrite(CSN, 1);  // Set CSN high again 
  return (reg_val);  // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'uint8_ts' #of uint8_ts from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
**************************************************/
uint8_t RECEIVE_ATTR SPI_Read_Buf(uint8_t reg, volatile uint8_t *pBuf, uint8_t bytes) {
  uint8_t sstatus, i;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSN, 0);   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);  // Select register to write to and read status uint8_t

  for (i = 0; i < bytes; i++) {
    pBuf[i] = SPI_RW(0);  // Perform SPI_RW to read uint8_t from nRF24L01
  }


  SPI.endTransaction();
   digitalWrite(CSN, 1);  // Set CSN high again 
  return (sstatus);  // return nRF24L01 status uint8_t
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
**************************************************/
uint8_t RECEIVE_ATTR SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes) {
  uint8_t sstatus, i;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSN, 0);        // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);       // Select register to write to and read status uint8_t
  for (i = 0; i < bytes; i++)  // then write all uint8_t in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }

    SPI.endTransaction();
   digitalWrite(CSN, 1);  // Set CSN high again   
  return (sstatus);      // return nRF24L01 status uint8_t
}
/**************************************************/

void RX_Mode(void) {
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Use the same address on the RX device as the TX device
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);                              // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + SETUP_AW, 0X02);
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);           // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, chRf[channel]);      // Select RF channel
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x06);            // TX_PWR:0dBm, Datarate:1Mps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x33);              // no crc and rx mode. 0x03
  digitalWrite(CE, 1);                               // Set CE pin high to enable RX device
  //  This device is now ready to receive one packet of 16 uint8_ts payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}



static uint8_t reverseBits(uint8_t input) {
  // reverse the bit order in a single byte
  uint8_t temp = 0;
  if (input & 0x80) temp |= 0x01;
  if (input & 0x40) temp |= 0x02;
  if (input & 0x20) temp |= 0x04;
  if (input & 0x10) temp |= 0x08;
  if (input & 0x08) temp |= 0x10;
  if (input & 0x04) temp |= 0x20;
  if (input & 0x02) temp |= 0x40;
  if (input & 0x01) temp |= 0x80;
  return temp;
}
static inline uint8_t bleWhitenStart(uint8_t chan) {
  //use left shifted one
  return reverseBits(chan) | 2;
}

void bleWhiten(uint8_t *data, uint8_t len, uint8_t whitenCoeff) {
  // Implementing whitening with LFSR
  uint8_t m;
  while (len--) {
    for (m = 1; m; m <<= 1) {
      if (whitenCoeff & 0x80) {
        whitenCoeff ^= 0x11;
        (*data) ^= m;
      }
      whitenCoeff <<= 1;
    }
    data++;
  }
}

