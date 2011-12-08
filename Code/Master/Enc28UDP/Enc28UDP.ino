#define DEBUG

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define SPI0_SS_BIT           BIT4  
#define SPI0_SCLK_BIT         BIT1
#define SPI0_MOSI_BIT         BIT2
#define SPI0_MISO_BIT         BIT3


// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address        (bits 0-4)
// - Bank number        (bits 5-6)
// - MAC/PHY indicator        (bit 7)
#define ADDR_MASK        0x1F
#define BANK_MASK        0x60
#define SPRD_MASK        0x80
// All-bank registers
#define EIE              0x1B
#define EIR              0x1C
#define ESTAT            0x1D
#define ECON2            0x1E
#define ECON1            0x1F
// Bank 0 registers
#define ERDPTL           (0x00|0x00)
#define ERDPTH           (0x01|0x00)
#define EWRPTL           (0x02|0x00)
#define EWRPTH           (0x03|0x00)
#define ETXSTL           (0x04|0x00)
#define ETXSTH           (0x05|0x00)
#define ETXNDL           (0x06|0x00)
#define ETXNDH           (0x07|0x00)
#define ERXSTL           (0x08|0x00)
#define ERXSTH           (0x09|0x00)
#define ERXNDL           (0x0A|0x00)
#define ERXNDH           (0x0B|0x00)
#define ERXRDPTL         (0x0C|0x00)
#define ERXRDPTH         (0x0D|0x00)
#define ERXWRPTL         (0x0E|0x00)
#define ERXWRPTH         (0x0F|0x00)
#define EDMASTL          (0x10|0x00)
#define EDMASTH          (0x11|0x00)
#define EDMANDL          (0x12|0x00)
#define EDMANDH          (0x13|0x00)
#define EDMADSTL         (0x14|0x00)
#define EDMADSTH         (0x15|0x00)
#define EDMACSL          (0x16|0x00)
#define EDMACSH          (0x17|0x00)
// Bank 1 registers
#define EHT0             (0x00|0x20)
#define EHT1             (0x01|0x20)
#define EHT2             (0x02|0x20)
#define EHT3             (0x03|0x20)
#define EHT4             (0x04|0x20)
#define EHT5             (0x05|0x20)
#define EHT6             (0x06|0x20)
#define EHT7             (0x07|0x20)
#define EPMM0            (0x08|0x20)
#define EPMM1            (0x09|0x20)
#define EPMM2            (0x0A|0x20)
#define EPMM3            (0x0B|0x20)
#define EPMM4            (0x0C|0x20)
#define EPMM5            (0x0D|0x20)
#define EPMM6            (0x0E|0x20)
#define EPMM7            (0x0F|0x20)
#define EPMCSL           (0x10|0x20)
#define EPMCSH           (0x11|0x20)
#define EPMOL            (0x14|0x20)
#define EPMOH            (0x15|0x20)
#define EWOLIE           (0x16|0x20)
#define EWOLIR           (0x17|0x20)
#define ERXFCON          (0x18|0x20)
#define EPKTCNT          (0x19|0x20)
// Bank 2 registers
#define MACON1           (0x00|0x40|0x80)
#define MACON2           (0x01|0x40|0x80)
#define MACON3           (0x02|0x40|0x80)
#define MACON4           (0x03|0x40|0x80)
#define MABBIPG          (0x04|0x40|0x80)
#define MAIPGL           (0x06|0x40|0x80)
#define MAIPGH           (0x07|0x40|0x80)
#define MACLCON1         (0x08|0x40|0x80)
#define MACLCON2         (0x09|0x40|0x80)
#define MAMXFLL          (0x0A|0x40|0x80)
#define MAMXFLH          (0x0B|0x40|0x80)
#define MAPHSUP          (0x0D|0x40|0x80)
#define MICON            (0x11|0x40|0x80)
#define MICMD            (0x12|0x40|0x80)
#define MIREGADR         (0x14|0x40|0x80)
#define MIWRL            (0x16|0x40|0x80)
#define MIWRH            (0x17|0x40|0x80)
#define MIRDL            (0x18|0x40|0x80)
#define MIRDH            (0x19|0x40|0x80)
// Bank 3 registers
#define MAADR1           (0x00|0x60|0x80)
#define MAADR0           (0x01|0x60|0x80)
#define MAADR3           (0x02|0x60|0x80)
#define MAADR2           (0x03|0x60|0x80)
#define MAADR5           (0x04|0x60|0x80)
#define MAADR4           (0x05|0x60|0x80)
#define EBSTSD           (0x06|0x60)
#define EBSTCON          (0x07|0x60)
#define EBSTCSL          (0x08|0x60)
#define EBSTCSH          (0x09|0x60)
#define MISTAT           (0x0A|0x60|0x80)
#define EREVID           (0x12|0x60)
#define ECOCON           (0x15|0x60)
#define EFLOCON          (0x17|0x60)
#define EPAUSL           (0x18|0x60)
#define EPAUSH           (0x19|0x60)
// PHY registers
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHHID1           0x02
#define PHHID2           0x03
#define PHCON2           0x10
#define PHSTAT2          0x11
#define PHIE             0x12
#define PHIR             0x13
#define PHLCON           0x14

// ENC28J60 ERXFCON Register Bit Definitions
#define ERXFCON_UCEN     0x80
#define ERXFCON_ANDOR    0x40
#define ERXFCON_CRCEN    0x20
#define ERXFCON_PMEN     0x10
#define ERXFCON_MPEN     0x08
#define ERXFCON_HTEN     0x04
#define ERXFCON_MCEN     0x02
#define ERXFCON_BCEN     0x01
// ENC28J60 EIE Register Bit Definitions
#define EIE_INTIE        0x80
#define EIE_PKTIE        0x40
#define EIE_DMAIE        0x20
#define EIE_LINKIE       0x10
#define EIE_TXIE         0x08
#define EIE_WOLIE        0x04
#define EIE_TXERIE       0x02
#define EIE_RXERIE       0x01
// ENC28J60 EIR Register Bit Definitions
#define EIR_PKTIF        0x40
#define EIR_DMAIF        0x20
#define EIR_LINKIF       0x10
#define EIR_TXIF         0x08
#define EIR_WOLIF        0x04
#define EIR_TXERIF       0x02
#define EIR_RXERIF       0x01
// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_INT        0x80
#define ESTAT_LATECOL    0x10
#define ESTAT_RXBUSY     0x04
#define ESTAT_TXABRT     0x02
#define ESTAT_CLKRDY     0x01
// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_AUTOINC    0x80
#define ECON2_PKTDEC     0x40
#define ECON2_PWRSV      0x20
#define ECON2_VRPS       0x08
// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST      0x80
#define ECON1_RXRST      0x40
#define ECON1_DMAST      0x20
#define ECON1_CSUMEN     0x10
#define ECON1_TXRTS      0x08
#define ECON1_RXEN       0x04
#define ECON1_BSEL1      0x02
#define ECON1_BSEL0      0x01
// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_LOOPBK    0x10
#define MACON1_TXPAUS    0x08
#define MACON1_RXPAUS    0x04
#define MACON1_PASSALL   0x02
#define MACON1_MARXEN    0x01
// ENC28J60 MACON2 Register Bit Definitions
#define MACON2_MARST     0x80
#define MACON2_RNDRST    0x40
#define MACON2_MARXRST   0x08
#define MACON2_RFUNRST   0x04
#define MACON2_MATXRST   0x02
#define MACON2_TFUNRST   0x01
// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG2   0x80
#define MACON3_PADCFG1   0x40
#define MACON3_PADCFG0   0x20
#define MACON3_TXCRCEN   0x10
#define MACON3_PHDRLEN   0x08
#define MACON3_HFRMLEN   0x04
#define MACON3_FRMLNEN   0x02
#define MACON3_FULDPX    0x01
// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIISCAN    0x02
#define MICMD_MIIRD      0x01
// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_NVALID    0x04
#define MISTAT_SCAN      0x02
#define MISTAT_BUSY      0x01
// ENC28J60 PHY PHCON1 Register Bit Definitions
#define PHCON1_PRST      0x8000
#define PHCON1_PLOOPBK   0x4000
#define PHCON1_PPWRSV    0x0800
#define PHCON1_PDPXMD    0x0100
// ENC28J60 PHY PHSTAT1 Register Bit Definitions
#define PHSTAT1_PFDPX    0x1000
#define PHSTAT1_PHDPX    0x0800
#define PHSTAT1_LLSTAT   0x0004
#define PHSTAT1_JBSTAT   0x0002
// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_FRCLINK   0x4000
#define PHCON2_TXDIS     0x2000
#define PHCON2_JABBER    0x0400
#define PHCON2_HDLDIS    0x0100

// ENC28J60 Packet Control Byte Bit Definitions
#define PKTCTRL_PHUGEEN  0x08
#define PKTCTRL_PPADEN   0x04
#define PKTCTRL_PCRCEN   0x02
#define PKTCTRL_POVERRIDE 0x01

// SPI operation codes
#define ENC28J60_READ_CTRL_REG       0x00
#define ENC28J60_READ_BUF_MEM        0x3A
#define ENC28J60_WRITE_CTRL_REG      0x40
#define ENC28J60_WRITE_BUF_MEM       0x7A
#define ENC28J60_BIT_FIELD_SET       0x80
#define ENC28J60_BIT_FIELD_CLR       0xA0
#define ENC28J60_SOFT_RESET          0xFF



// The RXSTART_INIT should be zero. See Rev. B4 Silicon Errata
// buffer boundaries applied to internal 8K ram
// the entire available packet buffer space is allocated
//
// start with recbuf at 0/
#define RXSTART_INIT     0x0
// receive buffer end
#define RXSTOP_INIT      (0x1FFF-0x0600-1)
// start TX buffer at 0x1FFF-0x0600, pace for one full ethernet frame (~1500 bytes)
#define TXSTART_INIT     (0x1FFF-0x0600)
// stp TX buffer at end of mem
#define TXSTOP_INIT      0x1FFF
//
// max frame length which the conroller will accept:
#define        MAX_FRAMELEN        1500        // (note: maximum ethernet frame length would be 1518)
//#define MAX_FRAMELEN     600

// set CS to 0 = active
#define CSACTIVE {PORTB&=~SPI0_SS_BIT;}

// set CS to 1 = passive
#define CSPASSIVE {PORTB|=SPI0_SS_BIT;} 

//
#define waitspi() while(!(SPSR&(1<<SPIF)))


static uint8_t myMAC[6] = {0x01,0x02,0x03,0x04,0x05,0x06}; 
static uint8_t myIP[4] = {192, 168, 1, 55};
static uint16_t NextPacketPtr;
static uint8_t Enc28j60Bank;
#define BUFFER_SIZE 500
static uint8_t buf[BUFFER_SIZE+1];


void Enc28j60Init(uint8_t* macaddr);
void Enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data);
void Enc28j60Write(uint8_t address, uint8_t data);
void Enc28j60SetBank(uint8_t address);
void enc28j60PhyWrite(uint8_t address, uint16_t data);
uint8_t Enc28j60Read(uint8_t address); 
uint8_t Enc28j60ReadOp(uint8_t op, uint8_t address);
void Enc28j60clkout(uint8_t clk);
uint8_t Enc28j60getrev(void);
uint16_t Enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet);
void Enc28j60ReadBuffer(uint16_t len, uint8_t* data);
void Enc28j60WriteBuffer(uint16_t len, uint8_t* data);
void Enc28j60PacketSend(uint16_t len, uint8_t* packet);



uint8_t Enc28j60getrev(void) {
	return(Enc28j60Read(EREVID));
}


void Enc28j60clkout(uint8_t clk) {
    //setup clkout: 2 is 12.5MHz:
    Enc28j60Write(ECOCON, clk & 0x7);
}



uint8_t Enc28j60ReadOp(uint8_t op, uint8_t address) {
      CSACTIVE;
      // issue read command
      SPDR = op | (address & ADDR_MASK);
      waitspi();
      // read data
      SPDR = 0x00;
      waitspi();
      // do dummy read if needed (for mac and mii, see datasheet page 29)
      if(address & 0x80) {
          SPDR = 0x00;
          waitspi();
      }
      // release CS
      CSPASSIVE;
      return(SPDR);
}




uint8_t Enc28j60Read(uint8_t address) {
    // set the bank
    Enc28j60SetBank(address);
    // do the read
    return Enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}




void Enc28j60PhyWrite(uint8_t address, uint16_t data) {
      // set the PHY register address
      Enc28j60Write(MIREGADR, address);
      // write the PHY data
      Enc28j60Write(MIWRL, data);
      Enc28j60Write(MIWRH, data>>8);
      // wait until the PHY write completes
      while(Enc28j60Read(MISTAT) & MISTAT_BUSY){
              delayMicroseconds(15);
      }
}


void Enc28j60SetBank(uint8_t address) {
    // set the bank (if needed)
    if((address & BANK_MASK) != Enc28j60Bank) {
        // set the bank
        Enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
        Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
        Enc28j60Bank = (address & BANK_MASK);
    }
}



void Enc28j60Write(uint8_t address, uint8_t data) {
    // set the bank
    Enc28j60SetBank(address);
    // do the write
    Enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}


void Enc28j60Init(uint8_t* macaddr) {
	// initialize I/O
	DDRB  |= SPI0_SS_BIT|SPI0_SCLK_BIT|SPI0_MOSI_BIT;
	PORTB |= SPI0_SS_BIT | BIT0; 
	PORTB &= ~(SPI0_SCLK_BIT|SPI0_MOSI_BIT);
	// initialize SPI interface
	// master mode and Fosc/2 clock:
        SPCR = (1<<SPE)|(1<<MSTR);
        SPSR |= (1<<SPI2X);
	// perform system reset
	Enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	delay(50);
	// check CLKRDY bit to see if reset is complete
        // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
        // Rx start
	Enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	Enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	Enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	Enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	Enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	Enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	Enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	Enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// TX end
	Enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
	Enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
	// do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these poitions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	Enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	Enc28j60Write(EPMM0, 0x3f);
	Enc28j60Write(EPMM1, 0x30);
	Enc28j60Write(EPMCSL, 0xf9);
	Enc28j60Write(EPMCSH, 0xf7);
        //
        //
	// do bank 2 stuff
	// enable MAC receive
	Enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	Enc28j60Write(MACON2, 0x00);
	// enable automatic padding to 60bytes and CRC operations
	Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	Enc28j60Write(MAIPGL, 0x12);
	Enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	Enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
        // Do not send packets longer than MAX_FRAMELEN:
	Enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
	Enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
	// do bank 3 stuff
        // write MAC address
        // NOTE: MAC address in ENC28J60 is byte-backward
        Enc28j60Write(MAADR5, macaddr[0]);
        Enc28j60Write(MAADR4, macaddr[1]);
        Enc28j60Write(MAADR3, macaddr[2]);
        Enc28j60Write(MAADR2, macaddr[3]);
        Enc28j60Write(MAADR1, macaddr[4]);
        Enc28j60Write(MAADR0, macaddr[5]);
	// no loopback of transmitted frames
	Enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
	// switch to bank 0
	Enc28j60SetBank(ECON1);
	// enable interrutps
	Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}


void Enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data) {
        CSACTIVE;
        // issue write command
        SPDR = op | (address & ADDR_MASK);
        waitspi();
        // write data
        SPDR = data;
        waitspi();
        CSPASSIVE;
}



void Enc28j60ReadBuffer(uint16_t len, uint8_t* data) {
    CSACTIVE;
    // issue read command
    SPDR = ENC28J60_READ_BUF_MEM;
    waitspi();
    while(len) {
        len--;
        // read data
        SPDR = 0x00;
        waitspi();
        *data = SPDR;
        data++;
    }
    *data='\0';
    CSPASSIVE;
}



void Enc28j60WriteBuffer(uint16_t len, uint8_t* data) {
        CSACTIVE;
        // issue write command
        SPDR = ENC28J60_WRITE_BUF_MEM;
        waitspi();
        while(len)
        {
                len--;
                // write data
                SPDR = *data;
                data++;
                waitspi();
        }
        CSPASSIVE;
}




void Enc28j60PacketSend(uint16_t len, uint8_t* packet) {
    // Set the write pointer to start of transmit buffer area
    Enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
    Enc28j60Write(EWRPTH, TXSTART_INIT>>8);
    // Set the TXND pointer to correspond to the packet size given
    Enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
    Enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
    // write per-packet control byte (0x00 means use macon3 settings)
    Enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
    // copy the packet into the transmit buffer
    Enc28j60WriteBuffer(len, packet);
    // send the contents of the transmit buffer onto the network
    Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
    if( (Enc28j60Read(EIR) & EIR_TXERIF) ){
        Enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
    }
}




void BuildEthHeader(uint8_t *p) {
}


uint16_t BuildArpReply(uint8_t *p, uint8_t *localMAC, uint8_t *remoteMAC, uint8_t *localIP, uint8_t *remoteIP) {
  uint16_t i;
  uint8_t *startP;

  startP=p;

  for (i=0; i<6; i++) {
      *p++=remoteMAC[i];
  } 

  for (i=0; i<6; i++) {
      *p++=localMAC[i];
  } 

  *p++=0x08;  // ARP
  *p++=0x06;  

  *p++=0x00;  // Hardware type
  *p++=0x01;

  *p++=0x08;  // Protocol type
  *p++=0x00;

  *p++=0x06;  // Hardware size

  *p++=0x04;  // Protocol size

  *p++=0x00;  // Opcode
  *p++=0x02;

  for (i=0; i<6; i++) {  // Sender MAC
      *p++=localMAC[i];
  } 
  
  for (i=0; i<4; i++) {  // Sender IP
      *p++=localIP[i];
  } 

  for (i=0; i<6; i++) {  // Remote MAC
      *p++=remoteMAC[i];
  } 
  
  for (i=0; i<4; i++) {  // Remote IP
      *p++=remoteIP[i];
  } 


  return (p-startP);
}



uint16_t BuildPingReply(uint8_t *dst, uint8_t *src, uint16_t length) {
  int i;
//   0/6 26/30  34op8->0      

  for (i=0; i<length; i++) {
    dst[i]=src[i];
  }
  
  for (i=0; i<6; i++) {  // Swap MAC
    dst[0+i]=src[6+i];
    dst[6+i]=src[0+i];
  }
  
  for (i=0; i<4; i++) {  // Swap IP
    dst[26+i]=src[30+i];
    dst[30+i]=src[26+i];
  }

  dst[34]=0;             // Reply opcode  

  dst[36]=0x00;             // Zero out checksum
  dst[37]=0x00;  

   uint32_t sum = 0;
  uint16_t chksum;
  uint8_t *p;
  uint16_t len;
 
 sum=0;
 p=&dst[34];
 len=length-34;

  // build the sum of 16bit words
  while(len >1){
          sum += 0xFFFF & (*p<<8|*(p+1));
          p+=2;
          len-=2;
  }
  // if there is a byte left then add it (padded with zero)
  if (len){
          sum += (0xFF & *p)<<8;
  }
  // now calculate the sum over the bytes in the sum
  // until the result is only 16bit long
  while (sum>>16){
          sum = (sum & 0xFFFF)+(sum >> 16);
  }
  // build 1's complement:
  chksum=( (uint16_t) sum ^ 0xFFFF);
  dst[36]=chksum>>8;
  dst[37]=chksum& 0xff;

}


void loop() {
}




void setup() { 
  Serial.begin(115200);
  Serial.println("Setup()");

  /*initialize enc28j60*/
    Enc28j60Init(myMAC);
    Enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
    delay(10);
        
    /* Magjack leds configuration, see enc28j60 datasheet, page 11 */
    // LEDA=greed LEDB=yellow

    // 0x880 is PHLCON LEDB=on, LEDA=on
    // enc28j60PhyWrite(PHLCON,0b0000 1000 1000 00 00);
    Enc28j60PhyWrite(PHLCON,0x880);
    delay(500);
    
    // 0x990 is PHLCON LEDB=off, LEDA=off
    // enc28j60PhyWrite(PHLCON,0b0000 1001 1001 00 00);
    Enc28j60PhyWrite(PHLCON,0x990);
    delay(500);

    // 0x880 is PHLCON LEDB=on, LEDA=on
    // enc28j60PhyWrite(PHLCON,0b0000 1000 1000 00 00);
    Enc28j60PhyWrite(PHLCON,0x880);
    delay(500);

    // 0x990 is PHLCON LEDB=off, LEDA=off
    // enc28j60PhyWrite(PHLCON,0b0000 1001 1001 00 00);
    Enc28j60PhyWrite(PHLCON,0x990);
    delay(500);

    // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
    // enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
    Enc28j60PhyWrite(PHLCON,0x476);
    delay(100);
        
    //init the ethernet/ip layer:
//    es.E_init_ip_arp_udp_tcp(mymac,myip,80);

  Serial.println("...done1");
  attachInterrupt(0, EncIRQ, FALLING);
  Serial.println("...done2");
  EncIRQ();
  Serial.println("...done3");
}





// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t Enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet) {
    uint16_t rxstat;
    uint16_t len;
    // check if a packet has been received and buffered
    //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
    // The above does not work. See Rev. B4 Silicon Errata point 6.
    if (Enc28j60Read(EPKTCNT)==0) {
    	return(0);
    }
    
    // Set the read pointer to the start of the received packet
    Enc28j60Write(ERDPTL, (NextPacketPtr));
    Enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
    // read the next packet pointer
    NextPacketPtr  = Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    NextPacketPtr |= Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    // read the packet length (see datasheet page 43)
    len  = Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    len |= Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    len-=4; //remove the CRC count
    // read the receive status (see datasheet page 43)
    rxstat  = Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    rxstat |= Enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    // limit retrieve length
    if (len>maxlen-1){
        len=maxlen-1;
    }
    // check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    if ((rxstat & 0x80)==0){
        // invalid
        len=0;
    } else {
        // copy the packet from the receive buffer
        Enc28j60ReadBuffer(len, packet);
    }

    // Move the RX read pointer to the start of the next received packet
    // This frees the memory we just read out
    Enc28j60Write(ERXRDPTL, (NextPacketPtr));
    Enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
    // decrement the packet counter indicate we are done with this packet
    Enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    return(len);
}






void EncIRQ() {
  uint16_t plen,i;
  uint8_t sendbuf[128];
  
  Serial.print("#");
    do {
      plen = Enc28j60PacketReceive(BUFFER_SIZE, buf);
      if(plen!=0) {
        if (buf[12]==8 && buf[13]==6) { // ARP
          i=BuildArpReply(sendbuf, myMAC, &buf[22], myIP, &buf[38]);
          Enc28j60PacketSend(i,sendbuf);
        } 
        
        if (buf[12]==8 && buf[13]==0 && buf[23]==1 && buf[34]==8) { // IP / ICMP / ECHO REQEST
          BuildPingReply(sendbuf, buf, plen);
          Enc28j60PacketSend(plen,sendbuf);
        }
        if (buf[12]==8 && buf[13]==0 && buf[23]==17 ) { // IP / UDP
//          Serial.print("u");
        }
      }
    } while (plen!=0);
}








/*
91	732.577193	192.168.1.2	192.168.1.55	IPv4	554	Fragmented IP protocol (proto=UDP 0x11, off=2048, ID=1234)

ffffffffffffe81132570b6d08004500021c12340100ff112313c0a80102c0a80137196780000208894255000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000aa

ffffffffffff (dst mac)
e81132570b6d (src mac)
0800 (ip)
4500
021c (length 540 bytes)
1234 (id)
0100
ff (ttl)
11 (protocol udp)
2313 (header checksum)
c0a80102 (src 192.168.1.2)
c0a80137 (dst 192.168.1.55)
1967 (src port)
8000 (dst port)
0208 (length upd header+data)
8942 (udp checksum)

55000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000aa (udp payload data)

0/6 26/30  34op8->0      
PING Request
0000  <LMAC>            <RMAC>             08 00 45 00   ........ 2W.m..E.
0010  00 3c 1e a7 00 00 80 01  00 00 <RIP>       <LIP>   .<...... ........
0020       <op>00 42 86 00 01  0a d5 61 62 63 64 65 66   .7..B... ..abcdef
0030  67 68 69 6a 6b 6c 6d 6e  6f 70 71 72 73 74 75 76   ghijklmn opqrstuv
0040  77 61 62 63 64 65 66 67  68 69                     wabcdefg hi      

PING Reply
0000  <RMAC>            <LMAC>             08 00 45 00   ..2W.m.. ......E.
0010  00 3c 1e a7 40 00 40 01  98 90 <LIP>       <RIP>   .<..@.@. .....7..
0020       <op>00 4a 86 00 01  0a d5 61 62 63 64 65 66   ....J... ..abcdef
0030  67 68 69 6a 6b 6c 6d 6e  6f 70 71 72 73 74 75 76   ghijklmn opqrstuv
0040  77 61 62 63 64 65 66 67  68 69                     wabcdefg hi           


*/

