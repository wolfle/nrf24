#ifndef _NRF24_H_
#define _NRF24_H_

#define NRF24_IOC_RD_MODE			_IOR(SPI_IOC_MAGIC, 0, __u8)
#define NRF24_IOC_WR_MODE			_IOW(SPI_IOC_MAGIC, 0, __u8)
#define NRF24_IOC_RD_ADDR0			_IOR(SPI_IOC_MAGIC, 1, char[5])
#define NRF24_IOC_WR_ADDR0			_IOW(SPI_IOC_MAGIC, 1, char[5])
#define NRF24_IOC_RD_ADDR1			_IOR(SPI_IOC_MAGIC, 2, char[5])
#define NRF24_IOC_WR_ADDR1			_IOW(SPI_IOC_MAGIC, 2, char[5])
#define NRF24_IOC_RD_ADDR2			_IOR(SPI_IOC_MAGIC, 3, __u8)
#define NRF24_IOC_WR_ADDR2			_IOW(SPI_IOC_MAGIC, 3, __u8)
#define NRF24_IOC_RD_ADDR3			_IOR(SPI_IOC_MAGIC, 4, __u8)
#define NRF24_IOC_WR_ADDR3			_IOW(SPI_IOC_MAGIC, 4, __u8)
#define NRF24_IOC_RD_ADDR4			_IOR(SPI_IOC_MAGIC, 5, __u8)
#define NRF24_IOC_WR_ADDR4			_IOW(SPI_IOC_MAGIC, 5, __u8)
#define NRF24_IOC_RD_ADDR5			_IOR(SPI_IOC_MAGIC, 6, __u8)
#define NRF24_IOC_WR_ADDR5			_IOW(SPI_IOC_MAGIC, 6, __u8)
#define NRF24_IOC_RD_CH				_IOR(SPI_IOC_MAGIC, 7, __u8)
#define NRF24_IOC_WR_CH				_IOW(SPI_IOC_MAGIC, 7, __u8)

#define WR_BIT	(9)
#define CD_BIT	(10)
#define TXFAIL_BIT (11)
#define SPIFAIL_BIT (12)

#define SPI_FAIL (1<<SPIFAIL_BIT)
#define TX_FAIL (1<<TXFAIL_BIT)
#define CD_FLAG (1<<CD_BIT)
#define WR_FLAG (1<<WR_BIT)

#define NOACK_BIT 6
#define RXP5_BIT  5
#define RXP4_BIT  4
#define RXP3_BIT  3
#define RXP2_BIT  2
#define RXP1_BIT  1
#define RXP0_BIT  0

#define NO_ACK  (1<<6)
#define RX_P5	(1<<5)
#define RX_P4	(1<<4)
#define RX_P3	(1<<3)
#define RX_P2	(1<<2)
#define RX_P1	(1<<1)
#define RX_P0	(1<<0)

#define RX_MASK (RX_P0|RX_P1|RX_P2|RX_P3|RX_P4|RX_P5)

#endif
