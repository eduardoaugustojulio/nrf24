#ifndef __LINUX_SPI_NRF24L01P_H
#define __LINUX_SPI_NRF24L01P_H

/* linux/spi/nRF24L01P */

#include <linux/ioctl.h>

struct nrf_exch {
        unsigned int timeout;
        unsigned char *rx;
        unsigned char *tx;
};

#define NRF24_MAGIC 'r'
#define NRF24_IOCBASE  0x20
#define NRF24_IOCSAA     _IOW(NRF24_MAGIC, NRF24_IOCBASE +  0, unsigned char)   /* Autoack, 7 bits, 0 = off, 1 = on */
#define NRF24_IOCGAA     _IOR(NRF24_MAGIC, NRF24_IOCBASE +  1, unsigned char *) 
#define NRF24_IOCSCRC    _IOW(NRF24_MAGIC, NRF24_IOCBASE +  2, unsigned char)   /* CRC, 0 = disabled, 1 = 1 byte, 2 = 2 bytes; */
#define NRF24_IOCGCRC    _IOR(NRF24_MAGIC, NRF24_IOCBASE +  3, unsigned char *)
#define NRF24_IOCSCH     _IOW(NRF24_MAGIC, NRF24_IOCBASE +  4, unsigned char) /* Channel, 7 bits, from 0 to 127 */
#define NRF24_IOCGCH     _IOR(NRF24_MAGIC, NRF24_IOCBASE +  5, unsigned char *)
#define NRF24_IOCSPW     _IOW(NRF24_MAGIC, NRF24_IOCBASE +  6, unsigned char) /* (RX) Payload width, 5 bits, from 0 to 32 */
#define NRF24_IOCGPW     _IOR(NRF24_MAGIC, NRF24_IOCBASE +  7, unsigned char *)
#define NRF24_IOCSTXADDR _IOW(NRF24_MAGIC, NRF24_IOCBASE +  8, unsigned char *) /* TX ADDR, from 3 to 5 bytes, depending on address width, see below */
#define NRF24_IOCGTXADDR _IOR(NRF24_MAGIC, NRF24_IOCBASE +  9, unsigned char *)
#define NRF24_IOCSRXADDR _IOW(NRF24_MAGIC, NRF24_IOCBASE + 10, unsigned char *) /* RX ADDR, same as TX ADDR */
#define NRF24_IOCGRXADDR _IOR(NRF24_MAGIC, NRF24_IOCBASE + 11, unsigned char *)
#define NRF24_IOCSAW     _IOW(NRF24_MAGIC, NRF24_IOCBASE + 12, unsigned char) /* Address Width, possible values: 3, 4 or 5 */
#define NRF24_IOCGAW     _IOR(NRF24_MAGIC, NRF24_IOCBASE + 13, unsigned char *)
#define NRF24_IOCSRETR   _IOW(NRF24_MAGIC, NRF24_IOCBASE + 14, unsigned char) /* Auto Retransmit, 7 bits from SETUP_RETR register */
#define NRF24_IOCGRETR   _IOR(NRF24_MAGIC, NRF24_IOCBASE + 15, unsigned char *)
#define NRF24_IOCSRF     _IOW(NRF24_MAGIC, NRF24_IOCBASE + 16, unsigned char) /* RF_SETUP, 7 bits from RF_SETUP register */
#define NRF24_IOCGRF     _IOR(NRF24_MAGIC, NRF24_IOCBASE + 17, unsigned char *)
#define NRF24_IOTTR      _IOWR(NRF24_MAGIC, NRF24_IOCBASE + 18, struct nrf_exch *) /* transmit then receive */
#define NRF24_IORTT      _IOWR(NRF24_MAGIC, NRF24_IOCBASE + 19, struct nrf_exch *) /* receive then transmit */
#define NRF24_IOCSDYNPLD _IOW(NRF24_MAGIC, NRF24_IOCBASE + 20, unsigned char)
#define NRF24_IOCGDYNPLD _IOW(NRF24_MAGIC, NRF24_IOCBASE + 20, unsigned char *)

#endif
