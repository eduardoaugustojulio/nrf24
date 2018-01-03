/* Helpers */
#ifndef RF_HELPERS_H
#define RF_HELPERS_H

#include <linux/spi/spi.h>
#include <linux/types.h>
#include "rf_module.h"

/* Avoid undefined function of TRACING isn't enabled */
#if !defined(CONFIG_TRACING)
#define trace_puts(x)
#endif

int write_reg(struct spi_device *spi, u8 reg, u8 *val, int len);
int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len);
int write_tx(struct spi_device *spi, u8 *val, int len);
void ce_high(struct spi_device *spi);
void ce_low(struct spi_device *spi);
void pwr_up(struct spi_device *spi);
void pwr_down(struct spi_device *spi);
void prx_mode(struct spi_device *spi);
void ptx_mode(struct spi_device *spi);
int flush_rx(struct spi_device *spi);
int flush_tx(struct spi_device *spi);
int read_rx(struct spi_device *spi, u8 *payload, int len);
int read_rx_pld_wid(struct spi_device *spi, u8 *pldwid);
int exchange_ttr(struct rf_data *rf, unsigned char *payload, size_t pldsiz,
		unsigned char *rxpayload, size_t *rxpldsiz, unsigned long timeout);
void rf_led_on(struct rf_data *rf);
void rf_led_off(struct rf_data *rf);

#endif
