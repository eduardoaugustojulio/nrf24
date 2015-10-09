#ifndef RF_HELPERS_H
#define RF_HELPERS_H

#include <linux/spi/spi.h>
#include <linux/types.h>

/* Avoid undefined function of TRACING isn't enabled */
#if !defined(CONFIG_TRACING)
#define trace_puts(x)
#endif

/* power up is assumed */
#define TX(rf, payload, pldsiz, timeout)                                \
        ({                                                              \
                int status;                                             \
                                                                        \
                _DEBUG("Transmitting, timeout %lu,  pldsiz %u",         \
                       timeout, pldsiz);                                \
                flush_tx((rf)->spi);                                    \
                write_tx((rf)->spi, (payload), (pldsiz));               \
                write_reg((rf)->spi, STATUS_REG, "\x70", 1);            \
                ptx_mode((rf)->spi);                                    \
                trace_puts("Waiting TX IRQ\n");                         \
                enable_irqs((rf)->spi);                                 \
                ce_high((rf)->spi);                                     \
                status = wait_for_completion_interruptible_timeout(     \
                        &(rf)->tx_complete,                             \
                        timeout);                                       \
                ce_low((rf)->spi);                                      \
                disable_irqs((rf)->spi);                                \
                status;                                                 \
        })

/* power up is assumed */
#define RX(rf, payload, pldsiz, timeout)                                \
        ({                                                              \
                int status;                                             \
                                                                        \
                _DEBUG("Receiving, timeout %lu", timeout);              \
                flush_rx((rf)->spi);                                    \
                write_reg((rf)->spi, STATUS_REG, "\x70", 1);            \
                prx_mode((rf)->spi);                                    \
                (rf)->new_rx_data = 0;                                  \
                trace_puts("Waiting RX IRQ\n");                         \
                enable_irqs((rf)->spi);                                 \
                ce_high((rf)->spi);                                     \
                status = wait_event_interruptible_timeout(              \
                        (rf)->rx_wq,                                    \
                        (rf)->new_rx_data,                              \
                        timeout);                                       \
                ce_low((rf)->spi);                                      \
                disable_irqs((rf)->spi);                                \
                if (status > 0) {                                       \
                        u8 reg;                                         \
                        u8 pldwid;                                      \
                        /* check for dynamic payload*/                  \
                        read_reg((rf)->spi, FEATURE_REG, &reg, 1);      \
                        if (IS_SETTED(reg, BIT(2)))                     \
                                read_rx_pld_wid((rf)->spi,              \
                                                &pldwid);               \
                        else                                            \
                                read_reg((rf)->spi,                     \
                                         RX_PW_P0_REG,                  \
                                         &pldwid,                       \
                                         1);                            \
                        read_rx((rf)->spi, payload, pldwid);            \
                        pldsiz = pldwid;                                \
                }                                                       \
                                                                        \
                status;                                                 \
})

#define PD_MODE(spi)                                    \
        do {                                            \
                _DEBUG("Change to POWER-DOWN MODE");    \
                ce_low((spi));                          \
                pwr_down((spi));                        \
        } while (0)

#define SB1_MODE(spi)                                   \
        do {                                            \
                _DEBUG("Change to STANDBY-I MODE");     \
                pwr_up((spi));                          \
                ce_low((spi));                          \
        } while (0)


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
void enable_irqs(struct spi_device *spi);
void disable_irqs(struct spi_device *spi);
int read_rx_pld_wid(struct spi_device *spi, u8 *pldwid);

#endif
