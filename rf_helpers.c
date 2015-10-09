#include "rf_module.h"
#include "rf_helpers.h"

int write_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
        u8 cmd = W_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .tx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while wrinting on SPI");
        
        return status;
}

int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
        u8 cmd = R_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading from SPI");
        
        return status;
}

int write_tx(struct spi_device *spi, u8 *val, int len)
{
        int status;
        u8 cmd = W_TX_PAYLOAD;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .tx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while writing TX payload on SPI");
        
        return status;
}

void ce_high(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 1);
}

void ce_low(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 0);
}
        
void pwr_up(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        if (IS_SETTED(BIT(1), config))
                return;
        config |= BIT(1);       /* set PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}
        
void pwr_down(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        if (!IS_SETTED(BIT(1), config))
                return;
        config &= ~(BIT(1));    /* clear PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}

void enable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config &= ~(BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, &config, 1);
}

void disable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config |= (BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, &config, 1);
}

void prx_mode(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        config |= BIT(0);
        write_reg(spi, CONFIG_REG, &config, 1);
}

void ptx_mode(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        config &= ~(BIT(0));    /* clear PRIM_RX */
        write_reg(spi, CONFIG_REG, &config, 1);
}

int flush_rx(struct spi_device *spi)
{
        int status;
        u8 cmd = FLUSH_RX;

        struct spi_message m;
        struct spi_transfer t = {
                .tx_buf = &cmd,
                .len = 1,
        };


        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_RX command on SPI");
        
        return status;
}

int flush_tx(struct spi_device *spi)
{
        int status;
        u8 cmd = FLUSH_TX;

        struct spi_message m;
        struct spi_transfer t = {
                .tx_buf = &cmd,
                .len = 1,
        };


        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_TX command on SPI");
        
        return status;
}

int read_rx(struct spi_device *spi, u8 *payload, int len)
{
        int status;
        u8 cmd = R_RX_PAYLOAD;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = payload,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload from SPI");

        return status;
}

int read_rx_pld_wid(struct spi_device *spi,
                    u8 *pldwid)
{
        int status;
        u8 cmd = R_RX_PL_WID;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = pldwid,
                        .len = 1,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload size from SPI");

        return status;
        
}
