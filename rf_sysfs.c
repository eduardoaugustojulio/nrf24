#include "rf_sysfs.h"
#include "rf_module.h"

ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct spi_device *spi = to_spi_device(dev);
        int off = 0;
        u8 reg;
        u8 regval[5];

        for (reg = CONFIG_REG; reg <= FIFO_STATUS_REG; reg++) {
               switch (reg) {
                case CONFIG_REG:
                case EN_AA_REG:
                case EN_RXADDR_REG:
                case SETUP_AW_REG :
                case SETUP_RETR_REG:
                case RF_CH_REG:
                case RF_SETUP_REG:
                case STATUS_REG:
                case OBSERVE_TX_REG:
                case CD_REG:
                case RX_PW_P0_REG:
                case RX_PW_P1_REG:
                case RX_PW_P2_REG:
                case RX_PW_P3_REG:
                case RX_PW_P4_REG:
                case RX_PW_P5_REG:
                case FIFO_STATUS_REG:
                case RX_ADDR_P2_REG:
                case RX_ADDR_P3_REG:
                case RX_ADDR_P4_REG:
                case RX_ADDR_P5_REG:
                        read_reg(spi, reg, regval, 1);
                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x\n", reg, regval[0]);
                        break;
                case TX_ADDR_REG:
                case RX_ADDR_P0_REG:
                case RX_ADDR_P1_REG:
                        do {
                                u8 aw;

                                read_reg(spi, SETUP_AW_REG, &aw, 1);
                                aw +=  2;
                                read_reg(spi, reg, regval, 5);
                                switch (aw) {
                                case 3:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2]);
                                        break;
                                case 4:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2], regval[3]);
                                        break;
                                case 5:
                                        off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x%02x%02x%02x%02x\n", reg, 
                                                         regval[0], regval[1], regval[2], regval[3], regval[4]);
                                        break;

                                }
                        } while (0);
                        break;
                default:
                        _DEBUG("Unknwon register addr %02x", reg);
                        break;
                }
        }

        for (reg = DYNPD_REG; reg <= FEATURE_REG; reg++) { 
                read_reg(spi, reg, regval, 1);
                off += scnprintf(buf + off, PAGE_SIZE - off, "%02x:%02x\n", reg, regval[0]);
        }

        return off;
}

ssize_t store_reg(struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
#ifdef DEBUG
        int i;
        int rval;
        u8   regaddr;
        /* The maximum allowed is 5 bytes, used in setting TX_ADDR for example
         * for each byte in string we have 2 chars, (e.g. FF) so 10 chars, plus 1 for '\0'*/
        char regval_str[11];
        int rvlen;
        char hexstr[3];         /* hold hexadecimal strings for a while */
        u8 regval[5];       

        rval = sscanf(buf, "%hhx:%11s", &regaddr, regval_str);
        if (rval != 2) {
                dev_err(dev, "%s: Bad format. Use REG_ADDR:REG_VAL", attr->attr.name);
                return -EINVAL;
        }

        rvlen = strlen(regval_str) / 2;
        for (i = 0; i < rvlen; i++) {
                sscanf(&regval_str[i*2], "%2s", hexstr);
                sscanf(hexstr, "%hhx", &regval[i]);
        }

        write_reg(to_spi_device(dev), regaddr, regval, rvlen);
        
        return count;
#else 
        return -ENOSYS;
#endif 
}

ssize_t show_statistics(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct spi_device *spi = to_spi_device(dev);
        struct rf_data *rf     = spi_get_drvdata(spi);

        return snprintf(buf, PAGE_SIZE,
                        "TX kbps: %5lu\n"
                        "RX kbps: %5lu\n"
                        "TX count: %5lu\n"
                        "RX count: %5lu\n",
                        rf->tx_kbps,
                        rf->rx_kbps,
                        rf->txcount,
                        rf->rxcount);

}

ssize_t show_config(struct device *dev, struct device_attribute *attr, char *buf)
{
        u8 regval;
        unsigned off = 0;
        struct spi_device *spi = to_spi_device(dev);

        read_reg(spi, RF_SETUP_REG, &regval, 1);
        if (IS_SETTED(BIT(5), regval))
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     250kbps\n");
        else if (IS_SETTED(BIT(3), regval))
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     2mbps\n");
        else
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Data rate:     1mbps\n");

        switch ((regval & 0x6) >> 1) { /* get bit 1 and 2 */
        case 0:                        /* 00 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -18dBm\n");
                break;
        case 1:                 /* 01 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -12dBm\n");
                break;
        case 2:                 /* 10 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   -6dBm\n");
                break;
        case 3:                 /* 11 */
                off += snprintf(buf + off, PAGE_SIZE - off,
                                "Outputpower:   0dBm\n");
                break;
        }

        read_reg(spi, RF_CH_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "Channel:       %u\n", regval);

        read_reg(spi, SETUP_AW_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "Address width: %u\n", regval + 2);

        read_reg(spi, FEATURE_REG, &regval, 1);
        off += snprintf(buf + off, PAGE_SIZE - off,
                        "EN_DPL:        %u\n"
                        "EN_ACK_PAY:    %u\n"
                        "EN_DYN_ACK:    %u\n",
                        (IS_SETTED(BIT(2), regval) ? 1 : 0),
                        (IS_SETTED(BIT(1), regval) ? 1 : 0),
                        (IS_SETTED(BIT(0), regval) ? 1 : 0));


        return off;
}
