#ifndef RF_SYSFS_H
#define RF_SYSFS_H

#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

#include "rf_helpers.h"

ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t store_reg(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t count);
ssize_t show_statistics(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t show_config(struct device *dev, struct device_attribute *attr, char *buf);

ssize_t show_scanlink(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t store_scanlink(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

ssize_t store_rftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#endif 
