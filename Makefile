CFLAGS_rf_helpers.o += -DDEBUG
CFLAGS_rf_sysfs.o += -DDEBUG
CFLAGS_rf_module.o += -DDEBUG
nrf24-y := rf_module.o rf_helpers.o rf_sysfs.o
obj-m += nrf24.o

KERNEL_SRC ?= "../linux-csi-mx6-3.10.53"

ifneq ($(M),,)
	ccflags-y += -I"$(M)/include/"
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
