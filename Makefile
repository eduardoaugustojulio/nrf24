CFLAGS_rf_helpers.o += -DDEBUG
CFLAGS_rf_sysfs.o += -DDEBUG
CFLAGS_rf_module.o += -DDEBUG
nrf24-y := rf_module.o rf_helpers.o rf_sysfs.o
obj-m += nrf24.o

ifdef M
	ccflags-y += -I"$(M)/include/"
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
