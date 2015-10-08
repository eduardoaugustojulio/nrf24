# README #

### nRF24L01+ SPI Linux driver for kernels >= 3.10 

### Compilation

```
#!bash
make KERNEL_SRC=path/to/kernel
# or
export KERNEL_SRC=path/to/kernel
make
```

### Device tree expected
```
        nrf24@0 {
                compatible = "nrf24";
                #address-cells = <1>;
                #size-cells = <0>;
                reg = <0>;
                spi-max-frequency = <8000000>;
                gpio-irq = <&gpio5 14 0>;
                gpio-ce  = <&gpio4 9 0>;
        };
```