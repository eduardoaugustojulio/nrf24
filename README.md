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

### Device tree expected (iMX.6)
```
/* nRF24L01+ */
&ecspi1 {
        fsl,spi-num-chipselects = <1>;
        cs-gpios = <&gpio4 10 0>;
        pinctrl-names = "default";
        pinctrl-0 = <&pinctrl_ecspi1_2>;
        status = "okay";

        pinctrl_ecspi1_cs_2: ecspi1_cs_grp-2 {
                fsl,pins = <
                MX6QDL_PAD_KEY_ROW1__GPIO4_IO09                 0x80000000 /* nRF24L01+ CE  pin */
                MX6QDL_PAD_KEY_COL2__GPIO4_IO10                 0x80000000 /* SPI1_CS1 */
                MX6QDL_PAD_DISP0_DAT20__GPIO5_IO14              0x80000000 /* nRF24L01+ IRQ pin */
                >;
        };

        nrf24@0 {
                compatible = "nrf24";
                #address-cells = <1>;
                #size-cells = <0>;
                reg = <0>;
                spi-max-frequency = <8000000>;
                gpio-irq = <&gpio5 14 0>;
                gpio-ce  = <&gpio4 9 0>;
        };
};

```