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

## Rapsbery Pi setup.

### Add this to config.txt
```
dtparam=spi=on
dtoverlay=nrf24
```

### Patch this in kernel 

* Tested with branch 3.18.16.y over commit 1bb18c8 at http://github.com/raspberrypi/linux.
```
From f9f7b0ede79835c5fcfa09ca321f48226caa8cad Mon Sep 17 00:00:00 2001
From: Daniel Hilst Selli <danielhilst@gmail.com>
Date: Mon, 12 Oct 2015 13:43:01 -0300
Subject: [PATCH] nrf24 device tree overlay config

---
 arch/arm/boot/dts/overlays/Makefile          |  1 +
 arch/arm/boot/dts/overlays/nrf24-overlay.dts | 44 ++++++++++++++++++++++++++++
 2 files changed, 45 insertions(+)
 create mode 100644 arch/arm/boot/dts/overlays/nrf24-overlay.dts

diff --git a/arch/arm/boot/dts/overlays/Makefile b/arch/arm/boot/dts/overlays/Makefile
index c766616..a21c3c7 100644
--- a/arch/arm/boot/dts/overlays/Makefile
+++ b/arch/arm/boot/dts/overlays/Makefile
@@ -37,6 +37,7 @@ dtb-$(RPI_DT_OVERLAYS) += spi-bcm2835-overlay.dtb
 dtb-$(RPI_DT_OVERLAYS) += tinylcd35-overlay.dtb
 dtb-$(RPI_DT_OVERLAYS) += w1-gpio-overlay.dtb
 dtb-$(RPI_DT_OVERLAYS) += w1-gpio-pullup-overlay.dtb
+dtb-$(RPI_DT_OVERLAYS) += nrf24-overlay.dtb
 
 targets += dtbs dtbs_install
 targets += $(dtb-y)
diff --git a/arch/arm/boot/dts/overlays/nrf24-overlay.dts b/arch/arm/boot/dts/overlays/nrf24-overlay.dts
new file mode 100644
index 0000000..5062754
--- /dev/null
+++ b/arch/arm/boot/dts/overlays/nrf24-overlay.dts
@@ -0,0 +1,44 @@
+/*
+ * Generic Device Tree overlay for the ADS7846 touch controller
+ *
+ */
+
+/dts-v1/;
+/plugin/;
+
+/ {
+	compatible = "brcm,bcm2835", "brcm,bcm2708", "brcm,bcm2709";
+
+	fragment@0 {
+		target = <&spi0>;
+		__overlay__ {
+			status = "okay";
+
+			spidev@0{
+				status = "disabled";
+			};
+
+			spidev@1{
+				status = "disabled";
+			};
+		};
+	};
+
+	fragment@1 {
+		target = <&spi0>;
+		__overlay__ {
+			/* needed to avoid dtc warning */
+			#address-cells = <1>;
+			#size-cells = <0>;
+
+                        nrf24@0 {
+                                compatible = "nrf24";
+                                reg = <0>;
+                                spi-max-frequency = <8000000>;
+                                gpio-irq = <&gpio 24 0>;
+                                gpio-ce  = <&gpio 25 0>;
+                                
+                        };
+		};
+	};
+};
-- 
2.6.0

```

### Compile the module
```
#!bash
cd nrf24 # module folder
export KERNEL_SRC=path/to/linux/raspberrypi
make
cp nrf24.ko path/to/rootfs/home/root/

```

### Load the module
```
@raspberrypi: instmod ~/nrf24.ko
```
If you have CONFIG_TRACING=y it will display a message about it
like this:
```
[   20.500258]
               **********************************************************
               [   20.508544] **   NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE   **
               [   20.516184] **                                                      **
               [   20.522731] ** trace_printk() being used. Allocating extra memory.  **
               [   20.530110] **                                                      **
               [   20.536878] ** This means that this is a DEBUG kernel and it is     **
               [   20.543440] ** unsafe for produciton use.                           **
               [   20.550799] **                                                      **
               [   20.557645] ** If you see this message and you are not debugging    **
               [   20.564210] ** the kernel, report this immediately to your vendor!  **
               [   20.571583] **                                                      **
               [   20.578205] **   NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE   **
               [   20.585386] **********************************************************
               
```

Remove CONFIG_TRACING to get rid of it.

### Testing
```
@raspberrypi: cat /sys/class/spi_master/spi0/spi0.0/config
Data rate:     250kbps
Outputpower:   0dBm
Channel:       2
Address width: 5
EN_DPL:        0
EN_ACK_PAY:    0
EN_DYN_ACK:    0

```
