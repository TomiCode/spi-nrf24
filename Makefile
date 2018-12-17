KERN_DIR ?= ../openwrt/build_dir/target-mipsel_24kc_musl/linux-ramips_mt76x8/linux-4.14.87
MAKE_OPTS= ARCH=mips \
	   CROSS_COMPILE=mipsel-openwrt-linux-musl-

obj-m := spi-nrf24.o

all default: modules
install: modules_install

modules modules_install help clean:
	$(MAKE) -C $(KERN_DIR) $(MAKE_OPTS) M=$$PWD $@

