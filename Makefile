ifneq ($(KERNELRELEASE),)

include Kbuild

else

KERNEL_SRC ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD clean

endif
