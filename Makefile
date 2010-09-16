# cross-compile module makefile

ifneq ($(KERNELRELEASE),)
    obj-m := omap_capture.o
else
        PWD := $(shell pwd)

default:
ifeq ($(strip $(KERNELDIR)),)
	$(error "KERNELDIR is undefined!")
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif


clean:
	rm -rf *~ *.ko *.o *.mod.c modules.order Module.symvers .omap_capture* .tmp_versions

endif


