CURRENT = $(shell uname -r)
KDIR = /lib/modules/${CURRENT}/build
PWD = $(shell pwd)
TARGET1 = ttyABC0
obj-m := $(TARGET1).o

default: 
	@echo try_buikd
	$(MAKE) -C ${KDIR} M=$(PWD) modules
clean:
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order
	@rm -f .*.*.cmd *~ *.*~ TODO.*
	@rm -fR .tmp*
	@rm -rf .tmp_versions
disclean: clean
	@rm *.ko *.symvers