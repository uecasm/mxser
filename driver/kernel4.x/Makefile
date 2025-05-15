MXVER:=`awk '{if($$2=="MX_SER_VERSION"){print $$3}}' ../../mx_ver.h | sed s/\"//g`
TARGET_DRIVER1:=mxser
TARGET_DRIVER2:=mxupcie
CPLD_PATH:=../CPLD/Source
LVER:=$(shell uname -r)
MOD_DIR:= /lib/modules/$(LVER)/kernel/drivers/char/
EXTRA_CFLAGS  += -DOS_LINUX
SP1 += -DRHEL8_PATCH1
KER_DVR1_DIR:= /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).ko

##############################################################
all: module
install: module
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	$(shell [ -e $(MOD_DIR)/mxser.ko.gz ] && mv $(MOD_DIR)/mxser.ko.gz $(MOD_DIR)/mxser.ko.gz.bk)
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/misc/
ifeq ($(KER_DVR1_DIR), $(wildcard $(KER_DVR1_DIR)))
	@mv /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).bak
	@depmod -a
	@mv /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).bak /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).ko
else
	@depmod -a
endif
	@echo " "
	@echo "**********************************************************************"
	@echo -n " MOXA Smartio/Industio Family Multiport Board Driver "
	@echo $(MXVER)
	@echo " installed successfully."
	@echo "**********************************************************************"

RHEL8P1: rhel8p1
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	$(shell [ -e $(MOD_DIR)/mxser.ko.gz ] && mv $(MOD_DIR)/mxser.ko.gz $(MOD_DIR)/mxser.ko.gz.bk)
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/misc/
ifeq ($(KER_DVR1_DIR), $(wildcard $(KER_DVR1_DIR)))
	@mv /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).bak
	@depmod -a
	@mv /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).bak /lib/modules/$(LVER)/kernel/drivers/tty/$(TARGET_DRIVER1).ko
else
	@depmod -a
endif
	@echo " "
	@echo "**********************************************************************"
	@echo -n " MOXA Smartio/Industio Family Multiport Board Driver "
	@echo $(MXVER)
	@echo " installed successfully."
	@echo "**********************************************************************"

ifneq ($(KERNELRELEASE),)
obj-m := $(TARGET_DRIVER1).o $(TARGET_DRIVER2).o 
$(TARGET_DRIVER2)-objs := mxpcie.o ${CPLD_PATH}/cpld.o ${CPLD_PATH}/gpio.o
else
KDIR	:= /lib/modules/$(LVER)/build
PWD	:= $(shell pwd)

module: envchk
	@$(MAKE) -s -C $(KDIR) M=$(PWD) modules 2>> build.log \
	&& cp -f ./build.log ../../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../../build.log \
	&& exit 1

rhel8p1: envchk
	@$(MAKE) EXTRA_CFLAGS+="-DOS_LINUX -DRHEL8_PATCH1" -s -C $(KDIR) M=$(PWD) modules 2>> build.log \
	&& cp -f ./build.log ../../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../../build.log \
	&& exit 1

envchk:
	@./mxconf | tee build.log

clean: clean_local

endif

uninstall: clean_lib unload_driver remove_msg

clean_local:
	@rm -f *.o
	@rm -rf ./.tmp_versions
	@rm -f $(TARGET_DRIVER1).mod*
	@rm -f $(TARGET_DRIVER2).mod*
	@rm -f .$(TARGET_DRIVER1)*
	@rm -f .$(TARGET_DRIVER2)*
	@rm -f $(TARGET_DRIVER1).ko
	@rm -f $(TARGET_DRIVER2).ko
	@rm -f mxser.o.ur-safe
	@rm -f mxpcie.o.ur-safe
	@rm -f .cache.mk
	@rm -f mxpcie.o
	@rm -f .mxpcie.o.cmd
	@rm -f mxupcie.o	
	@rm -f .mxupcie.o.cmd
	@rm -f *.symvers
	@rm -f *.log
	@rm -f ../../*.log
	@rm -f *.markers
	@rm -f *.order
	@rm -f ${CPLD_PATH}/*.o
	@rm -f ${CPLD_PATH}/.cpld.o.cmd ${CPLD_PATH}/.gpio.o.cmd

clean_lib:
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER1).ko
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER2).ko
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER1).ko
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER2).ko
	@rm -f /dev/mxser
	@rm -f /dev/mxupcie
	@rm -f /etc/moxa/moxa_unbind
	@rm -f /etc/systemd/system/moxa_unbind.service
	@depmod -a

unload_driver:	
	@if lsmod | grep -q $(TARGET_DRIVER1) ; then rmmod $(TARGET_DRIVER1) ; fi
	@if lsmod | grep -q $(TARGET_DRIVER2) ; then rmmod $(TARGET_DRIVER2) ; fi

remove_msg:
	@echo "**********************************************************************"
	@echo -n " MOXA Smartio/Industio Family Multiport Board Driver "
	@echo $(MXVER)
	@echo " removed compeleted."
	@echo "**********************************************************************"
