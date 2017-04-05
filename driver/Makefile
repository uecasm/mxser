MXVER:=1.16
TARGET_DRIVER1:=mxser
TARGET_DRIVER2:=mxupcie
LVER:=$(shell uname -r)
MOD_DIR:= /lib/modules/$(LVER)/kernel/drivers/char/
EXTRA_CFLAGS  += -DOS_LINUX
##############################################################
ifneq	"$(findstring 2.4., $(shell uname -a))" "2.4."

all: module

SP2: module2 

install: module
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	$(shell [ -e $(MOD_DIR)/mxser.ko.gz ] && mv $(MOD_DIR)/mxser.ko.gz $(MOD_DIR)/mxser.ko.gz.bk)
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/misc/
	@depmod -a
	@echo " "
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " installed successfully."
	@echo "**********************************************************************"

installsp2: module2
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).ko /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).ko /lib/modules/$(LVER)/misc/
	@depmod -a
	@echo " "
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " installed successfully."
	@echo "**********************************************************************"

ifneq ($(KERNELRELEASE),)
obj-m := $(TARGET_DRIVER1).o $(TARGET_DRIVER2).o 
$(TARGET_DRIVER2)-objs := mxpcie.o ./CPLD/Source/cpld.o ./CPLD/Source/gpio.o
else
KDIR	:= /lib/modules/$(LVER)/build
PWD	:= $(shell pwd)

module: envchk
	@$(MAKE) -s -C $(KDIR) SUBDIRS=$(PWD) modules 2>> build.log \
	&& cp -f ./build.log ../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../build.log \
	&& exit 1

module2: envchk 
	@$(MAKE) EXTRA_CFLAGS=-DSP2 -s -C $(KDIR) SUBDIRS=$(PWD) modules 2>> build.log \
	&& cp -f ./build.log ../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../build.log \
	&& exit 1

envchk:
	@./mxconf | tee build.log

clean: clean_local

endif

uninstall: clean_lib unload_driver

clean_local:
	@rm -f *.o
	@rm -rf ./.tmp_versions
	@rm -f $(TARGET_DRIVER1).mod*
	@rm -f $(TARGET_DRIVER2).mod*
	@rm -f .$(TARGET_DRIVER1)*
	@rm -f .$(TARGET_DRIVER2)*
	@rm -f $(TARGET_DRIVER1).ko
	@rm -f $(TARGET_DRIVER2).ko
	@rm -f mxpcie.o
	@rm -f .mxpcie.o.cmd
	@rm -f mxupcie.o	
	@rm -f .mxupcie.o.cmd
	@rm -f *.symvers
	@rm -f *.log
	@rm -f ../*.log
	@rm -f *.markers
	@rm -f *.order
	@rm -f CPLD/Source/*.o
	@rm -f CPLD/Source/.cpld.o.cmd CPLD/Source/.gpio.o.cmd

clean_lib:
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER1).ko
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER2).ko
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER1).ko
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER2).ko
	@rm -f /dev/mxser
	@rm -f /dev/mxupcie
	@depmod -a
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " removed successfully."
	@echo "**********************************************************************"

unload_driver:	
	@if lsmod | grep -q $(TARGET_DRIVER1) ; then rmmod $(TARGET_DRIVER1) ; fi
	@if lsmod | grep -q $(TARGET_DRIVER2) ; then rmmod $(TARGET_DRIVER2) ; fi



else      #Linux 2.4########################################

CC=gcc -c
ARCH=$(shell uname -m | sed s/i.86/i86/)

ifeq	($(ARCH),i86)
OPTS=-DMODULE -D__KERNEL__ -Wall -Wstrict-prototypes -O2 \
-fomit-frame-pointer -I/usr/src/linux-2.4/include \
-I/usr/src/linux/include
#OPTS=-DMODULE -D__KERNEL__ -Wall -Wstrict-prototypes -O2 \
#-fomit-frame-pointer -pipe -m486 -I/usr/src/linux-2.4/include \
#-I/usr/src/linux/include
else
OPTS=-DMODULE -D__KERNEL__ -Wall -Wstrict-prototypes -O2 \
-fomit-frame-pointer -pipe -I/usr/src/linux/include -ffixed-8
endif

ifeq	"$(findstring install, $(MAKECMDGOALS))" "install"
OPTS+=-DMODVERSIONS
endif


ifeq	"$(findstring SMP, $(shell uname -a))" "SMP"
OPTS+=-D__SMP__
endif

all: module

SP1: modulesp1

installmod: install

install: module
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER1).o /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).o /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).o /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).o /lib/modules/$(LVER)/misc/
	@depmod -a
	@echo " "
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " installed successfully."
	@echo "**********************************************************************"


installsp1: modulesp1
	@mkdir -p /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER1).o /lib/modules/$(LVER)/kernel/drivers/char/
	@cp -f ./$(TARGET_DRIVER2).o /lib/modules/$(LVER)/kernel/drivers/char/
	@mkdir -p /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER1).o /lib/modules/$(LVER)/misc/
	@cp -f ./$(TARGET_DRIVER2).o /lib/modules/$(LVER)/misc/
	@depmod -a
	@echo " "
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " installed successfully."
	@echo "**********************************************************************"


clean : clean_local

uninstall: clean_lib unload_driver

clean_local:
	@rm -f *.o
	@rm -f *.log
	@rm -f ../*.log

clean_lib:
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER1).o
	@rm -f /lib/modules/$(LVER)/kernel/drivers/char/$(TARGET_DRIVER2).o
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER1).o
	@rm -f /lib/modules/$(LVER)/misc/$(TARGET_DRIVER2).o
	@rm -f /dev/mxser
	@rm -f /dev/mxupcie
	@echo "**********************************************************************"
	@echo " MOXA Smartio/Industio Family Multiport Board Driver ver $(MXVER)"
	@echo " removed successfully."
	@echo "**********************************************************************"

unload_driver:
	@if lsmod | grep -q $(TARGET_DRIVER1) ; then rmmod $(TARGET_DRIVER1) ; fi
	@if lsmod | grep -q $(TARGET_DRIVER2) ; then rmmod $(TARGET_DRIVER2) ; fi

module : envchk $(TARGET_DRIVER1).c $(TARGET_DRIVER2).c
	@$(CC) $(OPTS) $(TARGET_DRIVER1).c $(TARGET_DRIVER2).c \
	2>> build.log && cp -f ./build.log ../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../build.log \
	&& exit 1
	
modulesp1 : envchk $(TARGET_DRIVER1).c $(TARGET_DRIVER2).c
	@$(CC) $(OPTS) -DSP1 $(TARGET_DRIVER1).c $(TARGET_DRIVER2).c \
	2>> build.log && cp -f ./build.log ../build.log \
	&& exit 0 \
	|| echo "Build Fail! Please check build.log for further information" \
	&& echo "and send it to support@moxa.com." \
	&& cp -f ./build.log ../build.log \
	&& exit 1
	
envchk:
	@./mxconf | tee build.log

dummy:

endif
