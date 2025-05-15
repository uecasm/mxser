PWD:=$(shell pwd)
MX_VER_TXT:=$(PWD)/VERSION.txt
MX_VER_MK:=$(PWD)/ver.mk
MX_VER_H:=$(PWD)/mx_ver.h
MX_BUILD_VER:=$(shell awk '{if($$1=="Version" && $$2=="Number:"){print $$3}}' $(MX_VER_TXT))
MX_BUILD_DATE:=$(shell awk '{if($$1=="Date:"){print $$2}}' $(MX_VER_TXT))
MX_CURR_DATE:=$(shell date +%g%m%d%H)

KERNEL_VERSION_MAJOR_NUMBER:=$(shell uname -r | cut -f1 -d.)
KERNEL_VERSION_MINOR_NUMBER:=$(shell uname -r | cut -f2 -d.)

DRIVER_PATH:=driver/kernel4.x

all: check_version mxser
install: check_version clean utility_install driver_install
RHEL8P1: check_version clean utility_install driver_install_rhel8p1

clean: check_version driver_clean utility_clean

remove: check_version uninstall
uninstall: check_version driver_uninstall utility_uninstall

check_version:
ifneq ("$(KERNEL_VERSION_MAJOR_NUMBER)","4")
	@echo "Error: Your kernel version is $(KERNEL_VERSION_MAJOR_NUMBER).x."
	@echo "       This driver only support linux kernel 4.x."
	@exit 1;
endif

utility_make :
	@cd utility;\
	make -s

mxser :
	@cd ${DRIVER_PATH};\
	make -s 

driver_install:
	@echo ""
	@echo " Build driver for Linux kernel ${KERNEL_VERSION_MAJOR_NUMBER}.x"
	@echo ""
	@cd ${DRIVER_PATH};\
	make install -s 

driver_install_rhel8p1:
	@echo ""
	@echo " Build driver for Linux kernel ${KERNEL_VERSION_MAJOR_NUMBER}.x"
	@echo ""
	@cd ${DRIVER_PATH};\
	make RHEL8P1 -s 

utility_install:
	@cd utility;\
	make install -s 


driver_clean:
	@cd ${DRIVER_PATH};\
	make clean -s

utility_clean:
	@cd utility;\
	make clean -s

driver_uninstall:
	@cd ${DRIVER_PATH};\
	make uninstall -s
	@rm -f /etc/systemd/system/moxa_unbind.service
	@rm -f /etc/moxa/moxa_unbind

utility_uninstall:
	@cd utility;\
	make uninstall -s

disk:
ifeq (,$(wildcard $(MX_VER_MK)))
	@touch $(MX_VER_MK)
endif
	@sudo $(MAKE) remove
	@sudo $(MAKE) clean
	@rm -f $(MX_VER_MK)
	@echo -n "DRV_VER=" > $(MX_VER_MK)
	@echo "$(MX_BUILD_VER)" >> $(MX_VER_MK)
	@echo -n "REL_DATE=" >> $(MX_VER_MK)
	@echo "$(MX_BUILD_DATE)" >> $(MX_VER_MK)
	@echo "New $(MX_VER_MK) is created."
	@rm -f $(MX_VER_H)
	@echo "#ifndef _MX_VER_H_" >> $(MX_VER_H)
	@echo "#define _MX_VER_H_" >> $(MX_VER_H)
	@echo -n "#define MX_SER_VERSION \"ver" >> $(MX_VER_H)
	@echo -n "$(MX_BUILD_VER)" >> $(MX_VER_H)
	@echo "\"" >> $(MX_VER_H)
	@echo -n "#define MX_SER_DATE \"" >> $(MX_VER_H)
	@echo -n "$(MX_BUILD_DATE)" >> $(MX_VER_H)
	@echo "\"" >> $(MX_VER_H)
	@echo "#endif" >> $(MX_VER_H)
	@echo "New $(MX_VER_H) is created."
	@rm -f $(MX_VER_MK)
	rm -rfi ../disk/*
	cp -f VERSION.txt ../disk
	cp -rf ../source ../mxser
	tar -cvzf ../disk/driv_linux_smart_v$(MX_BUILD_VER)_build_$(MX_CURR_DATE).tgz ../mxser
	@rm -rf ../mxser
	@echo "Done"
