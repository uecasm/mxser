LINUX_DIS := $(shell cat /etc/issue | cut -f1 -d' ')
DISTRIBUTION_VERSION_MAJOR_NUMBER := $(shell cat /etc/issue | cut -f2 -d' ' | cut -f1 -d'.')
DISTRIBUTION_VERSION_MINOR_NUMBER := $(shell cat /etc/issue | cut -f2 -d' ' | cut -f2 -d'.')

CC = cc -O -c -DLINUX -w
FLAGS = -Wno-implicit-function-declaration -fPIC
ARCH = $(shell uname -m)

ifeq ($(ARCH), x86_64)
FILES =  mon_p.o mon_pa.o ../global.o\
	../mxlib/mxlib_64.a ../mxwinlib/wlib_64.a
else
FILES =  mon_p.o mon_pa.o ../global.o\
	../mxlib/mxlib.a ../mxwinlib/wlib.a
endif

all: msmon

install: msmon
	@cp -p msmon /usr/bin

msmon: 	$(FILES)
ifeq ("$(LINUX_DIS)","Ubuntu") 
ifeq ("$(shell test $(DISTRIBUTION_VERSION_MAJOR_NUMBER) -ge 17; echo $$?)","0")
ifeq ("$(shell test $(DISTRIBUTION_VERSION_MINOR_NUMBER) -ge 04; echo $$?)","0")
	@cc -no-pie -static $(FILES) -o msmon
else
	@cc -no-pie $(FILES) -o msmon
endif
else
	@cc -no-pie $(FILES) -o msmon
endif
else
ifeq ("$(shell gcc -v 2>&1 | grep pie)", "")
	@cc $(FILES) -o msmon
else
	@cc -no-pie $(FILES) -o msmon
endif
endif
mon_p.o: 	mon_p.c ../global.h
	@$(CC) ${FLAGS} mon_p.c

mon_pa.o: 	mon_pa.c ../global.h
	@$(CC) ${FLAGS} mon_pa.c

../global.o:	../global.c
	@$(CC)	../global.c -o ../global.o


clean:
	@rm -f *.o
	@rm -f msmon

uninstall:
	@rm -f /usr/bin/msmon
