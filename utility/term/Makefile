#LINUX_DIS = $(shell grep '[0-9]' /etc/issue)
LINUX_DIS := $(shell cat /etc/issue | cut -f1 -d ' ')
DISTRIBUTION_VERSION_MAJOR_NUMBER := $(shell cat /etc/issue | cut -f2 -d' ' | cut -f1 -d'.')
DISTRIBUTION_VERSION_MINOR_NUMBER := $(shell cat /etc/issue | cut -f2 -d' ' | cut -f2 -d'.')

CC = cc -O -c -DLINUX -w
FLAGS = -Wno-implicit-function-declaration -fPIC
ARCH = $(shell uname -m)

ifeq ($(ARCH), x86_64)
FILES = term.o term_s.o term_m.o term_mp.o term_mf.o termvar.o recvmon.o\
	../mxlib/mxlib_64.a ../mxwinlib/wlib_64.a 
else
FILES = term.o term_s.o term_m.o term_mp.o term_mf.o termvar.o recvmon.o\
	../mxlib/mxlib.a ../mxwinlib/wlib.a 
endif

all: msterm

install: msterm
	@cp -p msterm /usr/bin

msterm:	$(FILES)
ifeq ("$(LINUX_DIS)","Ubuntu")
ifeq ("$(shell test $(DISTRIBUTION_VERSION_MAJOR_NUMBER) -ge 17; echo $$?)","0")
ifeq ("$(shell test $(DISTRIBUTION_VERSION_MINOR_NUMBER) -ge 04; echo $$?)","0")
	@cc -no-pie -static $(FILES) -o msterm
else
	@cc -no-pie $(FILES) -o msterm
endif
else
	@cc -no-pie $(FILES) -o msterm
endif
else
ifeq ("$(shell gcc -v 2>&1 | grep pie)", "")
	@cc $(FILES) -o msterm
else
	@cc -no-pie $(FILES) -o msterm
endif
endif
term.o: 	term.c term.h
	@$(CC) ${FLAGS} term.c

term_s.o: 	term_s.c term.h
	@$(CC) ${FLAGS} term_s.c

term_m.o: 	term_m.c term.h
	@$(CC) ${FLAGS} term_m.c 

term_mp.o: 	term_mp.c term.h
	@$(CC) ${FLAGS} term_mp.c 

term_mf.o: 	term_mf.c  term.h
	@$(CC) ${FLAGS} term_mf.c

termvar.o:	termvar.c term.h
	@$(CC) ${FLAGS} termvar.c 

recvmon.o:	recvmon.c  term.h
	@$(CC) ${FLAGS} recvmon.c


clean:
	@rm -f *.o
	@rm -f msterm

uninstall:
	@rm -f /usr/bin/msterm
