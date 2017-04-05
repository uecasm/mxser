CC = cc -O -c -DLINUX
ARCH = $(shell uname -m)

ifeq ($(ARCH), x86_64)
FILES = term.o term_s.o term_m.o term_mp.o term_mf.o termvar.o recvmon.o\
	../mxlib/mxlib_64.a ../mxwinlib/wlib_64.a 
else
FILES = term.o term_s.o term_m.o term_mp.o term_mf.o termvar.o recvmon.o\
	../mxlib/mxlib.a ../mxwinlib/wlib.a 
endif

all:		msterm

install: msterm
	@cp -p msterm /usr/bin

msterm:	$(FILES)
	@cc $(FILES) -o msterm

term.o: 	term.c term.h
	@$(CC)	term.c

term_s.o: 	term_s.c term.h
	@$(CC)	term_s.c

term_m.o: 	term_m.c term.h
	@$(CC)	term_m.c 

term_mp.o: 	term_mp.c term.h
	@$(CC)	term_mp.c 

term_mf.o: 	term_mf.c  term.h
	@$(CC)	term_mf.c

termvar.o:	termvar.c term.h
	@$(CC)	termvar.c 

recvmon.o:	recvmon.c  term.h
	@$(CC)	recvmon.c


clean:
	@rm -f *.o
	@rm -f msterm

uninstall:
	@rm -f /usr/bin/msterm
