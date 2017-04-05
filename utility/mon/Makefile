CC = cc -O -c -DLINUX
ARCH = $(shell uname -m)

ifeq ($(ARCH), x86_64)
FILES =  mon_p.o mon_pa.o ../global.o\
	../mxlib/mxlib_64.a ../mxwinlib/wlib_64.a
else
FILES =  mon_p.o mon_pa.o ../global.o\
	../mxlib/mxlib.a ../mxwinlib/wlib.a
endif

all:		msmon

install: msmon
	@cp -p msmon /usr/bin

msmon: 	$(FILES)
	@cc $(FILES) -o msmon


mon_p.o: 	mon_p.c ../global.h
	@$(CC)	mon_p.c

mon_pa.o: 	mon_pa.c ../global.h
	@$(CC)	mon_pa.c

../global.o:	../global.c
	@$(CC)	../global.c -o ../global.o


clean:
	@rm -f *.o
	@rm -f msmon

uninstall:
	@rm -f /usr/bin/msmon
