HDR_DIR = -I../../driver/CPLD/Source
CC = cc -O -c -DLINUX
FILES =  conftool.o ../global.o

all: muestty

install: muestty
	@cp -f muestty /usr/bin

muestty: $(FILES)
	@cc $(FILES) -o muestty

conftool.o: 	conftool.c ../global.h 
	@$(CC)	conftool.c $(HDR_DIR)

../global.o:	../global.c
	@$(CC) ../global.c -o ../global.o

clean:
	@rm -f *.o
	@rm -f muestty

uninstall:
	@rm -f /usr/bin/muestty
