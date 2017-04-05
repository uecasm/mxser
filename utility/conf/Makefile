HDR_DIR = -I../../driver/CPLD/Source
CC = cc -O -c -DLINUX
FILES =  conftool.o 

all: muestty

install: muestty
	@cp -f muestty /usr/bin

muestty: $(FILES)
	@cc $(FILES) -o muestty

conftool.o: 	conftool.c 
	@$(CC)	conftool.c $(HDR_DIR)

clean:
	@rm -f *.o
	@rm -f muestty

uninstall:
	@rm -f /usr/bin/muestty
