CC = cc -O -c -DLINUX
FILES =  diag.o ../global.o

all:		msdiag

install: msdiag
	@cp -p msdiag /usr/bin

msdiag: 	$(FILES)
	@cc $(FILES) -o msdiag

diag.o: 	diag.c ../global.h
	@$(CC)	diag.c

../global.o:	../global.c
	@$(CC)	../global.c -o ../global.o

clean:
	@rm -f *.o
	@rm -f msdiag

uninstall:
	@rm -f /usr/bin/msdiag
