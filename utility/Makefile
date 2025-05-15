

all: msdiag msmon msterm msestty

install: estty_install diag_install

msdiag:
	cd diag;\
	make;

msmon:
	cd mon;\
	make;

msterm:
	cd term;\
	make;

msestty:
	cd conf;\
	make;


diag_install:
	cd diag;\
	make install

mon_install:
	cd mon;\
	make install

term_install:
	cd term;\
	make install

estty_install:
	cd conf;\
	make install;



clean: global_clean estty_clean diag_clean

global_clean:
	rm -f *.o

diag_clean:
	cd diag;\
	make clean

mon_clean:
	cd mon;\
	make clean

term_clean:
	cd term;\
	make clean

estty_clean:
	cd conf;\
	make clean

uninstall: estty_uninstall diag_uninstall

diag_uninstall:
	cd diag;\
	make uninstall

mon_uninstall:
	cd mon;\
	make uninstall

term_uninstall:
	cd term;\
	make uninstall

estty_uninstall:
	cd conf;\
	make uninstall
