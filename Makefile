

all: utility_make mxser

SP1: utility_make mxsersp1

SP2: utility_make mxsersp2

install: clean utility_install driver_install
installsp1: utility_install driver_installsp1
installsp2: utility_install driver_installsp2

clean: driver_clean  utility_clean

uninstall: driver_uninstall utility_uninstall


utility_make :
	@cd utility;\
	make -s

mxser :
	@cd driver;\
	make -s 

mxsersp1 :
	@cd driver;\
	make SP1 -s

mxsersp2 :
	@cd driver;\
	make SP2 -s

driver_install:
	@cd driver;\
	make install -s 

driver_installsp1:
	@cd driver;\
	make installsp1 -s
	
driver_installsp2:
	@cd driver;\
	make installsp2 -s
	
utility_install:
	@cd utility;\
	make install -s 


driver_clean:
	@cd driver;\
	make clean -s


utility_clean:
	@cd utility;\
	make clean -s

driver_uninstall:
	@cd driver;\
	make uninstall -s

utility_uninstall:
	@cd utility;\
	make uninstall -s



