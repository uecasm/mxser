/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	conftool.c
*/

/*****************************************************************************/
/* Program:								      */
/* 		MOXA New PCIe Device Utility				      */
/*									      */
/* History:								      */
/* 2008/04/29								      */
/*									      */
/* Author:								      */
/* Joy Tu						      */
/*									      */
/* Discription:								      */
/* The utility is to set or get the MOXA new PCIe configurations	      */
/*									      */
/******************************************************************************/

#include        <stdlib.h>
#include	<stdio.h>
#include        <string.h>
#include        <fcntl.h>
#include	<getopt.h>
#include	<unistd.h>
#include	<errno.h>
#include	"cpld_param.h"
#include 	<termios.h>
#include 	<linux/version.h>
#include 	<sys/ioctl.h>

#include	"../global.h"

#define VERSION_CODE(ver,rel,seq)   ((ver << 16) | (rel << 8) | seq)
#define MX_IOCTL_CODE		0x400
//#define MOXA			MX_IOCTL_CODE
#define MX_SET_INTERFACE	(MX_IOCTL_CODE+79)
#define MX_GET_INTERFACE	(MX_IOCTL_CODE+80)
#define MX_SET_TERM		(MX_IOCTL_CODE+81)
#define MX_GET_TERM		(MX_IOCTL_CODE+82)
#define SMARTIO_PUART_SET_PULL_STATE    (MOXA + 83)
#define SMARTIO_PUART_GET_PULL_STATE    (MOXA + 84)
#define SMARTIO_PUART_SET_AUTO_MODE     (MOXA + 85)
#define SMARTIO_PUART_GET_AUTO_MODE     (MOXA + 86)
#define SMARTIO_PUART_SET_MASTER_SLAVE  (MOXA + 87)
#define SMARTIO_PUART_GET_MASTER_SLAVE  (MOXA + 88)
#define SMARTIO_PUART_SET_DIAGNOSE      (MOXA + 89)
#define SMARTIO_PUART_GET_ALARM		(MOXA + 90)

#define SMARTIO_SET_PCI_CAPABILITY	(MOXA + 91)
#define SMARTIO_GET_PCI_CAPABILITY	(MOXA + 92)

#define MOXA_GET_MAJOR			(MOXA + 63)
#define MOXA_GET_CUMAJOR		(MOXA + 64)


#define MX_RS232		0x00
#define MX_RS422		0x01
#define MX_RS485_2W		0x0f
#define MX_RS485_4W		0x0b
#define MX_TERM_NONE		0x00
#define MX_TERM_120		0x01

#define OPT_OPEN		1
#define OPT_HELP		2
#define OPT_SET_INTERFACE	4
#define OPT_GET_INTERFACE	8
#define OPT_SET_TERM		16
#define OPT_GET_TERM		32

#define OPT_SET_MODE		64	/* Set Auto-PNP mode under RS-485 2W */
#define OPT_GET_MODE		128
#define OPT_SET_STATE		256
#define OPT_GET_STATE		512	/* Set Pull High Resistor */
#define OPT_SET_DIAG		1024
#define OPT_SHOW_BOARD		2048
#define OPT_PCI_CAP		4096

static int mx_open_device(char *dev);
static int mx_set_interface(int fd, char *dev, char *parm);
static int mx_set_terminator(int fd, char *dev, char *parm);
static int mx_set_resistor_state(int fd, char *dev, char *parm);
static int mx_set_auto_pnp_mode(int fd, char *dev, char *parm);
static void mx_get_interface_term(int fd, char *dev);
static void mx_do_diagnose(int fd, char *dev, char *parm);
static void mx_show_help();
static void mx_set_pci_capability(char *dev, char *parm);
static void mx_show_board();
static void mx_make_mxupcie_node();
static int mx_get_mxupcie_major();

int main(int argc,char *argv[])
{
	int fd, c, opt;
	char *dev, *parm;
	extern char *optarg;
	extern int optind, opterr, optopt;

	opt = 0;
	while ((c = getopt(argc, argv, "i:t:a:gh:p:d:vv:")) != -1) {
		switch (c) {
			case 'i':
				opt |= (OPT_OPEN | OPT_SET_INTERFACE);
				parm = optarg;
				break;
			case 't':
				opt |= (OPT_OPEN | OPT_SET_TERM);
				parm = optarg;
				break;
			case 'g':
				opt |= (OPT_OPEN | OPT_GET_INTERFACE | 
					OPT_GET_TERM | OPT_GET_MODE |
					OPT_GET_STATE);
				break;
			case 'h':
			case '?':
				opt |= OPT_HELP;
				break;
			case 'a':
				if (argc != 4)      
					printf("muestty: Please specify the baud rate and device.\n");
				else {
					opt |= (OPT_OPEN | OPT_SET_MODE);
					parm = optarg;
				}
				break;
			case 'p':
				opt |= (OPT_OPEN | OPT_SET_STATE);
				parm = optarg;
				break;
			case 'd':
				if (argc != 4)
					printf("muestty: Please specify the baud rate and device.\n");
				else {
					opt |= (OPT_OPEN | OPT_SET_DIAG);
					parm = optarg;				
				}
				break;
			case 'v':
				if (argc != 4) {
					if (argc == 2)
					{
						opt |= (OPT_SHOW_BOARD);
					}
					else
						printf("muestty: Please input a parameter and board number.\n");
				}
				else {
					opt |= (OPT_PCI_CAP);
				}
				break;
		}
	}
	
	if (!opt)
		opt |= OPT_HELP;

	dev = argv[optind];
	if (opt & OPT_OPEN) {
		if ((fd = mx_open_device(dev)) < 0)
			exit(0);
	}
	if (opt & OPT_SET_INTERFACE)
		mx_set_interface(fd, dev, parm);
	else if (opt & OPT_SET_TERM)
		mx_set_terminator(fd, dev, parm);
	else if ((opt & OPT_GET_INTERFACE) && (opt & OPT_GET_TERM))
		mx_get_interface_term(fd,dev);
	else if(opt & OPT_HELP)
		mx_show_help();
	else if (opt & OPT_SET_STATE)
		mx_set_resistor_state(fd, dev, parm);
	else if (opt & OPT_SET_MODE)
		mx_set_auto_pnp_mode(fd, dev, parm);
	else if (opt & OPT_SET_DIAG)
		mx_do_diagnose(fd, dev, parm);
	else if (opt & OPT_SHOW_BOARD)
		mx_show_board();
	else if (opt & OPT_PCI_CAP){
		parm = argv[2];
		dev = argv[3];
		mx_set_pci_capability(dev, parm);
	}
	close(fd);

	return 0;
}


static int mx_open_device(char *dev)
{
	int fd;
	fd = open(dev,O_RDWR);

	if (fd < 0) {
		printf("muestty: Open device %s error (%d)!\n",dev,fd);
		return -1;
	}
	
	return fd;
}


static int mx_set_interface(int fd, char *dev, char *parm)
{
	int ret;
	char mode;
	
	mode = 0;

	if (strncmp(parm,"RS232",5)==0 && strlen(parm)==5)
		mode = MX_RS232;
	else if (strncmp(parm,"RS422",5)==0 && strlen(parm)==5)
		mode = MX_RS422;
	else if (strncmp(parm,"RS4852W",7)==0 && strlen(parm)==7)
		mode = MX_RS485_2W;
	else if (strncmp(parm,"RS4854W",7)==0 && strlen(parm)==7)
		mode = MX_RS485_4W;
	else 	
		mode = -1;

	if (mode == -1) {
		printf("muestty : Invalid parameter. '%s' is an illegal parameter.\n", parm);
		return -1;
	}

	if ((ret = ioctl(fd,MX_SET_INTERFACE,mode)) < 0) {
		if(errno == EINVAL)
			printf("muestty : Invalid operation, %s not support this operation.\n", dev);
		else if(errno == EOPNOTSUPP)
			printf("muestty : Invalid operation, %s not support this interface.\n", dev);
		else
			printf("muestty : Invalid operation of MOXA Smartio MUE series device.\n");
		return -5;
	}

	printf("muestty: Set interface of %s ok.\n", dev);

	return 0;
}

static int mx_set_terminator(int fd, char *dev, char *parm)
{
	int ret;
	char mode;

	ret = ioctl(fd, MX_GET_INTERFACE, &mode);
	if (mode == MX_RS232) {
		printf("muestty: Terminator cannot be set when interface is RS232 mode.\n");
		return -1;
	}

	mode = 1;
	if (strncmp(parm, "NONTERM", 7) == 0)
		mode = MX_TERM_NONE;
	else if (strncmp(parm, "120TERM", 7) == 0)
		mode = MX_TERM_120;	
	else	
		mode = -1;

	if (mode == -1) {
		printf("muestty : Invaild parameter. '%s' is an illegal parameter.\n", parm);
		return -1;
	}

	if ((ret = ioctl(fd, MX_SET_TERM, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}
	printf("muestty : Set terminator of %s ok.\n", dev);
	return 0;
}

static int mx_set_resistor_state(int fd, char *dev, char *parm)
{
	int ret;
	char mode;
	
        ioctl(fd, MX_GET_INTERFACE, &mode);
        if (mode == MX_RS232) {
                printf("muestty: Resistor state cannot be set when interface is RS232 mode.\n");
                return -1;
        }

	mode = 1;
	if (strncmp(parm, "150K", 4) == 0)
		mode = MX_CPLD_PULL_OFF;
	else if (strncmp(parm, "1K", 2) == 0)
		mode = MX_CPLD_PULL_ON;
	else {
		printf("muestty : Invaild parameter for setting resistor state.\n");
		printf("          '%s' is an illegal parameter.\n", parm);
		return -1;
	}

	if ((ret = ioctl(fd, SMARTIO_PUART_SET_PULL_STATE, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	printf("muestty : Set pull resistor state of %s ok.\n", dev);

	return 0;
}

static int mx_set_auto_pnp_mode(int fd, char *dev, char *parm)
{
	int ret, terminator, hl_resistor, alarm;
	int old_terminator = 0, old_hl_resistor = 0;
	int c, baud;
	char mode, inter;
	struct termios options;

	inter = 0, terminator = 0, hl_resistor = 0;

	ret = ioctl(fd, MX_GET_INTERFACE, &inter);
	if (inter != MX_RS485_2W) {
		printf("muestty : Invaild Operation, %s interface must be RS485_2W\r\n", dev);
		return -1;
	}

 	baud = atoi(parm);
	if (baud == 921600)
		baud = B921600;
	else if (baud == 460800)
		baud = B460800;
	else if (baud == 230400)
		baud = B230400;
	else if (baud == 115200)
		baud = B115200;
	else if (baud == 57600)
		baud = B57600;
	else if (baud == 38400)
		baud = B38400;
	else if (baud == 19200)
		baud = B19200;
	else if (baud == 9600)
		baud = B9600;
	else
		baud = -1;

	if(baud == -1) {
		printf("muestty : Invalid value of baud rate.\n");
		printf("          '%s' is an illegal parameter.\n", parm);
		return -1;
	}
	tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
	tcsetattr(fd, TCSANOW, &options);

	printf("\r\n");
	printf("Start tuning resistor...\r\n");
	printf("\r\n");

	/* Save old terminator/resistor register */
	if ((ret = ioctl(fd, MX_GET_TERM, &old_terminator)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	if ((ret = ioctl(fd, SMARTIO_PUART_GET_PULL_STATE, &old_hl_resistor)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}
	

	mode = MX_TERM_NONE;
	if ((ret = ioctl(fd, MX_SET_TERM, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}
	mode = MX_CPLD_PULL_OFF;

	if ((ret = ioctl(fd, SMARTIO_PUART_SET_PULL_STATE, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	if ((ret = ioctl(fd, SMARTIO_PUART_SET_AUTO_MODE, MX_CPLD_AUTO_ON)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	sleep(2);

	if ((ret = ioctl(fd, SMARTIO_PUART_SET_AUTO_MODE, MX_CPLD_AUTO_OFF)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	if ((ret = ioctl(fd, MX_GET_TERM, &terminator)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}
	if ((ret = ioctl(fd, SMARTIO_PUART_GET_PULL_STATE, &hl_resistor)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	printf("[Status]\r\n");
	printf("\r\n");	
	if (hl_resistor == MX_CPLD_PULL_OFF) {
		printf("Pull High/Low Resistor	: 150K\r\n");
	} else if (hl_resistor == MX_CPLD_PULL_ON) {
		printf("Pull High/Low Resistor	: 1K\r\n");
	} else {
		printf("Pull High/Low Resistor	: unknown\r\n");
	}

	if (terminator == MX_TERM_NONE) {
		printf("Terminator Resistor	: None\r\n");
	} else if (terminator == MX_TERM_120) {
		printf("Terminator Resistor	: 120 ohm\r\n");
	} else {
		printf("Terminator Resistor	: unknown\r\n");
	}

	/* Restore old terminator/resistor register */
	mode = old_terminator;

	if ((ret = ioctl(fd, MX_SET_TERM, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	mode = old_hl_resistor;

	if ((ret = ioctl(fd, SMARTIO_PUART_SET_PULL_STATE, mode)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	if ((ret = ioctl(fd, SMARTIO_PUART_GET_ALARM, &alarm)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return -1;
	}

	alarm &= 0x03;

	if (alarm == 0) {
		printf("Tuning Status		: OK\r\n");
	} else {
		printf("Tuning Status		: Fail\r\n");
	}	
	printf("\r\n");
	printf("Note 1. To execute this command again if the serial device\r\n");
	printf("        has been changed on the bus\r\n");
	printf("     2. If the communication is correctly, record the related\r\n");
	printf("	resistor value and init the setting on the rc.mxser file.\r\n");
	printf("	Otherwise, execute the diagnose to get the error status.\r\n");
	printf("\r\n");
	printf("Done.\r\n");


	printf("\r\n");
	printf("Make these values effective immediately? [Y/n] (Enter for default=Y):\r\n");
	c = getchar();
	if(c == 'n' || c == 'N') 
		;	// No => do nothing.
	else {
			// Yes => effect immediately.
		if ((ret = ioctl(fd, SMARTIO_PUART_SET_PULL_STATE, hl_resistor)) < 0) {
			if(ret == -EINVAL)
				printf("muestty : Invaild operation, %s not support this operation.\n", dev);
			else
				printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
			return -1; 
		}   
		if ((ret = ioctl(fd, MX_SET_TERM, terminator)) < 0) {
			if(ret == -EINVAL)
				printf("muestty : Invaild operation, %s not support this operation.\n", dev);
			else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
			return -1;
		}              
		printf("The values have been set now.\r\n");
	}

	return 0;
}

static void mx_do_diagnose(int fd, char *dev, char *parm)
{
	int ret, terminator, hl_resistor, alarm;
	int baud;
	char inter;
	struct termios options;

	inter = 0, terminator = 0, hl_resistor = 0;

	ret = ioctl(fd, MX_GET_INTERFACE, &inter);
	if (inter != MX_RS485_2W) {
		printf("muestty : Invaild Operation, %s interface must be RS485_2W\r\n", dev);
		return;
	}

	if ((ret = ioctl(fd, MX_GET_TERM, &terminator)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return;
	}
	if ((ret = ioctl(fd, SMARTIO_PUART_GET_PULL_STATE, &hl_resistor)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
		return;
	}

  	baud = atoi(parm);
	if (baud == 921600)
		baud = B921600;
	else if (baud == 460800)
		baud = B460800;
	else if (baud == 230400)
		baud = B230400;
	else if (baud == 115200)
		baud = B115200;
	else if (baud == 57600)
		baud = B57600;
	else if (baud == 38400)
		baud = B38400;
	else if (baud == 19200)
		baud = B19200;
	else if (baud == 9600)
		baud = B9600;
	else
		baud = -1;

	if(baud == -1) {
		printf("muestty : Invalid value of baud rate.\n");
		printf("          '%s' is an illegal parameter.\n", parm);
		return;
	}    
	tcgetattr(fd, &options);
	cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        tcsetattr(fd, TCSANOW, &options);


	printf("\r\n");
	printf("Start diagnosing... \r\n");

	// turn on diagnose
	if ((ret = ioctl(fd, SMARTIO_PUART_SET_DIAGNOSE, MX_CPLD_DIAG_ON)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
			printf("muestty : Invalid operation of MOXA Smartio MUE series device.\r\n");
		return ;
	}

	sleep(2);
	ioctl(fd, SMARTIO_PUART_GET_ALARM, &alarm);
	alarm &= 0x03;
	//while (1) {
		
		// exit when press ESC
		//printf("Press <Ctrl+C> to exit diagnosis.\r\n");
		
		// get alarm
		//ioctl(fd, SMARTIO_PUART_GET_ALARM, &alarm);
		//sleep(2);
		//printf("muestty : Alarm code = %d.\r\n", alarm);

	//	if (sleep(2) != 0)		// 2-second interval
	//		break;
	//}
		
	printf("\r\n");
	printf("[Status] \r\n");
	printf("\r\n");	
    printf("Notice: Following results are based on correct devices connection.\r\n");
	printf("\r\n");

	if (hl_resistor == MX_CPLD_PULL_OFF) {
		printf("Pull High/Low Resistor	: 150K\r\n");
	} else if (hl_resistor == MX_CPLD_PULL_ON) {
		printf("Pull High/Low Resistor	: 1K\r\n");
	} else {
		printf("Pull High/Low Resistor	: unknown\r\n");
	}

	if (terminator == MX_TERM_NONE) {
		printf("Terminator Resistor	: None\r\n");
	} else if (terminator == MX_TERM_120) {
		printf("Terminator Resistor	: 120 ohm\r\n");
	} else {
		printf("Terminator Resistor	: unknown\r\n");
	}

	if (alarm == MX_CPLD_ALARM_0) {
		printf("Alarm Status		: OK\r\n");
	} else if (alarm == MX_CPLD_ALARM_1) {
		printf("Alarm Status		: Waveform Distortion\r\n");
	} else if (alarm == MX_CPLD_ALARM_2) {
		printf("Alarm Status		: Receive Reflect Signal\r\n");
	} else if (alarm == MX_CPLD_ALARM_3) {
		printf("Alarm Status		: Data Error\r\n");
	}
	
	printf("\r\n");
	printf("Done.\r\n");
	ioctl(fd, SMARTIO_PUART_SET_DIAGNOSE, MX_CPLD_DIAG_OFF);

        if ((ret = ioctl(fd, MX_SET_TERM, terminator)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
        	        printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
                return;
        }

        if ((ret = ioctl(fd, SMARTIO_PUART_SET_PULL_STATE, hl_resistor)) < 0) {
		if(ret == -EINVAL)
			printf("muestty : Invaild operation, %s not support this operation.\n", dev);
		else
	                printf("muestty : Invaild operation of MOXA Smartio MUE series device.\n");
                return;
        }
}

static void mx_get_interface_term(int fd, char *dev)
{
	int mode, ret;

	mode = 0;
	ioctl(fd, MX_GET_INTERFACE, &mode);
	switch (mode) {
		case MX_RS232:
			printf("muestty: %s is set to RS-232 mode.\n",dev);
			break;
		case MX_RS422:
			printf("muestty: %s is set to RS-422 mode.\n",dev);
			break;
		case MX_RS485_2W:
			printf("muestty: %s is set to RS-485 2W mode.\n",dev);
			break;
		case MX_RS485_4W:
			printf("muestty: %s is set to RS-485 4W mode.\n",dev);
			break;
	}
	ret = ioctl(fd, MX_GET_TERM, &mode);
	switch (mode) {
		case MX_TERM_NONE:
			printf("muestty: %s none terminal resistor.\n",dev);
			break;
		case MX_TERM_120:
			printf("muestty: %s 120ohm terminal resistor.\n",dev);
			break;
	}

	ret = ioctl(fd, SMARTIO_PUART_GET_PULL_STATE, &mode);
	if (ret == 0) {
		switch (mode) {
			case MX_CPLD_PULL_OFF:
				printf("muestty: %s disable pull high/low resistor (150K ohm).\n",dev);
				break;
			case MX_CPLD_PULL_ON:
				printf("muestty: %s enable pull high/low resistor (1K ohm).\n",dev);
				break;
		} 
	} else if (ret < 0 && ret == EPERM) {
		printf("muestty: %s don't supportp pull high resistor feature.\n",dev);
	}
}

static void mx_set_pci_capability(char *dev, char *parm)
{
	int fd;
	int ret = 0;
	struct mxupcie_pci_setting pci_setting = {0};

	fd = open("/dev/mxupcie",O_RDWR);
	if(fd < 0) {
		if(errno == ENOENT) {
			//try to mknod
			mx_make_mxupcie_node();
			fd = open("/dev/mxupcie",O_RDWR);
			if(fd < 0) {
				printf("muestty: Open device %s error (%d)!\n", "/dev/mxupcie",fd);
				return;
			}
		}
		else if (errno == ENXIO){
			printf("muestty: Open device %s error, please load 'mxupcie' driver first.\n", "/dev/mxupcie");
			return;
		}
		else
		{
			printf("muestty: Open device %s error (%d)!\n", "/dev/mxupcie",fd);
			return;
		}
	}
	
	if(atoi(dev) > 4 || atoi(dev) <= 0)
	{
		printf("muestty : Invalid value of board number.\n");
		printf("          '%s' is an illegal parameter.\n", dev);
		close(fd);
		return;
	}
	//printf("parm %s\n", parm);
	//printf("dev %s\n", dev);
	
	if(atoi(parm) != 0 && atoi(parm)!=1)
	{
		printf("muestty : Invalid value of parameter.\n");
		printf("          '%s' is an illegal parameter.\n", parm);
		close(fd);
		return;
	}

	pci_setting.whichPciBoard = atoi(dev);
	pci_setting.cfg_value = atoi(parm);

	//printf("muestty: pci_setting.whichPciBoard = %d\n", pci_setting.whichPciBoard);
	//printf("pci_setting.cfg_value = %d\n",pci_setting.cfg_value);

	ret = ioctl(fd, SMARTIO_SET_PCI_CAPABILITY, &pci_setting);
	if(ret < 0) {
		printf("muestty: set pci capability failed: %d\n",ret);
		close(fd);
		return;
	}

	close(fd);
}

static void mx_show_board()
{
	int fd;
	int pcie_major, pcie_calloutmajor;
	int ret = 0;
	int ports;
	int i;
	unsigned char uchCap[MXSER_BOARDS];
	struct mxupcie_usr_hwconf mxupciecfg[MXSER_BOARDS];

	fd = open("/dev/mxupcie",O_RDWR);
	if (fd < 0) {
		if(errno == ENOENT) {
			mx_make_mxupcie_node();
			fd = open("/dev/mxupcie",O_RDWR);
			if(fd < 0) {
				printf("muestty: Open device %s error (%d)!\n", "/dev/mxupcie",fd);
				return;
			}
		}
		else if (errno == ENXIO){
			printf("muestty: Open device %s error, please load 'mxupcie' driver first.\n", "/dev/mxupcie");
			return;
		}
		else
		{
			printf("muestty: Open device %s error (%d)!\n", "/dev/mxupcie",fd);
			return;
		}	}

	ret = ioctl(fd, MOXA_GET_CONF, mxupciecfg);
	if (ret < 0) {
		printf("muestty: Show board slot information failed!\n");
		close(fd);
		return;
	}

	ret = ioctl(fd, SMARTIO_GET_PCI_CAPABILITY, &uchCap);
	if(ret < 0) {
		printf("muestty: Show board slot iformation failed!\n");
		close(fd);
		return;
	}

	close(fd);
	for (i = 0; i < MXSER_BOARDS; i++)
	{
		if(mxupciecfg[i].board_type == -1)
			continue;
		
		ports = mxupcie_numports[mxupciecfg[i].board_type-1];
		
		if(uchCap[i] == 0)
		{
			printf("\nBoard [ %d ] : %s \n\tttyMUE%d-ttyMUE%d, pci_capability=Enable, BusNo=%d, DevNo=%d\n", 
					i+1,
					mxupcie_brdname[mxupciecfg[i].board_type-1],
					i*MXSER_PORTS_PER_BOARD, 
					i*MXSER_PORTS_PER_BOARD +ports - 1,
					mxupciecfg[i].pciInfo.busNum,
					mxupciecfg[i].pciInfo.devNum >> 3);
		}
		else if(uchCap[i] == 1)
		{
			printf("\nBoard [ %d ] : %s \n\tttyMUE%d-ttyMUE%d, pci_capability=Disable, BusNo=%d, DevNo=%d\n", 
					i+1,
					mxupcie_brdname[mxupciecfg[i].board_type-1],
					i*MXSER_PORTS_PER_BOARD, 
					i*MXSER_PORTS_PER_BOARD +ports - 1,
					mxupciecfg[i].pciInfo.busNum,
					mxupciecfg[i].pciInfo.devNum >> 3);
		}

	}
}

static void mx_make_mxupcie_node()
{
	int systemRet = 0;
	int major = 31;
	char cmd[1024];
	memset(cmd, 0, sizeof(cmd));
	systemRet = system("rm -rf /dev/mxupcie");
	major = mx_get_mxupcie_major();
	if(systemRet != -1) {
		sprintf(cmd, "mknod /dev/mxupcie c %d 32", major);
		systemRet = system(cmd);
		if(systemRet == -1)
			return;
	}
}

static int mx_get_mxupcie_major()
{
	FILE *fstream = NULL;
	char buff[1024];
	int major = 31;
	memset(buff, 0, sizeof(buff));

	if(NULL == (fstream = popen("cat /proc/devices | grep -w ttyMUE | cut -c1-3 | cut -f2 -d' '","r")))
	{
		//fprintf(stderr, "execute command failed: %s", strerror(errno));

		//get major number failed, return default major number.
		return major;
	}

	while(NULL != fgets(buff, sizeof(buff), fstream))
	{
		//printf("%s", buff);
		if(atoi(buff) >= 0){
			major = atoi(buff);
		}
	}

	pclose(fstream);
	return major;
}

static void mx_show_help()
{
	printf("Usage: muestty <operation> device_node\n");
	printf("Usage: muestty -v enable board_number\n");
	printf("Usage: muestty -h | -v\n");
	printf("\n");
	printf("device_node: The MUE series device node, such as '/dev/ttyMUE0', '/dev/ttyMUE1' and so on. MUE series not include CP-118EL, CP-168EL, CP-104EL, ISA cards and PCI cards.\n");
	printf("\n");
	printf("board_number:The PCIe slot number, range is 1-4. You can use the operation '-v' to getthe board number and information.\n");
	printf("\n");
	printf("operation:	-h	        Help\n");
	printf("		-g	        Get the following information\n"); 
	printf("			        a) interface type\n");
	printf("			        b) terminator resistor\n");
	printf("			        c) pull high/low resistor\n");
	printf("		-i intf         Set interface type\n");
	printf("		-t value        Set terminator resistor\n");
	printf("		-p state        Set pull high/low resistor\n");
	printf("		-a baud         Auto tune and display the proper resistor on RS-485 2W bus\n");
	printf("        		        under specified baud rate\n");
	printf("		-d baud         Diagnose and display the error status when negotiation on\n");
	printf("        		        RS-485 2W bus under specified baud rate\n");
	printf("                -v              Show MOXA MUE series device board slot number and VM-Compatible information\n");
	printf("                -v enable board VM-Compatible (only available for specific model)\n");
	printf("Enable this setting to ignore PCI capability if this board has tramsmission issue on virtual machine.\n");
	printf("\n");	
	printf("intf 		RS232     RS-232 mode\n");
	printf("		RS422     RS-422 mode\n");
	printf("		RS4852W   RS-485 2 wire mode\n");
	printf("		RS4854W	  RS-485 4 wire mode\n");
	printf("\n");	
	printf("value		NONTERM	  None terminator resistor\n");
	printf("		120TERM	  120ohm terminator resistor\n");
	printf("\n");	
	printf("state		150K	  Disable pull high/low resistor (150K ohm)\n");
	printf("		1K	  Enable pull high/low resistor (1K ohm)\n");
	printf("\n");
	printf("baud\t\t921600    Baud rate = 921600\n");
	printf("    \t\t460800    Baud rate = 460800\n");
	printf("    \t\t230400    Baud rate = 230400\n");
	printf("    \t\t115200    Baud rate = 115200\n");
	printf("    \t\t 57600    Baud rate = 57600\n");
	printf("    \t\t 38400    Baud rate = 38400\n");
	printf("    \t\t 19200    Baud rate = 19200\n");
	printf("    \t\t  9600    Baud rate = 9600\n");
	printf("\n");
	printf("enable\t\t     1    Enable VM-Compatible\n");
	printf("      \t\t     0    Disable VM-Compatible\n");
	printf("board_number        Get by the command \"muestty -v\"\n");

}
