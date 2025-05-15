/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	diag.c
*/

/*****************************************************************************/
/* DIAG.C                                                                    */
/*									     */
/* Copyright (c) Moxa Inc. 2008. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*     8/5/99   Casper                                                       */
/*     3/9/2001 Casper     add Industio                                      */
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>

char DIAG_VER[] = "1.5";
#include "../global.h"

#define	MOXA_GET_MAJOR       (MOXA + 63)
#define	MOXA_GET_CUMAJOR     (MOXA + 64)

static void Printf(int verbose, char *msg);
static void mx_make_mxser_node();
static void mx_make_mxupcie_node();
static int mx_get_mxser_major();
static int mx_get_mxupcie_major();
static void mx_show_help();

int main(int argc, char *argv[])
{
	int	retpci = 0, retpcie = 0;
	int	fd,pcie_fd;
	int	i, j, brd_cnt;
	int	ports;
	int verbose = 0, c;
    int     calloutmajor, major, pcie_calloutmajor, pcie_major;
	char    title_str[100];
	struct mxser_usr_hwconf mxsercfg[MXSER_BOARDS];
	struct mxupcie_usr_hwconf mxupciecfg[MXSER_BOARDS];


    while ((c = getopt(argc, argv, "ph")) != -1) {
    	switch(c) {
			case 'p':
				verbose = 1;
				break;						
			case 'h':
				mx_show_help();
				return 0;
		}
	}

	printf("\n == MOXA Smartio/Industio Family Multiport Board Status Utility(%s) ==\n", DIAG_VER);

	brd_cnt = 0;
    printf("- PCI -\n");

	if ((fd = open("/dev/mxser", O_RDWR)) < 0) {
		if (errno == ENOENT) {
			mx_make_mxser_node();
			if ((fd = open("/dev/mxser", O_RDWR)) < 0) {
				Printf(verbose, "\t[Info] Open PCI failed, please run msmknod first.\n");
			    retpci = -1;
			    goto no_pci;
			}
		}
		else if (errno == ENXIO){
			Printf(verbose, "\t[Info] Open PCI failed, please load `mxser' driver first.\n");
			retpci = -1;
			goto no_pci;
		}
		else {
			retpci = -1;
			goto no_pci;
		}
	}
	if (ioctl(fd, MOXA_GET_MAJOR, &major)<0) {
            printf("\tCan't get tty major number.\n");
            close(fd);
            return 1;
    }
    printf("\tPCI tty device major number= %d.\n", major);
	
	
	if (ioctl(fd, MOXA_GET_CUMAJOR, &calloutmajor)<0) {
            printf("\tCan't get callout device major number.\n");
            close(fd);
            return 1;
    }
    printf("\tPCI callout device major number= %d.\n\n", calloutmajor);
	
	if (ioctl(fd, MOXA_GET_CONF, mxsercfg) < 0) {
            printf("\tCan't get driver configuration.\n");
            close(fd);
            return 1;
    }
	close(fd);
	for (i = 0; i < MXSER_BOARDS; i++) {
	    if (mxsercfg[i].board_type == -1)
	    	continue;

	    retpci = 1;
	    ports = mxser_numports[mxsercfg[i].board_type-1];
            if ((mxsercfg[i].pciInfo.busNum == 0) && 
               (mxsercfg[i].pciInfo.devNum == 0)) {
                //ISA board
		printf("\tBoard %d : %s \n", +1,
			mxser_brdname[mxsercfg[i].board_type-1]);
      	    
      	        for (j = 0; j < ports && j < MXSER_PORTS_PER_BOARD; j++) {
      		    if (mxsercfg[i].baud[j] == 115200)
       		        printf("\t\tPort %d: 0x%lx, max. baud rate = %s bps.\n", j+				1, mxsercfg[i].ioaddr[j], "115200");
      		    else
       		        printf("\t\tPort %d: 0x%lx, max. baud rate = %s bps.\n", j+				1, mxsercfg[i].ioaddr[j], "921600");
                }
            } else {
      	        printf("\tBoard %d : %s (BusNo=%d, DevNo=%d)\n",
                              i+1, mxser_brdname[mxsercfg[i].board_type-1],
                              mxsercfg[i].pciInfo.busNum,
                              mxsercfg[i].pciInfo.devNum >> 3);
                              
      	        for (j = 0; j < ports && j < MXSER_PORTS_PER_BOARD; j++) {
        		printf("\t\tPort %d: 0x%lx, max. baud rate = %d bps.\n", 
			j+1,mxsercfg[i].ioaddr[j], 
			mxsercfg[i].MaxCanSetBaudRate[j]);
		}
			
            }
	    printf("\n");
	    brd_cnt++;
	}

no_pci:	
	if(retpci != 1 && verbose == 0)
		printf("\tNo PCI device found.\n\n");
	else if(retpci == 0 && verbose == 1)
		printf("\tNo PCI device found.\n\n");

	printf("- PCIe -\n");

	if ((pcie_fd = open("/dev/mxupcie", O_RDWR)) < 0) {
		if (errno == ENOENT) {
			mx_make_mxupcie_node();
			if ((pcie_fd = open("/dev/mxupcie", O_RDWR)) < 0) {
				Printf(verbose, "\t[Info] Open PCIe failed, please run msmknod first.\n");
			    retpci = -1;
			    goto no_pcie;
			}
		}
		else if (errno == ENXIO){
			Printf(verbose, "\t[Info] Open PCIe failed, please load `mxupcie' driver first.\n");
			retpci = -1;
			goto no_pcie;
		}
		else {
			retpci = -1;
			goto no_pcie;
		}
	}
	
	if (ioctl(pcie_fd, MOXA_GET_MAJOR, &pcie_major ) < 0) {
		printf("\tCan't get PCIe tty major number.\n");
		close(pcie_fd);
		return 1;
	}
	printf("\tPCIe tty device major number= %d.\n", pcie_major);

	if (ioctl(pcie_fd, MOXA_GET_CUMAJOR, &pcie_calloutmajor) < 0) {
		printf("\tCant get PCIe callout device major number\n");
		close(pcie_fd);
		return 1;
	}
	printf("\tPCIe callout device major number= %d.\n\n", pcie_calloutmajor);
	/*get PCIe hw_configuration*/	
	if (ioctl(pcie_fd, MOXA_GET_CONF, mxupciecfg) < 0) {
            printf("\tCan't get driver configuration.\n");
            close(pcie_fd);
            return 1;
    }
	close(pcie_fd);
	for (i = 0; i < MXSER_BOARDS; i++) {
	    if (mxupciecfg[i].board_type == -1)
	    	continue;
		retpcie = 1;			
	    ports = mxupcie_numports[mxupciecfg[i].board_type-1];
            if ((mxupciecfg[i].pciInfo.busNum == 0) && 
               (mxupciecfg[i].pciInfo.devNum == 0)) {
                //ISA board
		printf("\tBoard %d : %s\n", i+1+brd_cnt,
 			mxupcie_brdname[mxupciecfg[i].board_type-1]);
      	         
      	        for (j = 0; j < ports && j < MXSER_PORTS_PER_BOARD; j++) {
      		    if (mxupciecfg[i].baud[j] == 115200)
       		        printf("\t\tPort %d: %p, max. baud rate = %s bps.\n", j+				1, mxupciecfg[i].ioaddr[j], "115200");
      		    else
       		        printf("\t\tPort %d: %p, max. baud rate = %s bps.\n", j+				1, mxupciecfg[i].ioaddr[j], "921600");
                }
            } else {
      	        printf("\tBoard %d : %s (BusNo=%d, DevNo=%d)\n", i+1+brd_cnt,
			mxupcie_brdname[mxupciecfg[i].board_type-1],
                              mxupciecfg[i].pciInfo.busNum,
                              mxupciecfg[i].pciInfo.devNum >> 3);
                              
      	        for (j = 0; j < ports && j < MXSER_PORTS_PER_BOARD; j++) {
        		printf("\t\tPort %d: %p, max. baud rate = %d bps.\n", 
			j+1,mxupciecfg[i].ioaddr[j], 
			mxupciecfg[i].MaxCanSetBaudRate[j]);
		}
			
            }
            printf("\n");
	}

no_pcie:
	if(retpcie != 1 && verbose == 0)
		printf("\tNo PCIe device found.\n");
	else if(retpcie == 0 && verbose == 1)
		printf("\tNo PCIe device found.\n");

    printf("\n");

	if(retpci == -1 && retpcie == -1)
		return 1;

        return 0;
}
       

static void Printf(int verbose, char *msg)    
{                                                                                                                                                                                          
    if(verbose)                                                                                                                                                          
        printf("%s", msg);                                                                                                                                  
}

static void mx_make_mxser_node()
{
	int ret = 0;
	int major = 20;
	char cmd[1024];
	memset(cmd, 0, sizeof(cmd));
	ret = system("rm -rf /dev/mxser");
	major = mx_get_mxser_major();
	if(ret != -1) {
		sprintf(cmd, "mknod /dev/mxser c %d 32", major);
		ret = system(cmd);
		if(ret == -1)
			return;
	}
}

static void mx_make_mxupcie_node()
{
	int ret = 0;
	int major = 31;
	char cmd[1024];
	memset(cmd, 0, sizeof(cmd));
	ret = system("rm -rf /dev/mxupcie");
	major = mx_get_mxupcie_major();
	if(ret != -1) {
		sprintf(cmd, "mknod /dev/mxupcie c %d 32", major);
		ret = system(cmd);
		if(ret == -1)
			return;
	}
}

static int mx_get_mxser_major()
{
	FILE *fstream = NULL;
	char buff[1024];
	int major = 30;
	memset(buff, 0, sizeof(buff));

	if(NULL == (fstream = popen("cat /proc/devices | grep -w ttyM | cut -c1-3 | cut -f2 -d' '","r")))
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
    printf("Usage: msdiag [operation]\n");                                                                                                                                           
	printf("\n");
	printf("operation:\t-h\tHelp\n");
    printf("          \t-p\tProbe the detail infomation while execution\n");
}           

