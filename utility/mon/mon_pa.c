/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mon_pa.c
*/

/*****************************************************************************/
/* MON_PA.C								     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	10/95	JOSE							     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

#include	<fcntl.h>
#include	<linux/types.h>
#include	<time.h>
//#include        <sys/ioctl.h>

#include	"../mxwinlib/win.h"
#include	"../mxlib/declare.h"

#include	"../global.h"
#include	"mon.h"

#define         MOXA_GETMSTATUS         (MOXA + 65)

/*****************************************************************************/
/* GLOBAL FUNCTIONS							     */
/*****************************************************************************/
int	mon_pa_setup();

/*****************************************************************************/
/* GLOBAL VARIABLES							     */
/*****************************************************************************/

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static	int	init_menu(void);
static	int	prepare_menu();
static 	int	getstatus();

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
#define Total_Item	8
#define Max_Col 	1
#define Col_Space	14
static  struct WItem	OnOffMenuItem[] = {
	{F_Active, "OFF"},
	{F_Active, "ON"}
};
static	int		itemflag[Total_Item] = {
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly
};
static	char		*itemdes[Total_Item] = {
	" Device Name  ",
	" Baud Rate    ",
	" Parity       ",
	" Data Bits    ",
	" Stop Bits    ",
	" CTS          ",
	" DSR          ",
	" DCD          "
};
static	struct	GEdit2	menu2 = {
	Edit2Type0, -1, 4, Total_Item,
	"Port Status", 0, 0,
	{ "PgDn:Next  PgUp:Prev ",
	  " Esc:Exit "},
	0
};


struct WItem	BaudRateMenuItem[] = {
	{F_Active, "50"},
	{F_Active, "75"},
	{F_Active, "110"},
	{F_Active, "134"},
	{F_Active, "150"},
	{F_Active, "200"},
	{F_Active, "300"},
	{F_Active, "600"},
	{F_Active, "1200"},
	{F_Active, "1800"},
	{F_Active, "2400"},
	{F_Active, "4800"},
	{F_Active, "9600"},
	{F_Active, "19200"},
	{F_Active, "38400"},
	{F_Active, "57600"},
	{F_Active, "115200"},
	{F_Active, "230400"},
	{F_Active, "460800"},
	{F_Active, "921600"}
};
struct WSelect	BaudRateMenu = {
	-1, 3, 0, 17, 0, 0, 0, 17, BaudRateMenuItem
};
struct WItem	ParityMenuItem[] = {
	{F_Active, " None "},
	{F_Active, "  Odd "},
	{F_Active, " Even "},
	{F_Active, " Even "},
	{F_Active, " Mark "},
	{F_Active, " Space"}
};
struct WSelect	ParityMenu = {
	-1, 10, 0, 3, 0, 0, 0, 3, ParityMenuItem
};
struct WItem	DataBitMenuItem[] = {
	{F_Active, " 5 "},
	{F_Active, " 6 "},
	{F_Active, " 7 "},
	{F_Active, " 8 "}
};
struct WSelect	DataBitMenu = {
	-1, 10, 0, 4, 0, 0, 0, 4, DataBitMenuItem
};
struct WItem	StopBitMenuItem[] = {
	{F_Active, " 1 "},
	{F_Active, " 2 "}
};
struct WSelect	StopBitMenu = {
	-1, 10, 0, 2, 0, 0, 0, 2, StopBitMenuItem
};
struct WItem	YesNoMenuItem[] = {
	{F_Active, "No "},
	{F_Active, "Yes"}
};
struct WSelect	YesNoMenu = {
	-1, 10, 0, 2, 0, 0, 0, 2, YesNoMenuItem
};
struct WItem	EnterMenuItem[] = {
	{F_Active, "CR-LF"},
	{F_Active, " CR  "},
	{F_Active, " LF  "},
	{F_Active, "LF-CR"}
};
struct WSelect	EnterMenu = {
	-1, 10, 0, 4, 0, 0, 0, 4, EnterMenuItem
};

long BaudMapTab[] = {
     B50, B75, B110, B134, B150, B200, B300, B600, B1200,
     B1800,B2400,B4800,B9600,B19200,B38400,B57600,B115200,
     B230400,B460800,B921600
};

static	int	old_space_ndx;

struct	pstat_info_str {
	uchar	baud;
	uchar	parity;
	uchar	databit;
	uchar	stopbit;
	uchar	CTS;
	uchar	DSR;
	uchar	DCD;
	int	iqueue;
	int	oqueue;
};
static	struct	pstat_info_str	pinfo;
#define CTS_SIG 0x01
#define DSR_SIG 0x02
#define DCD_SIG 0x04
static	time_t	t1, t3;
extern struct mxser_usr_hwconf	Gmxsercfg[MXSER_BOARDS];
extern struct mxupcie_usr_hwconf	Gmxupciecfg[MXSER_BOARDS];
/*****************************************************************************/
/*	FUNCTIONS							     */
/*****************************************************************************/
int	mon_pa_setup(char *devname, int interval,
                          int flag, int now_board, int now_pci_board, int port)
{
	int	exit_flag=0, err_flag=0, ret=0, old_space_ndx;

	old_space_ndx = init_menu();
	mw_edit2init(&menu2);
	t1 = time(&t3);
	if ( prepare_menu(devname, now_board, now_pci_board, port) < 0 )
	    err_flag = 1;
	else
	    edit2_flush_data(&menu2);
	while ( exit_flag == 0 && err_flag == 0 ) {
	    switch (mw_getkey_nb()) {
	    case K_ESC_A:
	    case K_ESC:
		exit_flag = 1;
		break;
	    case K_PGDOWN:
		mw_getkey_nb();
		if ( flag == MON_BOTTOM )
		    break;
		ret = 1;
		exit_flag = 1;
		break;
	    case K_PGUP:
		mw_getkey_nb();
		if ( flag == MON_TOP )
		    break;
		ret = -1;
		exit_flag = 1;
		break;
	    default:
		if ( time(&t3) - t1 < interval )
		    break;
		t1 = time(&t3);
		if ( prepare_menu(devname, now_board, now_pci_board, port) < 0 )
		    err_flag = 1;;
		edit2_flush_data(&menu2);
		break;
	    }
	}
	release_space(old_space_ndx);
	mw_edit2end();
	return(ret);
}

static int init_menu(void)
{
	int	i, j, k, old_space_ndx;

	old_space_ndx = get_space_ndx();
	j = sizeof(struct GItem2) * Total_Item;
	menu2.item = (struct GItem2 *)get_space(j);
	for ( i=0; i<Total_Item; i++ ) {
	    menu2.item[i].flag = itemflag[i];
	    menu2.item[i].des = itemdes[i];
	    menu2.item[i].act_item = 0;
	    menu2.item[i].total = Max_Col;
	    /* jose: to fit for each column space */
	    for ( j = 0; j < Max_Col; j++ ) {
		menu2.item[i].len[j] = Col_Space;
		menu2.item[i].str[j] = get_space(Col_Space + 1);
		for ( k=0; k<Col_Space; k++ )
		    menu2.item[i].str[j][k] = ' ';
		menu2.item[i].str[j][k] = 0;
	    }
	}
	menu2.act_item = 1;
	return(old_space_ndx);
}

static int prepare_menu(char *devname, int now_board, int now_pci_board, int port)
{
	int	j, i;
	char	str[20];

	for ( j=0; j<Max_Col; j++ ) {
	    if ( getstatus(devname, now_board, now_pci_board, port) < 0 )
		return(-1);
	    for ( i=0; i<10; i++ ) {
		str[i] = devname[i];
	 	if ( str[i] <= ' ' )
		    break;
	    }
	    str[i] = 0;
	    sprintf(menu2.item[0].str[j], " %9s   ", str);
	    sprintf(menu2.item[1].str[j], " %9s   ",
		    BaudRateMenuItem[pinfo.baud].str);
	    sprintf(menu2.item[2].str[j], "  %9s  ",
		    ParityMenuItem[pinfo.parity].str);
	    sprintf(menu2.item[3].str[j], "  %9s  ",
		    DataBitMenuItem[pinfo.databit].str);
	    sprintf(menu2.item[4].str[j], "  %9s  ",
		    StopBitMenuItem[pinfo.stopbit].str);
	    sprintf(menu2.item[5].str[j], "  %9s  ",
		    OnOffMenuItem[pinfo.CTS].str);
	    sprintf(menu2.item[6].str[j], "  %9s  ",
		    OnOffMenuItem[pinfo.DSR].str);
	    sprintf(menu2.item[7].str[j], "  %9s  ",
		    OnOffMenuItem[pinfo.DCD].str);
	}
	return(0);
}


static	int getstatus(char *devname, int now_board, int now_pci_board, int port)
{
	int		fd, i, j, idx;
	char		tmp[80];
	struct termios	tty;
        struct mxser_mstatus    mstatus[MXSER_MAXPORT];
#if 1
	if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) {
	fd = open("/dev/mxupcie", O_RDWR);
	if ( fd < 0 )
	    return(-1);
            
	ioctl(fd, MOXA_GETMSTATUS, mstatus);
        idx = (now_board - now_pci_board ) * MXSER_PORTS_PER_BOARD + port;

        for(i=0;i<20;i++){
            if((mstatus[idx].cflag & (CBAUD | CBAUDEX) ) == BaudMapTab[i]){
                pinfo.baud = i;
                break;
            }
        }

	if ( mstatus[idx].cflag & PARENB ) {
		if (mstatus[idx].cflag & PARODD){
#ifdef CMSPAR
			if(mstatus[idx].cflag & CMSPAR)
				pinfo.parity = 3;
			else
#endif
				pinfo.parity = 1;
		}else{
#ifdef CMSPAR
			if(mstatus[idx].cflag & CMSPAR)
				pinfo.parity = 4;
			else
#endif
				pinfo.parity = 2;
		}
	} else
	    pinfo.parity = 0;

	switch(mstatus[idx].cflag & CSIZE){
	    case CS5:
	        pinfo.databit = 0;
		break;
	    case CS6:
	        pinfo.databit = 1;
		break;
	    case CS7:
	        pinfo.databit = 2;
		break;
	    case CS8:
	        pinfo.databit = 3;
		break;
	}
	if ( mstatus[idx].cflag & CSTOPB )
	    pinfo.stopbit = 1;
	else
	    pinfo.stopbit = 0;

	pinfo.CTS = mstatus[idx].cts;
	pinfo.DSR = mstatus[idx].dsr;
	pinfo.DCD = mstatus[idx].dcd;
        
	close(fd);
	return(0);
#endif
	} else {
#if 1
	fd = open("/dev/mxser", O_RDWR);
	if ( fd < 0 )
	    return(-1);
            
	ioctl(fd, MOXA_GETMSTATUS, mstatus);
        idx = now_board * MXSER_PORTS_PER_BOARD + port;

        for(i=0;i<20;i++){
            if((mstatus[idx].cflag & (CBAUD | CBAUDEX) ) == BaudMapTab[i]){
                pinfo.baud = i;
                break;
            }
        }

	if ( mstatus[idx].cflag & PARENB ) {
		if (mstatus[idx].cflag & PARODD){
#ifdef CMSPAR
			if(mstatus[idx].cflag & CMSPAR)
				pinfo.parity = 3;
			else
#endif
				pinfo.parity = 1;
		}else{
#ifdef CMSPAR
			if(mstatus[idx].cflag & CMSPAR)
				pinfo.parity = 4;
			else
#endif
				pinfo.parity = 2;
		}
	} else
	    pinfo.parity = 0;

	switch(mstatus[idx].cflag & CSIZE){
	    case CS5:
	        pinfo.databit = 0;
		break;
	    case CS6:
	        pinfo.databit = 1;
		break;
	    case CS7:
	        pinfo.databit = 2;
		break;
	    case CS8:
	        pinfo.databit = 3;
		break;
	}
	if ( mstatus[idx].cflag & CSTOPB )
	    pinfo.stopbit = 1;
	else
	    pinfo.stopbit = 0;

	pinfo.CTS = mstatus[idx].cts;
	pinfo.DSR = mstatus[idx].dsr;
	pinfo.DCD = mstatus[idx].dcd;
        
	close(fd);
	return(0);
#endif
	}
}



