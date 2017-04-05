/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	termvar.c
*/

/******************************************************************************/
/* termvar.c 							              */
/*	term directory used global variable define			      */
/*									      */
/* Version History:							      */
/*	date		author		comment 			      */
/*	02/16/96	Victor		wrote it.			      */
/*	12/96		Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)*/
/*     8/5/99           Casper                                                */
/******************************************************************************/

#include	"../mxwinlib/win.h"
#include	"term.h"
#include 	"../global.h"	/* for B921600 */

int	SioFd=-1;
struct dterm_info	DTerm;
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
	-1, 3, 0, 20, 0, 0, 0, 20, BaudRateMenuItem
};
struct WItem	ParityMenuItem[] = {
	{F_Active, " None "},
	{F_Active, "  Odd "},
	{F_Active, " Even "}
#ifdef CMSPAR
	, {F_Active, " Mark "},
	{F_Active, " Space"}
#endif
};
struct WSelect	ParityMenu = {
#ifdef CMSPAR
	-1, 10, 0, 5, 0, 0, 0, 5, ParityMenuItem
#else
	-1, 10, 0, 3, 0, 0, 0, 3, ParityMenuItem
#endif
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

struct WItem	FlowMenuItem[] = {
	{F_Active, "RTS/CTS ", 'R'},
	{F_Active, "XON/XOFF", 'X'},
	{F_Active, "  Both  ", 'B'},
	{F_Active, "  None  ", 'N'}
};
struct WSelect 	FlowMenu = {
	-1, 10, 0, 4, 0, 0, 0, 4, FlowMenuItem
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
