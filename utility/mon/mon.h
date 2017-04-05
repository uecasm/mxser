/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mon.h
*/

/*****************************************************************************/
/* MON.H                                                                     */
/*									     */
/* Copyright (c) Moxa Technologies Inc. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

#ifndef _MON_H
#define _MON_H

#define MON_TOP		0
#define MON_BOTTOM	1
#define MON_NORMAL	2

struct mxser_mstatus{
       tcflag_t   cflag;
       int  cts;
       int  dsr;
       int  ri;
       int  dcd;
};

extern	struct	WItem		BaudRateMenuItem[];
extern	struct	WSelect		BaudRateMenu;
extern	struct	WItem		ParityMenuItem[];
extern	struct	WSelect		ParityMenu;
extern	struct	WItem		DataBitMenuItem[];
extern	struct	WSelect		DataBitMenu;
extern	struct	WItem		StopBitMenuItem[];
extern	struct	WSelect		StopBitMenu;
extern	struct	WItem		YesNoMenuItem[]; 
extern	struct	WSelect		YesNoMenu;
extern	struct	WItem		EnterMenuItem[];
extern	struct	WSelect		EnterMenu;

#endif
