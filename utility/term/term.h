/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term.h
*/

/*****************************************************************************/
/* term.h								     */
/*	global variable, function and define for term subdirectory	     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	05/95	JE							     */
/* 	02/96	Victor							     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

/*****************************************************************************/
/* global define value							     */
/*****************************************************************************/
#ifndef _TERM_H
#define _TERM_H

#include        "../mxwinlib/win.h"
#include        "../mxlib/declare.h"
#include	"../global.h"

#ifndef B921600
#define B921600 (B460800+1)
#endif

#define HIGH_SPEED_ITEM 20
#define NORM_SPEED_ITEM 17

struct dterm_info {
	char	dname[20];
	uchar	rate;
	uchar	parity;
	uchar	databit;
	uchar	stopbit;
	uchar	xflow;
	uchar	lecho;
	uchar	senter;
	uchar	dmode;
};

/*****************************************************************************/
/* extern global variable 						     */
/*****************************************************************************/
extern	struct	WItem		AbleMenuItem[];
extern	struct	WSelect 	AbleMenu;
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
extern	struct	WItem		FlowMenuItem[];
extern	struct	WSelect		FlowMenu;
extern	struct	WItem		EnterMenuItem[];
extern	struct	WSelect		EnterMenu;
extern	struct	dterm_info	DTerm;
extern	int			SioFd;
extern long BaudMapTab[];

/*****************************************************************************/
/* external public function						     */
/*****************************************************************************/
extern	void	term_menu();

#endif
