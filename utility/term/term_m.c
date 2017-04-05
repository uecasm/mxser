/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term_m.c
*/

/*****************************************************************************/
/* TERM_M.C								     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	10/95	JOSE							     */
/*	03/96	Victor		modified				     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

#include	<signal.h>
#include	"term.h"

#define SENDPATTERN	0
#define SENDFILE	1

/*****************************************************************************/
/* GLOBAL VARIABLES							     */
/*****************************************************************************/

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static	struct	WItem	dModeMenuitem[] = {
	{ F_Active, "Send Pattern " },
	{ F_Active, "Send file    " }
};
static	struct	WSelect dModeMenu = {
	-1, 6, "Advanced Transfer Modes", 2, 0, 0, 0, 2, dModeMenuitem,
	{ "Enter:Select  Esc:Exit"}, 0
};

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
void	mode_setup()
{
	int	key;

	mw_selectinit(&dModeMenu);
	do {
	    switch((key = mw_select())) {
	    case SENDPATTERN:
		mw_selectend();
		term_menu();
		pattern_setup();
		term_menu();
		mw_selectinit(&dModeMenu);
		break;
	    case SENDFILE:
		mw_selectend();
		term_menu();
		sfile_setup();
		mw_selectinit(&dModeMenu);
		break;
	    }
	} while ( key != K_ESC );
	mw_selectend();
}
