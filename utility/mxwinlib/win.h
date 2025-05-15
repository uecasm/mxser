/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	win.h
*/

/*****************************************************************************/
/* WIN.H								     */
/*	Definitions for window function.				     */
/*									     */
/* Copyright (c) 404 Technologies Inc. 1993. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	Kevin Li	06/08/93 Revised for UNIX version/Monochrom TEXT     */
/*	Kevin Li	12/08/93 Revised for UNIX version/VGA TEXT	     */
/*****************************************************************************/
#ifndef WIN404
#define WIN404

#include <sys/types.h>

#ifdef VENIX
#include <sys/at_ansi.h>
#include <sys/kd.h>
#include <sys/vt.h>
#endif

#ifdef STREAM
#include <sys/at_ansi.h>
#include <sys/kd.h>
#include <sys/vt.h>
#endif

#ifdef SCOUNIX
#ifndef LINUX
#include <sys/keyboard.h>
#include <sys/console.h>
#include <sys/vtkd.h>
#endif
#endif

#ifdef LINUX
/*#include <linux/keyboard.h>
*/
/*#include <linux/console.h>*/
#endif

#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include "kbd.h"

typedef unsigned char	uchar;

#ifndef STREAM
/*
typedef unsigned short	ushort;
typedef unsigned int	uint;
*/
#endif

#define SCRNSZ	2000

/* char attributes */
#define D_A_Normal	0
#define D_A_LIGHT	1
#define D_A_UNDER	4
#define D_A_Blink	5
#define D_A_INV 	7

#ifdef	STREAM
#define D_A_C_UNDER	0
int	Console_flag;
#endif

struct	WIN {
	int	flag,
		x0, y0, x1, y1,
		csr_x, csr_y,	/* cursor position */
		color,
		w, h,		/* width, height */
		norm_c, 	/* normal color */
		bar_c,		/* bar	    ''  */
		field_c;	/* field    ''  */
};
typedef struct	WIN	WINDOW;
#define 	MaxWindow	20
#define		MaxEditMsg	5
WINDOW		win[MaxWindow]; 	/* window struct */
WINDOW		*wstd;			/* standard screen */

struct	WItem {
	int	flag;		/* type of item; active or seperator */
	char	*str;		/* description string */
	int	key;		/* mnemonic key */
	int	x, y;
};
struct	WSelect {
	int		x, y;		/* window position */
	char		*title; 	/* select menu title */
	int		total, first;	/* total item, first cursor item */
	int		act_item;	/* cursor item */
	WINDOW		*wid;		/* window ptr */
	int		Ndisp;		/* Number of displayed items */
					/* scroll feature support */
	struct WItem	*item;
	/****add by jose ***/
	char		*msg[MaxEditMsg]; /* message line */ 
	int		len;	/* item string max length */
};

#define MW_SELECT	1
#define MW_EDIT 	2

union	Menu	{
	struct	WEdit		*editp;
	struct	WSelect 	*selectp;
};


/*
 *	Edit Menu
 */
struct	WEItem {
	int	flag;			/* Read only(0),Writeable(1) */
	char	*des;			/* description string */
	char	*str;			/* string for edit */
	union	Menu	menup;
	int	x, y;
	int	len;
	int	menu_act_item;
};
struct	WEdit {
	int		id;		/* edit menu id */
	int		x, y;		/* window position */
	char		*title; 	/* edit meunu title */
	char		*msg[MaxEditMsg];/* edit meunu msg */
	int		total;		/* total item*/
	int		first;		/* first cursor item */
	int		act_item;	/* cursor item */
	WINDOW		*wid;		/* system flag window prt */
	struct WEItem	*item;		/* Support Field value selection */
	int		Ndisp;		/* Number of displayed items */
	int		max_des_len;	/* max description string length */
	int		max_str_len;	/* max edit string length */
};
struct	GItem2 {
	int		flag;
	char		*des;
	int		total;		/* total sub-item */
	int		act_item;	/* sub - cursor item */
	char		*str[10];
	int		item_no[10];
	int		x[10];
	int		y;
	int		len[10];
	union Menu	menup[10];
};

#define Edit2Type0	0/*0x01*/		/* not drawing line */
#define Edit2Type1	1/*0x02*/		/* drawing line */
#define Edit2Type5	0x20		/* non-blocking read to be assigned */

struct GEdit2 {
	int		type;		/* GEdit2 type */
	int		x, y;		/* window position */
	int		total;		/* total item */
	char		*title; 	/* edit meunu title */
	int		act_item;	/* cursor item */
	WINDOW		*wid;
	char		*msg[MaxEditMsg];
	struct GItem2	*item;
};
struct	GItem3 {
	int		flag;
	char		*des;
	int		total;		/* total sub-item */
	int		act_item;	/* sub - cursor item */
	char		*str[16];
	int		item_no[16];
	int		x;
	int		y[16];
	int		len[16];
	union Menu	menup[16];
};

#define Edit3Type0	0x01		/* drawing V. lines only */
#define Edit3Type1	0x02		/* drawing v. line & h. lines */
#define Edit3Type2	0x04		/* drawing v. & h. even lines */
#define Edit3Type3	0x08		/* header 4 lines adding special opt. */
#define Edit3Type4	0x10		/* no up & down auto control */
#define Edit3Type5	0x20		/* non-blocking read to be assigned */

struct GEdit3 {
	int		type;		/* GEdit3 type */
	int		x, y;		/* window position */
	int		total;		/* total item */
	char		*title; 	/* edit meunu title */
	int		act_item;	/* cursor item */
	WINDOW		*wid;
	char		*msg[MaxEditMsg];
	struct GItem3	*item;
};

/*
 *	Define for WIN flag
 */
#define F_Used		0x80		/* identify the WINDOW used */

/*
 *	Define for WItem  or WEItem or WTItem flag
 */
#define F_NotAvailable	0x00		/* Not available yet entry */
#define F_Writable	0x01		/* same as  F_Active */
#define F_Active	0x01		/* identify the item active */
#define F_Readonly	0x02		/* read only entry */
#define F_NonEdit	0x04		/* non edit only entry */
#define F_Return	0x08		/* entry which return key code */
#define F_Select	0x10		/* entry which calls select menu */
#define F_Edit		0x20		/* entry which calls edit menu */
#define F_Seperator	0x80		/* edit2 menu: to draw line */

/* better used with fixed type of menu, instead of scrolling one */
#define F_MSG		0x00		/* message item indicator */
#define F_Sep		0x80		/* the item only a seperator */

/*
 *	Define for line direction/type
 */
#define L_SINGLE	0x00
#define L_DOUBLE	0x01
#define L_UP		0x00
#define L_RIGHT 	0x02
#define L_DOWN		0x04
#define L_LEFT		0x06

#define MaxSeleLevel	20
#define MaxEditLevel	20
#define MaxEdit2Level	20
#define MaxEdit3Level	20

#define SCRNSZ		2000	/* 25 * 80 */

/* win-0 functions declaration */
extern	void	mw_init();
extern	void	mw_end();
extern	int	mw_kbhit();
extern	int	mw_getkey();
extern	int	mw_inskey();
extern	void	mw_cursor(int, int);
extern	void	mw_cursor_type(int, int);
extern	void	mw_attr(int);
extern	void	mw_clearscreen();
extern	void	mw_putstr(uchar *, int);
extern	void	mw_putstr_xy(uchar *, int, int, int);
extern	void	mw_putchar(uchar);
extern	void	mw_putchar_c(uchar, int, int);
extern	void	mw_putnpchar(uchar);
extern	void	mw_putnpchar_c(uchar, int, int);
extern	void	mw_cursor_off();
extern	void	mw_cursor_on();
extern	void	mw_displaywindow(int, int, int, int, int);
extern	void	mw_redraw();
extern	void	mw_getwindow(int, int, int, int, uchar);
extern	void	mw_putwindow(int, int, int, int, uchar);
extern	void	mw_clearwindow(int, int, int, int);
extern	void	mw_fillbox(int, int, int, int, uchar);
extern	void	mw_scroll_up(int, int, int, int);
extern	void	mw_scroll_down(int, int, int, int);
extern	void	mw_line(int, int, int, int);
extern	void	mw_lineclip(int, int, int, int);

/* win-1 functions declaration */
extern	void	mw_winit();
extern	void	mw_wend();
extern	WINDOW	*mw_wopen(int, int, int, int);
extern	void	mw_wstore(WINDOW *);
extern	void	mw_wrestore(WINDOW *);
extern	void	mw_wscroll_up(WINDOW *, int, int, int, int);
extern	void	mw_wscroll_down(WINDOW *, int, int, int, int);
extern	void	mw_wputchc(WINDOW *, uchar, int);
extern	void	mw_wputsc(WINDOW *, uchar *, int, int);
extern	void	mw_wprintc(WINDOW *, uchar *, int);
extern	void	mw_wline(WINDOW *, int, int);
extern	void	mw_wdrawbox(WINDOW *, int);
extern	void	mw_wcputs(WINDOW *, int, char *);
extern	void	mw_wclseol(WINDOW *);
extern	int	_getstr();
#define mw_wclose(wid)		{ (wid)->flag = 0; }
#define mw_wcolor(wid, c)	{ (wid)->color = (c); }
#define mw_wallcolor(wid, n, b, f) { (wid)->norm_c = (n); \
				  (wid)->bar_c = (b); \
				  (wid)->field_c = (f); }
#define mw_wgotoxy(wid, x, y)	{ (wid)->csr_x = (x); \
				  (wid)->csr_y = (y); }
#define mw_wcursor(wid, x, y)	{ mw_cursor((x) + (wid)->x0, \
				(y) + (wid)->y0); mw_wgotoxy(wid, x, y); }
#define mw_wfill(wid, ch)	{ \
		mw_attr(wid->color);\
		mw_fillbox((wid)->x0, (wid)->y0, (wid)->x1, (wid)->y1, (ch));\
	}
#define mw_wputs(wid, str)	mw_wputsc(wid, str, strlen((char *)str), (wid)->color)
#define mw_wcls(wid)		mw_wfill(wid, ' ')
#define mw_wputch(wid, ch)	wputchc(wid, ch, (wid)->color)
#define mw_wprint(wid, str)	mw_wprintc(wid, str, (wid)->color)
#define mw_wgets(x, y, str, len, bar_c, field_c) \
				_getstr(x, y, str, len, bar_c, field_c)

/* win-1b functions declaration */
extern	void	display_select(struct WSelect *);
extern	int	_mw_select(struct WSelect *);
extern	int	_edit(struct WEdit *);
extern	int	edit_select(struct WEdit *);
extern	void	display_edit(struct WEdit *);

/* win-2 functions declaration */
extern	void	mw_selectinit(struct WSelect *);
extern	int	mw_select();
extern	void	mw_selectend();
extern	void	mw_select_set_act(struct WSelect *, int);
extern	void	mw_editinit(struct WEdit *);
extern	int	mw_edit();
extern	void	mw_editend();

/* win-3 functions declaration */
extern	void	mw_edit2init(struct GEdit2 *);
extern	int	mw_edit2();
extern	int	_edit2();
extern	int	mw_edit2end();
extern	void	edit2_flush_data(struct GEdit2 *);
extern	void	edit2_clear_data(struct GEdit2 *);
extern	int	edit2_select(struct GEdit2 *);

/* GLOBAL VARIABLES */
extern	uchar	Blank[80];		/* Blank string for convenience */
extern	int	tty_fd;

#endif
