/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term_mp.c
*/

/*****************************************************************************/
/* TERM_MP.C								     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	10/95	JOSE							     */
/*	03/96	Victor		modified				     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

#include	<sys/time.h>
#include	<sys/types.h>
#include	<unistd.h>
#include	"term.h"

#define MaxPtnSize	40
#define SENDDEFAULT	0
#define SENDUSERDEFINE	1
#define HEXMODE 	0
#define ASCIIMODE	1

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static void	send_Pattern();
static int	UPattern_setup();
static int	get_menu_data(void);
static int	get_hex_data(void);
static int	get_ascii_data(void);
static void	change_data_mode(int);

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static char	*HexDes  ="Hex codes(3F,...):";
static char	*AsciiDes="ASCII string     :";
static char	*ErrorMsg="Input pattern error !";
static char	dpattern[MaxPtnSize + 1], upattern[80];
static char	InputModeFlag=HEXMODE;
static char	ptn_string[MaxPtnSize+1];

static	struct	WItem	PatternMenuitem[] = {
	{ F_Active, "Send Default Pattern     " },
	{ F_Active, "Send User Defined Pattern" }
};
static	struct	WSelect PatternMenu = {
	-1, 6, "Send Pattern Menu", 2, 0, 0, 0, 2, PatternMenuitem,
	{"Enter:Select  Esc:Exit"}, 0
};
static	struct	WEItem	Uitem[] = {
	{ F_Active | F_Return, NULL, 0}
};
static	struct	WEdit	Umenu = {
	2, -1, 6,
	"User Defined Pattern",
	{"Enter:Start to Send  Ctrl-D:Change Input Mode  Esc:Exit"},
	 1, 0, 0, 0, Uitem, 1
};
static	struct	WEItem	DefItem[] = {
	{ F_Readonly | F_Return, "", dpattern},
};
static	struct	WEdit	DefMenu = {
	2, -1, 6,
	"Default Defined Pattern",
	{"Enter:Start to Send  Esc:Exit"},
	 1, 0, 0, 0, DefItem, 1
};

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
int	pattern_setup()
{
	int	key, len;

	Uitem[0].str = ptn_string;
	for ( len=0; len<MaxPtnSize; len++ ) {
	    dpattern[len] = (char)len + 0x30;
	    ptn_string[len] = ' ';
	}
	ptn_string[MaxPtnSize] = 0;
	dpattern[MaxPtnSize]=0;
	mw_selectinit(&PatternMenu);
	do {
	    switch( (key = mw_select()) ) {
	    case SENDDEFAULT:
		mw_selectend();
		mw_editinit(&DefMenu);
		while ( (len=mw_edit()) != K_ESC && len != K_ENTER );
		mw_editend();
		if ( len == K_ESC ) {
		    mw_selectinit(& PatternMenu);
		    break;
		}
		term_menu();
		send_Pattern(dpattern, MaxPtnSize, key);
		mw_selectinit(&PatternMenu);
		break;
	    case SENDUSERDEFINE:
		if ( (len = UPattern_setup()) < 0 )
		    break;
		mw_selectend();
		term_menu();
		send_Pattern(upattern, len, key);
		mw_selectinit(&PatternMenu);
		break;
	    }
	} while ( key != K_ESC );
	mw_selectend();
}

static void send_Pattern(char *ptn, int len, int flag)
{
	int		key, cx, times=100, oldcnt=0, count=0, fd;
	int		wflag=0, slen, wlen;
	fd_set		rfdset, wfdset;
	struct timeval	tm;
	WINDOW *	wid;
	char		temp[81], ttyset[80];

	if ( pattern_file_term() < 0 )
	    return;
	tm.tv_sec = 0;
	tm.tv_usec = 10000;

	/*
	 *	for transmit screen init
	 */
	wid = mw_wopen(0, 1, 79, 4);
	mw_wcolor(wid, D_A_INV);
	mw_wdrawbox(wid, L_SINGLE);
	mw_wclose(wid);
	mw_attr(D_A_Normal);
	mw_cursor(1, 2);
	if ( flag == SENDDEFAULT )
	    printf("Sending default pattern.");
	else
	    printf("Sending user defined pattern.");
	printf(" Press CTRL-X to stop.\n");
	get_ttysetup(ttyset);
	sprintf(temp, "%s, Length=%d, Count=", ttyset, len);
	cx = strlen(temp) + 1;
	mw_cursor(1, 3);
	printf("%s\n", temp);

	/*
	 *	for receive screen init
	 */
	recvmon_init();
	if ( tty_fd > SioFd )
	    fd = tty_fd;
	else
	    fd = SioFd;
	fd++;
	FD_ZERO(&rfdset);
	FD_ZERO(&wfdset);
	while ( 1 ) {
	    if ( wflag == 0 ) {
		slen = 0;
		wflag = 1;
	    }
	    FD_SET(SioFd, &wfdset);
	    FD_SET(tty_fd, &rfdset);
	    FD_SET(SioFd, &rfdset);
	    if ( select(fd, &rfdset, &wfdset, NULL, &tm) > 0 ) {
		if ( FD_ISSET(tty_fd, &rfdset) ) {
		    key = mw_getkey();
		    if ( key == K_CTRL_X || key == K_CTRL_N )
			break;
		}
		if ( FD_ISSET(SioFd, &rfdset) )
		    recvmon_disp(0);
		if ( wflag == 1 && FD_ISSET(SioFd, &wfdset) ) {
		    if ( (wlen = write(SioFd, &ptn[slen], (len-slen))) > 0 ) {
			slen += wlen;
			if ( slen == len ) {
			    count ++;
			    wflag = 0;
			}
		    }
		}
	    }
	    times++;
	    if ( times >= 100 ) {
		times = 0;
		if ( oldcnt == count )
		    continue;
		sprintf(temp, "%d", count);
		mw_putstr_xy((uchar *)temp, strlen(temp), cx, 3);
		oldcnt = count;
	    }
	}
	if ( oldcnt != count ) {
	    sprintf(temp, "%d", count);
	    mw_putstr_xy((uchar *)temp, strlen(temp), cx, 3);
	}
	recvmon_end();
	terminal_end();
}

static int UPattern_setup()
{
	int	len, exit_flag = 0;

	if ( InputModeFlag == HEXMODE )
	    Uitem[0].des = HexDes;
	else
	    Uitem[0].des = AsciiDes;
	mw_editinit(&Umenu);
	while ( exit_flag == 0 ) {
	    switch( mw_edit() ) {
	    case K_ESC :
		len = -1;
		exit_flag = 1;
		break;
	    case K_CTRL_D :
		if ( (len = get_menu_data()) < 0 ) {
		    confirm(ErrorMsg);
		    break;
		}
		if ( InputModeFlag == HEXMODE ) {
		    InputModeFlag = ASCIIMODE;
		    Uitem[0].des = AsciiDes;
		} else {
		    InputModeFlag = HEXMODE;
		    Uitem[0].des = HexDes;
		}
		mw_wgotoxy(Umenu.wid, (Uitem[0].x - strlen(Uitem[0].des) -1), Uitem[0].y);
		mw_wputs(Umenu.wid, (uchar *)Uitem[0].des);
		mw_wputs(Umenu.wid, (uchar *)" ");
		change_data_mode(len);
		mw_wputs(Umenu.wid, (uchar *)Uitem[0].str);
		break;
	    case K_ENTER :
		if ( (len = get_menu_data()) < 0 )
		    confirm(ErrorMsg);
		exit_flag = 1;
		break;
	    }
	}
	mw_editend();
	return(len);
}

/******************************************************************************/
/* to get the user define input data					      */
/* Return:								      */
/*	< 0	data error						      */
/*	>=0	input data length					      */
/******************************************************************************/
static int get_menu_data()
{

	if ( InputModeFlag == HEXMODE )
	    return(get_hex_data());
	else
	    return(get_ascii_data());
}

/******************************************************************************/
/* to get the user define input data					      */
/* Return:								      */
/*	< 0	data error						      */
/*	>=0	input data length					      */
/******************************************************************************/
static int get_hex_data()
{
	int	i, len=0, val=0;
	char	*ptr, ch;

	ptr = Uitem[0].str;
	for ( i=0; i<40; i++ ) {
	    ch = *ptr++;
	    if ( ch == ' ' )
		continue;
	    if ( ch >= '0' && ch <= '9' )
		val = val * 16 + ch - '0';
	    else if ( ch >= 'a' && ch <='f' )
		val = val * 16 + ch - 'a' + 10;
	    else if ( ch >= 'A' && ch <= 'F' )
		val = val * 16 + ch - 'A' + 10;
	    else if ( ch == ',' ) {
		if ( val >= 256 )
		    return(-1);
		upattern[len++] = (char)val;
		val = 0;
	    } else
		return(-1);
	}
	if ( val != 0 )
	    upattern[len++] = (char)val;
	return(len);
}

/******************************************************************************/
/* to get the user define input data					      */
/* Return:								      */
/*	< 0	data error						      */
/*	>=0	input data length					      */
/******************************************************************************/
static int get_ascii_data()
{
	int	i, len=0;
	char	*ptr, ch;

	ptr = Uitem[0].str;
	for ( i=0; i<10; i++ ) {
	    ch = *ptr++;
	    if ( ch == ' ' )
		continue;
	    upattern[len++] = ch;
	}
	return(len);
}

static void change_data_mode(int len)
{
	int	i, j=0;
	char	*ptr, temp[5];

	ptr = Uitem[0].str;
	for ( i=0; i<40; i++ )
	    *ptr++ = ' ';
	ptr = Uitem[0].str;
	for ( i=0; i<len; i++ ) {
	    if ( InputModeFlag == HEXMODE ) {
		sprintf(temp, "%02x", upattern[i]);
		*ptr++ = temp[0];
		*ptr++ = temp[1];
		j += 2;
		if ( j >= 38 || i >= (len - 1) )
		    break;
		*ptr++ = ',';
	    } else {
		*ptr++ = upattern[i];
		j ++;
		if ( j >= 40 )
		    break;
	    }
	}
}
