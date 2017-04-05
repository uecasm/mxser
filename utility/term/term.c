/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term.c
*/

/*****************************************************************************/
/* TERM.C								     */
/*									     */
/* Copyright (c) Moxa Technologies Inc. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	10/95	JOSE							     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*****************************************************************************/

/*****************************************************************************/
/* include file 							     */
/*****************************************************************************/
#include	<sys/time.h>
#include	<sys/types.h>
#include	<unistd.h>
#include	<errno.h>
#include	<termios.h>
#include	"term.h"

#define NO	0
#define YES	1
#define K_LF	0x0a
#define K_CR	0x0d
#define M_CRLF	0
#define M_CR	1
#define M_LF	2
#define M_LFCR	3

#define COMMSETUP	0
#define DUMBTERM	1
#define TRANMODE	2

/*****************************************************************************/
/* local define 							     */
/*****************************************************************************/

/*****************************************************************************/
/* GLOBAL VARIABLES							     */
/*****************************************************************************/
extern	int	errno;

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static	struct	termios	tty_old;
static	struct	WItem	mnMenuitem[] = {
	{ F_Active, "Communication Setup" },
	{ F_Active, "Dumb Terminal      " },
	{ F_Active, "Advanced Transfer Modes" }
};

static char	InstrMsg[]="Enter:Select  Esc:Exit";

static	struct	WSelect mnMenu = {
	-1, 6, "Dumb Terminal Emulation Menu", 3, 0, 0, 0, 3, mnMenuitem,
	{InstrMsg, 0}, 0
};
static	int	init_flag = 0;
static	char	*title_msg=
	"     Dumb Terminal Emulation         Ctrl-N: menu";

/*****************************************************************************/
/* GLOBAL FUNCTIONS							     */
/*****************************************************************************/
void	terminal_end();
void	term_menu();
int	pattern_file_term();
void	term_clr_screen();

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static	void	set_dterm();
static	void	sio_init();
static	void	con_init();
static	void	term_mode();

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
void	term_setup()
{
	int	key;
	int	ret;

	term_menu();
	tcgetattr(tty_fd, &tty_old);
	if ( init_flag == 0 ) {
	    set_dterm();
	    init_flag = 1;
	}
	mw_selectinit(&mnMenu);
	do {
	    key = mw_select();
	    switch ( key ) {
	    case COMMSETUP:
		term_s_setup();
		break;
	    case TRANMODE:
		mw_selectend();
		mode_setup();
		term_menu();
		mw_selectinit(&mnMenu);
		break;
	    case DUMBTERM:
		mw_selectend();
		term_mode();
		mw_selectinit(&mnMenu);
		break;
	    }
	} while ( key != K_ESC );
	mw_selectend();
	terminal_end();
	if ( SioFd > 0 ) {
	    tcflush(SioFd, 2);	    	    
	    close(SioFd);
	    SioFd = -1;
	}
	ret = system("clear");
}

void	term_menu()
{
	WINDOW	*wid;

	mw_clearscreen();
	wid = mw_wopen(0, 0, 79, 0);
	mw_wcolor(wid, D_A_INV);
	mw_wfill(wid, ' ');
	mw_wcputs(wid, 0, (uchar *)title_msg);
	mw_wclose(wid);
	mw_attr(D_A_Normal);
}

int	pattern_file_term()
{
	char	buf[81];

	sio_init();
	if ( SioFd < 0 ) {
	    sprintf(buf, "%s open error(errno = %d) !", DTerm.dname, errno);
	    confirm(buf);
	    return(-1);
	}
	con_init();
	return(0);
}

void	terminal_end()
{

	tcsetattr(tty_fd, TCSANOW, &tty_old);
}

static void term_mode()
{
	int	fd, i, len, exitflag = 0;
	uchar	key, buf[101];
	fd_set	rfdset, bakset;
	struct	timeval tm;
	int	ret;

	if ( pattern_file_term() < 0 )
	    return;
	term_menu();
        
	mw_cursor(0, 1);
	if ( tty_fd > SioFd )
	    fd = tty_fd;
	else
	    fd = SioFd;
	tm.tv_sec = 0;
	tm.tv_usec = 100000;
        FD_ZERO(&rfdset);
	FD_SET(tty_fd, &rfdset);
	FD_SET(SioFd, &rfdset);
        bakset = rfdset;
	while ( exitflag == 0 ) {
            rfdset = bakset;
	    if ( select(fd+1, &rfdset, NULL, NULL, NULL) > 0 ) {
		if ( FD_ISSET(tty_fd, &rfdset) ) {
		    key = getchar();
		    switch ( key ) {
		    case K_CTRL_N:
			exitflag = 1;
			break;
		    default:
			i = 0;
			if ( key == K_CR ) {
			    switch ( DTerm.senter ) {
			    case M_CRLF:
				buf[i++] = K_CR;
				buf[i++] = K_LF;
				break;
			    case M_CR:
				buf[i++] = K_CR;
				break;
			    case M_LF:
				buf[i++] = K_LF;
				break;
			    case M_LFCR:
				buf[i++] = K_LF;
				buf[i++] = K_CR;
				break;
			    }
			} else
			    buf[i++] = key;
			ret = write(SioFd, buf, i);
			break;
		    }
		}
		if ( FD_ISSET(SioFd, &rfdset) ) {
		    if ( (len = read(SioFd, buf, 100)) > 0 )
			ret = write(tty_fd, buf, len);
		}
	    }
	}
	terminal_end();
}

static void set_dterm()
{
        strcpy(DTerm.dname, "/dev/ttyS0");
	DTerm.rate = 12; /*B9600*/
	DTerm.parity = 0;
	DTerm.databit = 3;
	DTerm.stopbit = 0;
	DTerm.xflow = 0;
	DTerm.lecho = 0;
	DTerm.senter = 1;
}

static void con_init()
{
	struct termios tty;

	tcflush(tty_fd, 2);
	tcgetattr(tty_fd, &tty);

	switch ( DTerm.lecho ) {
	case 0:
	    tty.c_lflag &= ~ECHO;
	    break;
	case 1:
	    tty.c_lflag |= ECHO;
	    break;
	}
	tty.c_iflag &= ~ICRNL;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;
	tty.c_lflag &= ~(ICANON);
	tcsetattr(tty_fd, TCSANOW, &tty);
}

static void sio_init()
{
	int		cflag = CREAD | CLOCAL | HUPCL;
	struct termios	tty;

	if ( SioFd > 0 ) {
	    tcflush(SioFd, 2);
	    return;
	}
	if ( (SioFd = open(DTerm.dname, O_RDWR | O_NDELAY)) == -1 )
	    return;
	tcflush(tty_fd, 2);
	tcgetattr(tty_fd, &tty);

        cflag |= BaudMapTab[DTerm.rate];
        
#ifdef CMSPAR
	cflag &= ~(PARENB | PARODD | CMSPAR);
#else
	cflag &= ~(PARENB | PARODD);
#endif
	switch ( DTerm.parity ) {
	case 1:
	    cflag |= PARENB | PARODD;
	    break;
	case 2:
	    cflag |= PARENB;
	    break;
#ifdef CMSPAR
	case 3: /* mark */
	    cflag |= PARENB | CMSPAR | PARODD;
	    break;
	case 4: /* space */
	    cflag |= PARENB | CMSPAR;
	    break;
#endif
	}
	cflag |= (DTerm.databit * CS6);
	if ( DTerm.stopbit )
	    cflag |= CSTOPB;
	tty.c_cflag = cflag;
	switch ( DTerm.xflow ) {
	case 0:
		tty.c_cflag |= CRTSCTS;
		tty.c_iflag &= ~(IXON | IXOFF);
		break;
	case 1:
		tty.c_iflag |= IXON | IXOFF;
		tty.c_cflag &= (~CRTSCTS);
		break;
	case 2:
		tty.c_iflag |= IXON | IXOFF;
		tty.c_cflag |= CRTSCTS;
		break;
	case 3:
		tty.c_iflag &= ~(IXON | IXOFF);
		tty.c_cflag &= ~(CRTSCTS);
		break;
	}
	tty.c_lflag &= ~(ICANON | ECHO);
	tty.c_iflag &= ~(ICRNL | ISTRIP);
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;
	tcsetattr(SioFd, TCSANOW, &tty);
        tcgetattr(SioFd, &tty);
}


void main_end()
{
	mw_wend();
}

int main()
{
	int	fd, ret;
        int     i;
        int     boards;

	space_init();	
	mw_winit();
        
        term_setup();
	
	mw_wend();

        return 0;
}



