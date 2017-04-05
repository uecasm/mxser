/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term_mf.c
*/

/*****************************************************************************/
/* TERM_MF.C								     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1996. All Rights Reserved.	     */
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
#include	<signal.h>
#include	"term.h"

#define 	MaxPtnSize	40

static char	sfile_string[MaxPtnSize+1];

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static void	send_file();

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static	struct	WEItem	Uitem[] = {
	{ F_Active | F_Return, "Filename:", 0}
};
static	struct	WEdit	Umenu = {
	2, -1, 6,
	"Send File Name",
	{"Ctrl-D:Start to send  Esc:Exit", NULL},
	1, 0, 0, 0, Uitem, 1
};
static	char	filename[80];

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
void	sfile_setup()
{
	int	i, len, exit_flag=0;
	char	buf[80], *ptr, ch;

	Uitem[0].str = sfile_string;
	for ( i=0; i<MaxPtnSize; i++ )
	    Uitem[0].str[i] = ' ';
	Uitem[0].str[i] = 0;
	mw_editinit(&Umenu);
	while ( exit_flag == 0 ) {
	    switch( mw_edit() ) {
	    case K_ESC :
		exit_flag = 1;
		break;
	    case K_CTRL_D :
		exit_flag = 2;
	    case K_ENTER :
		ptr = Uitem[0].str;
		len = 0;
		for ( i=0; i<40; i++ ) {
		    ch = *ptr ++;
		    if ( ch == ' ' )
			continue;
		    filename[len++] = ch;
		}
		filename[len] = 0;
		if ( (i = open(filename, O_RDWR)) <= 0 ) {
		    sprintf(buf, "File %s does not exist !", filename);
		    confirm(buf);
		    exit_flag = 0;
		}
		close(i);
		break;
	    }
	}
	mw_editend();
	if ( exit_flag == 2 )
	    send_file();
}

static void send_file()
{
	int		wflag=0, readlen, filefd, key, cx, sendlen=0, fd;
	int		times=100, oldslen=0, slen, wlen;
	long		FileLen;
	fd_set		rfdset, wfdset;
	struct timeval	tm;
	WINDOW *	wid;
	char		temp[81], rbuf[129];

	if ( pattern_file_term() < 0 )
	    return;
	term_menu();
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
	filefd = open(filename, O_RDWR);
	FileLen = lseek(filefd, 0, SEEK_END);
	lseek(filefd, 0, SEEK_SET);
	mw_cursor(1, 2);
	printf("Sending file %s. Press CTRL-X to stop.\n", filename);
	get_ttysetup(rbuf);
	sprintf(temp, "%s, FileLength=%lu, SendLength=", rbuf, FileLen);
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
		if ( (readlen = read(filefd, rbuf, 128)) <= 0 )
		    break;
		else {
		    slen = 0;
		    wflag = 1;
		}
	    }
	    FD_SET(tty_fd, &rfdset);
	    FD_SET(SioFd, &rfdset);
	    FD_SET(SioFd, &wfdset);
	    if ( select(fd, &rfdset, &wfdset, NULL, &tm) > 0 ) {
		if ( FD_ISSET(tty_fd, &rfdset) ) {
		    key = mw_getkey();
		    if ( key == K_CTRL_N || key == K_CTRL_X )
			    break;
		}
		if ( FD_ISSET(SioFd, &rfdset) )
		    recvmon_disp(0);
		if ( wflag == 1 && FD_ISSET(SioFd, &wfdset) ) {
		    if ( (wlen = write(SioFd, &rbuf[slen], (readlen - slen)))
			 > 0 ) {
			sendlen += wlen;
			slen += wlen;
			if ( slen == readlen )
			    wflag = 0;
		    }
		}
	    }
	    times ++;
	    if ( times >= 100 ) {
		times = 0;
		if ( oldslen == sendlen )
		    continue;
		sprintf(temp, "%d", sendlen);
		mw_putstr_xy((uchar *)temp, strlen(temp), cx, 3);
		oldslen = sendlen;
	    }
	}
	if ( oldslen != sendlen ) {
	    sprintf(temp, "%d", sendlen);
	    mw_putstr_xy((uchar *)temp, strlen(temp), cx, 3);
	}
	close(filefd);
	recvmon_end();
	terminal_end();
}
