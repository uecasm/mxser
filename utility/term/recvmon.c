/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	recvmon.c
*/

/*****************************************************************************/
/* recvmon.c								     */
/*									     */
/* Copyright (c) Moxa Technologies Co., LTD. 1999. All Rights Reserved.	     */
/*									     */
/* Version History:							     */
/*	date		author	      comment 				     */
/*	13/03/1996	Victor	      wrote it				     */
/*	12/96		Hsu	      Modify for Linux 2.0.0 (C168/C104 ADM) */
/*       8/5/1999       Casper                                               */
/*****************************************************************************/

#include	"term.h"

#define K_CR		0x0d
#define K_LF		0x0a
#define LEN		156

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static WINDOW	*wid;
static int	y, x, rlen;
static char	clearstr[81], rbuf[LEN];

/*****************************************************************************/
/* global subroutine							     */
/*****************************************************************************/
void	recvmon_init(void);
void	recvmon_end(void);
void	recvmon_disp(int);

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
void	recvmon_init()
{
	int	i;

	mw_cursor(0, 6);
	printf("Received data monitor:\n");
	wid = mw_wopen(0, 7, 79, 22);
	mw_wcolor(wid, wid->norm_c);
	mw_wcls(wid);
	mw_wdrawbox(wid, L_SINGLE);
	for ( i=0; i<80; i++ )
	    clearstr[i] = ' ';
	y = 8;
	x = 1;
	rlen = 0;
}

void	recvmon_disp(int flag)
{
	char	ch, lflag=0, temp[81];
	int	slen, i, j, k, sx;

	if ( flag == 0 ) {
	    if ( (i = read(SioFd, &rbuf[rlen], LEN - rlen)) <= 0 )
		return;
	    rlen += i;
	    if ( rlen < LEN )
		return;
	}
	sx = x;
	slen = 0;
	for ( i=0 ; i<rlen; i++ ) {
	    ch = rbuf[i];
	    if ( ch == K_CR ) {
		lflag = 1;
		if ( rbuf[i+1] == K_LF )
		    i ++;
	    } else if ( ch == K_LF ) {
		lflag = 1;
		if ( rbuf[i+1] == K_CR )
		    i ++;
	    } else if ( ch == K_TAB ) {
		j = x;
		x += 7;
		x = (x / 8) * 8 + 1;
		if ( x >= 79 )
		    lflag = 1;
		else {
		    k = x - j;
		    for ( j=0; j<k; j++ )
			temp[slen++] =' ';
		}
	    } else {
		temp[slen++] = ch;
		x++;
		if ( x >= 79 )
		    lflag = 1;
	    }
	    if ( lflag == 1 ) {
		mw_putstr_xy((uchar *)temp, slen, sx, y);
		slen = 0;
		sx = x = 1;
		y++;
		if ( y >= 22 )
		    y = 8;
		lflag = 0;
		mw_putstr_xy((uchar *)clearstr, 78, 1, y);
	    }
	}
	if ( slen != 0 && lflag == 0 )
	    mw_putstr_xy((uchar *)temp, slen, sx, y);
	rlen = 0;
}

void recvmon_end()
{

	recvmon_disp(1);
	mw_wclose(wid);
}
