/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	term_s.c
*/

/*****************************************************************************/
/* term_s.c								     */
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

#define Total_Item	8
#define Col_Space	15

/*****************************************************************************/
/* GLOBAL VARIABLES							     */
/*****************************************************************************/
int	term_s_setup(void);
void	get_ttysetup(char *);

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static	int	init_menu(void);
static	void	prepare_menu(void);
static	void	get_menu_data();
static	int	change_setup();

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static	int		itemflag[Total_Item] = {
	F_Writable | F_Return,
	F_Select | F_Return,
	F_Select | F_Return,
	F_Select | F_Return,
	F_Select | F_Return,
	F_Select | F_Return,
	F_Select | F_Return,
	F_Select | F_Return
};
static	char		*itemdes[Total_Item] = {
	" Device Name            ",
	" Baud Rate              ",
	" Parity                 ",
	" Data Bits (5 - 8)      ",
	" Stop Bits (1 - 2)      ",
	" Flow Control           ",
	" Local Echo             ",
	" Send Enter as (CR/LF)  "
};

static char	InstrMsg[]="Enter:Select  Esc:Exit";

static	struct	GEdit2	menu2 = {
	Edit2Type0, -1, 4, Total_Item,
	"Port Setup", 0, 0,
	{InstrMsg},
	0
};
static	struct	dterm_info	old_term;

/*****************************************************************************/
/*	MAIN FUNCTIONS							     */
/*****************************************************************************/
int	term_s_setup()
{
	int	old_space_ndx, key;

	old_term = DTerm;
	old_space_ndx = init_menu();
	mw_edit2init(&menu2);
	prepare_menu();
	do {
	    key = mw_edit2();
	    if ( key == K_ENTER ) {
		get_menu_data();
		prepare_menu();
	    }
	} while ( key != K_ESC );
	get_menu_data();
	release_space(old_space_ndx);
	mw_edit2end();
	key = change_setup();
	if ( key && SioFd > 0 ) {
	    tcflush(SioFd, 2);	    
	    close(SioFd);
	    SioFd = -1;
	}
	return(0);
}

void	get_ttysetup(char *str)
{
	char	temp[81];
	int	i, fd;

        i = HIGH_SPEED_ITEM;
	if ( DTerm.rate >= i )
	    DTerm.rate = i-1;
	strcpy(temp, BaudRateMenuItem[DTerm.rate].str);
	i = strlen(temp);
	temp[i++] = ',';
	switch ( DTerm.parity ) {
	case 0 : temp[i++] = 'N'; break;
	case 1 : temp[i++] = 'O'; break;
	case 2 : temp[i++] = 'E'; break;
#ifdef CMSPAR
	case 3 : temp[i++] = 'M'; break;
	case 4 : temp[i++] = 'S'; break;
#endif
	}
	temp[i++] = ',';
	switch ( DTerm.databit ) {
	case 0 : temp[i++] = '5'; break;
	case 1 : temp[i++] = '6'; break;
	case 2 : temp[i++] = '7'; break;
	case 3 : temp[i++] = '8'; break;
	}
	temp[i++] = ',';
	switch ( DTerm.stopbit ) {
	case 0 : temp[i++] = '1'; break;
	case 1 : temp[i++] = '2'; break;
	}
	temp[i++] = 0;
	sprintf(str, "Device=%s, Set=%s", DTerm.dname, temp);
}

static int init_menu(void)
{
	int	i, k, old_space_ndx;

	old_space_ndx = get_space_ndx();
	i = sizeof(struct GItem2) * Total_Item;
	menu2.item = (struct GItem2 *)get_space(i);
	for ( i=0; i<Total_Item; i++ ) {
	    menu2.item[i].flag = itemflag[i];
	    menu2.item[i].des = itemdes[i];
	    menu2.item[i].act_item = 0;
	    menu2.item[i].total = 1;
	    menu2.item[i].item_no[0] = 0;
	    menu2.item[i].len[0] = Col_Space;
	    menu2.item[i].str[0] = get_space(Col_Space + 1);
	    for ( k = 0; k < Col_Space; k++ )
		menu2.item[i].str[0][k] = ' ';
	    menu2.item[i].str[0][k] = 0;
	}
	menu2.item[1].menup[0].selectp = &BaudRateMenu;
	menu2.item[2].menup[0].selectp = &ParityMenu;
	menu2.item[3].menup[0].selectp = &DataBitMenu;
	menu2.item[4].menup[0].selectp = &StopBitMenu;
	menu2.item[5].menup[0].selectp = &FlowMenu;
	menu2.item[6].menup[0].selectp = &YesNoMenu;
	menu2.item[7].menup[0].selectp = &EnterMenu;
	menu2.act_item = 0;
	return(old_space_ndx);
}

static void prepare_menu()
{
	int	fd;

	sprintf(menu2.item[0].str[0], "%-15s", DTerm.dname);
	menu2.item[1].item_no[0] = DTerm.rate;
	menu2.item[2].item_no[0] = DTerm.parity;
	menu2.item[3].item_no[0] = DTerm.databit;
	menu2.item[4].item_no[0] = DTerm.stopbit;
	menu2.item[5].item_no[0] = DTerm.xflow;
	menu2.item[6].item_no[0] = DTerm.lecho;
	menu2.item[7].item_no[0] = DTerm.senter;
        BaudRateMenu.total = BaudRateMenu.Ndisp = HIGH_SPEED_ITEM;
	edit2_flush_data(&menu2);
}

static void get_menu_data()
{
	int	i, ch;

	for ( i=0; i<Col_Space; i++ ) {
	    ch = menu2.item[0].str[0][i];
	    if ( ch == ' ' )
		break;
	    DTerm.dname[i] = ch;
	}
	DTerm.dname[i] = 0;
	DTerm.rate = menu2.item[1].item_no[0];
	DTerm.parity = menu2.item[2].item_no[0];
	DTerm.databit = menu2.item[3].item_no[0];
	DTerm.stopbit = menu2.item[4].item_no[0];
	DTerm.xflow = menu2.item[5].item_no[0];
	DTerm.lecho = menu2.item[6].item_no[0];
	DTerm.senter = menu2.item[7].item_no[0];
}

static int change_setup()
{

	if ( strcmp(DTerm.dname, old_term.dname) != 0 )
	    return(1);
	if ( DTerm.rate != old_term.rate )
	    return(1);
	if ( DTerm.parity != old_term.parity )
	    return(1);
	if ( DTerm.databit != old_term.databit )
	    return(1);
	if ( DTerm.stopbit != old_term.stopbit )
	    return(1);
	if ( DTerm.xflow != old_term.xflow )
	    return(1);
	if ( DTerm.lecho != old_term.lecho )
	    return(1);
	if ( DTerm.senter != old_term.senter )
	    return(1);
	return(0);
}
