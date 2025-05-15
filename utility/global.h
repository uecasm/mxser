/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	global.h
*/
/*****************************************************************************/
/* GLOBAL.H                                                                  */
/*									     */
/* Copyright (c) Moxa Technologies Inc. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*     8/5/99   Casper                                                       */
/*     3/1/01   Casper  Add Industio                                         */
/*****************************************************************************/

#ifndef _GLOBAL_H
#define _GLOBAL_H

#define		MXSER_PORTS_PER_BOARD	8
#define 	MXSER_BOARDS		4	/* Max. boards */

#define         MXSER_MAXPORT   (MXSER_PORTS_PER_BOARD * MXSER_BOARDS)

#define		MOXA	0x400
#define		MOXA_GET_CONF	(MOXA + 35)

#ifndef		B921600
#define		B921600 (B460800 + 1)
#endif

#ifndef		CMSPAR
#define		CMSPAR 010000000000 /* mark/space parity */
#endif

#define		BOARD_TYPE	38	

typedef struct __attribute__((__packed__)) _moxa_pci_usr_info{
	unsigned short busNum;
	unsigned short devNum;
} moxa_pci_usr_info;

struct __attribute__((__packed__)) mxser_usr_hwconf{
	int	board_type;
	unsigned long	ioaddr[MXSER_PORTS_PER_BOARD];
	int	baud[MXSER_PORTS_PER_BOARD];
	moxa_pci_usr_info pciInfo;
	int	IsMoxaMustChipFlag;
	int	MaxCanSetBaudRate[MXSER_PORTS_PER_BOARD];
};


struct __attribute__((__packed__)) mxupcie_usr_hwconf{
	int	board_type;
	unsigned char *	ioaddr[MXSER_PORTS_PER_BOARD];
	int	baud[MXSER_PORTS_PER_BOARD];
	moxa_pci_usr_info pciInfo;
	int 	IsMoxaMustChipFlag;
	int	MaxCanSetBaudRate[MXSER_PORTS_PER_BOARD];
};

struct __attribute__((__packed__)) mxupcie_pci_setting{
	int	whichPciBoard;
	int	cfg_value;
};

enum	{
	MXSER_BOARD_C168_ISA = 1,
	MXSER_BOARD_C104_ISA,
        MXSER_BOARD_CI104J,
	MXSER_BOARD_C168_PCI,
	MXSER_BOARD_C104_PCI,
	MXSER_BOARD_C102_ISA,
	MXSER_BOARD_CI132,
	MXSER_BOARD_CI134,
	MXSER_BOARD_CP132,
	MXSER_BOARD_CP114, //10
	MXSER_BOARD_CT114,
	MXSER_BOARD_CP102,
	MXSER_BOARD_CP104U,
	MXSER_BOARD_CP168U,
	MXSER_BOARD_CP132U,
	MXSER_BOARD_CP134U,
	MXSER_BOARD_CP104JU,
	MXSER_BOARD_RC7000,
	MXSER_BOARD_CP118U,
	MXSER_BOARD_CP102UL, //20
	MXSER_BOARD_CP102U,
	MXSER_BOARD_CP118EL,
	MXSER_BOARD_CP168EL,
	MXSER_BOARD_CP104EL,
	MXSER_BOARD_CB108,
	MXSER_BOARD_CB114,
	MXSER_BOARD_CB134I,
	MXSER_BOARD_CP138U,
	MXSER_BOARD_POS104UL,
	MXSER_BOARD_CP114UL, //30
	MXSER_BOARD_CP102UF,
	MXSER_BOARD_CP112UL,
	/* PC104 series */
	MXSER_PC_BOARD_CA104,
	MXSER_PC_BOARD_CA132,
	MXSER_PC_BOARD_CA132I,
	MXSER_PC_BOARD_CA108,
	MXSER_PC_BOARD_CA114,
	MXSER_PC_BOARD_CA134I
};

enum    {
        MXUPCIE_BOARD_CP102E = 1,
        MXUPCIE_BOARD_CP102EL,
        MXUPCIE_BOARD_CP132EL,
        MXUPCIE_BOARD_CP114EL,
        MXUPCIE_BOARD_CP104EL_A,
        MXUPCIE_BOARD_CP168EL_A,
        MXUPCIE_BOARD_CP118EL_A,
        MXUPCIE_BOARD_CP118E_A_I, /* Support New Ioctl AutoMode etc... */
        MXUPCIE_BOARD_CP138E_A,   /* Support New Ioctl AutoMode etc... */
        MXUPCIE_BOARD_CP134EL_A,  /* Support New Ioctl AutoMode etc... */
        MXUPCIE_BOARD_CP116E_A_A,  /* Support New Ioctl AutoMode etc, It the first part of 16 port */
        MXUPCIE_BOARD_CP116E_A_B,  /* Support New Ioctl AutoMode etc, It the second part of 16 port */
	MXUPCIE_BOARD_CP102N,
	MXUPCIE_BOARD_CP132N,
	MXUPCIE_BOARD_CP112N,
	MXUPCIE_BOARD_CP104N,
	MXUPCIE_BOARD_CP134N,
	MXUPCIE_BOARD_CP114N
};

extern  char *mxser_brdname[];
extern  int  mxser_numports[];
extern  char *mxupcie_brdname[];
extern  int  mxupcie_numports[];

#endif
