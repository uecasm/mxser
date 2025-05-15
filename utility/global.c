/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	global.c
*/

/*****************************************************************************/
/* GLOBAL.C                                                                  */
/*									     */
/* Copyright (c) Moxa Technologies Inc. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*     8/5/99   Casper                                                       */
/*     3/1/01   Casper  Add Industio                                         */
/*****************************************************************************/

#include "global.h"
char *mxupcie_brdname[] = {
        "CP-102E series",
        "CP-102EL series",
        "CP-132EL series",
        "CP-114EL series",
        "CP-104EL-A series",
        "CP-168EL-A series",
        "CP-118EL_A series",
        "CP-118E-A series",
        "CP-138E-A series",
        "CP-134EL-A series",
        "CP-116E-A series (A)",
        "CP-116E-A series (B)",
	"CP-102N series",
	"CP-132N series",
	"CP-112N series",
	"CP-104N series",
	"CP-134N series",
	"CP-114N series"
};
int mxupcie_numports[] = {
        2,      //CP-102E
        2,      //CP-102EL
        2,      //CP-132EL
        4,      //CP-114EL
        4,      //CP-104EL
        8,      //CP-168EL
        8,      //CP-118EL
        8,      //CP-118E-A-I
        8,      //CP-138E-A
        4,      //CP-134EL-A
        8,      //CP-116EA (A)
        8,      //CP-116EA (B)
	2,	//CP-102N
	2,	//CP-132N
	2,	//CP-112N
	4,	//CP-104N
	4,	//CP-134N
	4	//CP-114N
};
char *mxser_brdname[] = {
	"C168 series",
	"C104 series",
        "CI-104J series",
	"C168H/PCI series",
	"C104H/PCI series",
	"C102 series",
	"CI-132 series",
	"CI-134 series",
	"CP-132 series",
        "CP-114 series",
	"CT-114 series",
	"CP-102 series",
	"CP-104U series",
	"CP-168U series",
	"CP-132U series",
	"CP-134U series",
	"CP-104JU series",
	"Moxa UC7000 Serial",
	"CP-118U series",
	"CP-102UL series",
	"CP-102U series",
	"CP-118EL series",
	"CP-168EL series",
	"CP-104EL series",
	"CB-108 series",
	"CB-114 series",
	"CB-134I series",
	"CP-138U series",
	"POS-104UL series",
	"CP-114UL series", //Lion,2007/05/03
	"CP-102UF series",
	"CP-112UL series",
	/* PC104 series */
	"CA-104 series",
	"CA-132 series",
	"CA-132I series",
	"CA-108 series",
	"CA-114 series",
	"CA-134I series"
};

int mxser_numports[] = {
	8,	// C168
	4,	// C104
        4,	// CI-104I
	8,	// C168H/PCI
	4,	// C104H/PCI
	2,	// C102
	2,	// CI-132
	4,	// CI-134
	2,	// CP-132
	4,	// CP-114
	4,	// CT-114
	2,	// CP-102
	4,	// CP-104U
	8,	// CP-168U
	2,	// CP-132U
	4,	// CP-134U
	4,	// CP-104JU
	8,	// Moxa UC7000
	8,	// CP-118U
	2,	// CP-102UL
	2,	// CP-102U
	8,	// CP-118EL
	8,	// CP-168EL
	4,	// CP-104EL
	8,	// CB-108
	4,	// CB-114
	4,	// CB-134I
	8,	//CP-138U
	4,	//POS-104UL
	4,	//CP-114UL add by Lion ,2007/05/03
	2,	//CP-102UF
	2,	//CP-112UL
	/* PC104 series */
	4,	//CA104
	2,	//CA132
	2,	//CA132I
	8,	//CA108
	4,	//CA114
	4,	//CA134I
};
