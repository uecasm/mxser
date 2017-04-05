/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	kbd.h
*/

/**********************************************************************
 * kbd.h
 *	Definitions for keyboard
 *
 *	Copyright (c) 404 Technologies Inc. 1993. All Rights Reserved.
 *	by Kevin Li, Dec 1993
 *********************************************************************/
#ifndef KBD404
#define KBD404

#define K_BACKSPACE	8
#define K_TAB		9
#define	K_CTRL_A	0x01
#define	K_CTRL_B	0x02
#define	K_CTRL_C	0x03
#define	K_CTRL_D	0x04
#define	K_CTRL_E	0x05
#define	K_CTRL_F	0x06
#define	K_CTRL_G	0x07
#define	K_CTRL_H	0x08
#define	K_CTRL_I	0x09 
#define	K_CTRL_J	0x0a
#define	K_CTRL_K	0x0b
#define	K_CTRL_L	0x0c
#define	K_CTRL_N	0x0e
#define	K_CTRL_O	0x0f
#define	K_CTRL_P	0x10
#define	K_CTRL_R	0x12
#define	K_CTRL_T	0x14
#define	K_CTRL_U	0x15
#define	K_CTRL_V	0x16
#define	K_CTRL_W	0x17
#define	K_CTRL_X	0x18
#define	K_CTRL_Y	0x19
#define	K_CTRL_Z	0x1a

/* To prevent from confused with selected item */
#define K_ESC		(27 | 0x100)

#define K_ENTER_A	10
#define K_ESC_A 	27

/*#ifndef LINUX*/
#define K_ENTER 	(10 | 0x100)

#ifdef SCOUNIX
#define K_F1		(0x4d | 0x100)
#define K_F2		(0x4e | 0x100)
#define K_F3		(0x4f | 0x100)
#define K_F4		(0x50 | 0x100)
#define K_F5		(0x51 | 0x100)
#define K_F6		(0x52 | 0x100)
#define K_F7		(0x53 | 0x100)
#define K_F8		(0x54 | 0x100)
#define K_F9		(0x55 | 0x100)
#define K_F10		(0x56 | 0x100)
#define K_F11		(0x57 | 0x100)
#define K_F12		(0x58 | 0x100)

#define K_HOME		(0x48 | 0x100)
#define K_END		(0x46 | 0x100)
#define K_PGUP		(0x49 | 0x100)
#define K_PGDOWN	(0x47 | 0x100)
#define K_UP		(0x41 | 0x100)
#define K_DOWN		(0x42 | 0x100)
#define K_LEFT		(0x44 | 0x100)
#define K_RIGHT 	(0x43 | 0x100)
#define K_INS		(0x4c | 0x100)
#define K_DEL		(83 | 0x100)		/*??? */

#else 
#ifndef LINUX

#define K_F1		(0x50 | 0x100)
#define K_F2		(0x51 | 0x100)
#define K_F3		(0x52 | 0x100)
#define K_F4		(0x53 | 0x100)
#define K_F5		(0x54 | 0x100)
#define K_F6		(0x55 | 0x100)
#define K_F7		(0x56 | 0x100)
#define K_F8		(0x57 | 0x100)
#define K_F9		(0x58 | 0x100)
#define K_F10		(0x59 | 0x100)
#define K_F11		(0x5a | 0x100)
#define K_F12		(0x41 | 0x100)

#define K_HOME		(0x48 | 0x100)
#define K_END		(0x59 | 0x100)
#define K_PGUP		(0x56 | 0x100)
#define K_PGDOWN	(0x55 | 0x100)
#define K_UP		(0x41 | 0x100)
#define K_DOWN		(0x42 | 0x100)
#define K_LEFT		(0x44 | 0x100)
#define K_RIGHT 	(0x43 | 0x100)
#define K_INS		(0x40 | 0x100)
#define K_DEL		(83 | 0x100)		/*??? */
#endif

#endif
/*#else
#define K_HOME		(0x48 | 0x100)
#define K_END		(0x59 | 0x100)
#define K_PGDOWN	(0x55 | 0x100)
#define K_INS		(0x40 | 0x100)
#define K_DEL		(83 | 0x100)		
#endif
*/
#ifdef LINUX
#define K_F1		0x41	
#define K_F2		0x42	
#define K_F3		0x43	
#define K_F4		0x44	
#define K_F5		0x45	
#define K_F6		0x37	
#define K_F7		0x38
#define K_F8		0x39
#define K_F9		0x30
#define K_F10		0x31
#define K_F11		0x33
#define K_F12		0x34

#define K_HOME		(0x31 | 0x100)
#define K_END		(0x34 | 0x100)
#define K_PGUP		(0x35 | 0x100)
#define K_PGDOWN	(0x36 | 0x100)
#define K_UP		(0x41 | 0x100)
#define K_DOWN		(0x42 | 0x100)
#define K_LEFT		(0x44 | 0x100)
#define K_RIGHT 	(0x43 | 0x100)
#define K_INS		(0x4c | 0x100)
#define K_DEL		(83 | 0x100)		
#endif /*LINUX*/

#define K_ALT_Q 	0x171
#define K_ALT_W 	0x177
#define K_ALT_E 	0x165
#define K_ALT_R 	0x172
#define K_ALT_T 	0x174
#define K_ALT_Y 	0x179
#define K_ALT_U 	0x175
#define K_ALT_I 	0x169
#define K_ALT_O 	0x16f
#define K_ALT_P 	0x170
#define K_ALT_A 	0x161
#define K_ALT_S 	0x173
#define K_ALT_D 	0x164
#define K_ALT_F 	0x166
#define K_ALT_G 	0x167
#define K_ALT_H 	0x168
#define K_ALT_J 	0x16a
#define K_ALT_K 	0x16b
#define K_ALT_L 	0x16c
#define K_ALT_Z 	0x17a
#define K_ALT_X 	0x178
#define K_ALT_C 	0x163
#define K_ALT_V 	0x176
#define K_ALT_B 	0x162
#define K_ALT_N 	0x16e
#define K_ALT_M 	0x16d

#endif
