/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	declare.h
*/

/*****************************************************************************/
/* DECLARE.H								     */
/*									     */
/* Copyright (c) 404 Technologies Inc. 1993. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	05/95	JE							     */
/*****************************************************************************/

/*****************************************************************************/
/*	Functions declare						     */
/*****************************************************************************/
/* showmsg.c */
extern	void	show_message_init();
extern	void	show_message();
extern	void	show_message_end();
extern	void	show_Tmessage_init ();
/* showstatus.c */
extern 	void	show_status_init();
extern 	void	show_status();
extern  void	show_status_end();
extern  void	show_status_wh();
/* f_exist.c */
extern	int	file_exist();
/* confirm.c */
extern	int	confirm();
/* ecevsys.c */
extern	int	exec_system();
/* fileproc.c */
extern	int	f_open();
extern	int	f_next_line();
extern	int	f_find_str();
extern	int	f_find_next_str();
extern	int	f_del_line();
extern	int	f_ins_line();
extern	int	f_append_str();
extern	int	f_max_line();
extern	int	f_close();
/* f_alloc.c */
extern  void	space_init();
extern 	int	get_space_ndx();
extern 	char 	*get_space();
extern 	void	space_end();
extern 	void	release_space();

/*****************************************************************************/
/*	Variables declare						     */
/*****************************************************************************/
