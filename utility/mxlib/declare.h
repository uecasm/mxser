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
extern	void	show_message_init(int, int);
extern	void	show_message(char *);
extern	void	show_message_end();
extern	void	show_Tmessage_init (int, int, char *);
/* showstatus.c */
extern 	void	show_status_init(int, int, char *);
extern 	void	show_status(char *, char *);
extern  void	show_status_end();
extern  void	show_status_wh(char *, char *, int, int);
/* f_exist.c */
extern	int	file_exist(char *);
/* confirm.c */
extern	int	confirm(char *);
/* ecevsys.c */
extern	int	exec_system(char *);
/* fileproc.c */
extern	int	f_open(char *);
extern	int	f_next_line(int, char *);
extern	int	f_find_str(char *);
extern	int	f_find_next_str(int, char*);
extern	int	f_del_line(int);
extern	int	f_ins_line(int, char *);
extern	int	f_append_str(char *);
extern	int	f_max_line();
extern	int	f_close(int);
/* f_alloc.c */
extern  void	space_init();
extern 	int	get_space_ndx();
extern 	char 	*get_space(int);
extern 	void	space_end();
extern 	void	release_space(int);

/*****************************************************************************/
/*	Variables declare						     */
/*****************************************************************************/
