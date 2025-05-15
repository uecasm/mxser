/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mon_p.c
*/

/*****************************************************************************/
/* MON_P.C                                                                   */
/*									     */
/* Copyright (c) Moxa Technologies Inc. 1999. All Rights Reserved.	     */
/*									     */
/* Revision history:							     */
/*	12/95	JOSE							     */
/*	12/96	Hsu		Modify for Linux 2.0.0 (C168/C104 ADM)	     */
/*     8/5/99   Casper                                                       */
/*     3/9/2001 Casper          add Industio                                 */
/*****************************************************************************/

#include	<time.h>
#include	<errno.h>
#include	<string.h>
#include	<stdlib.h>
#include	<errno.h>
#include	"../mxwinlib/win.h"
#include	"../mxlib/declare.h"
#include	"../global.h"
#include        "mon.h"

#define	MOXA_GETDATACOUNT	(MOXA + 23)
#define	MOXA_GET_MAJOR		(MOXA + 63)

#define Total_Item	7
#define Max_Row 	16
#define Row_Space	10
#define Min_Val 	3
#define Max_Val 	10
#define Card		0
#define CBox		1


struct	Moniter_str {
	int	tick;
	unsigned long	rxcnt[MXSER_MAXPORT];
	unsigned long	txcnt[MXSER_MAXPORT];
};
typedef struct	Moniter_str	Moniter_t;
typedef struct	Moniter_str *	f_Moniter_t;


/*****************************************************************************/
/* GLOBAL FUNCTIONS							     */
/*****************************************************************************/
int	mon_p_setup(int boards);

/*****************************************************************************/
/* STATIC FUNCTIONS							     */
/*****************************************************************************/
static	int	init_menu(int now_board);
static	void	flush_menu_data(int now_board);
static	void	show_timer(int now_board);
static	void	reset_count();
static int	getdatacnt(struct Moniter_str *logdata);
static	void	mx_make_mxser_node();
static	void	mx_make_mxupcie_node();
static int	mx_get_mxser_major();
static int	mx_get_mxupcie_major();

/*****************************************************************************/
/* STATIC VARIABLES							     */
/*****************************************************************************/
static	int		row_space[Total_Item] = { 10, 10, 8, 8, 10, 8, 8 };
static	int		itemflag[Total_Item] = {
	F_Return,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly,
	F_Readonly
};
static	char		*itemdes[Total_Item] = {
	"",
	"",
	"",
	"",
	"",
	"",
	""
};
static	struct	GEdit3	menu3 = {
	Edit3Type0 | Edit3Type3 | Edit3Type5, -1, -1, Total_Item,
	0, 0, 0,
	{ "PgUp,PgDn:More Ports Home:Reset Count  Esc:Exit  Enter:Port Status"},
	0
};

static	char	*subt_des[] = {
	"           ----------- Rx ------------- ----------- Tx ------------",
	" Device    Char Count  Throughput(BPS)  Char Count  Throughput(BPS)",
	"  name       (bytes)  Interval  Average   (bytes)  Interval  Average"
};

static  char    GTtyName[(MXSER_PORTS_PER_BOARD * MXSER_BOARDS)][30];

static  char    GPcieTtyName[(MXSER_PORTS_PER_BOARD * MXSER_BOARDS)][30];

struct mxser_usr_hwconf	Gmxsercfg[MXSER_BOARDS];

struct mxupcie_usr_hwconf	Gmxupciecfg[MXSER_BOARDS * 2];

static	int	card_type[MXSER_BOARDS], total_card;

static	int	maxrow = 0;
static	struct	Moniter_str	slog_old, slog_s, slog_e;

static	unsigned long	interval;
static	time_t	t1, t2, t3;
static	int	port_enable;

void read_pcietty(int now_major);
void read_tty(int now_major);
static int now_board = 0;
static int Gpcibrd_cnt = 0;
/*****************************************************************************/
/*	FUNCTIONS							     */
/*****************************************************************************/
int	mon_p_setup(int boards)
{
	int	i, c, exit_flag=0, flag;
	int	need_prepare = 1, old_space_ndx;
	int	key;
	
	interval = 3;
	t1 = time((time_t *)&t3);
	if ( (old_space_ndx = init_menu(now_board)) < 0 ) {
	    confirm("No monitor message is available");
	    return (-1);
	}
	mw_edit3init(&menu3);
	for ( i=0; i<3; i++ ) {
	    mw_wgotoxy(menu3.wid, 1, i + 2);
	    mw_wputsc(menu3.wid, (uchar *)subt_des[i], strlen(subt_des[i]),
		      menu3.wid->norm_c);
	}
	while ( exit_flag == 0 ) {
	    if ( need_prepare ) {
		flush_menu_data(now_board);
		edit3_flush_data(&menu3);
		show_timer(now_board);
		need_prepare = 0;
	    }
	    switch ( mw_edit3() ) {
	    case K_ESC:
		exit_flag = 1;
		break;
	    case K_ENTER:
		c = menu3.item[menu3.act_item].act_item;
		if ( c == 0 )
		    flag = MON_TOP;
		else if ( c == (maxrow - 1) )
		    flag = MON_BOTTOM;
		else
		    flag = MON_NORMAL;
		while ( (i = mon_pa_setup(menu3.item[0].str[c],
                        interval, flag, now_board, Gpcibrd_cnt, c)) != 0 ) {
		    if ( i < 0 && c > 0 )
			c--;
		    if ( i > 0 && c < maxrow - 1 )
			c++;
		    if ( c == 0 )
			flag = MON_TOP;
		    else if ( c == (maxrow - 1) )
			flag = MON_BOTTOM;
		    else
			flag = MON_NORMAL;
		}
		menu3.item[menu3.act_item].act_item = c;
		need_prepare = 1;
		break;
            case K_PGUP:
                if(now_board!=0){
                    now_board--;
		    reset_count();
                    need_prepare = 1;
	            release_space(old_space_ndx);
	            mw_edit3end();
	            old_space_ndx = init_menu(now_board);
	            mw_edit3init(&menu3);
	            for ( i=0; i<3; i++ ) {
	                mw_wgotoxy(menu3.wid, 1, i + 2);
	                mw_wputsc(menu3.wid, (uchar *)subt_des[i], strlen(subt_des[i]),
		        menu3.wid->norm_c);
                    }
                }
                break;
                
            case K_PGDOWN:
                if(now_board < boards-1){
                    now_board++;
		    reset_count();
                    need_prepare = 1;
	            release_space(old_space_ndx);
	            mw_edit3end();
	            old_space_ndx = init_menu(now_board);
	            mw_edit3init(&menu3);
	            for ( i=0; i<3; i++ ) {
	                mw_wgotoxy(menu3.wid, 1, i + 2);
	                mw_wputsc(menu3.wid, (uchar *)subt_des[i], strlen(subt_des[i]),
		        menu3.wid->norm_c);
                    }
                }
                break;
            
	    case '+':
		if ( interval < Max_Val )
		    interval++;
		show_timer(now_board);
		break;
	    case '-':
		if ( interval > Min_Val )
		    interval--;
		show_timer(now_board);
		break;
	    case K_HOME :
		reset_count();
		need_prepare = 1;
		break;
	    default:
		if ( time((time_t *)&t3) - t2 < interval )
		    break;
		flush_menu_data(now_board);
		edit3_flush_data(&menu3);
		show_timer(now_board);
		break;
	    }
	}
	release_space(old_space_ndx);
	mw_edit3end();
	return(0);
}

static int init_menu(int now_board)
{
	int	i, j, k, old_space_ndx, bits;
        int     ports;
	char	pcie_tty[30];
	if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) { 
		ports = mxupcie_numports[Gmxupciecfg[now_board].board_type - 1];
	} else {	
        	ports = mxser_numports[Gmxsercfg[now_board].board_type-1];
	}
	maxrow = ports;
	if ( getdatacnt(&slog_old) < 0 )
	    return(-1);
	slog_s = slog_old;
	old_space_ndx = get_space_ndx();
	j = sizeof(struct GItem3) * Total_Item;
	menu3.item = (struct GItem3 *)get_space(j);
	menu3.x = 5;
	menu3.y = 5;
	for ( i=0; i<Total_Item; i++ ) {
	    menu3.item[i].flag = itemflag[i];
	    menu3.item[i].des = itemdes[i];
	    menu3.item[i].act_item = 0;
	    menu3.item[i].total = maxrow;
	    /* jose: to fit for each column space */
	    for ( j=0; j<maxrow; j++ ) {
		menu3.item[i].len[j] = row_space[i];
		menu3.item[i].str[j] = get_space(row_space[i] + 1);
		for ( k=0; k<row_space[i]; k++ )
		    menu3.item[i].str[j][k] = ' ';
		menu3.item[i].str[j][k] = 0;
	    }
	}
	menu3.act_item = 1;
        for (j=0; j<maxrow; j++){
		if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) {
	    		sprintf(menu3.item[0].str[j], "%s",					      	GPcieTtyName[(now_board - Gpcibrd_cnt )* 8 + j]);
		} else	
	    		sprintf(menu3.item[0].str[j], "%s",							GTtyName[now_board * 8 + j]);
		
        }

	return(old_space_ndx);
}

static void flush_menu_data(int now_board)
{
	int	i, j, b;
        int     ports,id;
	if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) 
		ports = mxupcie_numports[Gmxupciecfg[now_board].board_type - 1];
        else
		ports = mxser_numports[Gmxsercfg[now_board].board_type-1];
	getdatacnt(&slog_e);
	t2 = time((time_t *)&t3);
	for ( i=0, j=0; i<ports, j<maxrow; i++ ) {
	    b = t2 - t1;
	    if ( b == 0 )
		b = 1;
	    if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) 
            	id = (now_board - Gpcibrd_cnt)* MXSER_PORTS_PER_BOARD + i;
	    else 
            	id = (now_board)* MXSER_PORTS_PER_BOARD + i;
	    sprintf(menu3.item[1].str[j], "%10lu",
		    slog_e.rxcnt[id] - slog_old.rxcnt[id]);
	    sprintf(menu3.item[2].str[j], "%8lu",
		    (slog_e.rxcnt[id] - slog_s.rxcnt[id]) / interval);
	    sprintf(menu3.item[3].str[j], "%8lu",
		    (slog_e.rxcnt[id] - slog_old.rxcnt[id]) / b);
	    sprintf(menu3.item[4].str[j], "%10lu",
		    slog_e.txcnt[id] - slog_old.txcnt[id]);
	    sprintf(menu3.item[5].str[j], "%8lu",
		    (slog_e.txcnt[id] - slog_s.txcnt[id]) / interval);
	    sprintf(menu3.item[6].str[j], "%8lu",
		    (slog_e.txcnt[id] - slog_old.txcnt[id]) / b);
	    j++;
	}
	slog_s = slog_e;
}

static int getdatacnt(logdata)
struct	Moniter_str	*logdata;
{
	int ret, fd, pcie_fd, i;

	if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) {
	pcie_fd = open("/dev/mxupcie", O_RDWR);
	if (pcie_fd < 0)
	    return(-1);
	ret = ioctl(pcie_fd, MOXA_GETDATACOUNT, logdata);
	if (ret != 0) {
	    printf("pcie ioctl fail,ret=%d \n",ret);
	    return 1;
	}

	close(pcie_fd);
	
	} else {
	fd = open("/dev/mxser", O_RDWR);
	if ( fd < 0 )
	    return(-1);
	ioctl(fd, MOXA_GETDATACOUNT, logdata);
	close(fd);
	}
	return(0);
}

static void show_timer(int now_board)
{
	char	tmp[80], str[60];
	time_t	i;
	int	hr, min, sec;
        int     w;

	i = time((time_t *)&t3) - t1;
	sec = i % 60;
	min = i / 60;
	hr = min / 60;
	min = min % 60;
	mw_wcolor(menu3.wid, D_A_Normal);
	
	if (Gmxupciecfg[now_board].IsMoxaMustChipFlag == 3) {
	             sprintf(str, "     Board #%d :%s",
                     now_board+1,
                     mxupcie_brdname[Gmxupciecfg[now_board].board_type-1]);
	} else {
        sprintf(str, "     Board #%d :%s",
                     now_board+1,
                     mxser_brdname[Gmxsercfg[now_board].board_type-1]);
	}
	w = menu3.wid->w - 2;
	for ( i=0; i<w; i++ )
		tmp[i] = ' ';
	tmp[i] = 0;
	mw_wcputs(menu3.wid, 1, (uchar *)tmp);
	sprintf(tmp, "Time[%02d:%02d:%02d]%-32s Interval %lu sec <+/->", 
		hr, min, sec, str, interval); 
	mw_wcputs(menu3.wid, 1, (uchar *)tmp);
}

static void reset_count()
{
	getdatacnt(&slog_old);
	slog_s = slog_old;
	t1 = time((time_t *)&t3);
}

static void mx_make_mxser_node()
{
	int ret = 0;
	int major = 20;
	char cmd[1024];
	memset(cmd, 0, sizeof(cmd));
	ret = system("rm -rf /dev/mxser");
	major = mx_get_mxser_major();
	if(ret != -1) {
		sprintf(cmd, "mknod /dev/mxser c %d 32", major);
		ret = system(cmd);
		if(ret == -1)
			return;
	}
}

static void mx_make_mxupcie_node()
{
	int ret = 0;
	int major = 31;
	char cmd[1024];
	memset(cmd, 0, sizeof(cmd));
	ret = system("rm -rf /dev/mxupcie");
	major = mx_get_mxupcie_major();
	if(ret != -1) {
		sprintf(cmd, "mknod /dev/mxupcie c %d 32", major);
		ret = system(cmd);
		if(ret == -1)
			return;
	}
}

static int mx_get_mxser_major()
{
	FILE *fstream = NULL;
	char buff[1024];
	int major = 30;
	memset(buff, 0, sizeof(buff));

	if(NULL == (fstream = popen("cat /proc/devices | grep -w ttyM | cut -c1-3 | cut -f2 -d' '","r")))
        {
                //fprintf(stderr, "execute command failed: %s", strerror(errno));

		//get major number failed, return default major number.
		return major;
        }

        while(NULL != fgets(buff, sizeof(buff), fstream))
        {
                //printf("%s", buff);
                if(atoi(buff) >= 0){
                        major = atoi(buff);
                }
        }

        pclose(fstream);
        return major;
}

static int mx_get_mxupcie_major()
{
	FILE *fstream = NULL;
	char buff[1024];
	int major = 31;
	memset(buff, 0, sizeof(buff));

	if(NULL == (fstream = popen("cat /proc/deivces | grep -w ttyMUE | cut -c1-3 | cut -f2 -d' '","r")))
        {
                //fprintf(stderr, "execute command failed: %s", strerror(errno));

		//get major number failed, return default major number.
                return major;
        }

        while(NULL != fgets(buff, sizeof(buff), fstream))
        {
                //printf("%s", buff);
                if(atoi(buff) >= 0){
                        major = atoi(buff);
                }
        }

        pclose(fstream);
        return major;
}

void read_pcietty(int now_major)
{

        FILE    *fd;
        char    ttystr[40];
        char    s1[20],s2[20],s3[20],s4[20],s7[20],s8[20],s9[20];
        char    tmp[100];
        int     major, minor;
        int     i,ch;
	int	ret;

	for(i=0; i<(MXSER_PORTS_PER_BOARD * MXSER_BOARDS); i++){
		sprintf( GPcieTtyName[i], "Port %d", i);
	}

	//use grep will take a long time on redhat 9 (30 is too much)
        //sprintf(tmp,"ls -al /dev | grep %d > /tmp/mon.tmp 2> /dev/null", now_major);
        sprintf(tmp,"ls --time-style=iso -l /dev > /tmp/mon.tmp 2> /dev/null");
                   
        ret = system(tmp);

        fd = fopen("/tmp/mon.tmp", "r");
	if(fd<0) {
		return;
	}

        do{
            i = 0;
            do{
                ch = fgetc(fd);
                if((ch==-1) || (ch=='\n')){
                    tmp[i] = 0;
                    break;
                }else{
                    tmp[i++] = ch;
                }
            }while(1);
            if(ch==-1)
                break;
            sscanf(tmp,"%s %s %s %s %d, %d %s %s %s %s",
                      s1,s2,s3,s4,&major,&minor,s7,s8,s9,ttystr);
	  #if 0
		printf("%s\n",s1);
		printf("%s\n",s2);
		printf("%s\n",s3);
		printf("%s\n",s4);
		printf("%d\n",&major);
		printf("%d\n",&minor);
		printf("%s\n",s7);
		printf("%s\n",s8);
		printf("%s\n",s9);
		
		printf("%s %d\n",ttystr,strlen(ttystr));
	 	getchar();
	  #endif
	    if(s1[0] != 'c')
		continue;
            if(major!=now_major)
                continue;
            if(minor>=(MXSER_PORTS_PER_BOARD * MXSER_BOARDS))
                continue;
	    
	    strcpy(GPcieTtyName[minor], s9);
#if 0
	    if(strlen(ttystr) >= 10) {
                ttystr[9] = 0;
	    }	    
	    strcpy(GPcieTtyName[minor], ttystr);
	    
	    if(strlen(ttystr) <= 0 ) {
	        strcpy(GPcieTtyName[minor], s9);
	    }
	    if (strlen(ttystr) == 5) {
		strcpy(GPcieTtyName[minor], s9);
	    }
#endif
        }while(1);
        fclose(fd);
        ret = system("rm -f /tmp/mon.tmp 2> /dev/null");
}

void read_tty(int now_major)
{
        FILE    *fd;
        char    ttystr[40];
        char    s1[20],s2[20],s3[20],s4[20],s7[20],s8[20],s9[20];
        char    tmp[100];
        int     major, minor;
        int     i,ch;
	int	ret;
	
	for(i=0; i<(MXSER_PORTS_PER_BOARD * MXSER_BOARDS); i++){
		sprintf( GTtyName[i], "Port %d", i);
	}

	//use grep will take a long time on redhat 9 (30 is too much)
        //sprintf(tmp,"ls -al /dev | grep %d > /tmp/mon.tmp 2> /dev/null", now_major);
        sprintf(tmp,"ls --time-style=iso -l /dev > /tmp/mon.tmp 2> /dev/null");
        ret = system(tmp);

        fd = fopen("/tmp/mon.tmp", "r");
	if(fd<0) {
		return;
	}

        do{
            i = 0;
            do{
                ch = fgetc(fd);
                if((ch==-1) || (ch=='\n')){
                    tmp[i] = 0;
                    break;
                }else{
                    tmp[i++] = ch;
                }
            }while(1);
            if(ch==-1)
                break;
            sscanf(tmp,"%s %s %s %s %d, %d %s %s %s %s",
                      s1,s2,s3,s4,&major,&minor,s7,s8,s9,ttystr);
	    if(s1[0] != 'c')
		continue;
            if(major!=now_major)
                continue;
            if(minor>=(MXSER_PORTS_PER_BOARD * MXSER_BOARDS))
                continue;

	    strcpy(GTtyName[minor], s9);
#if 0
            if(strlen(ttystr) >= 10)
                ttystr[9] = 0;

            strcpy(GTtyName[minor], ttystr);

	    if(strlen(ttystr) <= 0)
	        strcpy(GTtyName[minor], s9);

	    if (strlen(ttystr) == 5) {
		
	        strcpy(GTtyName[minor], s9);
	    }
#endif
        }while(1);
        fclose(fd);
        ret = system("rm -f /tmp/mon.tmp 2> /dev/null");
        
}


void main_end()
{
	mw_wend();
}

int main()
{
	//int	brd_cnt;
	int	fd, pcie_fd; 
	int	ret, pci_ret, pcie_ret;
        int     i;
        int     boards;
        int     major, pcie_major;
	
	pci_ret = 0 ;
	boards = 0;
	Gpcibrd_cnt = 0;
	pcie_ret = 0;
	if ((fd = open("/dev/mxser", O_RDWR)) < 0) {
 		if (errno == ENOENT) {
			mx_make_mxser_node();
			if ((fd = open("/dev/mxser", O_RDWR)) < 0) {
				printf("open /dev/mxser fail\n");
			    	printf("Please run msmknod first.\n");
				pci_ret = -1;
				goto nopci;
			}
		} else if(errno == ENXIO) {
			printf("Please load `mxser' driver first.\n");
			pci_ret = -1;
			goto nopci;
		} else {
			pci_ret = -1;
			goto nopci;
		}
	}
	ret = ioctl(fd, MOXA_GET_CONF , Gmxsercfg);
	if(ret != 0){
	    printf("pci ioctl fail,ret=%d\n",ret);
	    return 1;
	}
	ioctl(fd, MOXA_GET_MAJOR, &major);
	close(fd);
	read_tty(major);
	for (i = 0; i < MXSER_BOARDS; i++) {
		if (Gmxsercfg[i].board_type != -1) {
                	boards++;
			Gpcibrd_cnt++;
		}
	}
nopci:
	if ((pcie_fd = open("/dev/mxupcie", O_RDWR)) < 0) {
		if (errno == ENOENT) {
		mx_make_mxupcie_node();
		if ((pcie_fd = open("/dev/mxupcie", O_RDWR)) < 0) {
		    	printf("open /dev/mxupcie fail\n");
	    		printf("Please run msmknod first.\n");
	    		pcie_ret = -1;
	    		goto nopcie;
		}
		} else if(errno == ENXIO) {
			printf("Please load `mxupcie' driver first.\n");
			pcie_ret = -1;
	    		goto nopcie;
		} else {
		    pcie_ret = -1;
		    goto nopcie;
		}
	}
	
	ret = ioctl(pcie_fd, MOXA_GET_CONF , &Gmxupciecfg[Gpcibrd_cnt]);
	if (ret != 0) {
	    printf("pcie ioctl fail,ret=%d \n",ret);
	    return 1;
	}
	ioctl(pcie_fd, MOXA_GET_MAJOR, &pcie_major);
	close(pcie_fd);
	read_pcietty(pcie_major);
	for (i = Gpcibrd_cnt; i < MXSER_BOARDS; i++) {
		if (Gmxupciecfg[i].board_type != -1)
			boards++;
	}

nopcie:	
	if (!boards) {
	    printf("No any MOXA Smartio/Industio Family board installed.\n");
	    return 1;
	}

	if (pci_ret == -1 && pcie_ret == -1)
		return 1;
	space_init();	

	mw_winit();
        
	mon_p_setup(boards);
	
	mw_wend();
        return 0;
}
