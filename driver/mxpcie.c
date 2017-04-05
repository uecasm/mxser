/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*	
	mxpcie.c

	MOXA Smartio/Industio family multiport serial driver.
	2009-09-10	Joy Tu	
			1. Fix a bug that CP-104EL-A can be set to
			   RS-422.

	2009-07-24	Joy Tu
			1. Add four models ,CP-104EL-A, CP-168EL-A, 
			   CP-118-EL-A, CP-118E-A-I.
			2. Support kernel up to 2.6.29.
						
*/

#ifdef 		MODVERSIONS
#ifndef 	MODULE
#define 	MODULE
#endif
#endif

#include <linux/version.h>
#define	VERSION_CODE(ver,rel,seq)	((ver << 16) | (rel << 8) | seq)

#ifdef MODULE
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,18))
#include <linux/config.h>
#endif
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#include <linux/module.h>
#else
#define	MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,32))
#include <linux/autoconf.h>
#endif
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,39))
#include <linux/smp_lock.h>
#endif
#include <linux/delay.h>

#if (LINUX_VERSION_CODE < VERSION_CODE(3,4,0))
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/segment.h>
#include <asm/bitops.h>


#include "mxpcie.h"
#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif

#if (LINUX_VERSION_CODE < VERSION_CODE(2,1,0))

#define copy_from_user memcpy_fromfs
#define copy_to_user memcpy_tofs
#define put_to_user(arg1, arg2) put_fs_long(arg1, arg2)
#define get_from_user(arg1, arg2) arg1 = get_fs_long(arg2)
#define schedule_timeout(x) {current->timeout = jiffies + (x); schedule();}
#define signal_pending(x) ((x)->signal & ~(x)->blocked)
#else
#include <asm/uaccess.h>
#define put_to_user(arg1, arg2) put_user(arg1, arg2)
#define get_from_user(arg1, arg2) get_user(arg1, arg2)
#endif

#define	MXUPCIE_VERSION	"1.16"
#define	MXUPCIEMAJOR	 31
#define	MXUPCIECUMAJOR	 34

#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif /* ENABLE_PCI */
#include "./CPLD/Source/cpld.h"
#include "./CPLD/Source/platform.h"
/* The definition of linear baud rate */
#define FREQUENCY       14745600
#define MAXDIVISOR      255
#define MAXSEQUENCE     46
#define MINSEQUENCE     4
#define MAX_SCR		12
#define MIN_SCR		0
#define MAX_CPRN	7
#define MIN_CPRN	0
#define MIN_CPRM	1
#define MAX_CPRM	2

enum	{
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
	MXUPCIE_BOARD_CP116E_A_B  /* Support New Ioctl AutoMode etc, It the second part of 16 port */
};

static char *mxupcie_brdname[] = {
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
	"CP-116E-A series (B)"
};

static int mxupcie_numports[] = {
	2,	//CP-102E
	2,	//CP-102EL
	2,	//CP-132EL
	4,	//CP-114EL
	4,	//CP-104EL
	8,	//CP-168EL
	8,	//CP-118EL
	8,	//CP-118E-A-I
	8,	//CP-138E-A
	4,	//CP-134EL-A
	8,	//CP-116EA (A)
	8	//CP-116EA (B)
};


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
typedef struct {
	unsigned short	vendor;
	unsigned short	device;
	unsigned short	driver_data;
}	mxupcie_pciinfo;

static mxupcie_pciinfo	mxupcie_pcibrds[] = {
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102E ,MXUPCIE_BOARD_CP102E},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102EL ,MXUPCIE_BOARD_CP102EL},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP132EL ,MXUPCIE_BOARD_CP132EL},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP114EL ,MXUPCIE_BOARD_CP114EL},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP104EL_A ,MXUPCIE_BOARD_CP104EL_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP168EL_A ,MXUPCIE_BOARD_CP168EL_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118EL_A ,MXUPCIE_BOARD_CP118EL_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118E_A_I ,MXUPCIE_BOARD_CP118E_A_I},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP138E_A ,MXUPCIE_BOARD_CP138E_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP134EL_A ,MXUPCIE_BOARD_CP134EL_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP116E_A_A ,MXUPCIE_BOARD_CP116E_A_A},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP116E_A_B ,MXUPCIE_BOARD_CP116E_A_B},
	{0}
};
 
#else

static  struct pci_device_id	mxupcie_pcibrds[] = {
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102E ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP102E},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP102EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP132EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP132EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP114EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP114EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP104EL_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP104EL_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP168EL_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP168EL_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118EL_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP118EL_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118E_A_I ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP118E_A_I},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP138E_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP138E_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP134EL_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP134EL_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP116E_A_A ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP116E_A_A},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP116E_A_B ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXUPCIE_BOARD_CP116E_A_B},
	{0}
};

MODULE_DEVICE_TABLE(pci, mxupcie_pcibrds);

#endif

typedef struct _moxa_pci_info {
	unsigned short busNum;
	unsigned short devNum;
struct pci_dev	*pdev;	
} moxa_pci_info;

static int ttymajor=MXUPCIEMAJOR;
static int calloutmajor=MXUPCIECUMAJOR;
static unsigned char interface=0;
static unsigned char terminator=0;


#ifdef MODULE
/* Variables for insmod */

MODULE_AUTHOR("Eric Lo");
MODULE_DESCRIPTION("MOXA Smartio/Industio Family Multiport Board Device Driver");
MODULE_LICENSE("GPL");

#if defined SP2 || (LINUX_VERSION_CODE > VERSION_CODE(2,6,9))
int mx_ioaddr_array_num;
module_param(ttymajor, int, 0);
module_param(calloutmajor, int, 0);
module_param(interface, byte, 0);
module_param(terminator, byte, 0);
#else
MODULE_PARM(ttymajor,        "i");
MODULE_PARM(calloutmajor,    "i");
MODULE_PARM(interface, 	"b");
MODULE_PARM(terminator,	"b");
#endif

#endif /* MODULE */


struct mxupcie_log {
	int	tick;
	unsigned long	rxcnt[MXUPCIE_PORTS];
	unsigned long	txcnt[MXUPCIE_PORTS];
};


struct mxupcie_mon{
        unsigned long   rxcnt;
        unsigned long   txcnt;
	unsigned long	up_rxcnt;
	unsigned long	up_txcnt;
        int             modem_status;
        unsigned char   hold_reason;
};


struct mxupcie_hwconf {
	int		board_type;
	int		ports;
	int		irq;
	unsigned long	iobar3_addr;
	unsigned long	vector_mask;
	int		uart_type;
	unsigned char	*ioaddr[MXUPCIE_PORTS_PER_BOARD];
	int		baud_base[MXUPCIE_PORTS_PER_BOARD];
	moxa_pci_info	pciInfo;
	int		IsMoxaMustChipFlag;	// add by Victor Yu. 08-30-2002
	int		MaxCanSetBaudRate[MXUPCIE_PORTS_PER_BOARD];	// add by Victor Yu. 09-04-2002
	unsigned long	*opmode_ioaddr[MXUPCIE_PORTS_PER_BOARD];	// add by Victor Yu. 01-05-2004
};


struct mxupcie_struct {
	int			port;
#if (LINUX_VERSION_CODE >= VERSION_CODE(3,7,0))
	struct tty_port ttyPort;
#endif	
	unsigned char		*base;		/* port base address */
	int			irq;		/* port using irq no. */
	int			baud_base;	/* max. speed */
	int			flags;		/* defined in tty.h */
	int			type;		/* UART type */
	struct tty_struct *	tty;
	int			read_status_mask;
	int			ignore_status_mask;
	int			rx_high_water;
	int			rx_trigger;	/* Rx fifo trigger level */
	int			rx_low_water;
	int			xmit_fifo_size;
	int			custom_divisor;
	int			close_delay;
	unsigned short		closing_wait;
	int			IER;		/* Interrupt Enable Register */
	int			MCR;		/* Modem control register */
	unsigned long		event;
	int			count;		/* # of fd on device */
	int			blocked_open;	/* # of blocked opens */
	unsigned char		*xmit_buf;
	int			xmit_head;
	int			xmit_tail;
	int			xmit_cnt;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))	
	struct tq_struct	tqueue;
#else
	struct work_struct tqueue;
#endif	
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
	struct ktermios		normal_termios;
	struct ktermios		callout_termios;
#else
	struct termios		normal_termios;
	struct termios		callout_termios;
#endif
	wait_queue_head_t open_wait;
	wait_queue_head_t close_wait;
	wait_queue_head_t delta_msr_wait;
	struct async_icount	icount; 	/* kernel counters for the 4 input interrupts */
	int			timeout;
	int			MaxCanSetBaudRate;	// add by Victor Yu. 09-04-2002
	long    realbaud;
	struct mxupcie_mon        mon_data;
	unsigned char           err_shadow;
	spinlock_t		slock;
	int			speed;
	int			custom_baud_rate;
	unsigned long		iobar3_addr;
	unsigned char		UIR;
	unsigned long		UIR_addr;
	unsigned char		terminator_flag;
	int			board_type;
};


struct mxupcie_mstatus{
       tcflag_t	cflag;
       int  	cts;
       int  	dsr;
       int  	ri;
       int  	dcd;
};


static  struct mxupcie_mstatus GMStatus[MXUPCIE_PORTS];

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
static struct tty_driver	mxvar_sdriver;
static struct tty_driver	mxvar_cdriver;
#else
static struct tty_driver	*mxvar_sdriver;
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
static int			mxvar_refcount;
#endif
static struct mxupcie_struct	mxvar_table[MXUPCIE_PORTS];
static struct tty_struct *	mxvar_tty[MXUPCIE_PORTS+1];
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static struct ktermios * 	mxvar_termios[MXUPCIE_PORTS+1];
#if (LINUX_VERSION_CODE < VERSION_CODE(3,3,0))
static struct ktermios * 	mxvar_termios_locked[MXUPCIE_PORTS+1];
#endif
#else
static struct termios * 	mxvar_termios[MXUPCIE_PORTS+1];
static struct termios * 	mxvar_termios_locked[MXUPCIE_PORTS+1];
#endif
static struct mxupcie_log 	mxvar_log;
static int			mxvar_diagflag;
static unsigned char mxupcie_msr[MXUPCIE_PORTS+1];
static int mxupcie_set_baud_method[MXUPCIE_PORTS+1];
static spinlock_t		gm_lock;

/*
 * This is used to figure out the divisor speeds and the timeouts
 */
static int mxvar_baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 0 };
#define BAUD_TABLE_NO	(sizeof(mxvar_baud_table)/sizeof(int))

struct mxupcie_hwconf mxupciecfg[MXUPCIE_BOARDS];

/*
 * static functions:
 */

#ifdef MODULE
int		init_module(void);
void		cleanup_module(void);
#endif

static int	mxupcie_open(struct tty_struct *, struct file *);
static void	mxupcie_close(struct tty_struct *, struct file *);

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
static int	mxupcie_write(struct tty_struct *, int from_user, const unsigned char *, int);
#else
static int	mxupcie_write(struct tty_struct *, const unsigned char *, int);
#endif
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,25))
static int	mxupcie_put_char(struct tty_struct *, unsigned char);
#else
static void	mxupcie_put_char(struct tty_struct *, unsigned char);
#endif
static void	mxupcie_flush_chars(struct tty_struct *);
static int	mxupcie_write_room(struct tty_struct *);
static int	mxupcie_chars_in_buffer(struct tty_struct *);
static void	mxupcie_flush_buffer(struct tty_struct *);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,39))
static int	mxupcie_ioctl(struct tty_struct *, struct file *, uint, ulong);
#else
static int	mxupcie_ioctl(struct tty_struct *, uint, ulong);
#endif
static void	mxupcie_throttle(struct tty_struct *);
static void	mxupcie_unthrottle(struct tty_struct *);
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static void	mxupcie_set_termios(struct tty_struct *, struct ktermios *);
#else
static void	mxupcie_set_termios(struct tty_struct *, struct termios *);
#endif
static void	mxupcie_stop(struct tty_struct *);
static void	mxupcie_start(struct tty_struct *);
static void	mxupcie_hangup(struct tty_struct *);
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,39))
static int	mxupcie_tiocmget(struct tty_struct *);
static int	mxupcie_tiocmset(struct tty_struct *, unsigned int, unsigned int);
#elif (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
static int	mxupcie_tiocmget(struct tty_struct *, struct file *);
static int	mxupcie_tiocmset(struct tty_struct *, struct file *, unsigned int, unsigned int);
#else
static int	mx_get_modem_info(struct mxupcie_struct *, unsigned int *);
static int	mx_set_modem_info(struct mxupcie_struct *, unsigned int, unsigned int *);
#endif
#if (LINUX_VERSION_CODE >= 131394 && LINUX_VERSION_CODE < VERSION_CODE(2,6,26))
static void	mxupcie_rs_break(struct tty_struct *, int);
static void 	mxupcie_wait_until_sent(struct tty_struct *tty, int timeout);
#else
static int	mxupcie_rs_break(struct tty_struct *, int);
static void 	mxupcie_wait_until_sent(struct tty_struct *tty, int timeout);
#endif
static void 	mx_getcfg(int board,struct mxupcie_hwconf *hwconf);
int		mx_init(void);
#ifdef CONFIG_PCI
static int      mx_get_PCI_conf(int ,int ,int ,struct mxupcie_hwconf *);
#endif
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static void     mx_do_softint(struct work_struct *work);
#else
static void	mx_do_softint(void *);
#endif
static int	mx_ioctl_special(unsigned int, unsigned long);
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,18))
static irqreturn_t mx_interrupt(int irq, void *dev_id);
#else
static IRQ_RET mx_interrupt(int, void *, struct pt_regs *);
#endif
static void	mx_receive_chars(struct mxupcie_struct *, int *);
static void	mx_transmit_chars(struct mxupcie_struct *);
static void	mx_check_modem_status(struct mxupcie_struct *, int);
static int	mx_block_til_ready(struct tty_struct *, struct file *, struct mxupcie_struct *);
static int	mx_startup(struct mxupcie_struct *);
static void	mx_shutdown(struct mxupcie_struct *);
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static int	mx_change_speed(struct mxupcie_struct *, struct ktermios *old_termios);
#else
static int	mx_change_speed(struct mxupcie_struct *, struct termios *old_termios);
#endif
static int	mx_get_serial_info(struct mxupcie_struct *, struct serial_struct *);
static int	mx_set_serial_info(struct mxupcie_struct *, struct serial_struct *);
static int	mx_get_lsr_info(struct mxupcie_struct *, unsigned int *);
static void	mx_send_break(struct mxupcie_struct *, int);
static int	mx_set_baud(struct mxupcie_struct *info, long newspd);
static void	mx_startrx(struct tty_struct * tty);
static void	mx_stoprx(struct tty_struct * tty);

static int	mx_set_interface(struct mxupcie_struct *info, unsigned char val);
static int	mx_set_terminator(struct mxupcie_struct *info, unsigned char val);
static void	mx_init_terminator(struct mxupcie_struct *info);
static void	mx_software_break_signal(struct mxupcie_struct *info, unsigned char state);
static void 	write_div_scr(unsigned char *base, signed short div, unsigned char scr);
static int 	set_linear_baud(unsigned char *base, long newspd);
static void mx_process_txrx_fifo(struct mxupcie_struct *info);

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
static struct tty_operations mxupcie_ops = {
	.open = mxupcie_open,
	.close = mxupcie_close,
	.write = mxupcie_write,
	.put_char = mxupcie_put_char,
	.flush_chars = mxupcie_flush_chars,
	.write_room = mxupcie_write_room,
	.chars_in_buffer = mxupcie_chars_in_buffer,
	.flush_buffer = mxupcie_flush_buffer,
	.ioctl = mxupcie_ioctl,
	.throttle = mxupcie_throttle,
	.unthrottle = mxupcie_unthrottle,
	.set_termios = mxupcie_set_termios,
	.stop = mxupcie_stop,
	.start = mxupcie_start,
	.hangup = mxupcie_hangup,
	.tiocmget = mxupcie_tiocmget,
	.tiocmset = mxupcie_tiocmset,
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,26))
	.break_ctl = mxupcie_rs_break,
	.wait_until_sent = mxupcie_wait_until_sent,
#endif
};
#endif


/*
 * The MOXA Smartio/Industio serial driver boot-time initialization code!
 */
INIT_FUNC_RET	INIT_FUNC(void)
{
	int ret;

	ret = mx_init();

	return ret;
}

CLEAR_FUNC_RET	CLEAR_FUNC(void)
{
	int i, err=0;

#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,27))
	struct ktermios *tp;
	void *p;
#endif

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
	if ((err |= tty_unregister_driver(&mxvar_cdriver)))
		printk("Couldn't unregister MOXA Smartio/Industio family callout driver\n");
#endif		
	if ((err |= tty_unregister_driver(DRV_VAR)))
		printk("Couldn't unregister MOXA Smartio/Industio family serial driver\n");
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,27))
	for (i = 0; i < DRV_VAR->num; i++) {
		tp = DRV_VAR->termios[i];
		if (tp) {
			DRV_VAR->termios[i] = NULL;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))		
			/* the ktermios of tty is not get through alloc after kernel 3.7, so we do not
			  *  have to free this */
			kfree(tp);
#endif
		}
#if (LINUX_VERSION_CODE < VERSION_CODE(3,3,0))
		tp = DRV_VAR->termios_locked[i];
		if (tp) {
			DRV_VAR->termios_locked[i] = NULL;
			kfree(tp);
		}
#endif
		if (!(DRV_VAR->flags & TTY_DRIVER_DYNAMIC_DEV))
			tty_unregister_device(DRV_VAR, i);
	}
	p = DRV_VAR->ttys;
	//proc_tty_unregister_driver(DRV_VAR);
	DRV_VAR->ttys = NULL;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,3,0))
	DRV_VAR->termios = DRV_VAR->termios_locked = NULL;
#else
	DRV_VAR->termios = NULL;
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))	
	kfree(p);
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	cdev_del(&DRV_VAR->cdev);
#endif
#endif
 
        for(i=0; i<MXUPCIE_BOARDS; i++){
		struct pci_dev *pdev;

		if(mxupciecfg[i].board_type == -1){
			continue;
		}
		else{
			pdev = mxupciecfg[i].pciInfo.pdev;
			free_irq(mxupciecfg[i].irq, &mxvar_table[i*MXUPCIE_PORTS_PER_BOARD]);

			if(pdev!=NULL){ //PCI
				iounmap(mxupciecfg[i].ioaddr[1]);
				release_mem_region(pci_resource_start(pdev, 1),	pci_resource_len(pdev, 1));
				release_region(pci_resource_start(pdev,2), pci_resource_len(pdev,2));
			}	
		}
        }
}


int mxupcie_initbrd(int board,struct mxupcie_hwconf *hwconf)
{
	struct mxupcie_struct *	info;
        int     retval;
	int	i,n;
        int temp_interface = 0;

	n = board*MXUPCIE_PORTS_PER_BOARD;
	info = &mxvar_table[n];
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
	printk("        ttyMUE%d/cumue%d - ttyMUE%d/cumue%d  ", 
			n, n, n+hwconf->ports-1, n+hwconf->ports-1);
#else
	printk("        ttyMUE%d - ttyMUE%d ", n, n+hwconf->ports-1);
#endif			
	printk(" max. baud rate = %d bps.\n", hwconf->MaxCanSetBaudRate[0]);
	
	for ( i=0; i<hwconf->ports; i++, n++, info++ ) {
		info->port = n;
		info->base = hwconf->ioaddr[i];
		info->irq = hwconf->irq;
		info->UIR = 0;
		info->terminator_flag = 0;
		info->iobar3_addr = hwconf->iobar3_addr;
		info->UIR_addr = hwconf->iobar3_addr + MOXA_UIR_OFFSET + (i / 2);
		info->board_type = hwconf->board_type;

		if(i == MX_PORT4){
			switch(hwconf->board_type){
				case MXUPCIE_BOARD_CP114EL:
				case MXUPCIE_BOARD_CP104EL_A:
				case MXUPCIE_BOARD_CP134EL_A:
					info->UIR_addr = hwconf->iobar3_addr + MOXA_UIR_OFFSET + 3;
				break;
			default:
				break;
			}
		}

		info->flags = ASYNC_SHARE_IRQ;
		info->type = hwconf->uart_type;
		info->baud_base = hwconf->baud_base[i];
		info->MaxCanSetBaudRate = hwconf->MaxCanSetBaudRate[i];
		info->xmit_fifo_size = MX_TX_FIFO_SIZE;
		info->rx_trigger= MOXA_RTL_96;
		info->rx_high_water = MOXA_FCH_110;
		info->rx_low_water = MOXA_FCL_16;
		info->custom_divisor = hwconf->baud_base[i] * 16;
		info->close_delay = 5*HZ/10;
		info->closing_wait = 30*HZ;
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
		INIT_WORK(&info->tqueue, mx_do_softint);
#else
		INIT_WORK(&info->tqueue, mx_do_softint, info);
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
		info->callout_termios = mxvar_cdriver.init_termios;
#endif		
		info->normal_termios = DRV_VAR_P(init_termios);
		init_waitqueue_head(&info->open_wait);
		init_waitqueue_head(&info->close_wait);
		init_waitqueue_head(&info->delta_msr_wait);
		info->speed = 9600;
                memset(&info->mon_data, 0, sizeof(struct mxupcie_mon));
		info->err_shadow = 0;
		spin_lock_init(&info->slock);

		MX_WRITE_IOBAR3_REG(MOXA_GPIO_SET_ALL_OUTPUT, info->iobar3_addr + MOXA_PUART_GPIO_EN);
		//mx_set_terminator(info, terminator);
		
		mx_init_terminator(info);
		
		if(!interface){
			switch(info->board_type){
				case MXUPCIE_BOARD_CP132EL:
				case MXUPCIE_BOARD_CP138E_A:
				case MXUPCIE_BOARD_CP134EL_A:                    
					temp_interface = MOXA_UIR_RS422;
					break;
				default:
					temp_interface = MOXA_UIR_RS232;
					break;
			}
		} else 
		    temp_interface = interface;

		mx_set_interface(info, temp_interface);
	}


	/*
	 * Allocate the IRQ if necessary
	 */
	

	/* before set INT ISR, disable all int */
	for(i=0; i<hwconf->ports; i++)
		MX_WRITE_REG(MX_READ_REG(hwconf->ioaddr[i] + UART_IER) & 0xf0, hwconf->ioaddr[i]+UART_IER);

	n = board*MXUPCIE_PORTS_PER_BOARD;
	info = &mxvar_table[n];
        retval = request_irq(hwconf->irq, mx_interrupt, IRQ_T(info), "mxupcie", info);

	if ( retval ) {
	    printk("Board %d: %s", board, mxupcie_brdname[hwconf->board_type-1]);
	    printk("  Request irq fail,IRQ (%d) may be conflit with another device.\n",info->irq);

	    return retval;
	}
	
        return 0;
}


static void mx_getcfg(int board,struct mxupcie_hwconf *hwconf)
{
	mxupciecfg[board] = *hwconf;
}


#ifdef CONFIG_PCI
static int mx_get_PCI_conf(int busnum,int devnum,int board_type,struct mxupcie_hwconf *hwconf)
{
	int		i;
	unsigned char	*ioaddress;
	struct pci_dev	*pdev=hwconf->pciInfo.pdev;
	unsigned long	iobar_addr;
	int		ret;
	gpio_param_t   cpld;

	//io address
	hwconf->board_type = board_type;
	hwconf->ports = mxupcie_numports[board_type-1];

	request_mem_region(pci_resource_start(pdev, 1),
	       pci_resource_len(pdev, 1),
	       "mxupcie(MEM)");

	ioaddress = ioremap(pci_resource_start(pdev,1),
			    pci_resource_len(pdev,1));

	iobar_addr = pci_resource_start(pdev, 2);
	request_region(pci_resource_start(pdev, 2),
	       pci_resource_len(pdev, 2),
	       "mxupcie(IOBAR3)");

	for (i = 0; i < hwconf->ports; i++) {
		hwconf->ioaddr[i] = ioaddress + (i * MX_PUART_SIZE);
		if(i == 3){
			switch(board_type){
				case MXUPCIE_BOARD_CP114EL:
				case MXUPCIE_BOARD_CP104EL_A:
				case MXUPCIE_BOARD_CP134EL_A:                    
					hwconf->ioaddr[i] = ioaddress + (MX_PORT8 * MX_PUART_SIZE);
				break;
			default:
				break;
			}
		}
	}

	//irq
	hwconf->irq = hwconf->pciInfo.pdev->irq;
	hwconf->iobar3_addr = iobar_addr;

	hwconf->uart_type = PORT_16550A;
	hwconf->IsMoxaMustChipFlag = MOXA_PUART_HWID;
	
        for (i = 0; i < hwconf->ports; i++) {
		hwconf->baud_base[i] = 921600;
		hwconf->MaxCanSetBaudRate[i] = 921600;
	}
	cpld.base = (unsigned char *)hwconf->iobar3_addr;
	ret = mxCPLDInit(cpld);
//	ret = mxCPLDInit(*(gpio_param_t *)hwconf->iobar3_addr);
	if (ret != 0) {
		printk(KERN_ALERT"[Init] CPLD Init Fail\r\n");	
	}

	return 0;
}
#endif

int mx_init(void)
{
	int			i, m, retval, b;
    int			ret1, ret2;
#if (LINUX_VERSION_CODE >= VERSION_CODE(3,7,0))	
	int 		port_idx;
#endif	
#ifdef CONFIG_PCI
	struct pci_dev	*pdev=NULL;
	int			index;
	unsigned char		busnum,devnum;
#endif
	struct mxupcie_hwconf	hwconf;
	uint8_t version, major, minor;
	gpio_param_t cpld;

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
	mxvar_sdriver = alloc_tty_driver(MXUPCIE_PORTS + 1);
	if (!mxvar_sdriver)
		return -ENOMEM;
#endif		
	spin_lock_init(&gm_lock);

	for(i=0; i<MXUPCIE_BOARDS; i++)
		mxupciecfg[i].board_type = -1;
	
	printk("MOXA Smartio/Industio family driver version %s\n",MXUPCIE_VERSION);

	/* Initialize the tty_driver structure */
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,28))
	memset(DRV_VAR, 0, sizeof(struct tty_driver));
#endif
	DRV_VAR_P(magic) = TTY_DRIVER_MAGIC;
	DRV_VAR_P(name) = "ttyMUE";
	DRV_VAR_P(major) = ttymajor;
	DRV_VAR_P(minor_start) = 0;
	DRV_VAR_P(num) = MXUPCIE_PORTS + 1;
	DRV_VAR_P(type) = TTY_DRIVER_TYPE_SERIAL;
	DRV_VAR_P(subtype) = SERIAL_TYPE_NORMAL;
	DRV_VAR_P(init_termios) = tty_std_termios;
	DRV_VAR_P(init_termios.c_cflag) = B9600|CS8|CREAD|HUPCL|CLOCAL;
	DRV_VAR_P(flags) = TTY_DRIVER_REAL_RAW;
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
	tty_set_operations(DRV_VAR, &mxupcie_ops);
	DRV_VAR_P(ttys) = mxvar_tty;
#else
	DRV_VAR_P(refcount) = &mxvar_refcount;
	DRV_VAR_P(table) = mxvar_tty;
#endif
	DRV_VAR_P(termios) = mxvar_termios;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,3,0))
	DRV_VAR_P(termios_locked) = mxvar_termios_locked;
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,26))
	DRV_VAR_P(open) = mxupcie_open;
	DRV_VAR_P(close) = mxupcie_close;
	DRV_VAR_P(write) = mxupcie_write;
	DRV_VAR_P(put_char) = mxupcie_put_char;
	DRV_VAR_P(flush_chars) = mxupcie_flush_chars;
	DRV_VAR_P(write_room) = mxupcie_write_room;
	DRV_VAR_P(chars_in_buffer) = mxupcie_chars_in_buffer;
	DRV_VAR_P(flush_buffer) = mxupcie_flush_buffer;
	DRV_VAR_P(ioctl) = mxupcie_ioctl;
	DRV_VAR_P(throttle) = mxupcie_throttle;
	DRV_VAR_P(unthrottle) = mxupcie_unthrottle;
	DRV_VAR_P(set_termios) = mxupcie_set_termios;
	DRV_VAR_P(stop) = mxupcie_stop;
	DRV_VAR_P(start) = mxupcie_start;
	DRV_VAR_P(hangup) = mxupcie_hangup;
#if (LINUX_VERSION_CODE >= 131394 && LINUX_VERSION_CODE < VERSION_CODE(2,6,26))
	DRV_VAR_P(break_ctl) = mxupcie_rs_break;
	DRV_VAR_P(wait_until_sent) = mxupcie_wait_until_sent;
#endif
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
	/*
	 * The callout device is just like normal device except for
	 * major number and the subtype code.
	 */
	mxvar_cdriver = mxvar_sdriver;
	mxvar_cdriver.name = "cumue";
	mxvar_cdriver.major = calloutmajor;
	mxvar_cdriver.subtype = SERIAL_TYPE_CALLOUT;

	printk("Tty devices major number = %d, callout devices major number = %d\n",ttymajor,calloutmajor);
#endif
	mxvar_diagflag = 0;
	memset(mxvar_table, 0, MXUPCIE_PORTS * sizeof(struct mxupcie_struct));
	memset(&mxvar_log, 0, sizeof(struct mxupcie_log));

	memset(&mxupcie_msr, 0, sizeof(unsigned char) * (MXUPCIE_PORTS+1));
	memset(&mxupcie_set_baud_method, 0, sizeof(int) * (MXUPCIE_PORTS+1));
	memset(&hwconf, 0, sizeof(struct mxupcie_hwconf));


	/* start finding PCI board here */
#ifdef CONFIG_PCI
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
	if (pci_present()) {
#else
	{		
#endif		
		int n = (sizeof(mxupcie_pcibrds) / sizeof(mxupcie_pcibrds[0])) - 1;
		index = 0;
		b = 0;
		m = 0;

		while (b < n) {
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,21))
		       pdev = pci_find_device(mxupcie_pcibrds[b].vendor,
		       		mxupcie_pcibrds[b].device, pdev);
#else
		       pdev = pci_get_device(mxupcie_pcibrds[b].vendor,
		       		mxupcie_pcibrds[b].device, pdev);
#endif
		       		
			if(pdev==NULL){
				b++;
				continue;
			}

			hwconf.pciInfo.busNum = busnum = pdev->bus->number;
			hwconf.pciInfo.devNum = devnum = PCI_SLOT(pdev->devfn)<<3;
			hwconf.pciInfo.pdev = pdev;
			printk("Found MOXA %s board(BusNo=%d,DevNo=%d)\n",mxupcie_brdname[(int)(mxupcie_pcibrds[b].driver_data)-1],busnum,devnum >> 3);
			index++;

			if ( m >= MXUPCIE_BOARDS) {
				printk("Too many Smartio/Industio family boards find (maximum %d),board not configured\n",MXUPCIE_BOARDS);
			}
			else {
				if ( pci_enable_device(pdev) ) {
					printk("Moxa SmartI/O PCI enable fail !\n");
					continue;
				}
				retval = mx_get_PCI_conf(busnum,devnum,
					(int)mxupcie_pcibrds[b].driver_data,&hwconf);
				if (retval < 0) {
					if (retval == MXUPCIE_ERR_IRQ)
						printk("Invalid interrupt number,board not configured\n");
					else if (retval == MXUPCIE_ERR_IRQ_CONFLIT)
						printk("Invalid interrupt number,board not configured\n");
					else if (retval == MXUPCIE_ERR_VECTOR)
						printk("Invalid interrupt vector,board not configured\n");
					else if (retval == MXUPCIE_ERR_IOADDR)
						printk("Invalid I/O address,board not configured\n");
					continue;

				}

				mx_getcfg(m,&hwconf);
				//init mxupciecfg first, or mxupciecfg data is not correct on ISR.
				//mxupcie_initbrd will hook ISR.
				if(mxupcie_initbrd(m,&hwconf)<0)
					continue;

                switch (hwconf.board_type) {
					case MXUPCIE_BOARD_CP118E_A_I:
					case MXUPCIE_BOARD_CP138E_A:
					case MXUPCIE_BOARD_CP134EL_A:
					case MXUPCIE_BOARD_CP116E_A_A:
					case MXUPCIE_BOARD_CP116E_A_B:
						cpld.base = (unsigned char *)hwconf.iobar3_addr;
						version = (uint8_t)mxCPLDGetVersion(cpld);
						major = version >> 6;
						minor = version & 0x3f;
						printk("MOXA CPLD version %d.%d\n", major, minor);
						break;
				}

				m++;
			}
		}
	}
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(3,7,0))
	for(m = 0; m < MXUPCIE_BOARDS; m++)
	{
		for(i = 0; i < MXUPCIE_PORTS_PER_BOARD;i++)
		{
			port_idx =  (m * MXUPCIE_PORTS_PER_BOARD) + i;
			tty_port_init(&mxvar_table[port_idx].ttyPort);
			tty_port_link_device(&mxvar_table[port_idx].ttyPort, mxvar_sdriver, port_idx);
		}
	}
	port_idx = MXUPCIE_BOARDS * MXUPCIE_PORTS_PER_BOARD;
	tty_port_init(&mxvar_table[port_idx].ttyPort);
	tty_port_link_device(&mxvar_table[port_idx].ttyPort, mxvar_sdriver, port_idx);
#endif

        ret1 = 0;
        ret2 = 0;

        if ( !(ret1=tty_register_driver(DRV_VAR)) ){
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))        	
            if ( !(ret2=tty_register_driver(&mxvar_cdriver)) ){

                return 0;
            }else{
                tty_unregister_driver(DRV_VAR);
		printk("Couldn't install MOXA Smartio/Industio family callout driver !\n");
            }
#else
	    return 0;	            
#endif            
        }else
	    printk("Couldn't install MOXA Smartio/Industio family driver !\n");


        if(ret1 || ret2){
		for(i=0; i<MXUPCIE_BOARDS; i++){
			if(mxupciecfg[i].board_type == -1)
				continue;
			else{
				free_irq(mxupciecfg[i].irq, &mxvar_table[i*MXUPCIE_PORTS_PER_BOARD]);
		    //todo: release io, vector
	                }
		}

		return -1;
        }

	return 0;
}


#if (LINUX_VERSION_CODE >  VERSION_CODE(2,6,19))
static void mx_do_softint(struct work_struct *work)
{
        struct mxupcie_struct *info = container_of(work, 
                                                 struct mxupcie_struct, tqueue);
#else
static void mx_do_softint(void *private_)
{
	struct mxupcie_struct *	info = (struct mxupcie_struct *)private_;
#endif
	struct tty_struct *	tty;

	tty = info->tty;

#if (LINUX_VERSION_CODE <  VERSION_CODE(2,6,9))
	if ( test_and_clear_bit(MXUPCIE_EVENT_TXLOW, &info->event) ) {
		if ( (tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup )
			(tty->ldisc.write_wakeup)(tty);
	        wake_up_interruptible(&tty->write_wait);
	}
#else
	if ( test_and_clear_bit(MXUPCIE_EVENT_TXLOW, &info->event) ) 
		tty_wakeup(tty);
#endif	    	    
	if ( test_and_clear_bit(MXUPCIE_EVENT_HANGUP, &info->event) )
		tty_hangup(tty);

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
	MX_MOD_DEC;
#endif
}


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
static unsigned char mxupcie_get_msr(unsigned char *baseaddr, int mode, int port, struct mxupcie_struct *info)
{
	unsigned char	status=0;

	status = MX_READ_REG(baseaddr + UART_MSR);
	mxupcie_msr[port] &= 0x0F;
	mxupcie_msr[port] |= status;
	status = mxupcie_msr[port];

	if( mode )
		mxupcie_msr[port] = 0;
		
	return status;
}
#endif


/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int mxupcie_open(struct tty_struct * tty, struct file * filp)
{
	struct mxupcie_struct *	info;
	int			retval, line;

	MX_LOCK_INIT();
	
	line = PORTNO(tty);

	if ( line == MXUPCIE_PORTS )
		return 0;
	    
    	MX_MOD_INC;
    	    
	if ( (line < 0) || (line > MXUPCIE_PORTS) )
		return -ENODEV;

	info = mxvar_table + line;

	if ( !info->base )
	    return -ENODEV;

	tty->driver_data = info;
	info->tty = tty;

//printk("port %d, mxupcie_open\r\n", info->port);
	/*
	 * Start up serial port
	 */
	info->count++;
	 
	retval = mx_startup(info);

	if ( retval )
	    return retval;

	retval = mx_block_til_ready(tty, filp, info);

	if ( retval )
	    return retval;

#if 0
	if ( (info->count == 1) && (info->flags & ASYNC_SPLIT_TERMIOS) ) {
#else
	if ( (info->count == 1) ) {
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	    if ( MX_TTY_DRV(subtype) == SERIAL_TYPE_NORMAL )
		*tty->termios = info->normal_termios;
	    else
		*tty->termios = info->callout_termios;
#else
	    if ( MX_TTY_DRV(subtype) == SERIAL_TYPE_NORMAL )
		(*tty).termios = info->normal_termios;
	    else
		(*tty).termios = info->callout_termios;	
#endif

	    MX_LOCK(&info->slock);
	    mx_change_speed(info, 0);
	    MX_UNLOCK(&info->slock);
	}

#ifdef TTY_DONT_FLIP
	clear_bit(TTY_DONT_FLIP, &tty->flags); // since VERSION_CODE >= 2.6.18
#endif

/* unmark here for very high baud rate (ex. 921600 bps) used
*/
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,1,0))
#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))
#if (LINUX_VERSION_CODE < VERSION_CODE(3,2,0))	
	tty->low_latency = 1;
#else
	tty->low_latency = 0;        
#endif
#else
	info->ttyPort.low_latency = 0;
#endif
#endif
	return 0;
}


/*
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 */
static void mxupcie_close(struct tty_struct * tty, struct file * filp)
{
	struct mxupcie_struct * info = (struct mxupcie_struct *)tty->driver_data;
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,9))
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,28))
	struct tty_ldisc *ld;
#endif
#endif
	unsigned long timeout;
	unsigned char reg_flag;	
	MX_LOCK_INIT();

	if ( PORTNO(tty) == MXUPCIE_PORTS )
	    return;

	if ( !info ){
	    MX_MOD_DEC;

	    return;
	}

	MX_LOCK(&info->slock);

	if ( tty_hung_up_p(filp) ) {
	    MX_UNLOCK(&info->slock);
	    MX_MOD_DEC;

	    return;
	}

#ifndef SP1
	if ( (tty->count == 1) && (info->count != 1) ) {
#else
#if (LINUX_VERSION_CODE < VERSION_CODE(2,4,21))
	if ( (tty->count == 1) && (info->count != 1) ) {
#else
	if ((atomic_read(&tty->count) == 1) && (info->count != 1)) {
#endif

#endif /* SP1 */
	    /*
	     * Uh, oh.	tty->count is 1, which means that the tty
	     * structure will be freed.  Info->count should always
	     * be one in these conditions.  If it's greater than
	     * one, we've got real problems, since it means the
	     * serial port won't be shutdown.
	     */
	    printk("mxupcie_close: bad serial port count; tty->count is 1, "
		   "info->count is %d\n", info->count);
	    info->count = 1;
	}

	if ( --info->count < 0 ) {
	    printk("mxupcie_close: bad serial port count for ttys%d: %d\n",
		   info->port, info->count);
	    info->count = 0;
	}

	if ( info->count ) {
	    MX_UNLOCK(&info->slock);
	    MX_MOD_DEC;

	    return;
	}

	info->flags |= ASYNC_CLOSING;
	MX_UNLOCK(&info->slock);
	/*
	 * Save the termios structure, since this port may have
	 * separate termios for callout and dialin.
	 */
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))	 
	if ( info->flags & ASYNC_NORMAL_ACTIVE )
	    info->normal_termios = *tty->termios;

	if ( info->flags & ASYNC_CALLOUT_ACTIVE )
	    info->callout_termios = *tty->termios;
#else
	if ( info->flags & ASYNC_NORMAL_ACTIVE )
	    info->normal_termios = (*tty).termios;

	if ( info->flags & ASYNC_CALLOUT_ACTIVE )
	    info->callout_termios = (*tty).termios;
#endif
	/*
	 * Now we wait for the transmit buffer to clear; and we notify
	 * the line discipline to only process XON/XOFF characters.
	 */
	
	reg_flag = MX_READ_REG(info->base + MOXA_PUART_EFR);
	reg_flag &= ~MOXA_EFR_AUTO_RTS;
	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_EFR);
	tty->closing = 1;

	if ( info->closing_wait != ASYNC_CLOSING_WAIT_NONE )
	    tty_wait_until_sent(tty, info->closing_wait);
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receive line status interrupts, and tell the
	 * interrupt driver to stop checking the data ready bit in the
	 * line status register.
	 */
	info->IER &= ~UART_IER_RLSI;

	if ( info->flags & ASYNC_INITIALIZED ) {
		MX_WRITE_REG(info->IER, info->base + UART_IER);
	    /*
	     * Before we drop DTR, make sure the UART transmitter
	     * has completely drained; this is especially
	     * important if there is a transmit FIFO!
	     */
		timeout = jiffies + HZ;
		while ( mxupcie_chars_in_buffer(tty) ) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(5);
			set_current_state(TASK_RUNNING);

			if ( time_after(jiffies, timeout) )
				break;
		    }
	}

	mx_shutdown(info);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,26))
	if ( MX_TTY_DRV(flush_buffer) )
		MX_TTY_DRV(flush_buffer)(tty);
#else
	mxupcie_flush_buffer(tty);
	
#endif
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
	if ( tty->ldisc.flush_buffer )
		tty->ldisc.flush_buffer(tty);
#else	    
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,27))
	ld = tty_ldisc_ref(tty);
	if (ld) {
		if(ld->flush_buffer)
			ld->flush_buffer(tty);

		tty_ldisc_deref(ld);
	}
#else
	tty_ldisc_flush(tty);
#endif
#endif	    
	    
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;

	if ( info->blocked_open ) {
		if ( info->close_delay ) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(info->close_delay);
			set_current_state(TASK_RUNNING);
		}

		wake_up_interruptible(&info->open_wait);
	}

	info->flags &= ~(ASYNC_NORMAL_ACTIVE | ASYNC_CALLOUT_ACTIVE | ASYNC_CLOSING);
	wake_up_interruptible(&info->close_wait);

	MX_MOD_DEC;
}

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
static int mxupcie_write(struct tty_struct * tty, int from_user,
		       const unsigned char * buf, int count)
#else
static int mxupcie_write(struct tty_struct * tty,
		       const unsigned char * buf, int count)
#endif
{
	int c, total = 0;
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

	if ( !tty || !info->xmit_buf  )
	    return(0);

	
	while ( 1 ) {
		c = MIN(count, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
			       SERIAL_XMIT_SIZE - info->xmit_head));
		if ( c <= 0 )
			break;

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
		if ( from_user ) {
			if(copy_from_user(info->xmit_buf+info->xmit_head, buf, c)==c){
				total = -EFAULT;	
				break;
			}
		
		} else
			memcpy(info->xmit_buf + info->xmit_head, buf, c);
#else
			memcpy(info->xmit_buf + info->xmit_head, buf, c);
#endif
		MX_LOCK(&info->slock);    
		info->xmit_head = (info->xmit_head + c) & (SERIAL_XMIT_SIZE - 1);
		info->xmit_cnt += c;
		MX_UNLOCK(&info->slock);

		buf += c;
		count -= c;
		total += c;
	}

	if ( info->xmit_cnt && !tty->stopped ) {
        	    MX_LOCK(&info->slock); 	
		    info->IER &= ~UART_IER_THRI	;
	            MX_WRITE_REG(info->IER, info->base + UART_IER);
	            info->IER |= UART_IER_THRI;
	            MX_WRITE_REG(info->IER, info->base + UART_IER);
	            MX_UNLOCK(&info->slock);
	}
	
//printk("%lu, mxupcie_write=%d\n", jiffies, total);
	
	return total;
}

#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,25))
static int mxupcie_put_char(struct tty_struct * tty, unsigned char ch)
#else
static void mxupcie_put_char(struct tty_struct * tty, unsigned char ch)
#endif
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

	if ( !tty || !info->xmit_buf )
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,25))
		return 0;
#else
		return;
#endif

	if ( info->xmit_cnt >= SERIAL_XMIT_SIZE - 1 )
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,25))
		return 0;
#else
		return;
#endif
	MX_LOCK(&info->slock);
	info->xmit_buf[info->xmit_head++] = ch;
	info->xmit_head &= SERIAL_XMIT_SIZE - 1;
	info->xmit_cnt++;
	MX_UNLOCK(&info->slock);

	if ( !tty->stopped ) {
        	if (!tty->hw_stopped) {
        		MX_LOCK(&info->slock);
		        info->IER &= ~UART_IER_THRI	;
	                MX_WRITE_REG(info->IER, info->base + UART_IER);
	    		info->IER |= UART_IER_THRI;
	    		MX_WRITE_REG(info->IER, info->base + UART_IER);
	    		MX_UNLOCK(&info->slock);
		}
	}
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,25))
	return 1;
#endif
//printk("%lu, mxupcie_put_char=%x\n", jiffies, ch);
}


static void mxupcie_flush_chars(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

//printk("%lu, mxupcie_flush_chars\n", jiffies);

	if ( info->xmit_cnt <= 0 || tty->stopped || !info->xmit_buf)
		return;

	MX_LOCK(&info->slock);
        info->IER &= ~UART_IER_THRI;
        MX_WRITE_REG(info->IER, info->base + UART_IER);
	info->IER |= UART_IER_THRI;
	MX_WRITE_REG(info->IER, info->base + UART_IER);
	MX_UNLOCK(&info->slock);
}


static int mxupcie_write_room(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	int	ret;

	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;

	if ( ret < 0 )
	    ret = 0;
	    
	return ret;
}


static int mxupcie_chars_in_buffer(struct tty_struct * tty)
{
	int len;
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	unsigned char t_cnt;
	
	len = info->xmit_cnt;
	t_cnt = MX_READ_REG(info->base + MOXA_PUART_TCNT);

	if(t_cnt)
		len+=(int)t_cnt;
//printk("%lu, mxupcie_chars_in_buffer=%d\n", jiffies, len);
	return len;
}


static void mxupcie_flush_buffer(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	char fcr;
	MX_LOCK_INIT();

//printk("%lu, flush_buffer\n", jiffies);

	MX_LOCK(&info->slock);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	MX_UNLOCK(&info->slock);
	
	fcr = MX_READ_REG(info->base + UART_FCR);
	MX_WRITE_REG((fcr | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT), 
		info->base + UART_FCR);
	MX_WRITE_REG(fcr, info->base+UART_FCR);
	
	wake_up_interruptible(&tty->write_wait);

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,27))
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && 
	     tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
#else
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,31))
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && 
             tty->ldisc.ops->write_wakeup)
		tty_wakeup(tty);
#else
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && 
             tty->ldisc->ops->write_wakeup)
		tty_wakeup(tty);
#endif
#endif
}


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,39))
static int mxupcie_ioctl(struct tty_struct * tty, struct file * file,
		       unsigned int cmd, unsigned long arg)
#else
static int mxupcie_ioctl(struct tty_struct * tty, unsigned int cmd,
		       unsigned long arg)
#endif
{
	int			error;
	struct mxupcie_struct *	info = (struct mxupcie_struct *)tty->driver_data;
	int			retval;
	struct async_icount	cprev, cnow;	    /* kernel counter temps */
	struct serial_icounter_struct *p_cuser;     /* user space */
	unsigned long 		templ;
	int			ret;
	int			port_idx = 0;
	int baudrate;
	gpio_param_t   cpld;
	MX_LOCK_INIT();
	port_idx = 0;
//printk("%lu, mxupcie_ioctl=%x\n", jiffies, cmd);

	if ( PORTNO(tty) == MXUPCIE_PORTS )
		return(mx_ioctl_special(cmd, arg));
	
	if ( cmd == MOXA_SET_SPECIAL_BAUD_RATE || cmd == MOXA_GET_SPECIAL_BAUD_RATE || cmd == SMARTIO_SET_SPECIAL_BAUD_RATE || cmd == SMARTIO_GET_SPECIAL_BAUD_RATE ) {
		int	speed, i;
		if ( cmd == MOXA_SET_SPECIAL_BAUD_RATE || cmd == SMARTIO_SET_SPECIAL_BAUD_RATE ) {
	    		error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(int));

	    		if ( MX_ERR(error) )
				return(error);

	    		get_from_user(speed,(int *)arg);

			if ( speed <= 0 || speed > info->MaxCanSetBaudRate )
				return -EFAULT;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))	
			if ( !info->tty || !info->tty->termios || !info->base )
				return 0;

			info->tty->termios->c_cflag &= ~(CBAUD | CBAUDEX);

			for ( i=0; i<BAUD_TABLE_NO && speed != mxvar_baud_table[i]; i++ );

			if ( i == BAUD_TABLE_NO ) {
				info->tty->termios->c_cflag |= B4000000;
			} else {
				switch ( mxvar_baud_table[i] ) {
					case 921600 : info->tty->termios->c_cflag |= B921600; break;
					case 460800 : info->tty->termios->c_cflag |= B460800; break;
					case 230400 : info->tty->termios->c_cflag |= B230400; break;
					case 115200 : info->tty->termios->c_cflag |= B115200; break;
					case 57600 : info->tty->termios->c_cflag |= B57600; break;
					case 38400 : info->tty->termios->c_cflag |= B38400; break;
					case 19200 : info->tty->termios->c_cflag |= B19200; break;
					case 9600 : info->tty->termios->c_cflag |= B9600; break;
					case 4800 : info->tty->termios->c_cflag |= B4800; break;
					case 2400 : info->tty->termios->c_cflag |= B2400; break;
					case 1800 : info->tty->termios->c_cflag |= B1800; break;
					case 1200 : info->tty->termios->c_cflag |= B1200; break;
					case 600 : info->tty->termios->c_cflag |= B600; break;
					case 300 : info->tty->termios->c_cflag |= B300; break;
					case 200 : info->tty->termios->c_cflag |= B200; break;
					case 150 : info->tty->termios->c_cflag |= B150; break;
					case 134 : info->tty->termios->c_cflag |= B134; break;
					case 110 : info->tty->termios->c_cflag |= B110; break;
					case 75 : info->tty->termios->c_cflag |= B75; break;
					case 50 : info->tty->termios->c_cflag |= B50; break;
				}
			}
#else
			if ( !info->tty || !info->base )
				return 0;

			info->tty->termios.c_cflag &= ~(CBAUD | CBAUDEX);

			for ( i=0; i<BAUD_TABLE_NO && speed != mxvar_baud_table[i]; i++ );

			if ( i == BAUD_TABLE_NO ) {
				info->tty->termios.c_cflag |= B4000000;
			} else {
				switch ( mxvar_baud_table[i] ) {
					case 921600 : info->tty->termios.c_cflag |= B921600; break;
					case 460800 : info->tty->termios.c_cflag |= B460800; break;
					case 230400 : info->tty->termios.c_cflag |= B230400; break;
					case 115200 : info->tty->termios.c_cflag |= B115200; break;
					case 57600 : info->tty->termios.c_cflag |= B57600; break;
					case 38400 : info->tty->termios.c_cflag |= B38400; break;
					case 19200 : info->tty->termios.c_cflag |= B19200; break;
					case 9600 : info->tty->termios.c_cflag |= B9600; break;
					case 4800 : info->tty->termios.c_cflag |= B4800; break;
					case 2400 : info->tty->termios.c_cflag |= B2400; break;
					case 1800 : info->tty->termios.c_cflag |= B1800; break;
					case 1200 : info->tty->termios.c_cflag |= B1200; break;
					case 600 : info->tty->termios.c_cflag |= B600; break;
					case 300 : info->tty->termios.c_cflag |= B300; break;
					case 200 : info->tty->termios.c_cflag |= B200; break;
					case 150 : info->tty->termios.c_cflag |= B150; break;
					case 134 : info->tty->termios.c_cflag |= B134; break;
					case 110 : info->tty->termios.c_cflag |= B110; break;
					case 75 : info->tty->termios.c_cflag |= B75; break;
					case 50 : info->tty->termios.c_cflag |= B50; break;
				}
			}
#endif
			info->speed = speed;
	   		MX_LOCK(&info->slock);
			mx_change_speed(info, 0);
		   	MX_UNLOCK(&info->slock);
		} else {
	    		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));

	    		if ( MX_ERR(error) )
				return(error);

	    		if(copy_to_user((int*)arg, &info->speed, sizeof(int)))
			     return -EFAULT;
		}

		return 0;
	}

	if ( (cmd != TIOCGSERIAL) && (cmd != TIOCMIWAIT) &&
	     (cmd != TIOCGICOUNT) ) {

		if ( tty->flags & (1 << TTY_IO_ERROR) )
			return -EIO;
	}
	
	switch ( cmd ) {
		case TCSBRK:	/* SVID version: non-zero arg --> no break */
			retval = tty_check_change(tty);

			if ( retval )
				return retval;

			tty_wait_until_sent(tty, 0);

			if ( !arg )
				mx_send_break(info, HZ/4);		/* 1/4 second */

			return 0;
		case TCSBRKP:	/* support for POSIX tcsendbreak() */
			retval = tty_check_change(tty);

			if ( retval )
				return retval;

			tty_wait_until_sent(tty, 0);
			mx_send_break(info, arg ? arg*(HZ/10) : HZ/4);

			return 0;
		case TIOCGSOFTCAR:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(long));

			if ( MX_ERR(error) )
				return error;

			put_to_user(C_CLOCAL(tty) ? 1 : 0, (unsigned long *)arg);

			return 0;
		case TIOCSSOFTCAR:
			error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(long));

			if ( MX_ERR(error) )
				return error;

			get_from_user(templ,(unsigned long *)arg);
			arg = templ;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))				
			tty->termios->c_cflag = ((tty->termios->c_cflag & ~CLOCAL) | (arg ? CLOCAL : 0));
#else
			tty->termios.c_cflag = ((tty->termios.c_cflag & ~CLOCAL) | (arg ? CLOCAL : 0));
#endif
			return 0;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
		case TIOCMGET:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(unsigned int));

			if ( MX_ERR(error) )
				return error;

			return mx_get_modem_info(info, (unsigned int *)arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return mx_set_modem_info(info, cmd, (unsigned int *)arg);
#endif	    
		case TIOCGSERIAL:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(struct serial_struct));

			if ( MX_ERR(error) )
				return error;

			return mx_get_serial_info(info, (struct serial_struct *)arg);
		case TIOCSSERIAL:
			error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg,sizeof(struct serial_struct));

			if ( MX_ERR(error) )
				return error;

			return mx_set_serial_info(info, (struct serial_struct *)arg);
		case TIOCSERGETLSR: /* Get line status register */
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(unsigned int));

			if ( MX_ERR(error) )
				return error;
			else
				return mx_get_lsr_info(info, (unsigned int *)arg);
	/*
	 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
	 * - mask passed in arg for lines of interest
	 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
	 * Caller should use TIOCGICOUNT to see which one it was
	 */
		case TIOCMIWAIT:{
			DECLARE_WAITQUEUE(wait, current);
			MX_LOCK(&info->slock);
			cprev = info->icount;   /* note the counters on entry */
			MX_UNLOCK(&info->slock);
	    
			while ( 1 ) {
			        add_wait_queue(&info->delta_msr_wait, &wait);
				set_current_state(TASK_INTERRUPTIBLE);
				schedule();
			        remove_wait_queue(&info->delta_msr_wait, &wait);

		                /* see if a signal did it */
				if ( signal_pending(current))
					return -ERESTARTSYS;

			        MX_LOCK(&info->slock);
				cnow = info->icount;	/* atomic copy */
				MX_UNLOCK(&info->slock);
				
                                if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
                                     ((arg & TIOCM_CD) && (cnow.dcd != cprev.dcd)) ||
				     ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
					return 0;
				}
				cprev = cnow;
			}
		}
	    /* NOTREACHED */
	/*
	 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
	 * Return: write counters to the user passed counter struct
	 * NB: both 1->0 and 0->1 transitions are counted except for
	 *     RI where only 0->1 is counted.
	 */
		case TIOCGICOUNT:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(struct serial_icounter_struct));

			if ( MX_ERR(error) )
				return error;

			MX_LOCK(&info->slock);
			cnow = info->icount;
			MX_UNLOCK(&info->slock);
			p_cuser = (struct serial_icounter_struct *)arg;

			if (put_user(cnow.frame, &p_cuser->frame))
				return -EFAULT;
			
			if (put_user(cnow.brk, &p_cuser->brk))
				return -EFAULT;

			if (put_user(cnow.overrun, &p_cuser->overrun))
				return -EFAULT;

			if (put_user(cnow.buf_overrun, &p_cuser->buf_overrun))
				return -EFAULT;

			if (put_user(cnow.parity, &p_cuser->parity))
				return -EFAULT;

			if (put_user(cnow.rx, &p_cuser->rx))
				return -EFAULT;

			if (put_user(cnow.tx, &p_cuser->tx))
				return -EFAULT;

			put_to_user(cnow.cts, &p_cuser->cts);
			put_to_user(cnow.dsr, &p_cuser->dsr);
			put_to_user(cnow.rng, &p_cuser->rng);
			put_to_user(cnow.dcd, &p_cuser->dcd);

			return 0;

		case SMARTIO_PUART_SET_INTERFACE:
			return mx_set_interface(info, (unsigned char)arg);

		case SMARTIO_PUART_GET_INTERFACE:
			if(copy_to_user((void*)arg,&info->UIR,sizeof(info->UIR)))
				return -EFAULT;

			return 0;

		case SMARTIO_PUART_SET_TERMINATOR:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				return mxCPLDSetTerminator(cpld,
					 		port_idx, 
							(int) arg);
			default : 
				return mx_set_terminator(info, 
							(unsigned char) arg);
			}
		case SMARTIO_PUART_GET_TERMINATOR:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				ret =  mxCPLDGetTerminator(cpld, port_idx);
				if (copy_to_user((void*)arg, &ret, 
					sizeof(ret)))
					return -EFAULT;
		
				return 0;
			default : 
				if (copy_to_user((void*)arg,
						&info->terminator_flag,
						sizeof(info->terminator_flag)))
					return -EFAULT;
				return 0;
			}
		case SMARTIO_PUART_SET_PULL_STATE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				return mxCPLDSetPullState(cpld, port_idx, (int)arg);
			default :
				return -EPERM; /* Other Not Supported */
			}	
		case SMARTIO_PUART_GET_PULL_STATE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				ret = mxCPLDGetPullState(cpld, port_idx);
			
				if (copy_to_user((void*)arg, 
						&ret, sizeof(ret)))
					return -EFAULT;

				return 0;
			default : 
				return -EPERM; /* Other Not Supported */
			}
		case SMARTIO_PUART_SET_AUTO_MODE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;

				baudrate = info->speed;
				if (baudrate == 921600)
					baudrate = MX_CPLD_BAUD_921600;
				else if (baudrate == 460800)
					baudrate = MX_CPLD_BAUD_460800;
				else if (baudrate == 230400)
					baudrate = MX_CPLD_BAUD_230400;
				else if (baudrate == 115200)
					baudrate = MX_CPLD_BAUD_115200;
				else if (baudrate == 57600)
					baudrate = MX_CPLD_BAUD_57600;
				else if (baudrate == 38400)
					baudrate = MX_CPLD_BAUD_38400;
				else if (baudrate == 19200)
					baudrate = MX_CPLD_BAUD_19200;
				else if (baudrate == 9600)
					baudrate = MX_CPLD_BAUD_9600;
				if(arg == 1 && mxCPLDSetBaudRate(cpld, port_idx, baudrate) == MX_CPLD_ERR)
					return -EPERM;

				return mxCPLDSetAutoMode(cpld, port_idx, (int)arg);
			default : 
				return -EPERM; /* Other Not Supported */
			}

		case SMARTIO_PUART_GET_AUTO_MODE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				ret = mxCPLDGetAutoMode(cpld, port_idx);
				if (copy_to_user((void*)arg, &ret, sizeof(ret)))
					return -EFAULT;
				return 0;
			default : 
				return -EPERM; /* Other Not Supported */
			}
		case SMARTIO_PUART_SET_MASTER_SLAVE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				return mxCPLDSetMasterSlave(cpld, port_idx, (int)arg);
			default : 
				return -EPERM; /* Other Not Supported */
			}
		case SMARTIO_PUART_GET_MASTER_SLAVE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				ret = mxCPLDGetMasterSlave(cpld, port_idx);
				if (copy_to_user((void*)arg, 
					&ret, sizeof(ret)))
					return -EFAULT;

				return 0;
			default : 
				return -EPERM; /* Other Not Supported */
			}
		case SMARTIO_PUART_SET_DIAGNOSE:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
                
				baudrate = info->speed;
                if (baudrate == 921600)
                    baudrate = MX_CPLD_BAUD_921600;
                else if (baudrate == 460800)
                    baudrate = MX_CPLD_BAUD_460800;
                else if (baudrate == 230400)
                    baudrate = MX_CPLD_BAUD_230400;
                else if (baudrate == 115200)
                    baudrate = MX_CPLD_BAUD_115200;
                else if (baudrate == 57600)
                    baudrate = MX_CPLD_BAUD_57600;
                else if (baudrate == 38400)
                    baudrate = MX_CPLD_BAUD_38400;
                else if (baudrate == 19200)
                    baudrate = MX_CPLD_BAUD_19200;
                else if (baudrate == 9600)
                    baudrate = MX_CPLD_BAUD_9600;
				if(arg == 1 && mxCPLDSetBaudRate(cpld, port_idx, baudrate) == MX_CPLD_ERR)
					return -EPERM;

				return mxCPLDSetDiagnose(cpld, port_idx, (int)arg);
			default : 
				return -EPERM; /* Other Not Supported */
			}
		case SMARTIO_PUART_GET_ALARM:
			switch (info->board_type) {
			case MXUPCIE_BOARD_CP118E_A_I: 
			case MXUPCIE_BOARD_CP138E_A: 
			case MXUPCIE_BOARD_CP134EL_A: 
			case MXUPCIE_BOARD_CP116E_A_A: 
            case MXUPCIE_BOARD_CP116E_A_B: 
				port_idx = info->port % 8;
				cpld.base = (unsigned char *)info->iobar3_addr;
				ret = mxCPLDGetAlarmCode(cpld, port_idx);
			
				if (copy_to_user((void*)arg, 
						&ret, sizeof(ret)))
					return -EFAULT;
				return 0;
			default : 
				return -EPERM; /* Other Not Supported */
			}
		default:
			return(-ENOIOCTLCMD);
	}

	return 0;
}

static int mx_ioctl_special(unsigned int cmd, unsigned long arg)
{
	int		error, i, result, status;

	switch ( cmd ) {
		case MOXA_GET_CONF:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(struct mxupcie_hwconf)*4);

			if ( MX_ERR(error) )
				return error;

			if(copy_to_user((struct mxupcie_hwconf *)arg, mxupciecfg,sizeof(struct mxupcie_hwconf)*4))
				return -EFAULT;

			return 0;

		case MOXA_GET_MAJOR:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));

			if ( MX_ERR(error) )
				return error;

			if(copy_to_user((int*)arg, &ttymajor, sizeof(int)))
				return -EFAULT;

			return 0;

		case MOXA_GET_CUMAJOR:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));

			if ( MX_ERR(error) )
				return error;

			if(copy_to_user((int*)arg, &calloutmajor, sizeof(int)))
				return -EFAULT;

			return 0;

		case MOXA_CHKPORTENABLE:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(long));

			if ( MX_ERR(error) )
				return error;

			result = 0;

			for ( i=0; i<MXUPCIE_PORTS; i++ ) {
				if ( mxvar_table[i].base )
					result |= (1 << i);
			}

			put_to_user(result, (unsigned long *)arg);

			return 0;

		case MOXA_GETDATACOUNT:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(struct mxupcie_log));

			if ( MX_ERR(error) )
				return error;

			if(copy_to_user((struct mxupcie_log *)arg, &mxvar_log, sizeof(mxvar_log)))
				return -EFAULT;
			
			return 0;

		case MOXA_GETMSTATUS:
			error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,sizeof(struct mxupcie_mstatus) * MXUPCIE_PORTS);

			if ( MX_ERR(error) )
				return error;

		        for(i=0; i<MXUPCIE_PORTS; i++){
				GMStatus[i].ri = 0;

				if ( !mxvar_table[i].base ){
					GMStatus[i].dcd = 0;
					GMStatus[i].dsr = 0;
					GMStatus[i].cts = 0;

					continue;
				}
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
				if ( !mxvar_table[i].tty || !mxvar_table[i].tty->termios )
					GMStatus[i].cflag=mxvar_table[i].normal_termios.c_cflag;
				else
					GMStatus[i].cflag = mxvar_table[i].tty->termios->c_cflag;
#else
				if ( !mxvar_table[i].tty )
					GMStatus[i].cflag=mxvar_table[i].normal_termios.c_cflag;
				else
					GMStatus[i].cflag = mxvar_table[i].tty->termios.c_cflag;
#endif

				status = MX_READ_REG(mxvar_table[i].base + UART_MSR);

				if(status  & 0x80/*UART_MSR_DCD*/)
					GMStatus[i].dcd = 1;
				else
					GMStatus[i].dcd = 0;

				if(status  & 0x20/*UART_MSR_DSR*/)
					GMStatus[i].dsr = 1;
				else
					GMStatus[i].dsr = 0;

				if(status  & 0x10/*UART_MSR_CTS*/)
					GMStatus[i].cts = 1;
				else
					GMStatus[i].cts = 0;
			}

			if(copy_to_user((struct mxupcie_mstatus *)arg, GMStatus,sizeof(struct mxupcie_mstatus) * MXUPCIE_PORTS))
				return -EFAULT;

			return 0;

		default:
			return -ENOIOCTLCMD;
	}

	return 0;
}


static void mx_stoprx(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	info->IER &= ~UART_IER_RDI;
	MX_WRITE_REG(info->IER, info->base + UART_IER);
	MX_UNLOCK(&info->slock);
}


static void mx_startrx(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	info->IER |= UART_IER_RDI;
	MX_WRITE_REG(info->IER, info->base + UART_IER);
	MX_UNLOCK(&info->slock);
}


/*
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 */
static void mxupcie_throttle(struct tty_struct * tty)
{
        mx_stoprx(tty);
}


static void mxupcie_unthrottle(struct tty_struct * tty)
{
        mx_startrx(tty);
}


#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static void mxupcie_set_termios(struct tty_struct * tty, 
                              struct ktermios * old_termios)
#else
static void mxupcie_set_termios(struct tty_struct * tty,
			      struct termios * old_termios)
#endif
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	if ( (tty->termios->c_cflag != old_termios->c_cflag) ||
	     (RELEVANT_IFLAG(tty->termios->c_iflag) !=
	      RELEVANT_IFLAG(old_termios->c_iflag)) ) {
		MX_LOCK(&info->slock);
		mx_change_speed(info, old_termios);
		MX_UNLOCK(&info->slock);

		if ( (old_termios->c_cflag & CRTSCTS) &&
	     	     !(tty->termios->c_cflag & CRTSCTS) ) {
	    		tty->hw_stopped = 0;
	    		mxupcie_start(tty);
		}
	}
#else
	if ( (tty->termios.c_cflag != old_termios->c_cflag) ||
	     (RELEVANT_IFLAG(tty->termios.c_iflag) !=
	      RELEVANT_IFLAG(old_termios->c_iflag)) ) {
		MX_LOCK(&info->slock);
		mx_change_speed(info, old_termios);
		MX_UNLOCK(&info->slock);

		if ( (old_termios->c_cflag & CRTSCTS) &&
	     	     !(tty->termios.c_cflag & CRTSCTS) ) {
	    		tty->hw_stopped = 0;
	    		mxupcie_start(tty);
		}
	}
#endif

/* Handle sw stopped */
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	if ( (old_termios->c_iflag & IXON) &&
     	     !(tty->termios->c_iflag & IXON) ) {
#else
	if ( (old_termios->c_iflag & IXON) &&
     	     !(tty->termios.c_iflag & IXON) ) {
#endif
    		tty->stopped = 0;
    		mxupcie_start(tty);
	}
}


/*
 * mxupcie_stop() and mxupcie_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 */
static void mxupcie_stop(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	if ( info->IER & UART_IER_THRI ) {
		info->IER &= ~UART_IER_THRI;
		MX_WRITE_REG(info->IER, info->base + UART_IER);
	}
	MX_UNLOCK(&info->slock);
}


static void mxupcie_start(struct tty_struct * tty)
{
	struct mxupcie_struct *info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();
	
	MX_LOCK(&info->slock);
	if ( info->xmit_cnt && info->xmit_buf ) {
		info->IER &= ~UART_IER_THRI;
		MX_WRITE_REG(info->IER, info->base + UART_IER);
		info->IER |= UART_IER_THRI;
		MX_WRITE_REG(info->IER, info->base + UART_IER);
	}
	MX_UNLOCK(&info->slock);
}

/*
 * mxupcie_wait_until_sent() --- wait until the transmitter is empty
 */
#if (LINUX_VERSION_CODE >= 131394)
static void mxupcie_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct mxupcie_struct * info = (struct mxupcie_struct *)tty->driver_data;
	unsigned long orig_jiffies, char_time;
	int lsr;

	if (info->type == PORT_UNKNOWN)
		return;

	if (info->xmit_fifo_size == 0)
		return; /* Just in case.... */

	orig_jiffies = jiffies;
	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send a single character, and make it at least 1.  The check
	 * interval should also be less than the timeout.
	 *
	 * Note: we have to use pretty tight timings here to satisfy
	 * the NIST-PCTS.
	 */
	char_time = (info->timeout - HZ/50) / info->xmit_fifo_size;
	char_time = char_time / 5;

	if (char_time == 0)
		char_time = 1;

	if (timeout && timeout < char_time)
		char_time = timeout;
	/*
	 * If the transmitter hasn't cleared in twice the approximate
	 * amount of time to send the entire FIFO, it probably won't
	 * ever clear.  This assumes the UART isn't doing flow
	 * control, which is currently the case.  Hence, if it ever
	 * takes longer than info->timeout, this is probably due to a
	 * UART bug of some kind.  So, we clamp the timeout parameter at
	 * 2*info->timeout.
	 */

	if (!timeout || timeout > 2*info->timeout)
		timeout = 2*info->timeout;
#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
	printk("In rs_wait_until_sent(%d) check=%lu...", timeout, char_time);
	printk("jiff=%lu...", jiffies);
#endif
	while (!((lsr = MX_READ_REG(info->base+ UART_LSR)) & UART_LSR_TEMT)) {
#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
		printk("lsr = %d (jiff=%lu)...", lsr, jiffies);
#endif
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(char_time);

		if (signal_pending(current))
			break;

		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
	set_current_state(TASK_RUNNING);

#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
	printk("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
}
#endif

/*
 * This routine is called by tty_hangup() when a hangup is signaled.
 */
void mxupcie_hangup(struct tty_struct * tty)
{
	struct mxupcie_struct * info = (struct mxupcie_struct *)tty->driver_data;

	mxupcie_flush_buffer(tty);
	mx_shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}


/*
 * mxupcie_rs_break() --- routine which turns the break handling on or off
 */
#if (LINUX_VERSION_CODE >= 131394 && LINUX_VERSION_CODE < VERSION_CODE(2,6,26))
static void mxupcie_rs_break(struct tty_struct *tty, int break_state)
{
#else
static int mxupcie_rs_break(struct tty_struct *tty, int break_state)
{
#endif
	struct mxupcie_struct * info = (struct mxupcie_struct *)tty->driver_data;
	MX_LOCK_INIT();
	
	MX_LOCK(&info->slock);
	if (break_state == -1){
		switch(info->UIR){
			case MOXA_UIR_RS485_4W:
			case MOXA_UIR_RS485_2W:
				mx_software_break_signal(info, MX_BREAK_ON);
				break;
			case MOXA_UIR_RS232:
			case MOXA_UIR_RS422:
				MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) | UART_LCR_SBC, info->base + UART_LCR);
				break;
		}
	}
	else{
		switch(info->UIR){
			case MOXA_UIR_RS485_4W:
			case MOXA_UIR_RS485_2W:
				mx_software_break_signal(info, MX_BREAK_OFF);
				break;
			case MOXA_UIR_RS232:
			case MOXA_UIR_RS422:
				MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) & ~UART_LCR_SBC, info->base + UART_LCR);
				break;
		}

	}
	MX_UNLOCK(&info->slock);
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,18))
	return 0;
#endif
}

/*
 * This is the serial driver's generic interrupt routine
 */
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,18))
static irqreturn_t mx_interrupt(int irq, void *dev_id)
#else
static IRQ_RET mx_interrupt(int irq, void *dev_id, struct pt_regs * regs)
#endif
{
	int			status, iir, i;
	struct mxupcie_struct *	info;
	struct mxupcie_struct *	port;
	int			max, msr;
	int			pass_counter = 0;
	int			int_cnt;
	int			handled=0;
	int			vect_flag;

	port = 0;

        for(i=0; i<MXUPCIE_BOARDS; i++){
            if(dev_id == &(mxvar_table[i*MXUPCIE_PORTS_PER_BOARD])){
                port = dev_id;
                break;
            }
        }

        if(i==MXUPCIE_BOARDS)
            goto irq_stop;

        if(port==0)
            goto irq_stop;

        max = mxupcie_numports[mxupciecfg[i].board_type-1];
	pass_counter = 0;

	do{
		vect_flag = 0;

		for(i=0;i<max;i++){
			info = port + i;
			int_cnt=0;

			do{
				iir = MX_READ_REG(info->base+UART_IIR);

				if ( iir == MOXA_IIR_NO_INT){
					vect_flag++;
					break;
				}

				if ( !info->tty ) {
					status = MX_READ_REG(info->base+UART_LSR);
					MX_WRITE_REG(0x27, info->base+UART_FCR);
					MX_READ_REG(info->base+UART_MSR);
					break;
				}
	
				handled = 1;

				spin_lock(&info->slock);
				status = MX_READ_REG(info->base+UART_LSR);

				if(iir == MOXA_IIR_RLSI){	
					if(status & UART_LSR_PE)
						info->err_shadow |= NPPI_NOTIFY_PARITY;
					if(status & UART_LSR_FE)
						info->err_shadow |= NPPI_NOTIFY_FRAMING;
					if(status & UART_LSR_OE) 
						info->err_shadow |= NPPI_NOTIFY_HW_OVERRUN;
					if(status & UART_LSR_BI)
						info->err_shadow |= NPPI_NOTIFY_BREAK;
				}

				if(iir & MOXA_IIR_RDI){
					status &= info->read_status_mask;
					if ( status & UART_LSR_DR )
						mx_receive_chars(info, &status);
				}

				msr = MX_READ_REG(info->base + UART_MSR);
				if ( msr & UART_MSR_ANY_DELTA ) {
					mx_check_modem_status(info, msr);
				}

				if(iir & MOXA_IIR_THRI){
					if ( status & UART_LSR_THRE ) {
						mx_transmit_chars(info);
					}
				}

				spin_unlock(&info->slock);
			}while(int_cnt++ < MXUPCIE_ISR_PASS_LIMIT);
		}

		if(vect_flag == max)
			break;

	}while(pass_counter++ < MXUPCIE_ISR_PASS_LIMIT);
	
irq_stop:
	return IRQ_RETVAL(handled);	
}

#if defined(_SCREEN_INFO_H) || (LINUX_VERSION_CODE >= VERSION_CODE(2,6,16))
static void mx_receive_chars(struct mxupcie_struct *info,
					 int *status)
{
	struct tty_struct *	tty = info->tty;
	unsigned char		ch, gdl=0;
	int			cnt = 0;
	int 			recv_room;
	int			max = 256;
	unsigned long 		flags;

	if ( *status & UART_LSR_SPECIAL )
		goto intr_old;

#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))
	recv_room = tty_buffer_request_room(tty,MX_RX_FIFO_SIZE);
#else
	recv_room = tty_buffer_request_room(&info->ttyPort, MX_RX_FIFO_SIZE);
#endif
	if(recv_room){
		gdl = MX_READ_REG(info->base + MOXA_PUART_RCNT);
	
		if(gdl > recv_room)
			gdl = recv_room;

		if(gdl){
#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))
			tty_insert_flip_string(tty, info->base + MOXA_PUART_MEMRBR, gdl);
#else
			tty_insert_flip_string(
				&info->ttyPort, info->base + MOXA_PUART_MEMRBR, gdl);
			
#endif
			cnt = gdl;
		}
	}
	else{
		set_bit(TTY_THROTTLED, &tty->flags);
	}

	goto end_intr;

intr_old:

	do {
		if(max-- <0)
			break;

		ch = MX_READ_REG(info->base + UART_RX);

		if ( *status & UART_LSR_SPECIAL ) {
			if ( *status & UART_LSR_BI ) {
				flags = TTY_BREAK;
				info->icount.brk++;

				if ( info->flags & ASYNC_SAK )
					do_SAK(tty);
			} else if ( *status & UART_LSR_PE ) {
				flags = TTY_PARITY;
				info->icount.parity++;
			} else if ( *status & UART_LSR_FE ) {
				flags = TTY_FRAME;			
				info->icount.frame++;
			} else if ( *status & UART_LSR_OE ) {
				flags = TTY_OVERRUN;
				info->icount.overrun++;
			} else
				flags = TTY_BREAK;

		} else
			flags = 0;

#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))
		tty_insert_flip_char(tty, ch, flags);
#else
		tty_insert_flip_char(&info->ttyPort, ch, flags);
#endif
		cnt++;

		*status = MX_READ_REG(info->base+UART_LSR);
	} while ( *status & UART_LSR_DR );

end_intr:	// add by Victor Yu. 09-02-2002

#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))
	tty_flip_buffer_push(tty);
#else
	tty_flip_buffer_push(&info->ttyPort);
#endif
	mxvar_log.rxcnt[info->port] += cnt;
	info->mon_data.rxcnt += cnt;
	info->mon_data.up_rxcnt += cnt;
}
#else
static void mx_receive_chars(struct mxupcie_struct *info,
					 int *status)
{
	struct tty_struct *	tty = info->tty;
	unsigned char		ch, gdl;
	int			cnt = 0;
	unsigned char           *cp;
	char                    *fp;
	int 			recv_room;
	int			max = 256;
	
        cp = tty->flip.char_buf;
        fp = tty->flip.flag_buf;

	if ( *status & UART_LSR_SPECIAL )
		goto intr_old;

	recv_room = tty->ldisc.receive_room(tty);

	if(recv_room){
		gdl = MX_READ_REG(info->base + MOXA_PUART_RCNT);
	
		if(gdl > recv_room)
			gdl = recv_room;

		if(gdl){
			cnt = gdl;
			memcpy(cp,info->base + MOXA_PUART_MEMRBR,cnt);
			memset(fp, TTY_NORMAL, cnt);
			cp+=cnt;
			fp+=cnt;
		}
	}
	else{
		set_bit(TTY_THROTTLED, &tty->flags);
	}

	goto end_intr;

intr_old:
	do {
		if(max-- <0)
			break;

		ch = MX_READ_REG(info->base + UART_RX);

		if ( *status & UART_LSR_SPECIAL ) {
			if ( *status & UART_LSR_BI ) {
				*fp++ = TTY_BREAK;
				info->icount.brk++;
				if ( info->flags & ASYNC_SAK )
					do_SAK(tty);
			} else if ( *status & UART_LSR_PE ) {
				*fp++ = TTY_PARITY;
				info->icount.parity++;
			} else if ( *status & UART_LSR_FE ) {
				*fp++ = TTY_FRAME;
				info->icount.frame++;
			} else if ( *status & UART_LSR_OE ) {
				*fp++ = TTY_OVERRUN;
				info->icount.overrun++;
			} else
				*fp++ = 0;
		} else
			*fp++ = 0;

		*cp++ = ch;
		cnt++;

		*status = MX_READ_REG(info->base+UART_LSR);
	} while ( *status & UART_LSR_DR );

end_intr:	// add by Victor Yu. 09-02-2002

	mxvar_log.rxcnt[info->port] += cnt;
	info->mon_data.rxcnt += cnt;
	info->mon_data.up_rxcnt += cnt;

	tty->ldisc.receive_buf(tty, tty->flip.char_buf, tty->flip.flag_buf, cnt);
//printk("%lu, mxsesr_receive_chars=%d, [%x]\n", 
//	jiffies, count, tty->flip.char_buf[0]);
}
#endif


static void mx_transmit_chars(struct mxupcie_struct *info)
{
	int	count, cnt;
	int tx_cnt;

	if ( info->xmit_buf == 0 ){
	    return;
	}

	if(info->xmit_cnt==0){
		if ( info->xmit_cnt < WAKEUP_CHARS ) {
			set_bit(MXUPCIE_EVENT_TXLOW,&info->event);
			MXQ_TASK();
		}
		return;
	}

	if (info->tty->stopped){
		info->IER &= ~UART_IER_THRI;
		MX_WRITE_REG(info->IER, info->base + UART_IER);
		return;
	}

	cnt = info->xmit_cnt;
	count = info->xmit_fifo_size;

	tx_cnt = MX_TX_FIFO_SIZE - MX_READ_REG(info->base + MOXA_PUART_TCNT);

	cnt = MIN(info->xmit_cnt, MIN(tx_cnt,
		       SERIAL_XMIT_SIZE - info->xmit_tail));

	if(cnt){
		memcpy(info->base + MOXA_PUART_MEMTHR,info->xmit_buf+info->xmit_tail,cnt);
		info->xmit_tail += cnt;
		info->xmit_tail &= (SERIAL_XMIT_SIZE - 1);
		info->xmit_cnt -= cnt;
	}

	mxvar_log.txcnt[info->port] += cnt;
	info->mon_data.txcnt += cnt;
	info->mon_data.up_txcnt += cnt;
        info->icount.tx += cnt;

	if ( info->xmit_cnt < WAKEUP_CHARS ) {
		set_bit(MXUPCIE_EVENT_TXLOW,&info->event);
		MXQ_TASK();
	}
}

static void mx_check_modem_status(struct mxupcie_struct *info,
					      int status)
{
	/* update input line counters */
	if ( status & UART_MSR_TERI )
	    info->icount.rng++;
	if ( status & UART_MSR_DDSR )
	    info->icount.dsr++;
	if ( status & UART_MSR_DDCD )
	    info->icount.dcd++;
	if ( status & UART_MSR_DCTS )
	    info->icount.cts++;

	info->mon_data.modem_status = status;
	wake_up_interruptible(&info->delta_msr_wait);

	if ( (info->flags & ASYNC_CHECK_CD) && (status & UART_MSR_DDCD) ) {
		if ( status & UART_MSR_DCD )
			wake_up_interruptible(&info->open_wait);
		else if ( !((info->flags & ASYNC_CALLOUT_ACTIVE) &&
			(info->flags & ASYNC_CALLOUT_NOHUP)) )

	        set_bit(MXUPCIE_EVENT_HANGUP,&info->event);
		MXQ_TASK();
	}

	if ( info->flags & ASYNC_CTS_FLOW ) {
		if ( info->tty->hw_stopped ) {
			if (status & UART_MSR_CTS ){
			    	info->tty->hw_stopped = 0;
		       		set_bit(MXUPCIE_EVENT_TXLOW,&info->event);
		       		MXQ_TASK();
	        	}
		} else {
			if ( !(status & UART_MSR_CTS) ){
			    	info->tty->hw_stopped = 1;
			}
		}
	}
}

static int mx_block_til_ready(struct tty_struct *tty, struct file * filp,
				 struct mxupcie_struct *info)
{
	DECLARE_WAITQUEUE(wait, current);
	int			retval;
	int			do_clocal = 0;
	MX_LOCK_INIT();

	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ( (filp->f_flags & O_NONBLOCK) || (tty->flags & (1 << TTY_IO_ERROR)) ) {
		if ( info->flags & ASYNC_CALLOUT_ACTIVE )
			return -EBUSY;

		info->flags |= ASYNC_NORMAL_ACTIVE;

		return 0;
	}

	if ( info->flags & ASYNC_CALLOUT_ACTIVE ) {
		if ( info->normal_termios.c_cflag & CLOCAL )
			do_clocal = 1;
	} else {
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))	
		if ( tty->termios->c_cflag & CLOCAL )
#else
		if ( tty->termios.c_cflag & CLOCAL )
#endif
			do_clocal = 1;
	}

	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * mxupcie_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
	
	MX_LOCK(&info->slock);
	if ( !tty_hung_up_p(filp) )
	    info->count--;
	MX_UNLOCK(&info->slock);

	info->blocked_open++;

	while ( 1 ) {
		MX_LOCK(&info->slock);
		if ( !(info->flags & ASYNC_CALLOUT_ACTIVE) )
			MX_WRITE_REG(MX_READ_REG(info->base + UART_MCR) | UART_MCR_DTR | UART_MCR_RTS,
		     info->base + UART_MCR);
		MX_UNLOCK(&info->slock);
		set_current_state(TASK_INTERRUPTIBLE);

		if ( tty_hung_up_p(filp) || !(info->flags & ASYNC_INITIALIZED) ) {
#ifdef SERIAL_DO_RESTART
			if ( info->flags & ASYNC_HUP_NOTIFY )
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;
#else
			retval = -EAGAIN;
#endif
			break;
		}

		if ( !(info->flags & ASYNC_CALLOUT_ACTIVE) && !(info->flags & ASYNC_CLOSING) &&
		 (do_clocal || (MX_READ_REG(info->base + UART_MSR) & UART_MSR_DCD)) )

			break;
		if ( signal_pending(current) ) {
			retval = -ERESTARTSYS;
			break;
		}

		schedule();
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&info->open_wait, &wait);

	if ( !tty_hung_up_p(filp) )
		info->count++;

	info->blocked_open--;

	if ( retval )
		return retval;

	info->flags |= ASYNC_NORMAL_ACTIVE;

	return 0;
}

static int mx_startup(struct mxupcie_struct * info)
{
	
	unsigned long page;
	unsigned char reg_flag;
	MX_LOCK_INIT();

	page = GET_FPAGE(GFP_KERNEL);
	if ( !page )
	    return -ENOMEM;

	MX_LOCK(&info->slock);

	if ( info->flags & ASYNC_INITIALIZED ) {
		free_page(page);
		MX_UNLOCK(&info->slock);

		return 0;
	}

	if ( !info->base || !info->type ) {
		if ( info->tty )
			set_bit(TTY_IO_ERROR, &info->tty->flags);
		free_page(page);
		MX_UNLOCK(&info->slock);

		return 0;
	}

	if ( info->xmit_buf )
		free_page(page);
	else
		info->xmit_buf = (unsigned char *)page;

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled in mx_change_speed())
	 */
	MX_WRITE_REG((UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT),info->base + UART_FCR);

	/*
	 * At this point there's no way the LSR could still be 0xFF;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */

	if ( MX_READ_REG(info->base + UART_LSR) == 0xff ) {
		MX_UNLOCK(&info->slock);

		if (capable(CAP_SYS_ADMIN)) {
			if ( info->tty )
				set_bit(TTY_IO_ERROR, &info->tty->flags);

			return(0);
		} else
			return -ENODEV;
	}

	/*
	 * Clear the interrupt registers.
	 */
	(void)MX_READ_REG(info->base + UART_LSR);
	(void)MX_READ_REG(info->base + UART_RX);
	(void)MX_READ_REG(info->base + UART_IIR);
	(void)MX_READ_REG(info->base + UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	MX_WRITE_REG(UART_LCR_WLEN8, info->base + UART_LCR);	/* reset DLAB */
	info->MCR = UART_MCR_DTR | UART_MCR_RTS;
	MX_WRITE_REG(info->MCR, info->base + UART_MCR);

	/*
	 * Initialize enhance mode register
	 */
	reg_flag = MOXA_EFR_ENHANCE;
	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_EFR);

	reg_flag = MOXA_SFR_950 | MOXA_SFR_ENABLE_TCNT;
	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_SFR);

	MX_WRITE_REG(info->xmit_fifo_size, info->base + MOXA_PUART_TTL);
	MX_WRITE_REG(info->rx_trigger, info->base + MOXA_PUART_RTL);
	MX_WRITE_REG(info->rx_low_water, info->base + MOXA_PUART_FCL);
	MX_WRITE_REG(info->rx_high_water, info->base + MOXA_PUART_FCH);

	/*
	 * Finally, enable interrupts
	 */
	info->IER = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;
	MX_WRITE_REG(info->IER, info->base + UART_IER); /* enable interrupts */

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void)MX_READ_REG(info->base + UART_LSR);
	(void)MX_READ_REG(info->base + UART_RX);
	(void)MX_READ_REG(info->base + UART_IIR);
	(void)MX_READ_REG(info->base + UART_MSR);

	if ( info->tty )
		test_and_clear_bit(TTY_IO_ERROR, &info->tty->flags);

	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	//mx_change_speed(info, 0);
	MX_UNLOCK(&info->slock);

	info->flags |= ASYNC_INITIALIZED;

	return 0;
}

/*
 * This routine will shutdown a serial port; interrupts maybe disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void mx_shutdown(struct mxupcie_struct * info)
{
	unsigned char reg_flag;
	int i;
	MX_LOCK_INIT();

	if ( !(info->flags & ASYNC_INITIALIZED) )
		return;

	MX_LOCK(&info->slock);

	/*
	 * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
	 * here so the queue might never be waken up
	 */
	wake_up_interruptible(&info->delta_msr_wait);

	/*
	 * Free the IRQ, if necessary
	 */
	if ( info->xmit_buf ) {
		free_page((unsigned long)info->xmit_buf);
		info->xmit_buf = 0;
	}

	reg_flag = 0;
	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_EFR);
	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_SFR);

	info->IER = 0;
	MX_WRITE_REG(0x00, info->base + UART_IER);
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	if ( !info->tty || (info->tty->termios->c_cflag & HUPCL) )
#else
	if ( !info->tty || (info->tty->termios.c_cflag & HUPCL) )
#endif
		info->MCR &= ~(UART_MCR_DTR | UART_MCR_RTS);
	MX_WRITE_REG(info->MCR, info->base + UART_MCR);

	if(info->speed < 9600) {
		int sleep_interval = 0;
		int reset_cnt = 0;

		if(info->speed <= 600) {
			sleep_interval = 10;
			reset_cnt = MX_FIFO_RESET_CNT;
		} else {
			sleep_interval = 1;
			reset_cnt = MX_FIFO_RESET_CNT / 10;
		}

		/* Workaround for clear FIFO in low baudrate */
		MX_WRITE_REG(0x0f, info->base+MOXA_PUART_ADJ_CLK);
		MX_WRITE_REG(0x03, info->base+MOXA_PUART_ADJ_ENABLE);

		/* clear Rx/Tx FIFO's */
		for(i = 0 ; i < reset_cnt;i++) {
			MX_WRITE_REG((UART_FCR_CLEAR_RCVR|UART_FCR_CLEAR_XMIT), info->base + UART_FCR);
			msleep(sleep_interval);
		}

		MX_WRITE_REG(0x00, info->base+MOXA_PUART_ADJ_CLK);
		MX_WRITE_REG(0x02, info->base+MOXA_PUART_ADJ_ENABLE);
	} else {
		/* clear Rx/Tx FIFO's */
		MX_WRITE_REG((UART_FCR_CLEAR_RCVR|UART_FCR_CLEAR_XMIT), info->base + UART_FCR);
	}
	/* read data port to reset things */
	(void)MX_READ_REG(info->base + UART_RX);

	if ( info->tty )
		set_bit(TTY_IO_ERROR, &info->tty->flags);

	info->flags &= ~ASYNC_INITIALIZED;

	MX_UNLOCK(&info->slock);
}

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
static int mx_change_speed(struct mxupcie_struct *info, 
                              struct ktermios *old_termios)
#else
static int mx_change_speed(struct mxupcie_struct *info,
                              struct termios *old_termios)
#endif
{
	unsigned	cflag, cval, fcr;
        int             ret = 0;
	long baud;
	int reg_flag;

#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	if ( !info->tty || !info->tty->termios )
		return ret;

	cflag = info->tty->termios->c_cflag;
#else	
	if ( !info->tty )
		return ret;

	cflag = info->tty->termios.c_cflag;
#endif

	if ( !(info->base) )
		return ret;

#ifndef B921600
#define B921600 (B460800 +1)
#endif
	switch( cflag & (CBAUD | CBAUDEX) ){
		case B4000000 : baud = info->speed; break;
        	case B921600 : info->speed = baud = 921600; break;
        	case B460800 : info->speed = baud = 460800; break;
        	case B230400 : info->speed = baud = 230400; break;
        	case B115200 : info->speed = baud = 115200; break;
        	case B57600 : info->speed = baud = 57600; break;
        	case B38400 : info->speed = baud = 38400;
			if((info->flags & ASYNC_SPD_MASK)==ASYNC_SPD_HI)
				info->speed = baud = 57600;
			else if((info->flags & ASYNC_SPD_MASK)==ASYNC_SPD_VHI)
				info->speed = baud = 115200;
#ifdef ASYNC_SPD_SHI
			else if((info->flags & ASYNC_SPD_MASK)==ASYNC_SPD_SHI)
				info->speed = baud = 230400;
#endif
#ifdef ASYNC_SPD_WARP
			else if((info->flags & ASYNC_SPD_MASK)==ASYNC_SPD_WARP)
				info->speed = baud = 460800;
#endif
#ifdef ASYNC_SPD_CUST
			else if((info->flags & ASYNC_SPD_MASK)==ASYNC_SPD_CUST){
				info->speed = baud = info->custom_baud_rate;
			}
#endif
			break;
        	case B19200 : info->speed = baud = 19200; break;
        	case B9600 : info->speed = baud = 9600; break;
        	case B4800 : info->speed = baud = 4800; break;
        	case B2400 : info->speed = baud = 2400; break;
        	case B1800 : info->speed = baud = 1800; break;
        	case B1200 : info->speed = baud = 1200; break;
        	case B600 : info->speed = baud = 600; break;
        	case B300 : info->speed = baud = 300; break;
        	case B200 : info->speed = baud = 200; break;
        	case B150 : info->speed = baud = 150; break;
        	case B134 : info->speed = baud = 134; break;
        	case B110 : info->speed = baud = 110; break;
        	case B75 : info->speed = baud = 75; break;
        	case B50 : info->speed = baud = 50; break;
        	default: info->speed = baud = 0; break;
	}

       	mx_set_baud(info, baud);

	/* byte size and parity */
	switch ( cflag & CSIZE ) {
		case CS5: cval = 0x00; break;
		case CS6: cval = 0x01; break;
		case CS7: cval = 0x02; break;
		case CS8: cval = 0x03; break;
		default:  cval = 0x00; break;	/* too keep GCC shut... */
	}

	if ( cflag & CSTOPB )
		cval |= 0x04;

	if ( cflag & PARENB )
		cval |= UART_LCR_PARITY;

#ifndef CMSPAR
#define	CMSPAR 010000000000
#endif
	if ( !(cflag & PARODD) )
		cval |= UART_LCR_EPAR;

	if ( cflag & CMSPAR )
		cval |= UART_LCR_SPAR;

	fcr = UART_FCR_ENABLE_FIFO;

	/* CTS flow control flag and modem status interrupts */
	info->IER &= ~UART_IER_MSI;
//	info->MCR &= ~UART_MCR_AFE;

	reg_flag = MX_READ_REG(info->base + MOXA_PUART_EFR);

	if ( cflag & CRTSCTS ) {
		info->flags |= ASYNC_CTS_FLOW;
		info->IER |= UART_IER_MSI;
		reg_flag |= (MOXA_EFR_AUTO_RTS | MOXA_EFR_AUTO_CTS);
	} else {
		info->flags &= ~ASYNC_CTS_FLOW;
		reg_flag &= ~(MOXA_EFR_AUTO_RTS | MOXA_EFR_AUTO_CTS);
	}

	MX_WRITE_REG(info->MCR, info->base + UART_MCR);

	if ( cflag & CLOCAL ){
		info->flags &= ~ASYNC_CHECK_CD;
	}else {
		info->flags |= ASYNC_CHECK_CD;
		info->IER |= UART_IER_MSI;
	}

	MX_WRITE_REG(info->IER, info->base + UART_IER);

	/*
	 * Set up parity check flag
	 */
	info->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;

	if ( I_INPCK(info->tty) )
		info->read_status_mask |= UART_LSR_FE | UART_LSR_PE;

	if ( I_BRKINT(info->tty) || I_PARMRK(info->tty) )
		info->read_status_mask |= UART_LSR_BI;

	info->ignore_status_mask = 0;

	if ( I_IGNBRK(info->tty) ) {
		info->ignore_status_mask |= UART_LSR_BI;
		info->read_status_mask |= UART_LSR_BI;
	    /*
	     * If we're ignore parity and break indicators, ignore
	     * overruns too.  (For real raw support).
	     */
		if ( I_IGNPAR(info->tty) ) {
			info->ignore_status_mask |= UART_LSR_OE|UART_LSR_PE|UART_LSR_FE;
			info->read_status_mask |= UART_LSR_OE|UART_LSR_PE|UART_LSR_FE;
		}
	}

	MX_WRITE_REG(START_CHAR(info->tty), info->base + MOXA_PUART_XON1);
	MX_WRITE_REG(START_CHAR(info->tty), info->base + MOXA_PUART_XON2);
	MX_WRITE_REG(STOP_CHAR(info->tty), info->base + MOXA_PUART_XOFF1);
	MX_WRITE_REG(STOP_CHAR(info->tty), info->base + MOXA_PUART_XOFF2);

	if ( I_IXON(info->tty) )
		reg_flag |= MOXA_EFR_TX_SW;
	else
		reg_flag &= ~MOXA_EFR_TX_SW;

	if ( I_IXOFF(info->tty) )
		reg_flag |= MOXA_EFR_RX_SW;
	else
		reg_flag &= ~MOXA_EFR_RX_SW;

	MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_EFR);
	MX_WRITE_REG(fcr, info->base + UART_FCR);		    /* set fcr */
	MX_WRITE_REG(cval, info->base + UART_LCR);  
	
        return ret;
}

static void write_div_scr(unsigned char *base, signed short div, unsigned char scr)
{
	unsigned char lcr;
	unsigned char oldsfr, sfr;

	oldsfr = MX_READ_REG(base + MOXA_PUART_SFR);
	sfr = oldsfr & (~(MOXA_SFR_ENABLE_TCNT));
	MX_WRITE_REG(sfr, base + MOXA_PUART_SFR);
	MX_WRITE_REG(scr, base + MOXA_PUART_SCR);
	MX_WRITE_REG(oldsfr, base + MOXA_PUART_SFR);
	
	lcr = MX_READ_REG(base + UART_LCR);
	MX_WRITE_REG(lcr | UART_LCR_DLAB, base + UART_LCR);  /* set DLAB */
	MX_WRITE_REG(div & 0xff, base + MOXA_PUART_LSB/*LSB*/);  
	MX_WRITE_REG((div & 0xff00) >> 8, base + MOXA_PUART_MSB/*MSB*/); 
	MX_WRITE_REG(lcr, base + UART_LCR);
}

static int set_linear_baud(unsigned char *base, long newspd)
{
	unsigned char scr, cpr;
	unsigned short div;
	int i, j, divisor = 0, sequence = 0;	
        int M , N = 0, SCR = 0;
	int set_value, min, ret_value, accuracy;
	
	M = MIN_CPRM;
	set_value = newspd;
	
	min = FREQUENCY;
    	ret_value = 0;

	for (i = 1; i <= MAXDIVISOR; i++) {
		for (j = MINSEQUENCE; j <= MAXSEQUENCE; ) {
			if (FREQUENCY / (i * j) > set_value) {
                  		accuracy = (FREQUENCY / (i * j)) - set_value;
            		} else {
                  		accuracy = set_value - (FREQUENCY / (i * j));
            		}
                        if (min > accuracy) {
                        	min = accuracy;
                        	ret_value = (FREQUENCY / (i * j));
                        	divisor = i;
                        	sequence = j;
                        }

            		if (j <= MAXSEQUENCE / 2) {
                		j += 1;
            		} else {
                		j += 2;
            		}
     		}	
     	}
	if ((min * 100) / (set_value * 100) <= 3) {
        	if (sequence > (MAXSEQUENCE / 2)) {
              		M = MAX_CPRM;
              		sequence /= 2;
         	}
        	for (i = MAX_SCR; i >= MIN_SCR; i--) {
            		for (j = MIN_CPRN; j <= MAX_CPRN; j++) {
                  		if ((16 - i + j) == sequence) {
                       			SCR = i;
                       			N = j;
                  		}
            		}
         	}
	}
	scr = (unsigned char) SCR;
	div = (unsigned short) divisor;
	cpr = (M << 3) + N;

	MX_WRITE_REG(MX_READ_REG(base + UART_MCR) | 0x80, base + UART_MCR);
	MX_WRITE_REG(cpr, base + MOXA_PUART_CPR);
	write_div_scr(base, div, scr);

	return 0;
}
static int mx_set_baud(struct mxupcie_struct *info, long newspd)
{
	int		i;
	int		quot = 0;
	unsigned char	cval;
        int             ret = 0;
#if (LINUX_VERSION_CODE < VERSION_CODE(3,7,0))
	if ( !info->tty || !info->tty->termios )
		return ret;
#else
	if ( !info->tty )
		return ret;
#endif
	if ( !(info->base) )
		return ret;

	if ( newspd > info->MaxCanSetBaudRate )
		return 0;

	info->realbaud = newspd;

	for ( i=0; i<BAUD_TABLE_NO && newspd != mxvar_baud_table[i]; i++ );

	if ( i == BAUD_TABLE_NO ){
		set_linear_baud(info->base, newspd);
		quot = info->baud_base / info->speed;	
		if ( info->speed <= 0 || info->speed > info->MaxCanSetBaudRate )
			quot = 0;
	}else{
		if ( newspd == 134 ) {
			quot = (2 * info->baud_base / 269);
			info->speed = 134;
		} else if ( newspd ) {
			quot = info->baud_base / newspd;
			/* info->baud_base = 921600 */	
			if(quot==0)
				quot = 1;
		} else {
			quot = 0;
		}
	}

	info->timeout = (int)((unsigned int)(info->xmit_fifo_size*HZ*10*quot) / (unsigned int)info->baud_base);
	info->timeout += HZ/50;		/* Add .02 seconds of slop */

	if ( quot ) {
		info->MCR |= UART_MCR_DTR;
		MX_WRITE_REG(info->MCR, info->base + UART_MCR);
	} else {
		info->MCR &= ~UART_MCR_DTR;
		MX_WRITE_REG(info->MCR, info->base + UART_MCR);

		return ret;
	}
	if (i != BAUD_TABLE_NO) {
		cval = MX_READ_REG(info->base + UART_LCR);
		MX_WRITE_REG(cval | UART_LCR_DLAB, info->base + UART_LCR);  /* set DLAB */
		MX_WRITE_REG(quot & 0xff, info->base + UART_DLL);	    /* LS of divisor */
		MX_WRITE_REG(quot >> 8, info->base + UART_DLM); 	    /* MS of divisor */
		MX_WRITE_REG(cval, info->base + UART_LCR);		    /* reset DLAB */
//		MX_WRITE_REG(0x00, info->base + MOXA_PUART_SCR);
		MX_WRITE_REG(0x08, info->base + MOXA_PUART_CPR);
	}
#if 0
	if ( i == BAUD_TABLE_NO ){
		quot = info->baud_base % info->speed;
		quot *= 8;
		if ( (quot % info->speed) > (info->speed / 2) ) {
			quot /= info->speed;
			quot++;
		} else {
			quot /= info->speed;
		}
	}
#endif
    return ret;
}



/*
 * ------------------------------------------------------------
 * friends of mxupcie_ioctl()
 * ------------------------------------------------------------
 */
static int mx_get_serial_info(struct mxupcie_struct * info,
				 struct serial_struct * retinfo)
{
	struct serial_struct	tmp;

	if ( !retinfo )
		return(-EFAULT);

	memset(&tmp, 0, sizeof(tmp));
	tmp.type = info->type;
	tmp.line = info->port;
	tmp.port = *info->base;
	tmp.irq = info->irq;
	tmp.flags = info->flags;
	tmp.baud_base = info->baud_base;
	tmp.close_delay = info->close_delay;
	tmp.closing_wait = info->closing_wait;
	tmp.custom_divisor = info->custom_divisor;
	tmp.hub6 = 0;

	if(copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;

	return 0;
}

static int mx_set_serial_info(struct mxupcie_struct * info,
				 struct serial_struct * new_info)
{
	struct serial_struct	new_serial;
	unsigned int		flags;
	int			retval = 0;

	MX_LOCK_INIT();

	if ( !new_info || !info->base )
		return -EFAULT;

	if(copy_from_user(&new_serial, new_info, sizeof(new_serial)))
		return -EFAULT;

	if ( (new_serial.irq != info->irq) ||
	     (new_serial.port != *info->base) )
		return -EPERM;

	flags = info->flags & ASYNC_SPD_MASK;

       if ( !capable(CAP_SYS_ADMIN)) {
		if ( (new_serial.baud_base != info->baud_base) ||
		     (new_serial.close_delay != info->close_delay) ||
		    ((new_serial.flags & ~ASYNC_USR_MASK) !=
		    (info->flags & ~ASYNC_USR_MASK)) )
			return(-EPERM);

		info->flags = ((info->flags & ~ASYNC_USR_MASK) | (new_serial.flags & ASYNC_USR_MASK));
	} else {
	    /*
	     * OK, past this point, all the error checking has been done.
	     * At this point, we start making changes.....
	     */
		info->flags = ((info->flags & ~ASYNC_FLAGS) | (new_serial.flags & ASYNC_FLAGS));
		info->close_delay = new_serial.close_delay * HZ/100;
		info->closing_wait = new_serial.closing_wait * HZ/100;
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,1,0))        
#if (LINUX_VERSION_CODE < VERSION_CODE(3,9,0))        
#if (LINUX_VERSION_CODE < VERSION_CODE(3,2,0))		
		info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;
#else
		info->tty->low_latency = 0;                
#endif
#else
		info->ttyPort.low_latency = 0;
#endif
#endif

		if( (new_serial.baud_base != info->baud_base) ||
		    (new_serial.custom_divisor != info->custom_divisor) )
			info->custom_baud_rate = new_serial.baud_base/new_serial.custom_divisor;
	}

	info->type = new_serial.type;
	
	mx_process_txrx_fifo(info);

	if ( info->flags & ASYNC_INITIALIZED ) {
		if ( flags != (info->flags & ASYNC_SPD_MASK) ){
			MX_LOCK(&info->slock);
			mx_change_speed(info,0);
			MX_UNLOCK(&info->slock);
		}
	} else{
	    retval = mx_startup(info);
	}

	return retval;
}

/*
 * mx_get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 *	    is emptied.  On bus types like RS485, the transmitter must
 *	    release the bus after transmitting. This must be done when
 *	    the transmit shift register is empty, not be done when the
 *	    transmit holding register is empty.  This functionality
 *	    allows an RS485 driver to be written in user space.
 */
static int mx_get_lsr_info(struct mxupcie_struct * info, unsigned int *value)
{
	unsigned char	status;
	unsigned int	result;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	status = MX_READ_REG(info->base + UART_LSR);
	MX_UNLOCK(&info->slock);
	result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	put_to_user(result, value);

	return 0;
}

/*
 * This routine sends a break character out the serial port.
 */
static void mx_send_break(struct mxupcie_struct * info, int duration)
{
	MX_LOCK_INIT();

	if ( !info->base )
		return;

	set_current_state(TASK_INTERRUPTIBLE);

	MX_LOCK(&info->slock);
	switch(info->UIR){
		case MOXA_UIR_RS485_4W:
		case MOXA_UIR_RS485_2W:
			mx_software_break_signal(info, MX_BREAK_ON);
			break;
		case MOXA_UIR_RS232:
		case MOXA_UIR_RS422:
			MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) | UART_LCR_SBC, info->base + UART_LCR);
			break;
	}
	MX_UNLOCK(&info->slock);

	schedule_timeout(duration);

	MX_LOCK(&info->slock);
	switch(info->UIR){
		case MOXA_UIR_RS485_4W:
		case MOXA_UIR_RS485_2W:
			mx_software_break_signal(info, MX_BREAK_OFF);
			break;
		case MOXA_UIR_RS232:
		case MOXA_UIR_RS422:
			MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) & ~UART_LCR_SBC, info->base + UART_LCR);
			break;
	}
	MX_UNLOCK(&info->slock);
	
	set_current_state(TASK_RUNNING);
}

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,39))
static int mxupcie_tiocmget(struct tty_struct *tty)
#else
static int mxupcie_tiocmget(struct tty_struct *tty, struct file *file)
#endif
{
	struct mxupcie_struct *info = (struct mxupcie_struct *) tty->driver_data;
	unsigned char control, status;
	MX_LOCK_INIT();
//printk("%lu,tiocmget\n", jiffies);

	if (PORTNO(tty) == MXUPCIE_PORTS)
		return -ENOIOCTLCMD;

	if (tty->flags & (1 << TTY_IO_ERROR))
		return -EIO;

	control = info->MCR;
	
	MX_LOCK(&info->slock);
	status = MX_READ_REG(info->base + UART_MSR);

	if (status & UART_MSR_ANY_DELTA)
		mx_check_modem_status(info, status);

	MX_UNLOCK(&info->slock);

	return ((control & UART_MCR_RTS) ? TIOCM_RTS : 0) |
	    ((control & UART_MCR_DTR) ? TIOCM_DTR : 0) |
	    ((status & UART_MSR_DCD) ? TIOCM_CAR : 0) |
	    ((status & UART_MSR_RI) ? TIOCM_RNG : 0) |
	    ((status & UART_MSR_DSR) ? TIOCM_DSR : 0) |
	    ((status & UART_MSR_CTS) ? TIOCM_CTS : 0);
}


#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,39))
static int mxupcie_tiocmset(struct tty_struct *tty,
			  unsigned int set, unsigned int clear)
#else
static int mxupcie_tiocmset(struct tty_struct *tty, struct file *file,
			  unsigned int set, unsigned int clear)
#endif			  
{
	struct mxupcie_struct *info = (struct mxupcie_struct *) tty->driver_data;
	MX_LOCK_INIT();


//printk("%lu,tiocmset\n", jiffies);

	if (PORTNO(tty) == MXUPCIE_PORTS)
		return (-ENOIOCTLCMD);

	if (tty->flags & (1 << TTY_IO_ERROR))
		return (-EIO);

	MX_LOCK(&info->slock);

	if (set & TIOCM_RTS)
		info->MCR |= UART_MCR_RTS;
	if (set & TIOCM_DTR)
		info->MCR |= UART_MCR_DTR;
	if (clear & TIOCM_RTS)
		info->MCR &= ~UART_MCR_RTS;
	if (clear & TIOCM_DTR)
		info->MCR &= ~UART_MCR_DTR;

	MX_WRITE_REG(info->MCR, info->base + UART_MCR);
	MX_UNLOCK(&info->slock);

	return 0;
}

#else
static int mx_get_modem_info(struct mxupcie_struct * info,
				unsigned int *value)
{
	unsigned char	control, status;
	unsigned int	result;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	control = info->MCR;
	status = mxupcie_get_msr(info->base, 0, info->port, info);
	MX_UNLOCK(&info->slock);

	if ( status & UART_MSR_ANY_DELTA )
		mx_check_modem_status(info, status);

	result = ((control & UART_MCR_RTS) ? TIOCM_RTS : 0) |
		 ((control & UART_MCR_DTR) ? TIOCM_DTR : 0) |
		 ((status  & UART_MSR_DCD) ? TIOCM_CAR : 0) |
		 ((status  & UART_MSR_RI)  ? TIOCM_RNG : 0) |
		 ((status  & UART_MSR_DSR) ? TIOCM_DSR : 0) |
		 ((status  & UART_MSR_CTS) ? TIOCM_CTS : 0);
	put_to_user(result, value);

	return 0;
}


static int mx_set_modem_info(struct mxupcie_struct * info, unsigned int cmd,
				unsigned int *value)
{
	int		error;
	unsigned int	arg;

	error = MX_ACCESS_CHK(VERIFY_READ, value, sizeof(int));

	if ( MX_ERR(error) )
		return(error);

	get_from_user(arg,value);

	switch ( cmd ) {
		case TIOCMBIS:
			if ( arg & TIOCM_RTS )
				info->MCR |= UART_MCR_RTS;
			if ( arg & TIOCM_DTR )
				info->MCR |= UART_MCR_DTR;
			break;
		case TIOCMBIC:
			if ( arg & TIOCM_RTS )
				info->MCR &= ~UART_MCR_RTS;
			if ( arg & TIOCM_DTR )
				info->MCR &= ~UART_MCR_DTR;
			break;
		case TIOCMSET:
			info->MCR = ((info->MCR & ~(UART_MCR_RTS | UART_MCR_DTR)) |
			((arg & TIOCM_RTS) ? UART_MCR_RTS : 0) |
			((arg & TIOCM_DTR) ? UART_MCR_DTR : 0));
			break;
		default:
			return -EINVAL;
	}

	MX_WRITE_REG(info->MCR, info->base + UART_MCR);

	return 0;
}
#endif

static int mx_set_interface(struct mxupcie_struct *info, unsigned char val)
{
	unsigned char intf = 0, chip_val = 0;
	gpio_param_t   cpld;
	int port_idx = 0;

	switch (val) {
	case MOXA_UIR_RS232:
		switch (info->board_type) {
		case MXUPCIE_BOARD_CP132EL:
		case MXUPCIE_BOARD_CP138E_A:
		case MXUPCIE_BOARD_CP134EL_A:                    
			return -EINVAL;
		case MXUPCIE_BOARD_CP118E_A_I:
		case MXUPCIE_BOARD_CP116E_A_A:
		case MXUPCIE_BOARD_CP116E_A_B:
			port_idx = info->port % 8;
			cpld.base = (unsigned char *)info->iobar3_addr;
			if(mxCPLDSetTerminator(cpld, port_idx, MX_TERM_NONE) < 0)
				return -EINVAL;
			break;
		default:
			if(mx_set_terminator(info, MX_TERM_NONE) < 0)
				return -EINVAL;
			break;
		}		
	case MOXA_UIR_RS422:
	case MOXA_UIR_RS485_4W:
	case MOXA_UIR_RS485_2W:
		switch (info->board_type) {
		case MXUPCIE_BOARD_CP102E:
		case MXUPCIE_BOARD_CP102EL:
		case MXUPCIE_BOARD_CP168EL_A:
		case MXUPCIE_BOARD_CP104EL_A:
			return -EINVAL;
		default:
			break;
		}

		info->UIR = val;
		chip_val = MX_READ_IOBAR3_REG(info->UIR_addr);
	
		if (info->port % 2) {
			intf = val << MOXA_UIR_EVEN_PORT_VALUE_OFFSET;
			chip_val &= 0x0F;
			chip_val |= intf;
			
		} else {
			intf = val;
			chip_val &= 0xF0;
			chip_val |= intf;
		}


		MX_WRITE_IOBAR3_REG(chip_val, info->UIR_addr);
		break;			
	default:
		return -EINVAL;
	}

	return 0;
}

static void mx_init_terminator(struct mxupcie_struct *info)
{
	switch(info->board_type){
		case MXUPCIE_BOARD_CP118EL_A://422 or 485 and 8port
			break;
		case MXUPCIE_BOARD_CP132EL:  // 2port		
		case MXUPCIE_BOARD_CP114EL:  // 4&8 port
		case MXUPCIE_BOARD_CP104EL_A://232 Only
		case MXUPCIE_BOARD_CP102E:   //232 Only
		case MXUPCIE_BOARD_CP102EL:  //232 Only
		case MXUPCIE_BOARD_CP168EL_A://232 Only
		default:
			return ;
		}
		
	MX_WRITE_IOBAR3_REG(0xff, info->iobar3_addr + \
		MOXA_PUART_GPIO_EN);
	MX_WRITE_IOBAR3_REG(0x00, info->iobar3_addr + \
		MOXA_PUART_GPIO_OUT);
}


static int mx_set_terminator(struct mxupcie_struct *info, unsigned char val)
{
	unsigned char chip_val = 0;

	if(info->UIR == MOXA_UIR_RS232)
		return -EINVAL;

	switch(val){
		case MX_TERM_NONE:
		case MX_TERM_120:

			info->terminator_flag = val;
			chip_val = MX_READ_IOBAR3_REG(info->iobar3_addr + 
					MOXA_PUART_GPIO_IN);
			switch(info->board_type){
				case MXUPCIE_BOARD_CP132EL: //2port		
					chip_val &= ~(1<<(info->port+2));
					chip_val |= (val << (info->port+2));
					break;
				case MXUPCIE_BOARD_CP114EL: // 4&8 port
				case MXUPCIE_BOARD_CP118EL_A:
					chip_val &= ~(1<<info->port);
					chip_val |= (val << info->port);
					break;
				case MXUPCIE_BOARD_CP104EL_A:
				case MXUPCIE_BOARD_CP102E: //232 Only
				case MXUPCIE_BOARD_CP102EL:
				case MXUPCIE_BOARD_CP168EL_A:
				default:
					return -EINVAL;
			}
		
			MX_WRITE_IOBAR3_REG(0xff, info->iobar3_addr + \
				MOXA_PUART_GPIO_EN);
			MX_WRITE_IOBAR3_REG(chip_val, info->iobar3_addr + \
				MOXA_PUART_GPIO_OUT);

			break;
		default:
			return -EINVAL;
	}
	
	return 0;
}


static void  mx_software_break_signal(struct mxupcie_struct *info, unsigned char state)
{
	unsigned char cval,reg_flag;
	unsigned char tx_byte=0x01;
	int origin_speed;

	origin_speed = info->speed;

	if(state == MX_BREAK_ON){
		cval = MX_READ_REG(info->base + UART_LCR);
		MX_WRITE_REG(cval | UART_LCR_DLAB, info->base + UART_LCR);
		MX_WRITE_REG(0, info->base + UART_DLL);
		MX_WRITE_REG(0, info->base + UART_DLM);
		MX_WRITE_REG(cval, info->base + UART_LCR);

		memcpy(info->base + MOXA_PUART_MEMTHR,&tx_byte,1);
			
		reg_flag = MX_READ_REG(info->base + MOXA_PUART_SFR);
		reg_flag |= MOXA_SFR_FORCE_TX;
		MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_SFR);

		MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) | UART_LCR_SBC, info->base + UART_LCR);
	}

	if(state == MX_BREAK_OFF){
		MX_WRITE_REG(MX_READ_REG(info->base + UART_LCR) & ~UART_LCR_SBC, info->base + UART_LCR);

		reg_flag = MX_READ_REG(info->base + MOXA_PUART_SFR);
		reg_flag &= ~MOXA_SFR_FORCE_TX;
		MX_WRITE_REG(reg_flag, info->base + MOXA_PUART_SFR);

		MX_WRITE_REG(UART_FCR_CLEAR_XMIT, info->base + UART_FCR);

		mx_set_baud(info, origin_speed);
	}
}

static void mx_process_txrx_fifo(struct mxupcie_struct *info)
{
	
	if ( (info->type == PORT_16450) || (info->type == PORT_8250) ){
		info->rx_trigger = 1;
		info->rx_high_water = 1;
		info->rx_low_water = 1;
		info->xmit_fifo_size = 1;
	}else{
		info->rx_trigger = MOXA_RTL_96;
		info->rx_high_water = MOXA_FCH_110;
		info->rx_low_water = MOXA_FCL_16;
		info->xmit_fifo_size = MX_TX_FIFO_SIZE;
	}
}



module_init(mxupcie_module_init);
module_exit(mxupcie_module_exit); 
