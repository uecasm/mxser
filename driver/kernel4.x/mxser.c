/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mxser.c
*/

#ifdef 		MODVERSIONS
#ifndef 	MODULE
#define 	MODULE
#endif
#endif

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "../../mx_ver.h"

#include <linux/version.h>
#define	VERSION_CODE(ver,rel,seq)	((ver << 16) | (rel << 8) | seq)

#ifdef MODULE
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#include <linux/module.h>
#else
#define	MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
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
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/segment.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#include "mxser.h"

#ifdef CONFIG_PCI
	#include <linux/pci.h>
#endif

#define	MXSER_VERSION	MX_SER_VERSION	
#define	MXSERMAJOR	 30
#define	MXSERCUMAJOR	 35

#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif /* ENABLE_PCI */

#include <asm/uaccess.h>
#define put_to_user(arg1, arg2) put_user(arg1, (unsigned int *)arg2)
#define get_from_user(arg1, arg2) get_user(arg1, (unsigned int *)arg2)

#define	MXSER_EVENT_TXLOW	 1
#define	MXSER_EVENT_HANGUP	 2

#define SERIAL_DO_RESTART
#define MXSER_BOARDS		4	/* Max. boards */
#define MXSER_PORTS		32	/* Max. ports */
#define MXSER_PORTS_PER_BOARD	8	/* Max. ports per board*/
#define MXSER_ISR_PASS_LIMIT	99999L	

#define	MXSER_ERR_IOADDR	-1
#define	MXSER_ERR_IRQ		-2
#define	MXSER_ERR_IRQ_CONFLIT	-3
#define	MXSER_ERR_VECTOR	-4

#define SERIAL_TYPE_NORMAL	1
#define SERIAL_TYPE_CALLOUT	2

#define WAKEUP_CHARS		256

#define UART_MCR_AFE		0x20
#define UART_LSR_SPECIAL	0x1E

#define MX_LOCK_INIT()		unsigned long sp_flags=0

#define MX_LOCK(lock)		{\
			if(!in_interrupt())\
				spin_lock_irqsave(lock, sp_flags);\
			}
#define MX_UNLOCK(lock)		{\
			if(!in_interrupt())\
				spin_unlock_irqrestore(lock, sp_flags);\
			}

#define PORTNO(x)	((x)->index)

#define RELEVANT_IFLAG(iflag)	(iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|IXON|IXOFF))

#define IRQ_T(info) ((info->flags & ASYNC_SHARE_IRQ) ? IRQF_SHARED : IRQF_TRIGGER_NONE)

#ifndef MIN
	#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

/*
 *	Define the Moxa PCI vendor and device IDs.
 */
#ifndef	PCI_VENDOR_ID_MOXA
#define	PCI_VENDOR_ID_MOXA	0x1393
#endif

#ifndef PCI_DEVICE_ID_C168
#define PCI_DEVICE_ID_C168	0x1680
#endif

#ifndef PCI_DEVICE_ID_C104
#define PCI_DEVICE_ID_C104	0x1040
#endif

#ifndef PCI_DEVICE_ID_CP132
#define PCI_DEVICE_ID_CP132	0x1320
#endif

#ifndef PCI_DEVICE_ID_CP114
#define PCI_DEVICE_ID_CP114	0x1141
#endif

#ifndef PCI_DEVICE_ID_CT114
#define PCI_DEVICE_ID_CT114	0x1140
#endif

#ifndef PCI_DEVICE_ID_CP102
#define PCI_DEVICE_ID_CP102	0x1020
#endif

#ifndef PCI_DEVICE_ID_CP104U
#define PCI_DEVICE_ID_CP104U	0x1041
#endif

#ifndef PCI_DEVICE_ID_CP168U
#define PCI_DEVICE_ID_CP168U	0x1681
#endif

#ifndef PCI_DEVICE_ID_CP132U
#define PCI_DEVICE_ID_CP132U	0x1321
#endif

#ifndef PCI_DEVICE_ID_CP134U
#define PCI_DEVICE_ID_CP134U	0x1340
#endif

#ifndef PCI_DEVICE_ID_CP104JU
#define PCI_DEVICE_ID_CP104JU	0x1042
#endif

#ifndef PCI_DEVICE_ID_CP118U
#define PCI_DEVICE_ID_CP118U	0x1180
#endif

#ifndef PCI_DEVICE_ID_CP102UL
#define PCI_DEVICE_ID_CP102UL	0x1021
#endif

#ifndef PCI_DEVICE_ID_CP102U
#define PCI_DEVICE_ID_CP102U	0x1022
#endif

#ifndef PCI_DEVICE_ID_CP118EL
#define PCI_DEVICE_ID_CP118EL	0x1181
#endif

#ifndef PCI_DEVICE_ID_CP168EL
#define PCI_DEVICE_ID_CP168EL	0x1682
#endif

#ifndef PCI_DEVICE_ID_CP104EL
#define PCI_DEVICE_ID_CP104EL	0x1043
#endif

#ifndef PCI_DEVICE_ID_RC7000
#define PCI_DEVICE_ID_RC7000	0x0001
#endif

#ifndef	PCI_DEVICE_ID_CB108
#define	PCI_DEVICE_ID_CB108	0x1080
#endif

#ifndef	PCI_DEVICE_ID_CB114
#define	PCI_DEVICE_ID_CB114	0x1142
#endif

#ifndef	PCI_DEVICE_ID_CB134I
#define	PCI_DEVICE_ID_CB134I	0x1341
#endif

#ifndef	PCI_DEVICE_ID_CP138U
#define	PCI_DEVICE_ID_CP138U	0x1380
#endif

#ifndef	PCI_DEVICE_ID_POS104UL
#define	PCI_DEVICE_ID_POS104UL	0x1044
#endif

#ifndef	PCI_DEVICE_ID_CP114UL
#define	PCI_DEVICE_ID_CP114UL	0x1143
#endif

#ifndef	PCI_DEVICE_ID_CP102UF
#define	PCI_DEVICE_ID_CP102UF	0x1023
#endif

#ifndef	PCI_DEVICE_ID_CP112UL
#define	PCI_DEVICE_ID_CP112UL	0x1120
#endif

#define C168_ASIC_ID    1
#define C104_ASIC_ID    2
#define C102_ASIC_ID	0xB
#define CI132_ASIC_ID	4
#define CI134_ASIC_ID	3
#define CI104J_ASIC_ID  5

/* PC104 series */
#define CA104_ID    	1
#define CA132_ID    	2
#define CA132I_ID    	3
#define	CA108_ID    	4
#define	CA114_ID    	5
#define	CA134I_ID    	6

enum	{
	MXSER_BOARD_C168_ISA = 1,
	MXSER_BOARD_C104_ISA,
	MXSER_BOARD_CI104J,
	MXSER_BOARD_C168_PCI,
	MXSER_BOARD_C104_PCI,	/* 5 */
	MXSER_BOARD_C102_ISA,
	MXSER_BOARD_CI132,
	MXSER_BOARD_CI134,
	MXSER_BOARD_CP132,
	MXSER_BOARD_CP114,		/* 10 */
	MXSER_BOARD_CT114,
	MXSER_BOARD_CP102,
	MXSER_BOARD_CP104U,
	MXSER_BOARD_CP168U,
	MXSER_BOARD_CP132U,		/* 15 */
	MXSER_BOARD_CP134U,
	MXSER_BOARD_CP104JU,
	MXSER_BOARD_RC7000,
	MXSER_BOARD_CP118U,
	MXSER_BOARD_CP102UL,	/* 20 */
	MXSER_BOARD_CP102U,
	MXSER_BOARD_CP118EL,
	MXSER_BOARD_CP168EL,
	MXSER_BOARD_CP104EL,
	MXSER_BOARD_CB108,		/* 25 */
	MXSER_BOARD_CB114,
	MXSER_BOARD_CB134I,
	MXSER_BOARD_CP138U,
	MXSER_BOARD_POS104UL,
	MXSER_BOARD_CP114UL,	/* 30 */
	MXSER_BOARD_CP102UF,
	MXSER_BOARD_CP112UL,
	/*PC104 series*/
	MXSER_PC_BOARD_CA104,
	MXSER_PC_BOARD_CA132,
	MXSER_PC_BOARD_CA132I,	/* 35 */
	MXSER_PC_BOARD_CA108,
	MXSER_PC_BOARD_CA114,
	MXSER_PC_BOARD_CA134I
};

static char *mxser_brdname[] = {
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
	"CP-114UL series",
	"CP-102UF series",
	"CP-112UL series",
	/* PC104 series*/
	"CA-104 series",
	"CA-132 series",
	"CA-132I series",
	"CA-108 series",
	"CA-114 series",
	"CA-134I series"
};

static int mxser_numports[] = {
	8,	// C168-ISA
	4,	// C104-ISA
	4,	// CI104J
	8,	// C168-PCI
	4,	// C104-PCI
	2,	// C102-ISA
	2,	// CI132
	4,	// CI134
	2,	// CP132
	4,	// CP114
	4,	// CT114
	2,	// CP102
	4,	// CP104U
	8,	// CP168U
	2,	// CP132U
	4,	// CP134U
	4,	// CP104JU
	8,	// RC7000
	8,	// CP118U 
	2,	// CP102UL 
	2,	// CP102U
	8,	// CP118EL 
	8,	// CP168EL 
	4,	// CP104EL
	8,	//CB-108
	4,	//CB-114
	4,	//CB-134I
	8,	//CP-138U
	4,		//POS-104UL
	4,		//CP-114UL
	2,	//CP-102UF
	2,	//CP-112UL
	/* PC104 series */
	4,	// CA104
	2,	// CA132
	2,	// CA132I
	8,	// CA108
	4,	// CA114
	4		// CA134I
};

/*
 *	MOXA ioctls
 */
#define MOXA			0x400
#define MOXA_GETDATACOUNT     (MOXA + 23)
#define	MOXA_GET_CONF         (MOXA + 35)
#define MOXA_DIAGNOSE         (MOXA + 50)
#define MOXA_CHKPORTENABLE    (MOXA + 60)
#define MOXA_HighSpeedOn      (MOXA + 61)
#define MOXA_GET_MAJOR        (MOXA + 63)
#define MOXA_GET_CUMAJOR      (MOXA + 64)
#define MOXA_GETMSTATUS       (MOXA + 65)

// following add by Victor Yu. 01-05-2004
#define MOXA_SET_OP_MODE      (MOXA + 66)
#define MOXA_GET_OP_MODE      (MOXA + 67)

#define RS232_MODE		0
#define RS485_2WIRE_MODE	1
#define RS422_MODE		2
#define RS485_4WIRE_MODE	3
#define OP_MODE_MASK		3
// above add by Victor Yu. 01-05-2004

#define TTY_THRESHOLD_THROTTLE  128

// added by James. 03-11-2004.
#define MOXA_SDS_GETICOUNTER  (MOXA + 68)
#define MOXA_SDS_RSTICOUNTER  (MOXA + 69)
// (above) added by James.

#define MOXA_ASPP_OQUEUE  (MOXA + 70)
#define MOXA_ASPP_SETBAUD (MOXA + 71)
#define MOXA_ASPP_GETBAUD (MOXA + 72)
#define MOXA_ASPP_MON     (MOXA + 73)
#define MOXA_ASPP_LSTATUS (MOXA + 74)
#define MOXA_ASPP_MON_EXT (MOXA + 75)
#define MOXA_SET_BAUD_METHOD	(MOXA + 76)
#define MOXA_SET_SPECIAL_BAUD_RATE	(MOXA+100)
#define MOXA_GET_SPECIAL_BAUD_RATE	(MOXA+101)
#define SMARTIO_SET_SPECIAL_BAUD_RATE	(MOXA+77)
#define SMARTIO_GET_SPECIAL_BAUD_RATE	(MOXA+78)

#define NPPI_NOTIFY_PARITY	0x01
#define NPPI_NOTIFY_FRAMING	0x02
#define NPPI_NOTIFY_HW_OVERRUN	0x04
#define NPPI_NOTIFY_SW_OVERRUN	0x08
#define NPPI_NOTIFY_BREAK	0x10

#define NPPI_NOTIFY_CTSHOLD         0x01    // Tx hold by CTS low
#define NPPI_NOTIFY_DSRHOLD         0x02    // Tx hold by DSR low
#define NPPI_NOTIFY_XOFFHOLD        0x08    // Tx hold by Xoff received
#define NPPI_NOTIFY_XOFFXENT        0x10    // Xoff Sent

#define UART_TYPE_NUM	2
unsigned int Gmoxa_uart_id[UART_TYPE_NUM]= {
	MOXA_MUST_MU150_HWID,
	MOXA_MUST_MU860_HWID
};


// This is only for PCI
#define UART_INFO_NUM	3
struct mxpciuart_info{
	int	type;
	int	tx_fifo;
	int	rx_fifo;
	int	xmit_fifo_size;
	int	rx_high_water;
	int	rx_trigger;
	int	rx_low_water;
	long	max_baud;
};

struct mxpciuart_info Gpci_uart_info[UART_INFO_NUM] = {
	{MOXA_OTHER_UART, 16, 16, 16, 14, 14, 1, 921600L},
	{MOXA_MUST_MU150_HWID, 64, 64, 64, 48, 48, 16, 230400L},
	{MOXA_MUST_MU860_HWID, 128, 128, 128, 96, 96, 32, 921600L}
};


#ifdef CONFIG_PCI

#ifndef PCI_ANY_ID
#define PCI_ANY_ID (~0)
#endif

static  struct pci_device_id	mxser_pcibrds[] = {
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_C168 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_C168_PCI},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_C104 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_C104_PCI},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP132 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP132},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP114 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP114},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CT114 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CT114},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP102},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP104U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP104U},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP168U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP168U},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP132U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP132U},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP134U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP134U},
	{PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP104JU ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP104JU},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_RC7000 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_RC7000},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP118U},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102UL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP102UL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP102U},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP118EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP118EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP168EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP168EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP104EL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP104EL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CB108 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CB108},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CB114 ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CB114},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CB134I ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CB134I},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP138U ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP138U},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_POS104UL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_POS104UL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP114UL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP114UL},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP102UF ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP102UF},
        {PCI_VENDOR_ID_MOXA,PCI_DEVICE_ID_CP112UL ,PCI_ANY_ID, PCI_ANY_ID, 0, 0,MXSER_BOARD_CP112UL},
	{0}
};

MODULE_DEVICE_TABLE(pci, mxser_pcibrds);

#endif

typedef struct _moxa_pci_info {
	unsigned short busNum;
	unsigned short devNum;
struct pci_dev	*pdev;	// add by Victor Yu. 06-23-2003
} moxa_pci_info;

typedef struct __attribute__((__packed__)) _moxa_pci_usr_info{
	unsigned short busNum;
	unsigned short devNum;
} moxa_pci_usr_info;

static int ioaddr[MXSER_BOARDS]={0,0,0,0};
static int iovect[MXSER_BOARDS]={0,0,0,0};
static int irq[MXSER_BOARDS]={0,0,0,0};
static int ttymajor=MXSERMAJOR;
static int calloutmajor=MXSERCUMAJOR;
static int verbose=0;

#ifdef MODULE
/* Variables for insmod */
MODULE_AUTHOR("Eric Lo");
MODULE_DESCRIPTION("MOXA Smartio/Industio Family Multiport Board Device Driver");
int mx_ioaddr_array_num;
int mx_iovect_array_num;
int mx_irq_array_num;

module_param_array(ioaddr, int, &mx_ioaddr_array_num, 0);
module_param_array(iovect, int, &mx_iovect_array_num, 0);
module_param_array(irq, int, &mx_irq_array_num, 0);

module_param(ttymajor, int, 0);
module_param(calloutmajor, int, 0);
module_param(verbose, int, 0);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
#endif /* MODULE */

struct mxser_log {
	int	tick;
	unsigned long	rxcnt[MXSER_PORTS];
	unsigned long	txcnt[MXSER_PORTS];
};

struct mxser_mon{
        unsigned long   rxcnt;
        unsigned long   txcnt;
	unsigned long	up_rxcnt;
	unsigned long	up_txcnt;
        int             modem_status;
        unsigned char   hold_reason;
};

struct mxser_mon_ext
{
	unsigned long rx_cnt[32];
	unsigned long tx_cnt[32];
	unsigned long up_rxcnt[32];
	unsigned long up_txcnt[32];
	int	modem_status[32];
	long baudrate[32];
	int databits[32];
	int stopbits[32];
	int parity[32];
	int flowctrl[32];
	int fifo[32];
	int iftype[32];
};

struct mxser_hwconf {
	int		board_type;
	int		ports;
	int		irq;
	unsigned long	vector;
	unsigned long	vector_mask;
	int		uart_type;
	unsigned long	ioaddr[MXSER_PORTS_PER_BOARD];
	int		baud_base[MXSER_PORTS_PER_BOARD];
	moxa_pci_info	pciInfo;
	int		IsMoxaMustChipFlag;	// add by Victor Yu. 08-30-2002
	int		MaxCanSetBaudRate[MXSER_PORTS_PER_BOARD];	// add by Victor Yu. 09-04-2002
	unsigned long	opmode_ioaddr[MXSER_PORTS_PER_BOARD];	// add by Victor Yu. 01-05-2004
};

struct __attribute__((__packed__)) mxser_usr_hwconf{
	int	board_type;
	unsigned long	ioaddr[MXSER_PORTS_PER_BOARD];
	int	baud[MXSER_PORTS_PER_BOARD];
	moxa_pci_usr_info pciInfo;
	int	IsMoxaMustChipFlag;
	int	MaxCanSetBaudRate[MXSER_PORTS_PER_BOARD];
};

struct mxser_struct {
	int			port;
	struct tty_port ttyPort;
	unsigned long		base;		/* port base address */
	int			irq;		/* port using irq no. */
	unsigned long		vector; 	/* port irq vector */
	unsigned long		vectormask;	/* port vector mask */
	int			rx_high_water;
	int			rx_trigger;	/* Rx fifo trigger level */
	int			rx_low_water;
	int			baud_base;	/* max. speed */
	int			flags;		/* defined in tty.h */
	int			type;		/* UART type */
	struct tty_struct *	tty;
	int			read_status_mask;
	int			ignore_status_mask;
	int			xmit_fifo_size;
	int			custom_divisor;
	int			x_char; 	/* xon/xoff character */
	int			close_delay;
	unsigned short		closing_wait;
	int			IER;		/* Interrupt Enable Register */
	int			MCR;		/* Modem control register */
	unsigned long		event;
	int			count;		/* # of fd on device */
	int			blocked_open;	/* # of blocked opens */
	long			session;	/* Session of opening process */
	long			pgrp;		/* pgrp of opening process */
	unsigned char		*xmit_buf;
	int			xmit_head;
	int			xmit_tail;
	int			xmit_cnt;
	struct work_struct tqueue;
	struct ktermios		normal_termios;
	struct ktermios		callout_termios;
	wait_queue_head_t open_wait;
	wait_queue_head_t close_wait;
	wait_queue_head_t delta_msr_wait;
	struct async_icount	icount; 	/* kernel counters for the 4 input interrupts */
	int			timeout;
	int			IsMoxaMustChipFlag;	// add by Victor Yu. 08-30-2002
	int			MaxCanSetBaudRate;	// add by Victor Yu. 09-04-2002
	unsigned long		opmode_ioaddr;	// add by Victor Yu. 01-05-2004
	unsigned char		stop_rx;
	unsigned char       ldisc_stop_rx;
	long    realbaud;
	struct mxser_mon        mon_data;
	unsigned char           err_shadow;
	spinlock_t		slock;
	int			speed;
	int			custom_baud_rate;
#if (LINUX_VERSION_CODE >= VERSION_CODE(4,6,0))
	int close;
#endif
};


struct mxser_mstatus{
       tcflag_t	cflag;
       int  	cts;
       int  	dsr;
       int  	ri;
       int  	dcd;
};

static  struct mxser_mstatus GMStatus[MXSER_PORTS];

static int mxserBoardCAP[MXSER_BOARDS]  = {
	0,0,0,0
       /*  0x180, 0x280, 0x200, 0x320   */
};

static int mxserBoardIRQ[MXSER_BOARDS] = {
	0,0,0,0
};

static int mxserBoardVECT[MXSER_BOARDS] = {
	0,0,0,0
};

static struct tty_driver	*mxvar_sdriver;
static struct mxser_struct	mxvar_table[MXSER_PORTS+1];
static struct tty_struct *	mxvar_tty[MXSER_PORTS+1];
static struct ktermios * 	mxvar_termios[MXSER_PORTS+1];
static struct mxser_log 	mxvar_log;
static int			mxvar_diagflag;
static unsigned char mxser_msr[MXSER_PORTS+1];
static struct mxser_mon_ext mon_data_ext;
static int mxser_set_baud_method[MXSER_PORTS+1];
static spinlock_t		gm_lock;
//static int                      moxaTimer_on;
//static struct timer_list        moxaTimer;

/*
 * This is used to figure out the divisor speeds and the timeouts
 */
static int mxvar_baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 0 };
#define BAUD_TABLE_NO	(sizeof(mxvar_baud_table)/sizeof(int))

struct mxser_hwconf mxsercfg[MXSER_BOARDS];

/*
 * static functions:
 */

#ifdef MODULE
int		init_module(void);
void		cleanup_module(void);
#endif

static void 	mxser_getcfg(int board,struct mxser_hwconf *hwconf);
int		mxser_init(void);

//static void	mxser_poll(unsigned long);
static int	mxser_get_PC_ISA_conf(int cap,int vector,	struct mxser_hwconf *hwconf, int irq);
static int	mxser_get_ISA_conf(int, struct mxser_hwconf *);
#ifdef CONFIG_PCI
static int      mxser_get_PCI_conf(int ,int ,int ,struct mxser_hwconf *);
#endif
static void     mxser_do_softint(struct work_struct *work);
static int	mxser_open(struct tty_struct *, struct file *);
static void	mxser_close(struct tty_struct *, struct file *);

static int	mxser_write(struct tty_struct *, const unsigned char *, int);
static int	mxser_write_room(struct tty_struct *);
static void	mxser_flush_buffer(struct tty_struct *);
static int	mxser_chars_in_buffer(struct tty_struct *);
static void	mxser_flush_chars(struct tty_struct *);
static int	mxser_put_char(struct tty_struct *, unsigned char);
static int	mxser_ioctl(struct tty_struct *, uint, ulong);
static int	mxser_ioctl_special(unsigned int, unsigned long);
static void	mxser_throttle(struct tty_struct *);
static void	mxser_unthrottle(struct tty_struct *);
static void	mxser_set_ldisc(struct tty_struct *);
static void	mxser_set_termios(struct tty_struct *, struct ktermios *);
static void	mxser_stop(struct tty_struct *);
static void	mxser_start(struct tty_struct *);
static void	mxser_hangup(struct tty_struct *);
static int  mxser_rs_break(struct tty_struct *, int);
static void mxser_wait_until_sent(struct tty_struct *tty, int timeout);
static irqreturn_t mxser_interrupt(int irq, void *dev_id);
static void mxser_receive_chars(struct mxser_struct *, int *);
static void mxser_transmit_chars(struct mxser_struct *);
static void mxser_check_modem_status(struct mxser_struct *, int);
static int	mxser_block_til_ready(struct tty_struct *, struct file *, struct mxser_struct *);
static int	mxser_startup(struct mxser_struct *);
static void	mxser_shutdown(struct mxser_struct *);
static int	mxser_change_speed(struct mxser_struct *, struct ktermios *old_termios);
#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
static int	mxser_get_serial_info(struct tty_struct *, struct serial_struct *);
static int	mxser_set_serial_info(struct tty_struct *, struct serial_struct *);
#else
static int	mxser_get_serial_info(struct mxser_struct *, struct serial_struct *);
static int	mxser_set_serial_info(struct mxser_struct *, struct serial_struct *);
#endif
static int	mxser_get_lsr_info(struct mxser_struct *, unsigned int *);
static void	mxser_send_break(struct mxser_struct *, int);
static int mxser_tiocmget(struct tty_struct *);
static int mxser_tiocmset(struct tty_struct *, unsigned int, unsigned int);
static int mxser_set_baud(struct mxser_struct *info, long newspd);

static void mxser_startrx(struct tty_struct * tty);
static void mxser_stoprx(struct tty_struct * tty);


static int CheckIsMoxaMust(int io)
{
	UCHAR	oldmcr, hwid;
	int	i;

	outb(0, io+UART_LCR);
	DISABLE_MOXA_MUST_ENCHANCE_MODE(io);
	oldmcr = inb(io+UART_MCR);
	outb(0, io+UART_MCR);
	SET_MOXA_MUST_XON1_VALUE(io, 0x11);
	if ( (hwid=inb(io+UART_MCR)) != 0 ) {
		outb(oldmcr, io+UART_MCR);
		return(MOXA_OTHER_UART);
	}
	
	GET_MOXA_MUST_HARDWARE_ID(io, &hwid);
	for(i=0; i<UART_TYPE_NUM; i++){
		if(hwid == Gmoxa_uart_id[i])
			return (int)hwid;
	}
	return MOXA_OTHER_UART;
}
// above is modified by Victor Yu. 08-15-2002

static struct tty_operations mxser_ops = {
	.open = mxser_open,
	.close = mxser_close,
	.write = mxser_write,
	.put_char = mxser_put_char,
	.flush_chars = mxser_flush_chars,
	.write_room = mxser_write_room,
	.chars_in_buffer = mxser_chars_in_buffer,
	.flush_buffer = mxser_flush_buffer,
	.ioctl = mxser_ioctl,
	.throttle = mxser_throttle,
	.unthrottle = mxser_unthrottle,
	.set_ldisc = mxser_set_ldisc,
	.set_termios = mxser_set_termios,
	.stop = mxser_stop,
	.start = mxser_start,
	.hangup = mxser_hangup,
	.tiocmget = mxser_tiocmget,
	.tiocmset = mxser_tiocmset,
#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
	.set_serial = mxser_set_serial_info,
	.get_serial = mxser_get_serial_info,
#endif
	.break_ctl = mxser_rs_break,
	.wait_until_sent = mxser_wait_until_sent,
};

/*
 * The MOXA Smartio/Industio serial driver boot-time initialization code!
 */
INIT_FUNC_RET	INIT_FUNC(void)
{
	int	ret;
	if (verbose)
		pr_info("Loading module mxser ...\n");
	ret = mxser_init();
	if (verbose)
		pr_info("Done.\n");
	return (ret);
}

CLEAR_FUNC_RET	CLEAR_FUNC(void)
{
	int i,err = 0;
	struct ktermios *tp;
	void *p;

	if (verbose)
		pr_info("Unloading module mxser ...\n");

	if ((err |= tty_unregister_driver(DRV_VAR)))
		pr_info("Couldn't unregister MOXA Smartio/Industio family serial driver\n");

	for (i = 0; i < DRV_VAR->num; i++) {
		tp = DRV_VAR->termios[i];
		if (tp) {
			DRV_VAR->termios[i] = NULL;
			kfree(tp);
		}
		if (!(DRV_VAR->flags & TTY_DRIVER_DYNAMIC_DEV))
			tty_unregister_device(DRV_VAR, i);
	}
	p = DRV_VAR->ttys;
	//proc_tty_unregister_driver(DRV_VAR);
	DRV_VAR->ttys = NULL;
	DRV_VAR->termios = NULL;
        
	for(i=0; i<MXSER_BOARDS; i++){
	    struct pci_dev *pdev;

	    if(mxsercfg[i].board_type == -1)
	        continue;
            else{
		pdev = mxsercfg[i].pciInfo.pdev;
	        free_irq(mxsercfg[i].irq, &mxvar_table[i*MXSER_PORTS_PER_BOARD]);
		if(pdev!=NULL){ //PCI
			release_region(pci_resource_start(pdev, 2),
	       			pci_resource_len(pdev, 2));
			release_region(pci_resource_start(pdev, 3),
		       		pci_resource_len(pdev, 3));
		}else{
			release_region(mxsercfg[i].ioaddr[0],
					8*mxsercfg[i].ports);
			release_region(mxsercfg[i].vector, 1);
		}
            }
        }
	
	if (verbose)
		pr_info("Done.\n");

	pr_info("Unregister MOXA Smartio/Industio family serial driver\n");
}

static void process_txrx_fifo(struct mxser_struct *info)
{
	int i;
	
	if ( (info->type == PORT_16450) || (info->type == PORT_8250) ){
		info->rx_trigger = 1;
		info->rx_high_water = 1;
		info->rx_low_water = 1;
		info->xmit_fifo_size = 1;
	}else{
		for(i=0; i<UART_INFO_NUM; i++){
			if(info->IsMoxaMustChipFlag == Gpci_uart_info[i].type){
				info->rx_trigger = Gpci_uart_info[i].rx_trigger;
				info->rx_low_water = Gpci_uart_info[i].rx_low_water;
				info->rx_high_water = Gpci_uart_info[i].rx_high_water;	
				info->xmit_fifo_size = Gpci_uart_info[i].xmit_fifo_size;	
				break;
			}
		}
	}
}

int mxser_initbrd(int board,struct mxser_hwconf *hwconf)
{
	struct mxser_struct *	info;
//	unsigned long	flags;
        int     retval;
	int	i,n;

	n = board*MXSER_PORTS_PER_BOARD;
	info = &mxvar_table[n];
	/*if (verbose)*/ {
		pr_info("        ttyM%d - ttyM%d ", 
			n, n+hwconf->ports-1);
		pr_info(" max. baud rate = %d bps.\n", hwconf->MaxCanSetBaudRate[0]);
	}
	
	for ( i=0; i<hwconf->ports; i++, n++, info++ ) {
		info->port = n;
		info->base = hwconf->ioaddr[i];
		info->irq = hwconf->irq;
		info->vector = hwconf->vector;
		info->vectormask = hwconf->vector_mask;
		info->opmode_ioaddr = hwconf->opmode_ioaddr[i];	// add by Victor Yu. 01-05-2004
		info->stop_rx = 0;
	        info->ldisc_stop_rx = 0;

		info->IsMoxaMustChipFlag = hwconf->IsMoxaMustChipFlag;
		//Enhance mode enabled here
		if(info->IsMoxaMustChipFlag!=MOXA_OTHER_UART){
			ENABLE_MOXA_MUST_ENCHANCE_MODE(info->base);
		}
		info->flags = ASYNC_SHARE_IRQ;
		info->type = hwconf->uart_type;
		info->baud_base = hwconf->baud_base[i];
		
		info->MaxCanSetBaudRate = hwconf->MaxCanSetBaudRate[i];

		process_txrx_fifo(info);
		

		info->custom_divisor = hwconf->baud_base[i] * 16;
		info->close_delay = 5*HZ/10;
		info->closing_wait = 30*HZ;
		INIT_WORK(&info->tqueue, mxser_do_softint);
		info->normal_termios = DRV_VAR_P(init_termios);
		init_waitqueue_head(&info->open_wait);
		init_waitqueue_head(&info->close_wait);
		init_waitqueue_head(&info->delta_msr_wait);
		info->speed = 9600;
                memset(&info->mon_data, 0, sizeof(struct mxser_mon));
		info->err_shadow = 0;
		spin_lock_init(&info->slock);
	}
	/*
	 * Allocate the IRQ if necessary
	 */
	

	/* before set INT ISR, disable all int */
	for(i=0; i<hwconf->ports; i++){
		outb(inb(hwconf->ioaddr[i] + UART_IER) & 0xf0, hwconf->ioaddr[i]+UART_IER);
	}

	n = board*MXSER_PORTS_PER_BOARD;
	info = &mxvar_table[n];
	
        //MX_LOCK(&info->slock);
        retval = request_irq(hwconf->irq, mxser_interrupt, IRQ_T(info),
				 "mxser", info);
	if ( retval ) {
	    //MX_UNLOCK(&info->slock);
	    pr_info("Board %d: %s", board, mxser_brdname[hwconf->board_type-1]);
	    pr_info("  Request irq fail,IRQ (%d) may be conflit with another device.\n",info->irq);
	    return(retval);
	}	
	//MX_UNLOCK(&info->slock);
        return 0;
}


static void mxser_getcfg(int board,struct mxser_hwconf *hwconf)
{
	mxsercfg[board] = *hwconf;
}

#ifdef CONFIG_PCI
static int mxser_get_PCI_conf(int busnum,int devnum,int board_type,struct mxser_hwconf *hwconf)
{
	int		i, j;
//	unsigned int	val;
	unsigned long	ioaddress;
	struct pci_dev	*pdev=hwconf->pciInfo.pdev;

	//io address
	hwconf->board_type = board_type;
	hwconf->ports = mxser_numports[board_type-1];
	ioaddress = pci_resource_start(pdev, 2);
	request_region(pci_resource_start(pdev, 2),
	       pci_resource_len(pdev, 2),
	       "mxser(IO)");

	for (i = 0; i < hwconf->ports; i++) {
		hwconf->ioaddr[i] = ioaddress + 8*i;
	}

	//vector
	ioaddress = pci_resource_start(pdev, 3);
	request_region(pci_resource_start(pdev, 3),
	       pci_resource_len(pdev, 3),
	       "mxser(vector)");
	hwconf->vector = ioaddress;
	
	//irq
	hwconf->irq = hwconf->pciInfo.pdev->irq;

	hwconf->IsMoxaMustChipFlag = CheckIsMoxaMust(hwconf->ioaddr[0]);
	hwconf->uart_type = PORT_16550A;
	hwconf->vector_mask = 0;
	
	
	for (i = 0; i < hwconf->ports; i++) {
		for(j=0; j<UART_INFO_NUM ;j++){
			if(Gpci_uart_info[j].type == hwconf->IsMoxaMustChipFlag) {
				hwconf->MaxCanSetBaudRate[i] = Gpci_uart_info[j].max_baud;
				
				//exception....CP-102
				if(board_type == MXSER_BOARD_CP102)
					hwconf->MaxCanSetBaudRate[i] = 921600;
				break;
			}
		}
	}
	
	if(hwconf->IsMoxaMustChipFlag == MOXA_MUST_MU860_HWID){
		for (i = 0; i < hwconf->ports; i++) {
			if ( i < 4 )
				hwconf->opmode_ioaddr[i] = ioaddress + 4;
			else
				hwconf->opmode_ioaddr[i] = ioaddress + 0x0c;
		}
		outb(0, ioaddress+4);	// default set to RS232 mode
		outb(0, ioaddress+0x0c); //default set to RS232 mode
	}
	
        for (i = 0; i < hwconf->ports; i++) {
		hwconf->vector_mask |= (1<<i);
		hwconf->baud_base[i] = 921600;
	}
	return(0);
}
#endif

int mxser_init(void)
{
	int			i, m, retval, b;
	int         ret1, ret2;
	int			port_idx;
#ifdef CONFIG_PCI
	struct pci_dev	*pdev=NULL;
	int			index;
	unsigned char		busnum,devnum;
	int n = 0;
#endif
	struct mxser_hwconf	hwconf;

	mxvar_sdriver = alloc_tty_driver(MXSER_PORTS + 1);
	if (!mxvar_sdriver)
		return -ENOMEM;
	spin_lock_init(&gm_lock);

	for(i=0; i<MXSER_BOARDS; i++){
		mxsercfg[i].board_type = -1;
	}
	
	pr_info("MOXA Smartio/Industio family driver version %s\n",MXSER_VERSION);

	/* Initialize the tty_driver structure */
	DRV_VAR_P(magic) = TTY_DRIVER_MAGIC;
	DRV_VAR_P(name) = "ttyM";
	DRV_VAR_P(major) = ttymajor;
	DRV_VAR_P(minor_start) = 0;
	DRV_VAR_P(num) = MXSER_PORTS + 1;
	DRV_VAR_P(type) = TTY_DRIVER_TYPE_SERIAL;
	DRV_VAR_P(subtype) = SERIAL_TYPE_NORMAL;
	DRV_VAR_P(init_termios) = tty_std_termios;
	DRV_VAR_P(init_termios.c_cflag) = B9600|CS8|CREAD|HUPCL|CLOCAL;
	DRV_VAR_P(flags) = TTY_DRIVER_REAL_RAW ;
	tty_set_operations(DRV_VAR, &mxser_ops);
	DRV_VAR_P(ttys) = mxvar_tty;
	DRV_VAR_P(termios) = mxvar_termios;

	mxvar_diagflag = 0;
	memset(mxvar_table, 0, (MXSER_PORTS+1) * sizeof(struct mxser_struct));
	memset(&mxvar_log, 0, sizeof(struct mxser_log));

	memset(&mxser_msr, 0, sizeof(unsigned char) * (MXSER_PORTS+1));
	memset(&mon_data_ext, 0, sizeof(struct mxser_mon_ext));
	memset(&mxser_set_baud_method, 0, sizeof(int) * (MXSER_PORTS+1));
	memset(&hwconf, 0, sizeof(struct mxser_hwconf));

	m = 0;
	/* Start finding PC104 ISA boards here */
	for ( b=0; b<MXSER_BOARDS && m<MXSER_BOARDS; b++ ) {
            int cap,vector,irq;
	    
	    if ( !(cap=mxserBoardCAP[b]) )
	        continue;
	    if ( !(vector=mxserBoardVECT[b]) )
	        continue;
	    if ( !(irq=mxserBoardIRQ[b]) )
	        continue;
        
	    retval = mxser_get_PC_ISA_conf(cap, vector, &hwconf, irq);

	    if ( retval != 0 ) 
	    	pr_info("Found MOXA %s board (IO=0x%x,Vector=0x%x,IRQ=%d)\n",
				mxser_brdname[hwconf.board_type-1],
				mxserBoardCAP[b],mxserBoardVECT[b],
				mxserBoardIRQ[b]);
				
	    if ( retval <= 0 ) {
		if (retval == MXSER_ERR_IRQ)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_IRQ_CONFLIT)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_VECTOR)
			pr_info("Invalid interrupt vector,board not configured\n");
		else if (retval == MXSER_ERR_IOADDR)
			pr_info("Invalid I/O address,board not configured\n");

		continue;
	    }
	    
	    hwconf.pciInfo.busNum = 0;
	    hwconf.pciInfo.devNum = 0;
	    mxser_getcfg(m,&hwconf);
	    
	    if(mxser_initbrd(m,&hwconf)<0)
                continue;
	    
				  
	    m++;
	}
	/* Start finding PC104 ISA boards from module arg */
	for ( b=0; b<MXSER_BOARDS && m<MXSER_BOARDS; b++ ) {
            int cap,vector;

	    if ( !(cap=ioaddr[b]) )
	        continue;
	    if ( !(vector=iovect[b]) )
		continue;
	    if ( !(irq[b]) )
		continue;
                
	    retval = mxser_get_PC_ISA_conf(cap, vector, &hwconf, irq[b]);

	    if ( retval != 0 ) 
	    	pr_info("Found MOXA %s board (IO=0x%x,Vector=0x%x,IRQ=%d)\n",
				mxser_brdname[hwconf.board_type-1],
				ioaddr[b],iovect[b],irq[b]);
				
	    if ( retval <= 0 ) {
		if (retval == MXSER_ERR_IRQ)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_IRQ_CONFLIT)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_VECTOR)
			pr_info("Invalid interrupt vector,board not configured\n");
		else if (retval == MXSER_ERR_IOADDR)
			pr_info("Invalid I/O address,board not configured\n");

		continue;
	    }
	    
	    hwconf.pciInfo.busNum = 0;
	    hwconf.pciInfo.devNum = 0;
	    
	    mxser_getcfg(m,&hwconf);

	    if(mxser_initbrd(m,&hwconf)<0)
                continue;
	    
	    m++;
	}
	
	/* Start finding ISA boards here */
	for ( b=0; b<MXSER_BOARDS && m<MXSER_BOARDS; b++ ) {
            int cap;
	    if ( !(cap=mxserBoardCAP[b]) )
	        continue;

	    retval = mxser_get_ISA_conf(cap, &hwconf);

	    if ( retval != 0 )
	    	pr_info("Found MOXA %s board (CAP=0x%x)\n",
				mxser_brdname[hwconf.board_type-1],
				ioaddr[b]);

	    if ( retval <= 0 ) {
		if (retval == MXSER_ERR_IRQ)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_IRQ_CONFLIT)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_VECTOR)
			pr_info("Invalid interrupt vector,board not configured\n");
		else if (retval == MXSER_ERR_IOADDR)
			pr_info("Invalid I/O address,board not configured\n");

		continue;
	    }

	    hwconf.pciInfo.busNum = 0;
	    hwconf.pciInfo.devNum = 0;
	    hwconf.pciInfo.pdev = NULL;

	    mxser_getcfg(m,&hwconf);
	    //init mxsercfg first, or mxsercfg data is not correct on ISR.
	    //mxser_initbrd will hook ISR.
	    if(mxser_initbrd(m,&hwconf)<0)
                continue;


	    m++;
	}

	/* Start finding ISA boards from module arg */
	for ( b=0; b<MXSER_BOARDS && m<MXSER_BOARDS; b++ ) {
            unsigned long cap;
	    if ( !(cap=ioaddr[b]) )
	        continue;

	    retval = mxser_get_ISA_conf(cap, &hwconf);

	    if ( retval != 0 )
	    	pr_info("Found MOXA %s board (CAP=0x%x)\n",
				mxser_brdname[hwconf.board_type-1],
				ioaddr[b]);

	    if ( retval <= 0 ) {
		if (retval == MXSER_ERR_IRQ)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_IRQ_CONFLIT)
			pr_info("Invalid interrupt number,board not configured\n");
		else if (retval == MXSER_ERR_VECTOR)
			pr_info("Invalid interrupt vector,board not configured\n");
		else if (retval == MXSER_ERR_IOADDR)
			pr_info("Invalid I/O address,board not configured\n");

		continue;
	    }

	    hwconf.pciInfo.busNum = 0;
	    hwconf.pciInfo.devNum = 0;
	    hwconf.pciInfo.pdev = NULL;

	    mxser_getcfg(m,&hwconf);
	    //init mxsercfg first, or mxsercfg data is not correct on ISR.
	    //mxser_initbrd will hook ISR.
	    if(mxser_initbrd(m,&hwconf)<0)
                continue;

	    m++;
	}

	/* start finding PCI board here */
#ifdef CONFIG_PCI	
	n = (sizeof(mxser_pcibrds) / sizeof(mxser_pcibrds[0])) - 1;
	index = 0;
	b = 0;
	while (b < n) {
	       pdev = pci_get_device(mxser_pcibrds[b].vendor,
	       		mxser_pcibrds[b].device, pdev);
	       		
		if(pdev==NULL){
			b++;
			continue;
		}
		hwconf.pciInfo.busNum = busnum = pdev->bus->number;
		hwconf.pciInfo.devNum = devnum = PCI_SLOT(pdev->devfn)<<3;
		hwconf.pciInfo.pdev = pdev;
		pr_info("Found MOXA %s board(BusNo=%d,DevNo=%d)\n",mxser_brdname[(int)(mxser_pcibrds[b].driver_data)-1],busnum,devnum >> 3);
		index++;
		if ( m >= MXSER_BOARDS) {
			pr_info("Too many Smartio/Industio family boards find (maximum %d),board not configured\n",MXSER_BOARDS);
		}
		else {
			if ( pci_enable_device(pdev) ) {
				pr_info("Moxa SmartI/O PCI enable fail !\n");
				continue;
			}
			retval = mxser_get_PCI_conf(busnum,devnum,
				(int)mxser_pcibrds[b].driver_data,&hwconf);
			if (retval < 0) {
				if (retval == MXSER_ERR_IRQ)
					pr_info("Invalid interrupt number,board not configured\n");
				else if (retval == MXSER_ERR_IRQ_CONFLIT)
					pr_info("Invalid interrupt number,board not configured\n");
				else if (retval == MXSER_ERR_VECTOR)
					pr_info("Invalid interrupt vector,board not configured\n");
				else if (retval == MXSER_ERR_IOADDR)
					pr_info("Invalid I/O address,board not configured\n");
				continue;
			}
			mxser_getcfg(m,&hwconf);
			//init mxsercfg first, or mxsercfg data is not correct on ISR.
			//mxser_initbrd will hook ISR.
			if(mxser_initbrd(m,&hwconf)<0)
				continue;
			m++;
		}
	}
#endif

	for(m = 0;m < MXSER_BOARDS; m++)
	{
		for(i = 0; i < MXSER_PORTS_PER_BOARD;i++)
		{
			port_idx =  (m * MXSER_PORTS_PER_BOARD) + i;
			tty_port_init(&mxvar_table[port_idx].ttyPort);		
			tty_port_link_device(&mxvar_table[port_idx].ttyPort, mxvar_sdriver, port_idx);
		}
	}
	port_idx = MXSER_BOARDS * MXSER_PORTS_PER_BOARD;
	tty_port_init(&mxvar_table[port_idx].ttyPort);
	tty_port_link_device(&mxvar_table[port_idx].ttyPort, mxvar_sdriver, port_idx);

	ret1 = 0;
	ret2 = 0;
	if ( !(ret1=tty_register_driver(DRV_VAR)) )
	    return 0;	            
        else
	    pr_info("Couldn't install MOXA Smartio/Industio family driver !\n");

        if(ret1 || ret2){
            for(i=0; i<MXSER_BOARDS; i++){
	        if(mxsercfg[i].board_type == -1)
	            continue;
                else{
		    free_irq(mxsercfg[i].irq, &mxvar_table[i*MXSER_PORTS_PER_BOARD]);
		    //todo: release io, vector
                }
            }
            return -1;
        }

	return(0);
}

static void mxser_do_softint(struct work_struct *work)
{
	struct mxser_struct *info = container_of(work, struct mxser_struct, tqueue);
	struct tty_struct *	tty;

	tty = info->tty;

	if ( test_and_clear_bit(MXSER_EVENT_TXLOW, &info->event) ) 
		tty_wakeup(tty);
	if ( test_and_clear_bit(MXSER_EVENT_HANGUP, &info->event) )
		tty_hangup(tty);
}

static unsigned char mxser_get_msr(int baseaddr, int mode, int port, struct mxser_struct *info)
{
	unsigned char	status=0;

	status = inb(baseaddr + UART_MSR);

	mxser_msr[port] &= 0x0F;
	mxser_msr[port] |= status;
	status = mxser_msr[port];
	if( mode )
		mxser_msr[port] = 0;
		
	return status;
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int mxser_open(struct tty_struct * tty, struct file * filp)
{
	struct mxser_struct *	info;
	int			retval, line;

	MX_LOCK_INIT();
	
	line = PORTNO(tty);
	if ( line == MXSER_PORTS )
		return(0);
	
    	MX_MOD_INC;
    	
	if ( (line < 0) || (line > MXSER_PORTS) )
		return(-ENODEV);

	info = mxvar_table + line;
	if ( !info->base )
		return(-ENODEV);

	tty->driver_data = info;
	info->tty = tty;

//pr_info("port %d, mxser_open\r\n", info->port);
	/*
	 * Start up serial port
	 */
	info->count++;
	
	retval = mxser_startup(info);
	if ( retval )
		return(retval);

	retval = mxser_block_til_ready(tty, filp, info);
	if ( retval )
		return(retval);

	if ( (info->count == 1) ) {
		if ( MX_TTY_DRV(subtype) == SERIAL_TYPE_NORMAL )
			(*tty).termios = info->normal_termios;
		else
			(*tty).termios = info->callout_termios;
		MX_LOCK(&info->slock);
		mxser_change_speed(info, 0);
		MX_UNLOCK(&info->slock);
	}

#ifdef TTY_DONT_FLIP
	clear_bit(TTY_DONT_FLIP, &tty->flags); // since VERSION_CODE >= 2.6.18
#endif
	//status = mxser_get_msr(info->base, 0, info->port);
	//mxser_check_modem_status(info, status);

	/* unmark here for very high baud rate (ex. 921600 bps) used */
	info->ttyPort.low_latency = 0;
	return(0);
}

/*
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 */
static void mxser_close(struct tty_struct * tty, struct file * filp)
{
	struct mxser_struct * info = (struct mxser_struct *)tty->driver_data;
	unsigned long	timeout;
	
	MX_LOCK_INIT();

	if ( PORTNO(tty) == MXSER_PORTS )
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
	if ( (tty->count == 1) && (info->count != 1) ) {
		/*
		 * Uh, oh.	tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		pr_info("mxser_close: bad serial port count; tty->count is 1, "
			"info->count is %d\n", info->count);
		info->count = 1;
	}
	if ( --info->count < 0 ) {
		pr_info("mxser_close: bad serial port count for ttys%d: %d\n",
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
	if ( info->flags & ASYNC_NORMAL_ACTIVE )
		info->normal_termios = (*tty).termios;
	if ( info->flags & ASYNC_CALLOUT_ACTIVE )
		info->callout_termios = (*tty).termios;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify
	 * the line discipline to only process XON/XOFF characters.
	 */
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
	if ( info->IsMoxaMustChipFlag )
		info->IER &= ~MOXA_MUST_RECV_ISR;
/* by William
	info->read_status_mask &= ~UART_LSR_DR;
*/
	if ( info->flags & ASYNC_INITIALIZED ) {
		outb(info->IER, info->base + UART_IER);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		timeout = jiffies + HZ;
		while ( !(inb(info->base + UART_LSR) & UART_LSR_TEMT) ) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(5);
			if ( time_after(jiffies, timeout) )
				break;
		}
	}
	mxser_shutdown(info);
	mxser_flush_buffer(tty);
	tty_ldisc_flush(tty);//joy
	
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;
	if ( info->blocked_open ) {
		if ( info->close_delay ) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(info->close_delay);
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ASYNC_NORMAL_ACTIVE | ASYNC_CALLOUT_ACTIVE |
			 ASYNC_CLOSING);
	wake_up_interruptible(&info->close_wait);
	MX_MOD_DEC;
}

static int mxser_write(struct tty_struct * tty,
		       const unsigned char * buf, int count)
{
	int		c, total = 0;
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

	if ( !tty || !info->xmit_buf  )
		return(0);

	while ( 1 ) {
		c = MIN(count, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
			SERIAL_XMIT_SIZE - info->xmit_head));
		if ( c <= 0 )
			break;
		memcpy(info->xmit_buf + info->xmit_head, buf, c);
		MX_LOCK(&info->slock);    
		info->xmit_head = (info->xmit_head + c) & (SERIAL_XMIT_SIZE - 1);
		info->xmit_cnt += c;
		MX_UNLOCK(&info->slock);

		buf += c;
		count -= c;
		total += c;
	}
	   
	if ( info->xmit_cnt && !tty->stopped /*&& !(info->IER & UART_IER_THRI)*/ ) {
		if (!tty->hw_stopped || (info->type == PORT_16550A) || (info->IsMoxaMustChipFlag)) {
			MX_LOCK(&info->slock); 	
			info->IER &= ~UART_IER_THRI;
			outb(info->IER, info->base + UART_IER);
			info->IER |= UART_IER_THRI;
			outb(info->IER, info->base + UART_IER);
			MX_UNLOCK(&info->slock);
		}
	}
//pr_info("%lu, mxser_write=%d\n", jiffies, total);
	return total;
}

static int mxser_put_char(struct tty_struct * tty, unsigned char ch)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

	if ( !tty || !info->xmit_buf )
		return 0;

	if ( info->xmit_cnt >= SERIAL_XMIT_SIZE - 1 )
		return 0;

	MX_LOCK(&info->slock);
	info->xmit_buf[info->xmit_head++] = ch;
	info->xmit_head &= SERIAL_XMIT_SIZE - 1;
	info->xmit_cnt++;
	MX_UNLOCK(&info->slock);
	if ( !tty->stopped /*&& !(info->IER & UART_IER_THRI)*/ ) {
		if (!tty->hw_stopped || (info->type == PORT_16550A) || info->IsMoxaMustChipFlag) {
			MX_LOCK(&info->slock);
			info->IER &= ~UART_IER_THRI;
			outb(info->IER, info->base + UART_IER);
			info->IER |= UART_IER_THRI;
			outb(info->IER, info->base + UART_IER);
			MX_UNLOCK(&info->slock);
		}
	}
	return 1;
//pr_info("%lu, mxser_put_char=%x\n", jiffies, ch);
}


static void mxser_flush_chars(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

//pr_info("%lu, mxser_flush_chars\n", jiffies);

	if ( info->xmit_cnt <= 0 || tty->stopped || !info->xmit_buf ||
		(tty->hw_stopped && (info->type!=PORT_16550A) && (!info->IsMoxaMustChipFlag)))
		return;

	MX_LOCK(&info->slock);
	info->IER &= ~UART_IER_THRI	;
	outb(info->IER, info->base + UART_IER);
	info->IER |= UART_IER_THRI;
	outb(info->IER, info->base + UART_IER);

	MX_UNLOCK(&info->slock);
}

static int mxser_write_room(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	int	ret;

	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
	if ( ret < 0 )
		ret = 0;
	
//pr_info("%lu, mxser_write_room=%d\n", jiffies, ret);
	
	return(ret);
}

static int mxser_chars_in_buffer(struct tty_struct * tty)
{
	int len;
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	len = info->xmit_cnt;

	if(!(inb(info->base + UART_LSR) & UART_LSR_THRE))  
		len++;
		
//pr_info("%lu, mxser_chars_in_buffer=%d\n", jiffies, len);

	return len;
}

static void mxser_flush_buffer(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	char fcr;
	MX_LOCK_INIT();

//pr_info("%lu, flush_buffer\n", jiffies);

	MX_LOCK(&info->slock);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	MX_UNLOCK(&info->slock);
	
/* below added by shinhay */
	//outb(0x05, info->base+UART_FCR);
	fcr = inb(info->base + UART_FCR);
	outb((fcr | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT), 
		info->base + UART_FCR);
	outb(fcr, info->base+UART_FCR);

/* above added by shinhay */
	
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc->ops->write_wakeup)
		tty_wakeup(tty);
}

static int mxser_ioctl(struct tty_struct * tty, unsigned int cmd,
		       unsigned long arg)
{
	int			error;
	struct mxser_struct *	info = (struct mxser_struct *)tty->driver_data;
	int			retval;
	struct async_icount	cprev, cnow;	    /* kernel counter temps */
	struct serial_icounter_struct *p_cuser;     /* user space */
	unsigned long 		templ;
	MX_LOCK_INIT();

//pr_info("%lu, mxser_ioctl=%x\n", jiffies, cmd);

	if ( PORTNO(tty) == MXSER_PORTS )
	    return(mxser_ioctl_special(cmd, arg));

	// following add by Victor Yu. 01-05-2004
	if ( cmd == MOXA_SET_OP_MODE || cmd == MOXA_GET_OP_MODE ) {
		int	p;
		unsigned long opmode;
		static unsigned char ModeMask[]={0xfc, 0xf3, 0xcf, 0x3f};
		int             shiftbit;
		unsigned char	val, mask;

		p = info->port % 4;
		if ( cmd == MOXA_SET_OP_MODE ) {
	    		error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(int));
	    		if ( MX_ERR(error) )
				return(error);
	    		get_from_user(opmode,(int *)arg);
			if ( opmode != RS232_MODE && opmode != RS485_2WIRE_MODE && opmode != RS422_MODE && opmode != RS485_4WIRE_MODE )
				return -EFAULT;
			mask = ModeMask[p];
			shiftbit = p * 2;
			val = inb(info->opmode_ioaddr);
			val &= mask;
			val |= (opmode << shiftbit);
			outb(val, info->opmode_ioaddr);
		} else {
	    		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
	    		if ( MX_ERR(error) )
				return(error);
			shiftbit = p * 2;
			opmode = inb(info->opmode_ioaddr) >> shiftbit;
			opmode &= OP_MODE_MASK;
	    		if(copy_to_user((int*)arg, &opmode, sizeof(int)))
			     return -EFAULT;
		}
		return 0;
	}
	// above add by Victor Yu. 01-05-2004
	
	if ( cmd == MOXA_SET_SPECIAL_BAUD_RATE || cmd == MOXA_GET_SPECIAL_BAUD_RATE || cmd == SMARTIO_SET_SPECIAL_BAUD_RATE || cmd == SMARTIO_GET_SPECIAL_BAUD_RATE ) {
		int	speed, i;
		if ( cmd == MOXA_SET_SPECIAL_BAUD_RATE || cmd == SMARTIO_SET_SPECIAL_BAUD_RATE ) {
	    		error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(int));
	    		if ( MX_ERR(error) )
				return(error);

	    		get_from_user(speed,(int *)arg);
			if ( speed <= 0 || speed > info->MaxCanSetBaudRate )
				return -EFAULT;
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
			info->speed = speed;
	   		MX_LOCK(&info->slock);
			mxser_change_speed(info, 0);
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
		return(-EIO);
	}
	switch ( cmd ) {
	case TCSBRK:	/* SVID version: non-zero arg --> no break */
	    retval = tty_check_change(tty);
	    if ( retval )
		return(retval);
	    tty_wait_until_sent(tty, 0);
	    if ( !arg )
		mxser_send_break(info, HZ/4);		/* 1/4 second */
	    return(0);
	case TCSBRKP:	/* support for POSIX tcsendbreak() */
	    retval = tty_check_change(tty);
	    if ( retval )
		return(retval);
	    tty_wait_until_sent(tty, 0);
	    mxser_send_break(info, arg ? arg*(HZ/10) : HZ/4);
	    return(0);
	case TIOCGSOFTCAR:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(long));
	    if ( MX_ERR(error) )
		return(error);
	    put_to_user(C_CLOCAL(tty) ? 1 : 0, (unsigned long *)arg);
	    return 0;
	case TIOCSSOFTCAR:
	    error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(long));
	    if ( MX_ERR(error) )
		return(error);
	    get_from_user(templ,(unsigned long *)arg);
	    arg = templ;
	    tty->termios.c_cflag = ((tty->termios.c_cflag & ~CLOCAL) |
				    (arg ? CLOCAL : 0));		
	    return(0);
	case TIOCGSERIAL:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
				sizeof(struct serial_struct));
	    if ( MX_ERR(error) )
		return(error);
#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
	    return(mxser_get_serial_info(tty, (struct serial_struct *)arg));
#else
            return(mxser_get_serial_info(info, (struct serial_struct *)arg));
#endif
	case TIOCSSERIAL:
	    error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg,
				sizeof(struct serial_struct));
	    if ( MX_ERR(error) )
		return(error);
#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
	    return(mxser_set_serial_info(tty, (struct serial_struct *)arg));
#else
	    return(mxser_set_serial_info(info, (struct serial_struct *)arg));
#endif
	case TIOCSERGETLSR: /* Get line status register */
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
				sizeof(unsigned int));
	    if ( MX_ERR(error) )
		return(error);
	    else
		return(mxser_get_lsr_info(info, (unsigned int *)arg));
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
		if ( signal_pending(current) ){
		    return -ERESTARTSYS;
		}
		MX_LOCK(&info->slock);
		cnow = info->icount;	/* atomic copy */
	        MX_UNLOCK(&info->slock);
		if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
		     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
		     ((arg & TIOCM_CD)	&& (cnow.dcd != cprev.dcd)) ||
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
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
				sizeof(struct serial_icounter_struct));
	    if ( MX_ERR(error) )
		return(error);
	    MX_LOCK(&info->slock);
	    cnow = info->icount;
	    MX_UNLOCK(&info->slock);
	    p_cuser = (struct serial_icounter_struct *)arg;
/* modified by casper 1/11/2000 */
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

/* */
	    return(0);
	case MOXA_HighSpeedOn:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
	    if ( MX_ERR(error) )
		return(error);
	    put_to_user(info->baud_base != 115200 ? 1 : 0, (int *)arg);
	    return(0);

	case MOXA_SDS_RSTICOUNTER: {
	    info->mon_data.rxcnt = 0;
	    info->mon_data.txcnt = 0;
	    return(0);
    }
// (above) added by James.
    case MOXA_ASPP_SETBAUD:{
        long    baud;

		error = MX_ACCESS_CHK(VERIFY_READ, (void *)arg, sizeof(long));
		if ( MX_ERR(error) )
    		return(error);
		get_from_user(baud, (long *)arg);
		    MX_LOCK(&info->slock);
        mxser_set_baud(info, baud);
		    MX_UNLOCK(&info->slock);

        return (0);
        }
    case MOXA_ASPP_GETBAUD:
		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(long));
		if ( MX_ERR(error) )
    		return(error);

	    if (copy_to_user((long *)arg, &info->realbaud, sizeof(long)))
	        return -EFAULT;

        return (0);

    case MOXA_ASPP_OQUEUE:{
        int len, lsr;

		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
		if ( MX_ERR(error) )
    		return(error);
        len = mxser_chars_in_buffer(tty);

        lsr = inb(info->base+ UART_LSR) & UART_LSR_THRE;

        len += (lsr ? 0: 1);

		if (copy_to_user((int *)arg, &len, sizeof(int)))
	        return -EFAULT;

        return (0);
    }
    case MOXA_ASPP_MON:{
        int mcr, status;
		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(struct mxser_mon));
		if ( MX_ERR(error) )
    		return(error);
//    	info->mon_data.ser_param = tty->termios->c_cflag;

		status = mxser_get_msr(info->base, 1, info->port, info);
		mxser_check_modem_status(info, status);

		mcr = inb(info->base + UART_MCR);
		if(mcr & MOXA_MUST_MCR_XON_FLAG)
	        info->mon_data.hold_reason &= ~NPPI_NOTIFY_XOFFHOLD;
		else
	        info->mon_data.hold_reason |= NPPI_NOTIFY_XOFFHOLD;
	
		if(mcr & MOXA_MUST_MCR_TX_XON)
	        info->mon_data.hold_reason &= ~NPPI_NOTIFY_XOFFXENT;
		else
	        info->mon_data.hold_reason |= NPPI_NOTIFY_XOFFXENT;
    	
    	if(info->tty->hw_stopped)
    	    info->mon_data.hold_reason |= NPPI_NOTIFY_CTSHOLD;
        else
    	    info->mon_data.hold_reason &= ~NPPI_NOTIFY_CTSHOLD;
    	
   	
		if (copy_to_user((struct mxser_mon *)arg, &(info->mon_data), sizeof(struct mxser_mon)))
	        return -EFAULT;

        return (0);
        
        }

    case MOXA_ASPP_LSTATUS:{
	error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(struct mxser_mon));
	if ( MX_ERR(error) )
    		return(error);
    		
	if (copy_to_user((struct mxser_mon *)arg, &(info->err_shadow), 
		sizeof(unsigned char)))
	        return -EFAULT;

        info->err_shadow = 0;
        return (0);
        
        }
	case MOXA_SET_BAUD_METHOD:{
		int method;
		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
		if ( MX_ERR(error) )
    		return(error);
   	    get_from_user(method,(int *)arg);
		mxser_set_baud_method[info->port] = method;
		if (copy_to_user((int *)arg, &method, sizeof(int)))
	        return -EFAULT;

        return (0);
    }
	default:
	    return(-ENOIOCTLCMD);
	}
	return(0);
}

static int mxser_ioctl_special(unsigned int cmd, unsigned long arg)
{
	int		error, i, result, status;
	struct mxser_usr_hwconf usr_mxsercfg[MXSER_BOARDS] = {0};

	switch ( cmd ) {
	case MOXA_GET_CONF:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
			    sizeof(struct mxser_usr_hwconf)*MXSER_BOARDS);

	    if ( MX_ERR(error) )
	    	return(error);

	    for(i=0; i<MXSER_BOARDS; i++ ){
			usr_mxsercfg[i].IsMoxaMustChipFlag = mxsercfg[i].IsMoxaMustChipFlag;
			usr_mxsercfg[i].board_type = mxsercfg[i].board_type;
			memcpy( usr_mxsercfg[i].ioaddr, mxsercfg[i].ioaddr, sizeof(mxsercfg[i].ioaddr) );
			memcpy( usr_mxsercfg[i].MaxCanSetBaudRate, mxsercfg[i].MaxCanSetBaudRate, sizeof(mxsercfg[i].MaxCanSetBaudRate) );
			memcpy( usr_mxsercfg[i].baud, mxsercfg[i].baud_base, sizeof(mxsercfg[i].baud_base));
			memcpy( &usr_mxsercfg[i].pciInfo, &mxsercfg[i].pciInfo, sizeof(moxa_pci_usr_info) );
		}

	    if(copy_to_user((char *)arg, usr_mxsercfg,
			    sizeof(usr_mxsercfg)))
	    		return -EFAULT;
	    return 0;

        case MOXA_GET_MAJOR:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
	    if ( MX_ERR(error) )
                return(error);
	    if(copy_to_user((int*)arg, &ttymajor, sizeof(int)))
		return -EFAULT;
            return 0;

        case MOXA_GET_CUMAJOR:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(int));
	    if ( MX_ERR(error) )
                return(error);
	    if(copy_to_user((int*)arg, &calloutmajor, sizeof(int)))
		return -EFAULT;
            return 0;

	case MOXA_CHKPORTENABLE:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(long));
	    if ( MX_ERR(error) )
		return(error);
	    result = 0;
	    for ( i=0; i<MXSER_PORTS; i++ ) {
		if ( mxvar_table[i].base )
		    result |= (1 << i);
	    }
	    put_to_user(result, (unsigned long *)arg);
	    return(0);
	case MOXA_GETDATACOUNT:
	    //pr_info("MOXA_GETDATACOUNT\n");
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
				sizeof(struct mxser_log));
	    if ( MX_ERR(error) )
		return(error);
	    //pr_info("mxvar_log rx[0]: %d\n", mxvar_log.rxcnt[0]);
	    //pr_info("mxvar_log tx[0]: %d\n", mxvar_log.txcnt[0]);
	    if(copy_to_user((struct mxser_log *)arg, &mxvar_log, sizeof(mxvar_log)))
		return -EFAULT;
	    return(0);
        case MOXA_GETMSTATUS:
	    error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg,
				sizeof(struct mxser_mstatus) * MXSER_PORTS);
	    if ( MX_ERR(error) )
		return(error);

        for(i=0; i<MXSER_PORTS; i++){
            GMStatus[i].ri = 0;
		if ( !mxvar_table[i].base ){
                    GMStatus[i].dcd = 0;
                    GMStatus[i].dsr = 0;
                    GMStatus[i].cts = 0;
		    continue;
                }
		if ( !mxvar_table[i].tty)
                    GMStatus[i].cflag=mxvar_table[i].normal_termios.c_cflag;
                else
                    GMStatus[i].cflag = mxvar_table[i].tty->termios.c_cflag;

                status = inb(mxvar_table[i].base + UART_MSR);
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
            if(copy_to_user((struct mxser_mstatus *)arg, GMStatus,
                                    sizeof(struct mxser_mstatus) * MXSER_PORTS))
		return -EFAULT;
            return 0;
    case MOXA_ASPP_MON_EXT:{
        int status;
		int	p;
		unsigned long opmode;
		int shiftbit;
		unsigned	cflag, iflag;

		error = MX_ACCESS_CHK(VERIFY_WRITE, (void *)arg, sizeof(struct mxser_mon_ext));
		if ( MX_ERR(error) )
    		return(error);

		for(i=0; i<MXSER_PORTS; i++)
		{

			if ( !mxvar_table[i].base )
				continue;

			status = mxser_get_msr(mxvar_table[i].base, 0, i, &(mxvar_table[i]));
//			mxser_check_modem_status(&mxvar_table[i], status);
			if ( status & UART_MSR_TERI )	    mxvar_table[i].icount.rng++;
			if ( status & UART_MSR_DDSR )	    mxvar_table[i].icount.dsr++;
			if ( status & UART_MSR_DDCD )	    mxvar_table[i].icount.dcd++;
			if ( status & UART_MSR_DCTS )	    mxvar_table[i].icount.cts++;

			mxvar_table[i].mon_data.modem_status = status;
			mon_data_ext.rx_cnt[i] = mxvar_table[i].mon_data.rxcnt;
			mon_data_ext.tx_cnt[i] = mxvar_table[i].mon_data.txcnt;
			mon_data_ext.up_rxcnt[i] = mxvar_table[i].mon_data.up_rxcnt;
			mon_data_ext.up_txcnt[i] = mxvar_table[i].mon_data.up_txcnt;
			mon_data_ext.modem_status[i] = mxvar_table[i].mon_data.modem_status;
			mon_data_ext.baudrate[i] = mxvar_table[i].realbaud;
	        if ( !mxvar_table[i].tty )		
	        {
            	cflag = mxvar_table[i].normal_termios.c_cflag;
            	iflag = mxvar_table[i].normal_termios.c_iflag;
            }
            else
            {
		cflag = mxvar_table[i].tty->termios.c_cflag;
            	iflag = mxvar_table[i].tty->termios.c_iflag;
            }

			mon_data_ext.databits[i] = cflag & CSIZE;

			mon_data_ext.stopbits[i] = cflag & CSTOPB;

			mon_data_ext.parity[i] = cflag & (PARENB | PARODD | CMSPAR);

			mon_data_ext.flowctrl[i] = 0x00;

			if( cflag & CRTSCTS )
				mon_data_ext.flowctrl[i] |= 0x03;

			if( iflag & (IXON | IXOFF) )
				mon_data_ext.flowctrl[i] |= 0x0C;		
			
			if( mxvar_table[i].type == PORT_16550A)
				mon_data_ext.fifo[i] = 1;
			else
				mon_data_ext.fifo[i] = 0;
			
			p = i % 4;
			shiftbit = p * 2;
			opmode = inb(mxvar_table[i].opmode_ioaddr) >> shiftbit;
			opmode &= OP_MODE_MASK;

			mon_data_ext.iftype[i] = opmode;

		}
		if (copy_to_user((struct mxser_mon_ext *)arg, &mon_data_ext, sizeof(struct mxser_mon_ext)))
	        return -EFAULT;

        return (0);
        
        }
	default:
	    return(-ENOIOCTLCMD);
	}
	return(0);
}

static void mxser_stoprx(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
#if !defined CONFIG_PREEMPT_RT_FULL
	MX_LOCK_INIT();
#endif

        info->ldisc_stop_rx = 1;
	if ( I_IXOFF(tty) ) {
#if !defined CONFIG_PREEMPT_RT_FULL
	    MX_LOCK(&info->slock);
#endif
	    // following add by Victor Yu. 09-02-2002
	    if ( info->IsMoxaMustChipFlag ) {
		info->IER &= ~MOXA_MUST_RECV_ISR;
		outb(info->IER, info->base+UART_IER);
	    } else {
		if(!(info->flags & ASYNC_CLOSING)){
	            info->x_char = STOP_CHAR(tty);
	            outb(0, info->base+UART_IER);
	            info->IER |= UART_IER_THRI;
	            outb(info->IER, info->base + UART_IER);
		}
	    }
#if !defined CONFIG_PREEMPT_RT_FULL
	    MX_UNLOCK(&info->slock);
#endif
	}
	if ( info->tty->termios.c_cflag & CRTSCTS ) {
#if !defined CONFIG_PREEMPT_RT_FULL
	    MX_LOCK(&info->slock);
#endif
	    info->MCR &= ~UART_MCR_RTS;
	    outb(info->MCR, info->base + UART_MCR);
#if !defined CONFIG_PREEMPT_RT_FULL
	    MX_UNLOCK(&info->slock);
#endif
	}
}

static void mxser_startrx(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

        info->ldisc_stop_rx = 0;
        if ( I_IXOFF(tty) ) {
	    if ( info->x_char )
		info->x_char = 0;
	    else {
		MX_LOCK(&info->slock);

		// following add by Victor Yu. 09-02-2002
		if ( info->IsMoxaMustChipFlag ) {
			info->IER |= MOXA_MUST_RECV_ISR;
			outb(info->IER, info->base+UART_IER);
		} else {
		    if(!(info->flags & ASYNC_CLOSING)){
		        info->x_char = START_CHAR(tty);
		        outb(0, info->base+UART_IER);	
		        info->IER |= UART_IER_THRI;	
		        outb(info->IER, info->base + UART_IER);
                    }
		}
		MX_UNLOCK(&info->slock);
	    }
	}
	if ( info->tty->termios.c_cflag & CRTSCTS ) {
	    MX_LOCK(&info->slock);
	    info->MCR |= UART_MCR_RTS;
	    outb(info->MCR, info->base + UART_MCR);
	    MX_UNLOCK(&info->slock);
	}
}

/*
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 */
static void mxser_throttle(struct tty_struct * tty)
{
#if defined CONFIG_PREEMPT_RT_FULL
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();
	MX_LOCK(&info->slock);
#endif	
        mxser_stoprx(tty);
#if defined CONFIG_PREEMPT_RT_FULL
	MX_UNLOCK(&info->slock);
#endif
}

static void mxser_unthrottle(struct tty_struct * tty)
{
	//struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	//MX_LOCK_INIT();
	//MX_LOCK(&info->slock);
        mxser_startrx(tty);
	//MX_UNLOCK(&info->slock);
}

static void mxser_set_ldisc(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

	if ( tty->termios.c_line == N_PPS ) {
		info->flags |= ASYNC_HARDPPS_CD;
		MX_LOCK(&info->slock);
		info->IER |= UART_IER_MSI;
		outb(info->IER, info->base + UART_IER);
		MX_UNLOCK(&info->slock);
	} else {
		info->flags &= ~ASYNC_HARDPPS_CD;
		MX_LOCK(&info->slock);
		info->IER &= ~UART_IER_MSI;
		outb(info->IER, info->base + UART_IER);
		MX_UNLOCK(&info->slock);
	}
}

static void mxser_set_termios(struct tty_struct * tty, 
                              struct ktermios * old_termios)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();
	if ( (tty->termios.c_cflag != old_termios->c_cflag) ||
	     (RELEVANT_IFLAG(tty->termios.c_iflag) !=
	      RELEVANT_IFLAG(old_termios->c_iflag)) ) {
		MX_LOCK(&info->slock);
		mxser_change_speed(info, old_termios);
		MX_UNLOCK(&info->slock);
		if ( (old_termios->c_cflag & CRTSCTS) &&
	     	     !(tty->termios.c_cflag & CRTSCTS) ) {
	    		tty->hw_stopped = 0;
	    		mxser_start(tty);
		}
	}

/* Handle sw stopped */
	if ( (old_termios->c_iflag & IXON) &&
     	     !(tty->termios.c_iflag & IXON) ) {
    		tty->stopped = 0;

		// following add by Victor Yu. 09-02-2002
		if ( info->IsMoxaMustChipFlag ) {
			MX_LOCK(&info->slock);
			DISABLE_MOXA_MUST_RX_SOFTWARE_FLOW_CONTROL(info->base);
			MX_UNLOCK(&info->slock);
		}
		// above add by Victor Yu. 09-02-2002

    		mxser_start(tty);
	}
}

/*
 * mxser_stop() and mxser_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 */
static void mxser_stop(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	if ( info->IER & UART_IER_THRI ) {
	    info->IER &= ~UART_IER_THRI;
	    outb(info->IER, info->base + UART_IER);
	}
	MX_UNLOCK(&info->slock);
}

static void mxser_start(struct tty_struct * tty)
{
	struct mxser_struct *info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();
	
	MX_LOCK(&info->slock);
	if ( info->xmit_cnt && info->xmit_buf /*&&
	     !(info->IER & UART_IER_THRI)*/ ) {
	    info->IER &= ~UART_IER_THRI	;
	    outb(info->IER, info->base + UART_IER);
	    info->IER |= UART_IER_THRI;
	    outb(info->IER, info->base + UART_IER);
	}
	MX_UNLOCK(&info->slock);
}

/*
 * mxser_wait_until_sent() --- wait until the transmitter is empty
 */
static void mxser_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct mxser_struct * info = (struct mxser_struct *)tty->driver_data;
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
	if (timeout > 0 && timeout < char_time)
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
	pr_info("In rs_wait_until_sent(%d) check=%d...", timeout, char_time);
	pr_info("jiff=%lu...", jiffies);
#endif
	while (!((lsr = inb(info->base+ UART_LSR)) & UART_LSR_TEMT)) {
#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
		pr_info("lsr = %d (jiff=%lu)...", lsr, jiffies);
#endif
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(char_time);
		if (signal_pending(current))
			break;
		if (timeout > 0 && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
		set_current_state(TASK_RUNNING);

#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
	pr_info("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
}

/*
 * This routine is called by tty_hangup() when a hangup is signaled.
 */
void mxser_hangup(struct tty_struct * tty)
{
	struct mxser_struct * info = (struct mxser_struct *)tty->driver_data;

	mxser_flush_buffer(tty);
	mxser_shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}


// added by James 03-12-2004.
/*
 * mxser_rs_break() --- routine which turns the break handling on or off
 */
static int mxser_rs_break(struct tty_struct *tty, int break_state)
{
	struct mxser_struct * info = (struct mxser_struct *)tty->driver_data;
	MX_LOCK_INIT();
	
	MX_LOCK(&info->slock);
	if (break_state == -1)
		outb(inb(info->base + UART_LCR) | UART_LCR_SBC, info->base + UART_LCR);
	else
		outb(inb(info->base + UART_LCR) & ~UART_LCR_SBC, info->base + UART_LCR);
	MX_UNLOCK(&info->slock);
	return 0;
}
// (above) added by James.


/*
 * This is the serial driver's generic interrupt routine
 */
static irqreturn_t mxser_interrupt(int irq, void *dev_id)
{
	int			status, iir, i;
	struct mxser_struct *	info;
	struct mxser_struct *	port;
	int			max, irqbits, bits, msr;
	int			pass_counter = 0;
	int			int_cnt;
	int			handled = 0;

	port = 0;
	//spin_lock(&gm_lock);

        for(i=0; i<MXSER_BOARDS; i++){
            if(dev_id == &(mxvar_table[i*MXSER_PORTS_PER_BOARD])){
                port = dev_id;
                break;
            }
        }

        if(i==MXSER_BOARDS){
            goto irq_stop;
	}
        if(port==0){
            goto irq_stop;
	}
        max = mxser_numports[mxsercfg[i].board_type-1];
	while ( 1 ) {
	    irqbits = inb(port->vector) & port->vectormask;
	    if ( irqbits == port->vectormask ){
		break;
	    }
	    
	    handled = 1;
	    for ( i=0, bits=1; i<max; i++, irqbits |= bits, bits <<= 1 ) {
		if ( irqbits == port->vectormask ){
		    break;
		}
		if ( bits & irqbits )
		    continue;
		info = port + i;

		int_cnt = 0;
		do{
		    // following add by Victor Yu. 09-13-2002
		    iir = inb(info->base+UART_IIR);
		    if ( iir & UART_IIR_NO_INT )
			break;
		    iir &= MOXA_MUST_IIR_MASK;
		    if ( !info->tty ) {
			status = inb(info->base+UART_LSR);
			outb(0x27, info->base+UART_FCR);
			inb(info->base+UART_MSR);
			break;
		    }
		    // above add by Victor Yu. 09-13-2002

		    spin_lock(&info->slock);
		    // following add by Victor Yu. 09-02-2002
		    status = inb(info->base+UART_LSR);
	
		    if(status & UART_LSR_PE)
		        info->err_shadow |= NPPI_NOTIFY_PARITY;
		    if(status & UART_LSR_FE)
		        info->err_shadow |= NPPI_NOTIFY_FRAMING;
		    if(status & UART_LSR_OE) 
		        info->err_shadow |= NPPI_NOTIFY_HW_OVERRUN;
		    if(status & UART_LSR_BI)
		        info->err_shadow |= NPPI_NOTIFY_BREAK;
		        
		    if ( info->IsMoxaMustChipFlag ) {
			/*
			if ( (status & 0x02) && !(status & 0x01) ) {
				outb(info->base+UART_FCR,  0x23);
				continue;
			}
			*/
			if ( iir == MOXA_MUST_IIR_GDA ||
			     iir == MOXA_MUST_IIR_RDA ||
			     iir == MOXA_MUST_IIR_RTO ||
			     iir == MOXA_MUST_IIR_LSR )
				mxser_receive_chars(info, &status);
			
		    } else {
		    // above add by Victor Yu. 09-02-2002

			status &= info->read_status_mask;
			if ( status & UART_LSR_DR )
			    mxser_receive_chars(info, &status);
		    }
		    msr = inb(info->base + UART_MSR);
		    if ( msr & UART_MSR_ANY_DELTA ) {
		        mxser_check_modem_status(info, msr);
		    }

		    // following add by Victor Yu. 09-13-2002
		    if ( info->IsMoxaMustChipFlag ) {
			if ((iir == 0x02 ) && (status & UART_LSR_THRE)){
				mxser_transmit_chars(info);
			}
		    } else {
		    // above add by Victor Yu. 09-13-2002

			if ( status & UART_LSR_THRE ) {
/* 8-2-99 by William
			    if ( info->x_char || (info->xmit_cnt > 0) )
*/
				mxser_transmit_chars(info);
			}
		    }
		    spin_unlock(&info->slock);
		}while(int_cnt++ < MXSER_ISR_PASS_LIMIT);
	    }
	    if ( pass_counter++ > MXSER_ISR_PASS_LIMIT ) 
		break;	/* Prevent infinite loops */
	}
	
irq_stop:
        //spin_unlock(&gm_lock);
	return IRQ_RETVAL(handled);	
}

static void mxser_receive_chars(struct mxser_struct *info,
					 int *status)
{
	struct tty_struct *	tty = info->tty;
	unsigned char		ch, gdl;
	int			ignored = 0;
	int			cnt = 0;
	int			count;
	int 			recv_room;
	int			max = 256;
	unsigned long 		flags;

        count = 0;

	// following add by Victor Yu. 09-02-2002
	if ( info->IsMoxaMustChipFlag != MOXA_OTHER_UART) {

		if ( *status & UART_LSR_SPECIAL ) {
			goto intr_old;
		}

		// following add by Victor Yu. 02-11-2004
		if ( info->IsMoxaMustChipFlag == MOXA_MUST_MU860_HWID &&
		     (*status & MOXA_MUST_LSR_RERR) )
			goto intr_old;
		// above add by Victor Yu. 02-14-2004
		if(*status & MOXA_MUST_LSR_RERR)
			goto intr_old;

		gdl = inb(info->base+MOXA_MUST_GDL_REGISTER);

		if ( info->IsMoxaMustChipFlag == MOXA_MUST_MU150_HWID )
			gdl &= MOXA_MUST_GDL_MASK;

		while ( gdl-- ) {
	    		ch = inb(info->base + UART_RX);
	    		count++;
			tty_insert_flip_char(&info->ttyPort, ch, 0);
			cnt++;
		}
		goto end_intr;
	}
intr_old:
	// above add by Victor Yu. 09-02-2002

	do {
	    if(max-- <0)
 		break;

	    ch = inb(info->base + UART_RX);
	    // following add by Victor Yu. 09-02-2002
	    if ( info->IsMoxaMustChipFlag && (*status&UART_LSR_OE) /*&& !(*status&UART_LSR_DR)*/ )
			outb(0x23, info->base+UART_FCR);
	    	*status &= info->read_status_mask;
	    // above add by Victor Yu. 09-02-2002
	    	if ( *status & info->ignore_status_mask ) {
			if ( ++ignored > 100 )
		    	break;
	    } else {
		count++;
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
		tty_insert_flip_char(&info->ttyPort, ch, flags);
		cnt++;

	    }

	    // following add by Victor Yu. 09-02-2002
	    if ( info->IsMoxaMustChipFlag )
		break;
	    // above add by Victor Yu. 09-02-2002

	    /* mask by Victor Yu. 09-02-2002
	    *status = inb(info->base + UART_LSR) & info->read_status_mask;
	    */
	    // following add by Victor Yu. 09-02-2002
	    *status = inb(info->base+UART_LSR);
	    // above add by Victor Yu. 09-02-2002
	} while ( *status & UART_LSR_DR );

end_intr:	// add by Victor Yu. 09-02-2002

	mxvar_log.rxcnt[info->port] += cnt;
	info->mon_data.rxcnt += cnt;
	info->mon_data.up_rxcnt += cnt;
        info->icount.rx += cnt;

	recv_room = tty->receive_room;
	if(recv_room < (128*2)){
		set_bit(TTY_THROTTLED, &tty->flags);
        	mxser_stoprx(tty);
	}	

    /*   
     * Follow the code snippets in 8250_core.c,
     * we release the lock and then hold the lock again when calling tty_flip_buffer_push().
     * Otherwise, it will easily get kernel panic in real time Linux. 
     */
    spin_unlock(&info->slock);
	tty_flip_buffer_push(&info->ttyPort);
	spin_lock(&info->slock);

}

static void mxser_transmit_chars(struct mxser_struct *info)
{
	int	count, cnt;
	//MX_LOCK_INIT();

	if ( info->x_char ) {
	    //MX_LOCK(&info->slock);
	    outb(info->x_char, info->base + UART_TX);
	    info->x_char = 0;
	    mxvar_log.txcnt[info->port]++;
	    info->mon_data.txcnt++;
	    info->mon_data.up_txcnt++;

/* added by casper 1/11/2000 */
	    info->icount.tx++;
/* */
	    //MX_UNLOCK(&info->slock);
	    return;
	}

	//MX_LOCK(&info->slock);
	if ( info->xmit_buf == 0 ){
	    //MX_UNLOCK(&info->slock);
	    return;
	}

	if(info->xmit_cnt==0){
		if ( info->xmit_cnt < WAKEUP_CHARS ) {
			set_bit(MXSER_EVENT_TXLOW,&info->event);
			MXQ_TASK();
		}
	    	//MX_UNLOCK(&info->slock);
		return;
	}

	if (/*(info->xmit_cnt <= 0) ||*/ info->tty->stopped ||
	    (info->tty->hw_stopped && (info->type != PORT_16550A) &&(!info->IsMoxaMustChipFlag))) {
		info->IER &= ~UART_IER_THRI;
		outb(info->IER, info->base + UART_IER);
	        //MX_UNLOCK(&info->slock);
		return;
	}

	cnt = info->xmit_cnt;
	count = info->xmit_fifo_size;
	do {

//pr_info("%lu, mxser_transmit_chars=[%x]\n", 
//	jiffies, info->xmit_buf[info->xmit_tail]);

	    outb(info->xmit_buf[info->xmit_tail++], info->base + UART_TX);
	    info->xmit_tail = info->xmit_tail & (SERIAL_XMIT_SIZE - 1);
	    if ( --info->xmit_cnt <= 0 )
		break;
	} while ( --count > 0 );
	mxvar_log.txcnt[info->port] += (cnt - info->xmit_cnt);

// added by James 03-12-2004.
	info->mon_data.txcnt += (cnt - info->xmit_cnt);
	info->mon_data.up_txcnt += (cnt - info->xmit_cnt);
// (above) added by James.

/* added by casper 1/11/2000 */
        info->icount.tx += (cnt - info->xmit_cnt);
/* */

	if ( info->xmit_cnt < WAKEUP_CHARS ) {
		set_bit(MXSER_EVENT_TXLOW,&info->event);
		MXQ_TASK();
	}
/*
	if (info->xmit_cnt <= 0) {
		info->IER &= ~UART_IER_THRI;
		outb(info->IER, info->base + UART_IER);
	}*/
	//MX_UNLOCK(&info->slock);
}

static void mxser_check_modem_status(struct mxser_struct *info,
					      int status)
{
	struct tty_struct *tty = info->tty;
	struct tty_ldisc *ld;

	/* update input line counters */
	if ( status & UART_MSR_TERI )
	    info->icount.rng++;
	if ( status & UART_MSR_DDSR )
	    info->icount.dsr++;
	if ( status & UART_MSR_DDCD ) {
	    if( tty ) {
		ld = tty_ldisc_ref(tty);
		if( ld )
		    if ( ld->ops->dcd_change)
			ld->ops->dcd_change(tty, ((status & UART_MSR_DCD)? TIOCM_CAR : 0));
		tty_ldisc_deref(ld);
	    }
	    info->icount.dcd++;
	}
	if ( status & UART_MSR_DCTS )
	    info->icount.cts++;
	info->mon_data.modem_status = status;
	wake_up_interruptible(&info->delta_msr_wait);
	

	if ( (info->flags & ASYNC_CHECK_CD) && (status & UART_MSR_DDCD) ) {
	    if ( status & UART_MSR_DCD )
		wake_up_interruptible(&info->open_wait);
	    else if ( !((info->flags & ASYNC_CALLOUT_ACTIVE) &&
		      (info->flags & ASYNC_CALLOUT_NOHUP)) )

	        set_bit(MXSER_EVENT_HANGUP,&info->event);
            MXQ_TASK();
	}

	if ( info->flags & ASYNC_CTS_FLOW ) {
	    if ( info->tty->hw_stopped ) {
			if (status & UART_MSR_CTS ){
		    	info->tty->hw_stopped = 0;

		    	if ((info->type != PORT_16550A) && (!info->IsMoxaMustChipFlag)){
					info->IER &= ~UART_IER_THRI;
					outb(info->IER, info->base + UART_IER);
					info->IER |= UART_IER_THRI;
					outb(info->IER, info->base + UART_IER);
		    	}
	       		set_bit(MXSER_EVENT_TXLOW,&info->event);
	       		MXQ_TASK();
        	}
	    } else {
			if ( !(status & UART_MSR_CTS) ){
		    	info->tty->hw_stopped = 1;
		    	if ((info->type != PORT_16550A) && (!info->IsMoxaMustChipFlag)) {
					info->IER &= ~UART_IER_THRI;
					outb(info->IER, info->base + UART_IER);
		    	}
			}
	    }
	}
}

static int mxser_block_til_ready(struct tty_struct *tty, struct file * filp,
				 struct mxser_struct *info)
{
	DECLARE_WAITQUEUE(wait, current);
	int			retval;
	int			do_clocal = 0;
	MX_LOCK_INIT();

#if 0
	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if ( tty_hung_up_p(filp) || (info->flags & ASYNC_CLOSING) ) {
	    if ( info->flags & ASYNC_CLOSING )
		interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
	    if ( info->flags & ASYNC_HUP_NOTIFY )
		return(-EAGAIN);
	    else
		return(-ERESTARTSYS);
#else
	    return(-EAGAIN);
#endif
	}
#endif
	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ( (filp->f_flags & O_NONBLOCK) ||
	     (tty->flags & (1 << TTY_IO_ERROR)) ) {
	    if ( info->flags & ASYNC_CALLOUT_ACTIVE )
		return(-EBUSY);
	    info->flags |= ASYNC_NORMAL_ACTIVE;
	    return(0);
	}

	if ( info->flags & ASYNC_CALLOUT_ACTIVE ) {
	    if ( info->normal_termios.c_cflag & CLOCAL )
		do_clocal = 1;
	} else {
	    if ( tty->termios.c_cflag & CLOCAL ) 		
		do_clocal = 1;
	}

	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * mxser_close() knows when to free things.  We restore it upon
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
		outb(inb(info->base + UART_MCR) | UART_MCR_DTR | UART_MCR_RTS,
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
	    if ( !(info->flags & ASYNC_CALLOUT_ACTIVE) &&
		 !(info->flags & ASYNC_CLOSING) &&
		 (do_clocal || (inb(info->base + UART_MSR) & UART_MSR_DCD)) )
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
	    return(retval);
	info->flags |= ASYNC_NORMAL_ACTIVE;
	return(0);
}

static int mxser_startup(struct mxser_struct * info)
{
	
	unsigned long	page;
	MX_LOCK_INIT();

	page = GET_FPAGE(GFP_KERNEL);
	if ( !page )
	    return(-ENOMEM);

	MX_LOCK(&info->slock);

	if ( info->flags & ASYNC_INITIALIZED ) {
	    free_page(page);
	    MX_UNLOCK(&info->slock);
	    return(0);
	}

	if ( !info->base || !info->type ) {
	    if ( info->tty )
		set_bit(TTY_IO_ERROR, &info->tty->flags);
	    free_page(page);
	    MX_UNLOCK(&info->slock);
	    return(0);
	}
	if ( info->xmit_buf )
	    free_page(page);
	else
	    info->xmit_buf = (unsigned char *)page;

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled in mxser_change_speed())
	 */
	if ( info->IsMoxaMustChipFlag )
	    outb((UART_FCR_CLEAR_RCVR|UART_FCR_CLEAR_XMIT|MOXA_MUST_FCR_GDA_MODE_ENABLE), info->base+UART_FCR);
	else
	    outb((UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT),
		 info->base + UART_FCR);

	/*
	 * At this point there's no way the LSR could still be 0xFF;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if ( inb(info->base + UART_LSR) == 0xff ) {
	    MX_UNLOCK(&info->slock);
            if (capable(CAP_SYS_ADMIN)) {
		if ( info->tty )
		    set_bit(TTY_IO_ERROR, &info->tty->flags);
		return(0);
	    } else
		return(-ENODEV);
	}

	/*
	 * Clear the interrupt registers.
	 */
	(void)inb(info->base + UART_LSR);
	(void)inb(info->base + UART_RX);
	(void)inb(info->base + UART_IIR);
	(void)inb(info->base + UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	outb(UART_LCR_WLEN8, info->base + UART_LCR);	/* reset DLAB */
	info->MCR = UART_MCR_DTR | UART_MCR_RTS;
	outb(info->MCR, info->base + UART_MCR);

	/*
	 * Finally, enable interrupts
	 */
	info->IER = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;

	// following add by Victor Yu. 08-30-2002
	if ( info->IsMoxaMustChipFlag )
		info->IER |= MOXA_MUST_IER_EGDAI;
	// above add by Victor Yu. 08-30-2002
	outb(info->IER, info->base + UART_IER); /* enable interrupts */

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void)inb(info->base + UART_LSR);
	(void)inb(info->base + UART_RX);
	(void)inb(info->base + UART_IIR);
	(void)inb(info->base + UART_MSR);

	if ( info->tty )
	    test_and_clear_bit(TTY_IO_ERROR, &info->tty->flags);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	/*
	 * and set the speed of the serial port
	 */
	MX_UNLOCK(&info->slock);

	info->flags |= ASYNC_INITIALIZED;
	return(0);
}

/*
 * This routine will shutdown a serial port; interrupts maybe disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void mxser_shutdown(struct mxser_struct * info)
{
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

	info->IER = 0;
	outb(0x00, info->base + UART_IER);
	if ( !info->tty || (info->tty->termios.c_cflag & HUPCL) )
	    info->MCR &= ~(UART_MCR_DTR | UART_MCR_RTS);
	outb(info->MCR, info->base + UART_MCR);

	/* clear Rx/Tx FIFO's */
	// following add by Victor Yu. 08-30-2002
	if ( info->IsMoxaMustChipFlag )
		outb((UART_FCR_CLEAR_RCVR|UART_FCR_CLEAR_XMIT|MOXA_MUST_FCR_GDA_MODE_ENABLE), info->base + UART_FCR);
	else
	// above add by Victor Yu. 08-30-2002
		outb((UART_FCR_CLEAR_RCVR|UART_FCR_CLEAR_XMIT), info->base + UART_FCR);

	/* read data port to reset things */
	(void)inb(info->base + UART_RX);

	if ( info->tty )
	    set_bit(TTY_IO_ERROR, &info->tty->flags);

	info->flags &= ~ASYNC_INITIALIZED;

	// following add by Victor Yu. 09-23-2002
	if ( info->IsMoxaMustChipFlag ) {
		SET_MOXA_MUST_NO_SOFTWARE_FLOW_CONTROL(info->base);
	}
	// above add by Victor Yu. 09-23-2002

	MX_UNLOCK(&info->slock);
}

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
static int mxser_change_speed(struct mxser_struct *info, 
                              struct ktermios *old_termios)
{
	unsigned	cflag, cval, fcr;
        int             ret = 0;
	unsigned char status;
	long baud;
	//MX_LOCK_INIT();

	if ( !info->tty ) 
	    return ret;
	cflag = info->tty->termios.c_cflag;
	if ( !(info->base) )
	    return ret;

#ifndef B921600
#define B921600 (B460800 +1)
#endif
	if( mxser_set_baud_method[info->port] == 0)
	{
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
       	mxser_set_baud(info, baud);
	}

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
	if ( !(cflag & PARODD) ){
	    cval |= UART_LCR_EPAR;
	}
	if ( cflag & CMSPAR )
	   cval |= UART_LCR_SPAR;

	if ( (info->type == PORT_8250) || (info->type == PORT_16450) ) {
	    if ( info->IsMoxaMustChipFlag ) {
			fcr = UART_FCR_ENABLE_FIFO;
			fcr |= MOXA_MUST_FCR_GDA_MODE_ENABLE;
			SET_MOXA_MUST_FIFO_VALUE(info);
	    }else
	    	fcr = 0;
	} else {
	    fcr = UART_FCR_ENABLE_FIFO;
	    // following add by Victor Yu. 08-30-2002
	    if ( info->IsMoxaMustChipFlag ) {
			fcr |= MOXA_MUST_FCR_GDA_MODE_ENABLE;
			SET_MOXA_MUST_FIFO_VALUE(info);
	    } else {
	    // above add by Victor Yu. 08-30-2002

	    	switch ( info->rx_trigger ) {
	    		case 1:  fcr |= UART_FCR_TRIGGER_1; break;
	    		case 4:  fcr |= UART_FCR_TRIGGER_4; break;
	    		case 8:  fcr |= UART_FCR_TRIGGER_8; break;
	    		default: fcr |= UART_FCR_TRIGGER_14;break;
	    	}
	    }
	}

	/* CTS flow control flag and modem status interrupts */
	info->IER &= ~UART_IER_MSI;  
	info->MCR &= ~UART_MCR_AFE;

	if( info->flags & ASYNC_HARDPPS_CD ) {
	    info->IER |= UART_IER_MSI;
	}

	if ( cflag & CRTSCTS ) {
	    info->flags |= ASYNC_CTS_FLOW;
	    info->IER |= UART_IER_MSI;   
	    if (( info->type == PORT_16550A ) || (info->IsMoxaMustChipFlag)){
			info->MCR |= UART_MCR_AFE;
		}else {
	                status = inb(info->base + UART_MSR);
			if (info->tty->hw_stopped) {
				if (status & UART_MSR_CTS) {
					info->tty->hw_stopped = 0;
					if ((info->type != PORT_16550A) && (!info->IsMoxaMustChipFlag)){
						info->IER &= ~UART_IER_THRI;
						outb(info->IER, info->base + UART_IER);
						info->IER |= UART_IER_THRI;
						outb(info->IER, info->base + UART_IER);
					}
					set_bit(MXSER_EVENT_TXLOW, &info->event);
					MXQ_TASK();
				}
			} else {
				if (!(status & UART_MSR_CTS)) {
					info->tty->hw_stopped = 1;
					if ((info->type != PORT_16550A) && (!info->IsMoxaMustChipFlag)){
						info->IER &= ~UART_IER_THRI;
						outb(info->IER, info->base + UART_IER);
					}
				}
			}
		}
	} else {
	    info->flags &= ~ASYNC_CTS_FLOW;
	}
	outb(info->MCR, info->base + UART_MCR);
	if ( cflag & CLOCAL ){
	    info->flags &= ~ASYNC_CHECK_CD;
	}else {
	    info->flags |= ASYNC_CHECK_CD;
	    info->IER |= UART_IER_MSI;
	}
	outb(info->IER, info->base + UART_IER);

	/*
	 * Set up parity check flag
	 */
	info->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if ( I_INPCK(info->tty) )
	    info->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if ( I_BRKINT(info->tty) || I_PARMRK(info->tty) )
	    info->read_status_mask |= UART_LSR_BI;

	info->ignore_status_mask = 0;
#if 0
	/* This should be safe, but for some broken bits of hardware... */
	if ( I_IGNPAR(info->tty) ) {
	    info->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	    info->read_status_mask |= UART_LSR_PE | UART_LSR_FE;
	}
#endif
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

	// following add by Victor Yu. 09-02-2002
	if ( info->IsMoxaMustChipFlag ) {
		SET_MOXA_MUST_XON1_VALUE(info->base, START_CHAR(info->tty));
		SET_MOXA_MUST_XOFF1_VALUE(info->base, STOP_CHAR(info->tty));
		if ( I_IXON(info->tty) ) {
			ENABLE_MOXA_MUST_RX_SOFTWARE_FLOW_CONTROL(info->base);
		} else {
			DISABLE_MOXA_MUST_RX_SOFTWARE_FLOW_CONTROL(info->base);
		}
		if ( I_IXOFF(info->tty) ) {
			ENABLE_MOXA_MUST_TX_SOFTWARE_FLOW_CONTROL(info->base);
		} else {
			DISABLE_MOXA_MUST_TX_SOFTWARE_FLOW_CONTROL(info->base);
		}
		/*
		if ( I_IXANY(info->tty) ) {
			info->MCR |= MOXA_MUST_MCR_XON_ANY;
			ENABLE_MOXA_MUST_XON_ANY_FLOW_CONTROL(info->base);
		} else {
			info->MCR &= ~MOXA_MUST_MCR_XON_ANY;
			DISABLE_MOXA_MUST_XON_ANY_FLOW_CONTROL(info->base);
		}
		*/
	}
	// above add by Victor Yu. 09-02-2002

	outb(fcr, info->base + UART_FCR);		    /* set fcr */
	outb(cval, info->base + UART_LCR);  
	
        return ret;
}


static int mxser_set_baud(struct mxser_struct *info, long newspd)
{
	int		i;
	int		quot = 0;
	unsigned char	cval;
        int             ret = 0;
	//MX_LOCK_INIT();
	if ( !info->tty )
	    return ret;

	if ( !(info->base) )
	    return ret;

    if ( newspd > info->MaxCanSetBaudRate )
	    return 0;

    info->realbaud = newspd;
	for ( i=0; i<BAUD_TABLE_NO && newspd != mxvar_baud_table[i]; i++ );
	if ( i == BAUD_TABLE_NO ){
		quot = info->baud_base / info->speed;
		if ( info->speed <= 0 || info->speed > info->MaxCanSetBaudRate )
			quot = 0;
	}else{
	if ( newspd == 134 ) {
	    quot = (2 * info->baud_base / 269);
	    info->speed = 134;
	} else if ( newspd ) {
	    quot = info->baud_base / newspd;

	    if(quot==0)
	        quot = 1;

	} else {
	    quot = 0;
	}
	}

	info->timeout = (int)((unsigned int)(info->xmit_fifo_size*HZ*10*quot) / (unsigned int)info->baud_base);
	info->timeout += HZ/50;		/* Add .02 seconds of slop */

	if ( quot ) {
	    //MX_LOCK(&info->slock);
	    info->MCR |= UART_MCR_DTR;
	    outb(info->MCR, info->base + UART_MCR);
	    //MX_UNLOCK(&info->slock);
	} else {
	    //MX_LOCK(&info->slock);
	    info->MCR &= ~UART_MCR_DTR;
	    outb(info->MCR, info->base + UART_MCR);
	    //MX_UNLOCK(&info->slock);
	    return ret;
	}

    cval = inb(info->base + UART_LCR);

	outb(cval | UART_LCR_DLAB, info->base + UART_LCR);  /* set DLAB */

	outb(quot & 0xff, info->base + UART_DLL);	    /* LS of divisor */
	outb(quot >> 8, info->base + UART_DLM); 	    /* MS of divisor */
	outb(cval, info->base + UART_LCR);		    /* reset DLAB */

	if ( i == BAUD_TABLE_NO ){
		quot = info->baud_base % info->speed;
		quot *= 8;
		if ( (quot % info->speed) > (info->speed / 2) ) {
			quot /= info->speed;
			quot++;
		} else {
			quot /= info->speed;
		}
		SET_MOXA_MUST_ENUM_VALUE(info->base, quot);
	} else {
		SET_MOXA_MUST_ENUM_VALUE(info->base, 0);
	}

    return ret;
}



/*
 * ------------------------------------------------------------
 * friends of mxser_ioctl()
 * ------------------------------------------------------------
 */

#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
static int mxser_get_serial_info(struct tty_struct * tty,
				 struct serial_struct * retinfo)
{
	struct mxser_struct	*info;

	if ( !retinfo || !tty)
	    return(-EFAULT);

	info = (struct mxser_struct *)tty->driver_data;

	retinfo->type = info->type;
	retinfo->line = info->port;
	retinfo->port = info->base;
	retinfo->irq = info->irq;
	retinfo->flags = info->flags;
	retinfo->baud_base = info->baud_base;
	retinfo->close_delay = info->close_delay;
	retinfo->closing_wait = info->closing_wait;
	retinfo->custom_divisor = info->custom_divisor;
	retinfo->hub6 = 0;

	return(0);
}
#else
static int mxser_get_serial_info(struct mxser_struct * info,
				 struct serial_struct * retinfo)
{
	struct serial_struct	tmp;

	if ( !retinfo )
	    return(-EFAULT);
	memset(&tmp, 0, sizeof(tmp));
	tmp.type = info->type;
	tmp.line = info->port;
	tmp.port = info->base;
	tmp.irq = info->irq;
	tmp.flags = info->flags;
	tmp.baud_base = info->baud_base;
	tmp.close_delay = info->close_delay;
	tmp.closing_wait = info->closing_wait;
	tmp.custom_divisor = info->custom_divisor;
	tmp.hub6 = 0;
	if(copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
	    return -EFAULT;
	return(0);
}
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)) || defined(RHEL8_PATCH1)
static int mxser_set_serial_info(struct tty_struct * tty,
				 struct serial_struct * new_info)
{
	struct mxser_struct	*info;
	unsigned int		flags;
	int			retval = 0;

	MX_LOCK_INIT();

	if ( !new_info || !tty )
	    return(-EFAULT);

	info = (struct mxser_struct *)tty->driver_data;

	if ( (new_info->irq != info->irq) ||
	     (new_info->port != info->base) )
	    return(-EPERM);

	flags = info->flags & ASYNC_SPD_MASK;

       if ( !capable(CAP_SYS_ADMIN)) {
	    if ( (new_info->baud_base != info->baud_base) ||
		 (new_info->close_delay != info->close_delay) ||
		 ((new_info->flags & ~ASYNC_USR_MASK) !=
		 (info->flags & ~ASYNC_USR_MASK)) )
		return(-EPERM);
	    info->flags = ((info->flags & ~ASYNC_USR_MASK) |
			  (new_info->flags & ASYNC_USR_MASK));
	} else {
	    /*
	     * OK, past this point, all the error checking has been done.
	     * At this point, we start making changes.....
	     */
	    info->flags = ((info->flags & ~ASYNC_FLAGS) |
			  (new_info->flags & ASYNC_FLAGS));
	    info->close_delay = new_info->close_delay * HZ/100;
	    info->closing_wait = new_info->closing_wait * HZ/100;
	    //info->ttyPort.low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;
	    info->ttyPort.low_latency = 0;
	    if( (new_info->baud_base != info->baud_base) ||
	    (new_info->custom_divisor != info->custom_divisor) )
		info->custom_baud_rate = new_info->baud_base/new_info->custom_divisor;
      }

/* added by casper, 3/17/2000, for mouse */
	info->type = new_info->type;

	process_txrx_fifo(info);

	if ( info->flags & ASYNC_INITIALIZED ) {
	    if ( flags != (info->flags & ASYNC_SPD_MASK) ){
		MX_LOCK(&info->slock);
		mxser_change_speed(info,0);
		MX_UNLOCK(&info->slock);
	    }
	} else{
	    retval = mxser_startup(info);
	}
	return(retval);
}
#else
static int mxser_set_serial_info(struct mxser_struct * info,
				 struct serial_struct * new_info)
{
	struct serial_struct	new_serial;
	unsigned int		flags;
	int			retval = 0;

	MX_LOCK_INIT();

	if ( !new_info || !info->base )
	    return(-EFAULT);
	if(copy_from_user(&new_serial, new_info, sizeof(new_serial)))
	    return -EFAULT;

	if ( (new_serial.irq != info->irq) ||
	     (new_serial.port != info->base) )
	    return(-EPERM);

	flags = info->flags & ASYNC_SPD_MASK;

       if ( !capable(CAP_SYS_ADMIN)) {
	    if ( (new_serial.baud_base != info->baud_base) ||
		 (new_serial.close_delay != info->close_delay) ||
		 ((new_serial.flags & ~ASYNC_USR_MASK) !=
		 (info->flags & ~ASYNC_USR_MASK)) )
		return(-EPERM);
	    info->flags = ((info->flags & ~ASYNC_USR_MASK) |
			  (new_serial.flags & ASYNC_USR_MASK));
	} else {
	    /*
	     * OK, past this point, all the error checking has been done.
	     * At this point, we start making changes.....
	     */
	    info->flags = ((info->flags & ~ASYNC_FLAGS) |
			  (new_serial.flags & ASYNC_FLAGS));
	    info->close_delay = new_serial.close_delay * HZ/100;
	    info->closing_wait = new_serial.closing_wait * HZ/100;
	    //info->ttyPort.low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;
	    info->ttyPort.low_latency = 0;
	    if( (new_serial.baud_base != info->baud_base) ||
	    (new_serial.custom_divisor != info->custom_divisor) )
		info->custom_baud_rate = new_serial.baud_base/new_serial.custom_divisor;
      }

/* added by casper, 3/17/2000, for mouse */
	info->type = new_serial.type;

	process_txrx_fifo(info);

	if ( info->flags & ASYNC_INITIALIZED ) {
	    if ( flags != (info->flags & ASYNC_SPD_MASK) ){
		MX_LOCK(&info->slock);
		mxser_change_speed(info,0);
		MX_UNLOCK(&info->slock);
	    }
	} else{
	    retval = mxser_startup(info);
	}
	return(retval);
}
#endif

/*
 * mxser_get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 *	    is emptied.  On bus types like RS485, the transmitter must
 *	    release the bus after transmitting. This must be done when
 *	    the transmit shift register is empty, not be done when the
 *	    transmit holding register is empty.  This functionality
 *	    allows an RS485 driver to be written in user space.
 */
static int mxser_get_lsr_info(struct mxser_struct * info, unsigned int *value)
{
	unsigned char	status;
	unsigned int	result;
	MX_LOCK_INIT();

	MX_LOCK(&info->slock);
	status = inb(info->base + UART_LSR);
	MX_UNLOCK(&info->slock);
	result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	put_to_user(result, value);
	return(0);
}

/*
 * This routine sends a break character out the serial port.
 */
static void mxser_send_break(struct mxser_struct * info, int duration)
{
	MX_LOCK_INIT();

	if ( !info->base )
	    return;
	set_current_state(TASK_INTERRUPTIBLE);
	MX_LOCK(&info->slock);
	outb(inb(info->base + UART_LCR) | UART_LCR_SBC, info->base + UART_LCR);
	MX_UNLOCK(&info->slock);
	schedule_timeout(duration);
	MX_LOCK(&info->slock);
	outb(inb(info->base + UART_LCR) & ~UART_LCR_SBC, info->base + UART_LCR);
	MX_UNLOCK(&info->slock);
}

static int mxser_tiocmget(struct tty_struct *tty)
{
	struct mxser_struct *info = (struct mxser_struct *) tty->driver_data;
	unsigned char control, status;
	MX_LOCK_INIT();
//pr_info("%lu,tiocmget\n", jiffies);

	if (PORTNO(tty) == MXSER_PORTS)
		return (-ENOIOCTLCMD);
	if (tty->flags & (1 << TTY_IO_ERROR))
		return (-EIO);

	control = info->MCR;
	
	MX_LOCK(&info->slock);
	status = inb(info->base + UART_MSR);
	if (status & UART_MSR_ANY_DELTA)
		mxser_check_modem_status(info, status);
	MX_UNLOCK(&info->slock);
	return ((control & UART_MCR_RTS) ? TIOCM_RTS : 0) |
	    ((control & UART_MCR_DTR) ? TIOCM_DTR : 0) |
	    ((status & UART_MSR_DCD) ? TIOCM_CAR : 0) |
	    ((status & UART_MSR_RI) ? TIOCM_RNG : 0) |
	    ((status & UART_MSR_DSR) ? TIOCM_DSR : 0) |
	    ((status & UART_MSR_CTS) ? TIOCM_CTS : 0);
}

static int mxser_tiocmset(struct tty_struct *tty, unsigned int set,
			  unsigned int clear)
{
	struct mxser_struct *info = (struct mxser_struct *) tty->driver_data;
	MX_LOCK_INIT();


//pr_info("%lu,tiocmset\n", jiffies);

	if (PORTNO(tty) == MXSER_PORTS)
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

	outb(info->MCR, info->base + UART_MCR);
	MX_UNLOCK(&info->slock);
	return (0);
}

static int	mxser_read_register(int, unsigned short *);
static void mxser_pc_read_register(int, unsigned char *);
static int	mxser_program_mode(int);
static void	mxser_normal_mode(int);

static int mxser_get_PC_ISA_conf(int cap,int vector,
	struct mxser_hwconf *hwconf, int irq)
{
	int		id, i;
	unsigned char	scratch, scratch2;
	unsigned char	value[8];

	if ( !vector )
	    return(MXSER_ERR_VECTOR);

	if ( !irq ) 
	    return(MXSER_ERR_IRQ);

	mxser_pc_read_register(vector,value);

	id = value[2];
	if (id == CA104_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA104;
	    hwconf->ports = 4;
	}else if (id == CA132_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA132;
	    hwconf->ports = 2;
	}else if (id == CA132I_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA132I;
	    hwconf->ports = 2;
  }else if(id == CA108_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA108;
	    hwconf->ports = 8;
  }else if(id == CA114_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA114;
	    hwconf->ports = 4;
  }else if(id == CA134I_ID){
	    hwconf->board_type = MXSER_PC_BOARD_CA134I;
	    hwconf->ports = 4;
  }else{
	    return(0);
	}

	hwconf->irq = (int)irq;
	
	for ( i=0; i<hwconf->ports; i++ ){
		hwconf->ioaddr[i] = (int)(cap + (8 * i));
	}
	
	hwconf->vector = vector;
	if ( (id == CA104_ID) ||
			 (id == CA114_ID) ||
			 (id == CA134I_ID))
	    hwconf->vector_mask = 0x0F;
	else if(id == CA108_ID)
			hwconf->vector_mask = 0xFF;
	else 
	    hwconf->vector_mask = 0x03;

	hwconf->IsMoxaMustChipFlag = CheckIsMoxaMust(hwconf->ioaddr[0]);
	for ( i=0 ; i<hwconf->ports ; i++ ){
	    hwconf->baud_base[i] = 921600;
/*
	    if(hwconf->board_type < MXPCDRV_BOARD_INDEX)
	    		hwconf->MaxCanSetBaudRate[i] = 230400;
	    else
*/
	    		hwconf->MaxCanSetBaudRate[i] = 921600;
	}

	scratch2 = inb(cap + UART_LCR) & (~UART_LCR_DLAB);
	outb(scratch2 | UART_LCR_DLAB, cap + UART_LCR);
	outb(0, cap + UART_EFR);	/* EFR is the same as FCR */
	outb(scratch2, cap + UART_LCR);
	outb(UART_FCR_ENABLE_FIFO, cap + UART_FCR);
	scratch = inb(cap + UART_IIR);

	if ( scratch & 0xC0 )
	    hwconf->uart_type = PORT_16550A;
	else
	    hwconf->uart_type = PORT_16450;
	
	return(hwconf->ports);
}

static int mxser_get_ISA_conf(int cap,struct mxser_hwconf *hwconf)
{
	int		id, i, bits;
	unsigned short	regs[16], irq;
	unsigned char	scratch, scratch2;

	hwconf->IsMoxaMustChipFlag = MOXA_OTHER_UART;
 
	id = mxser_read_register(cap, regs);
	if (id == C168_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_C168_ISA;
	    hwconf->ports = 8;
	}else if (id == C104_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_C104_ISA;
	    hwconf->ports = 4;
	}else if (id == C102_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_C102_ISA;
	    hwconf->ports = 2;
	}else if (id == CI132_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_CI132;
	    hwconf->ports = 2;
	}else if (id == CI134_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_CI134;
	    hwconf->ports = 4;
	}else if (id == CI104J_ASIC_ID){
	    hwconf->board_type = MXSER_BOARD_CI104J;
	    hwconf->ports = 4;
        }else
	    return(0);

	irq = 0;
	if(hwconf->ports==2){
	    irq = regs[9] & 0xF000;
	    irq = irq | (irq>>4);
	    if (irq != (regs[9] & 0xFF00))
	        return(MXSER_ERR_IRQ_CONFLIT);
	}else if (hwconf->ports==4){
	    irq = regs[9] & 0xF000;
	    irq = irq | (irq>>4);
	    irq = irq | (irq>>8);
	    if (irq != regs[9])
	        return(MXSER_ERR_IRQ_CONFLIT);
	}else if (hwconf->ports==8){
	    irq = regs[9] & 0xF000;
	    irq = irq | (irq>>4);
	    irq = irq | (irq>>8);
	    if ((irq != regs[9]) || (irq != regs[10]))
	        return(MXSER_ERR_IRQ_CONFLIT);
        }

	if ( !irq ) {
	    return(MXSER_ERR_IRQ);
	}
	hwconf->irq = ((int)(irq & 0xF000) >>12);
	for ( i=0; i<8; i++ )
	    hwconf->ioaddr[i] = (int)regs[i + 1] & 0xFFF8;
	if ( (regs[12] & 0x80) == 0 ) {
	    return(MXSER_ERR_VECTOR);
	}
	hwconf->vector = (int)regs[11];		/* interrupt vector */
	if ( id == 1 )
	    hwconf->vector_mask = 0x00FF;
	else
	    hwconf->vector_mask = 0x000F;
	for ( i=7, bits=0x0100; i>=0; i--, bits <<= 1 ) {
	    if ( regs[12] & bits ) {
		hwconf->baud_base[i] = 921600;
		hwconf->MaxCanSetBaudRate[i] = 921600;	// add by Victor Yu. 09-04-2002
	    } else {
		hwconf->baud_base[i] = 115200;
		hwconf->MaxCanSetBaudRate[i] = 115200;	// add by Victor Yu. 09-04-2002
	    }
	}
	scratch2 = inb(cap + UART_LCR) & (~UART_LCR_DLAB);
	outb(scratch2 | UART_LCR_DLAB, cap + UART_LCR);
	outb(0, cap + UART_EFR);	/* EFR is the same as FCR */
	outb(scratch2, cap + UART_LCR);
	outb(UART_FCR_ENABLE_FIFO, cap + UART_FCR);
	scratch = inb(cap + UART_IIR);

	if ( scratch & 0xC0 )
	    hwconf->uart_type = PORT_16550A;
	else
	    hwconf->uart_type = PORT_16450;

	request_region(hwconf->ioaddr[0], 8*hwconf->ports, "mxser(IO)"); 
	request_region(hwconf->vector, 1, "mxser(vector)");  	    
	return(hwconf->ports);
}

#define CHIP_SK 	0x01		/* Serial Data Clock  in Eprom */
#define CHIP_DO 	0x02		/* Serial Data Output in Eprom */
#define CHIP_CS 	0x04		/* Serial Chip Select in Eprom */
#define CHIP_DI 	0x08		/* Serial Data Input  in Eprom */
#define EN_CCMD 	0x000		/* Chip's command register     */
#define EN0_RSARLO	0x008		/* Remote start address reg 0  */
#define EN0_RSARHI	0x009		/* Remote start address reg 1  */
#define EN0_RCNTLO	0x00A		/* Remote byte count reg WR    */
#define EN0_RCNTHI	0x00B		/* Remote byte count reg WR    */
#define EN0_DCFG	0x00E		/* Data configuration reg WR   */
#define EN0_PORT	0x010		/* Rcv missed frame error counter RD */
#define ENC_PAGE0	0x000		/* Select page 0 of chip registers   */
#define ENC_PAGE3	0x0C0		/* Select page 3 of chip registers   */
static void mxser_pc_read_register(int vect, unsigned char *buf)
{
	int i;

	for(i=0;i<8;i++){
		buf[i]  = inb((vect+i));
	}
	return;
}

static int mxser_read_register(int port, unsigned short *regs)
{
	int		i, k, value, id;
	unsigned int	j;

	id = mxser_program_mode(port);
	if ( id < 0 )
	    return(id);
	for ( i=0; i<14; i++ ) {
	    k = (i & 0x3F) | 0x180;
	    for ( j=0x100; j>0; j>>=1 ) {
		outb(CHIP_CS, port);
		if ( k & j ) {
		    outb(CHIP_CS | CHIP_DO, port);
		    outb(CHIP_CS | CHIP_DO | CHIP_SK, port);	/* A? bit of read */
		} else {
		    outb(CHIP_CS, port);
		    outb(CHIP_CS | CHIP_SK, port);	/* A? bit of read */
		}
	    }
	    (void)inb(port);
	    value = 0;
	    for ( k=0, j=0x8000; k<16; k++, j>>=1 ) {
		outb(CHIP_CS, port);
		outb(CHIP_CS | CHIP_SK, port);
		if ( inb(port) & CHIP_DI )
		    value |= j;
	    }
	    regs[i] = value;
	    outb(0, port);
	}
	mxser_normal_mode(port);
	return(id);
}

static int mxser_program_mode(int port)
{
	int	id, i, j, n;
	//unsigned long	flags;

	spin_lock(&gm_lock);
	outb(0, port);
	outb(0, port);
	outb(0, port);
	(void)inb(port);
	(void)inb(port);
	outb(0, port);
	(void)inb(port);
	//restore_flags(flags);
	spin_unlock(&gm_lock);
	
	id = inb(port + 1) & 0x1F;
	if ( (id != C168_ASIC_ID) &&
		(id != C104_ASIC_ID) &&
		(id != C102_ASIC_ID) &&
		(id != CI132_ASIC_ID) &&
		(id != CI134_ASIC_ID) &&
		(id != CI104J_ASIC_ID) )
	    return(-1);
	for ( i=0, j=0; i<4; i++ ) {
	    n = inb(port + 2);
	    if ( n == 'M' ) {
		j = 1;
	    } else if ( (j == 1) && (n == 1) ) {
		j = 2;
		break;
	    } else
		j = 0;
	}
	if ( j != 2 )
	    id = -2;
	return(id);
}

static void mxser_normal_mode(int port)
{
	int	i, n;

	outb(0xA5, port + 1);
	outb(0x80, port + 3);
	outb(12, port + 0);		    /* 9600 bps */
	outb(0, port + 1);
	outb(0x03, port + 3);		    /* 8 data bits */
	outb(0x13, port + 4);		    /* loop back mode */
	for ( i=0; i<16; i++ ) {
	    n = inb(port + 5);
	    if ( (n & 0x61) == 0x60 )
		break;
	    if ( (n & 1) == 1 )
		(void)inb(port);
	}
	outb(0x00, port + 4);
}

// added by James 03-05-2004.
// for secure device server:
// stat = 1, the port8 DTR is set to ON.
// stat = 0, the port8 DTR is set to OFF.
void SDS_PORT8_DTR(int stat)
{
        int _sds_oldmcr;
	    _sds_oldmcr = inb(mxvar_table[7].base + UART_MCR);    // get old MCR
        if (stat == 1) {
    	    outb(_sds_oldmcr | 0x01, mxvar_table[7].base + UART_MCR);    // set DTR ON
        }
        if (stat == 0) {
    	    outb(_sds_oldmcr & 0xfe, mxvar_table[7].base + UART_MCR);    // set DTR OFF
        }
        return;
}

module_init(mxser_module_init);
module_exit(mxser_module_exit); 
