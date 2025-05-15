/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mxpcie.h
*/

#ifndef _MXPCIE_H

#define MOXA_PUART_HWID		0x03

#define MX_READ_REG	ioread8
#define MX_WRITE_REG	iowrite8
#define MX_READ_IOBAR3_REG inb
#define MX_WRITE_IOBAR3_REG outb

#include <asm/uaccess.h>
//#define put_to_user(arg1, arg2) put_user(arg1, (unsigned long *)arg2)
//#define get_from_user(arg1, arg2) get_user(arg1, (unsigned int *)arg2)

#define	MXUPCIE_EVENT_TXLOW	1
#define	MXUPCIE_EVENT_HANGUP	2

#define SERIAL_DO_RESTART
#define MXUPCIE_BOARDS		4	/* Max. boards */
#define MXUPCIE_PORTS		32	/* Max. ports */
#define MXUPCIE_PORTS_PER_BOARD	8	/* Max. ports per board*/
#ifdef CONFIG_PREEMPT_RT_FULL   /* On real time Linux, we can have a more delays */
#define MXUPCIE_ISR_PASS_LIMIT	1000000L
#else
#define MXUPCIE_ISR_PASS_LIMIT  512
#endif

#define	MXUPCIE_ERR_IOADDR	-1
#define	MXUPCIE_ERR_IRQ		-2
#define	MXUPCIE_ERR_IRQ_CONFLIT	-3
#define	MXUPCIE_ERR_VECTOR	-4

#define SERIAL_TYPE_NORMAL	1
#define SERIAL_TYPE_CALLOUT	2

#define WAKEUP_CHARS		256

#define UART_MCR_AFE		0x20
#define UART_LSR_SPECIAL	0x1E

#define MX_LOCK_INIT()		unsigned long sp_flags=0
#if 0
#define MX_LOCK(lock)		{\
				if(!in_interrupt())\
					printk("LOCK");\
					spin_lock_irqsave(lock, sp_flags);\
				}
#define MX_UNLOCK(lock)		{\
				if(!in_interrupt())\
					printk("UNLOCK");\
					spin_unlock_irqrestore(lock, sp_flags);\
				}
#else
#define MX_LOCK(lock)		{\
				if(!in_interrupt())\
					spin_lock_irqsave(lock, sp_flags);\
				}
#define MX_UNLOCK(lock)		{\
				if(!in_interrupt())\
					spin_unlock_irqrestore(lock, sp_flags);\
				}
#endif

#define MX_CPLD_LOCK(lock)		spin_lock(lock)
#define MX_CPLD_UNLOCK(lock)	spin_unlock(lock)

#define PORTNO(x)	((x)->index)

#define RELEVANT_IFLAG(iflag)	(iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|IXON|IXOFF))

#define IRQ_T(info) ((info->flags & ASYNC_SHARE_IRQ) ? IRQF_SHARED : IRQF_TRIGGER_NONE)

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

#define MOXA			0x400
#define MOXA_GETDATACOUNT	(MOXA + 23)
#define	MOXA_GET_CONF		(MOXA + 35)
#define MOXA_CHKPORTENABLE	(MOXA + 60)
#define MOXA_GET_MAJOR		(MOXA + 63)
#define MOXA_GET_CUMAJOR	(MOXA + 64)
#define MOXA_GETMSTATUS		(MOXA + 65)

#define MOXA_SET_SPECIAL_BAUD_RATE	(MOXA+100)
#define MOXA_GET_SPECIAL_BAUD_RATE	(MOXA+101)
#define SMARTIO_SET_SPECIAL_BAUD_RATE	(MOXA+77)
#define SMARTIO_GET_SPECIAL_BAUD_RATE	(MOXA+78)

#define NPPI_NOTIFY_PARITY	0x01
#define NPPI_NOTIFY_FRAMING	0x02
#define NPPI_NOTIFY_HW_OVERRUN	0x04
#define NPPI_NOTIFY_BREAK	0x10

#define SMARTIO_PUART_SET_INTERFACE	(MOXA + 79)
#define SMARTIO_PUART_GET_INTERFACE	(MOXA + 80)
#define SMARTIO_PUART_SET_TERMINATOR	(MOXA + 81)
#define SMARTIO_PUART_GET_TERMINATOR	(MOXA + 82)

#define SMARTIO_PUART_SET_PULL_STATE	(MOXA + 83)
#define SMARTIO_PUART_GET_PULL_STATE	(MOXA + 84)
#define SMARTIO_PUART_SET_AUTO_MODE	(MOXA + 85)
#define SMARTIO_PUART_GET_AUTO_MODE	(MOXA + 86)
#define SMARTIO_PUART_SET_MASTER_SLAVE	(MOXA + 87)
#define SMARTIO_PUART_GET_MASTER_SLAVE	(MOXA + 88)
#define SMARTIO_PUART_SET_DIAGNOSE	(MOXA + 89)
#define SMARTIO_PUART_GET_ALARM		(MOXA + 90)

#define SMARTIO_SET_PCI_CAPABILITY	(MOXA + 91)
#define SMARTIO_GET_PCI_CAPABILITY	(MOXA + 92)

#ifdef CONFIG_PCI

#ifndef PCI_ANY_ID
#define PCI_ANY_ID (~0)
#endif

#endif

/*
 *	Define the Moxa PCI vendor and device IDs.
 */
#ifndef	PCI_VENDOR_ID_MOXA
#define	PCI_VENDOR_ID_MOXA	0x1393
#endif

#ifndef	PCI_DEVICE_ID_CP102E
#define	PCI_DEVICE_ID_CP102E	0x1024
#endif

#ifndef	PCI_DEVICE_ID_CP102EL
#define	PCI_DEVICE_ID_CP102EL	0x1025
#endif

#ifndef	PCI_DEVICE_ID_CP132EL
#define	PCI_DEVICE_ID_CP132EL	0x1322
#endif

#ifndef	PCI_DEVICE_ID_CP114EL
#define	PCI_DEVICE_ID_CP114EL	0x1144
#endif

#ifndef	PCI_DEVICE_ID_CP104EL_A
#define	PCI_DEVICE_ID_CP104EL_A	 0x1045
#endif

#ifndef	PCI_DEVICE_ID_CP168EL_A
#define	PCI_DEVICE_ID_CP168EL_A	 0x1683
#endif

#ifndef	PCI_DEVICE_ID_CP118EL_A
#define	PCI_DEVICE_ID_CP118EL_A	 0x1182
#endif

#ifndef	PCI_DEVICE_ID_CP118E_A_I
#define	PCI_DEVICE_ID_CP118E_A_I	 0x1183
#endif

#ifndef	PCI_DEVICE_ID_CP138E_A
#define	PCI_DEVICE_ID_CP138E_A	 0x1381
#endif

#ifndef	PCI_DEVICE_ID_CP134EL_A
#define	PCI_DEVICE_ID_CP134EL_A	 0x1342
#endif

#ifndef	PCI_DEVICE_ID_CP116E_A_A
#define	PCI_DEVICE_ID_CP116E_A_A	 0x1160
#endif

#ifndef	PCI_DEVICE_ID_CP116E_A_B
#define	PCI_DEVICE_ID_CP116E_A_B	 0x1161
#endif

#ifndef PCI_DEVICE_ID_CP102N
#define PCI_DEVICE_ID_CP102N	0x1027
#endif

#ifndef PCI_DEVICE_ID_CP132N
#define PCI_DEVICE_ID_CP132N	0x1323
#endif

#ifndef PCI_DEVICE_ID_CP112N
#define PCI_DEVICE_ID_CP112N	0x1121
#endif

#ifndef PCI_DEVICE_ID_CP104N
#define PCI_DEVICE_ID_CP104N	0x1046
#endif

#ifndef PCI_DEVICE_ID_CP134N
#define PCI_DEVICE_ID_CP134N	0x1343
#endif

#ifndef PCI_DEVICE_ID_CP114N
#define PCI_DEVICE_ID_CP114N	0x1145
#endif

#define MOXA_PUART_SFR			0x07
#define MOXA_PUART_EFR			0x0A
#define MOXA_PUART_XON1			0x0B
#define MOXA_PUART_XON2			0x0C
#define MOXA_PUART_XOFF1		0x0D
#define MOXA_PUART_XOFF2		0x0E
#define MOXA_PUART_ACR			0x0F
#define MOXA_PUART_TTL			0x10
#define MOXA_PUART_RTL			0x11
#define MOXA_PUART_FCL			0x12
#define MOXA_PUART_FCH			0x13
#define MOXA_PUART_CPR			0x14
#define MOXA_PUART_RCNT			0x15
#define MOXA_PUART_LSRCNT		0x15
#define MOXA_PUART_TCNT			0x16
#define MOXA_PUART_SCR			0x16
#define MOXA_PUART_GLSR			0x17
#define MOXA_PUART_MEMRBR		0x100
#define MOXA_PUART_MEMTHR		0x100
#define MOXA_PUART_0UIR			0x04
#define MOXA_PUART_1UIR			0x04
#define MOXA_PUART_2UIR			0x05
#define MOXA_PUART_3UIR			0x05
#define MOXA_PUART_4UIR			0x06
#define MOXA_PUART_5UIR			0x06
#define MOXA_PUART_6UIR			0x07
#define MOXA_PUART_7UIR			0x07
#define MOXA_PUART_GPIO_IN		0x08
#define MOXA_PUART_GPIO_EN		0x09
#define MOXA_PUART_GPIO_OUT		0x0A
#define MOXA_PUART_LSB			0x08
#define MOXA_PUART_MSB			0x09

#define MOXA_PUART_ADJ_CLK		0x24
#define MOXA_PUART_ADJ_ENABLE		0x25

#define MOXA_SFR_FORCE_TX		0x01
#define MOXA_SFR_950			0x20
#define MOXA_SFR_ENABLE_TCNT		0x80

#define MOXA_EFR_TX_SW			0x02
#define MOXA_EFR_RX_SW			0x08
#define MOXA_EFR_ENHANCE		0x10
#define MOXA_EFR_AUTO_RTS		0x40
#define MOXA_EFR_AUTO_CTS		0x80

#define MOXA_IIR_NO_INT			0xC1
#define MOXA_IIR_RLSI			0xC6
#define MOXA_IIR_RDI			0x04
#define MOXA_IIR_THRI			0x02

#define MOXA_TTL_1			0x01
#define MOXA_RTL_1			0x01
#define MOXA_RTL_96			0x60
#define MOXA_RTL_120			0x78
#define MOXA_FCL_16			0x10
#define MOXA_FCH_96			0x60
#define MOXA_FCH_110			0x6E
#define MOXA_FCH_120			0x78

#define MOXA_UIR_RS232			0x00
#define MOXA_UIR_RS422			0x01
#define MOXA_UIR_RS485_4W		0x0B
#define MOXA_UIR_RS485_2W		0x0F
#define MOXA_UIR_OFFSET			0x04
#define MOXA_UIR_EVEN_PORT_VALUE_OFFSET	4

#define MOXA_GPIO_SET_ALL_OUTPUT	0x0F
#define MOXA_GPIO_OUTPUT_VALUE_OFFSET	16

#define MX_RS232			1
#define MX_RS422			2
#define MX_RS485_2W			4
#define MX_RS485_4W			8
#define MX_TERM_NONE			0x00
#define MX_TERM_120			0x01

#define MX_PORT4			3
#define MX_PORT8			7
#define MX_TX_FIFO_SIZE			128
#define MX_RX_FIFO_SIZE			128
#define MX_PUART_SIZE			0x200
#define MX_BREAK_ON			0x01
#define MX_BREAK_OFF			0x00

#define MX_FIFO_RESET_CNT		100

#define CLEAR_FUNC 	mxupcie_module_exit
#define CLEAR_FUNC_RET	static void __exit

#define INIT_FUNC 	mxupcie_module_init
#define INIT_FUNC_RET	static int __init


#define DRV_VAR		(mxvar_sdriver)
#define DRV_VAR_P(x)	mxvar_sdriver->x

#ifndef INIT_WORK
#define INIT_WORK(_work, _func, _data){	\
	_data->tqueue.routine = _func;\
	_data->tqueue.data = _data;\
	}
#endif

#ifndef set_current_state
#define	set_current_state(x) 		current->state = x
#endif


#define IRQ_RET irqreturn_t

#define	MXQ_TASK()	schedule_work(&info->tqueue)

#define MX_MOD_INC	try_module_get(THIS_MODULE)
#define MX_MOD_DEC	module_put(THIS_MODULE)	

#ifndef ASYNC_CALLOUT_ACTIVE
#define ASYNC_CALLOUT_ACTIVE 0
#endif

#define MX_TTY_DRV(x)	tty->driver->x

#ifdef VERIFY_WRITE
#define MX_ACCESS_CHK(type, addr, size)		access_ok(type, addr, size)
#else
#define MX_ACCESS_CHK(type, addr, size)		access_ok(addr, size)
#endif

#define MX_ERR(x)	!(x)	

#define GET_FPAGE	__get_free_page	

#ifndef atomic_read
#define atomic_read(v)	v
#endif


#ifndef UCHAR
typedef unsigned char	UCHAR;
#endif


#ifndef ASYNCB_SHARE_IRQ
	#define ASYNCB_SHARE_IRQ	24
#endif
#ifndef ASYNC_SHARE_IRQ
	#define ASYNC_SHARE_IRQ		(1U << ASYNCB_SHARE_IRQ)
#endif
#ifndef ASYNCB_CHECK_CD
	#define ASYNCB_CHECK_CD		25
#endif
#ifndef ASYNC_CHECK_CD
	#define ASYNC_CHECK_CD		(1U << ASYNCB_CHECK_CD)
#endif
#ifndef ASYNCB_CTS_FLOW
	#define ASYNCB_CTS_FLOW		26
#endif
#ifndef ASYNC_CTS_FLOW
	#define ASYNC_CTS_FLOW		(1U << ASYNCB_CTS_FLOW)
#endif
#ifndef ASYNCB_CLOSING
	#define ASYNCB_CLOSING		27
#endif
#ifndef ASYNC_CLOSING
	#define ASYNC_CLOSING		(1U << ASYNCB_CLOSING)
#endif
#ifndef ASYNCB_NORMAL_ACTIVE
	#define ASYNCB_NORMAL_ACTIVE	29
#endif

#ifndef ASYNC_NORMAL_ACTIVE
	#define ASYNC_NORMAL_ACTIVE	(1U << ASYNCB_NORMAL_ACTIVE)
#endif
#ifndef ASYNCB_INITIALIZED
	#define ASYNCB_INITIALIZED	31
#endif
#ifndef ASYNC_INITIALIZED
	#define ASYNC_INITIALIZED	(1U << ASYNCB_INITIALIZED)
#endif


#endif
