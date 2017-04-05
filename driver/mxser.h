/* 	Copyright (C) MOXA Inc. All rights reserved.
   
   	This is free software distributed under the terms of the
   	GNU Public License. See the file COPYING-GPL for details.
*/

/*
	mxser.h
*/

#ifndef _MXSER_H

//CheckIsMoxaMust return value
#define MOXA_OTHER_UART			0x00
#define MOXA_MUST_MU150_HWID		0x01
#define MOXA_MUST_MU860_HWID		0x02

// follow just for Moxa Must chip define.
//
// when LCR register (offset 0x03) write following value,
// the Must chip will enter enchance mode. And write value
// on EFR (offset 0x02) bit 6,7 to change bank.
#define MOXA_MUST_ENTER_ENCHANCE	0xBF

// when enhance mode enable, access on general bank register
#define MOXA_MUST_GDL_REGISTER		0x07
#define MOXA_MUST_GDL_MASK		0x7F
#define MOXA_MUST_GDL_HAS_BAD_DATA	0x80

#define MOXA_MUST_LSR_RERR		0x80	// error in receive FIFO
// enchance register bank select and enchance mode setting register
// when LCR register equal to 0xBF
#define MOXA_MUST_EFR_REGISTER		0x02
// enchance mode enable
#define MOXA_MUST_EFR_EFRB_ENABLE	0x10
// enchance reister bank set 0, 1, 2
#define MOXA_MUST_EFR_BANK0		0x00
#define MOXA_MUST_EFR_BANK1		0x40
#define MOXA_MUST_EFR_BANK2		0x80
#define MOXA_MUST_EFR_BANK3		0xC0
#define MOXA_MUST_EFR_BANK_MASK		0xC0

// set XON1 value register, when LCR=0xBF and change to bank0
#define MOXA_MUST_XON1_REGISTER		0x04

// set XON2 value register, when LCR=0xBF and change to bank0
#define MOXA_MUST_XON2_REGISTER		0x05

// set XOFF1 value register, when LCR=0xBF and change to bank0
#define MOXA_MUST_XOFF1_REGISTER	0x06

// set XOFF2 value register, when LCR=0xBF and change to bank0
#define MOXA_MUST_XOFF2_REGISTER	0x07

#define MOXA_MUST_RBRTL_REGISTER	0x04
#define MOXA_MUST_RBRTH_REGISTER	0x05
#define MOXA_MUST_RBRTI_REGISTER	0x06
#define MOXA_MUST_THRTL_REGISTER	0x07
#define MOXA_MUST_ENUM_REGISTER		0x04
#define MOXA_MUST_HWID_REGISTER		0x05
#define MOXA_MUST_ECR_REGISTER		0x06
#define MOXA_MUST_CSR_REGISTER		0x07

// good data mode enable
#define MOXA_MUST_FCR_GDA_MODE_ENABLE	0x20
// only good data put into RxFIFO
#define MOXA_MUST_FCR_GDA_ONLY_ENABLE	0x10

// enable CTS interrupt
#define MOXA_MUST_IER_ECTSI		0x80
// eanble RTS interrupt
#define MOXA_MUST_IER_ERTSI		0x40
// enable Xon/Xoff interrupt
#define MOXA_MUST_IER_XINT		0x20
// enable GDA interrupt
#define MOXA_MUST_IER_EGDAI		0x10

#define MOXA_MUST_RECV_ISR		(UART_IER_RDI | MOXA_MUST_IER_EGDAI)

// GDA interrupt pending
#define MOXA_MUST_IIR_GDA		0x1C
#define MOXA_MUST_IIR_RDA		0x04
#define MOXA_MUST_IIR_RTO		0x0C
#define MOXA_MUST_IIR_LSR		0x06

// recieved Xon/Xoff or specical interrupt pending
#define MOXA_MUST_IIR_XSC		0x10

// RTS/CTS change state interrupt pending
#define MOXA_MUST_IIR_RTSCTS		0x20
#define MOXA_MUST_IIR_MASK		0x3E

#define MOXA_MUST_MCR_XON_FLAG		0x40
#define MOXA_MUST_MCR_XON_ANY		0x80
#define MOXA_MUST_MCR_TX_XON		0x08


// software flow control on chip mask value
#define MOXA_MUST_EFR_SF_MASK		0x0F
// send Xon1/Xoff1
#define MOXA_MUST_EFR_SF_TX1		0x08
// send Xon2/Xoff2
#define MOXA_MUST_EFR_SF_TX2		0x04
// send Xon1,Xon2/Xoff1,Xoff2
#define MOXA_MUST_EFR_SF_TX12		0x0C
// don't send Xon/Xoff
#define MOXA_MUST_EFR_SF_TX_NO		0x00
// Tx software flow control mask
#define MOXA_MUST_EFR_SF_TX_MASK	0x0C
// don't receive Xon/Xoff
#define MOXA_MUST_EFR_SF_RX_NO		0x00
// receive Xon1/Xoff1
#define MOXA_MUST_EFR_SF_RX1		0x02
// receive Xon2/Xoff2
#define MOXA_MUST_EFR_SF_RX2		0x01
// receive Xon1,Xon2/Xoff1,Xoff2
#define MOXA_MUST_EFR_SF_RX12		0x03
// Rx software flow control mask
#define MOXA_MUST_EFR_SF_RX_MASK	0x03

//#define MOXA_MUST_MIN_XOFFLIMIT		66
//#define MOXA_MUST_MIN_XONLIMIT		20
//#define ID1_RX_TRIG			120


#define CHECK_MOXA_MUST_XOFFLIMIT(info) { 	\
	if ( (info)->IsMoxaMustChipFlag && 	\
	 (info)->HandFlow.XoffLimit < MOXA_MUST_MIN_XOFFLIMIT ) {	\
		(info)->HandFlow.XoffLimit = MOXA_MUST_MIN_XOFFLIMIT;	\
		(info)->HandFlow.XonLimit = MOXA_MUST_MIN_XONLIMIT;	\
	}	\
}

#define ENABLE_MOXA_MUST_ENCHANCE_MODE(baseio) { \
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr |= MOXA_MUST_EFR_EFRB_ENABLE;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define DISABLE_MOXA_MUST_ENCHANCE_MODE(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_EFRB_ENABLE;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_XON1_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK0;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_XON1_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_XON2_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK0;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_XON2_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_XOFF1_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK0;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_XOFF1_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_XOFF2_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK0;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_XOFF2_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_RBRTL_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_RBRTL_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_RBRTH_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_RBRTH_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_RBRTI_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_RBRTI_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_THRTL_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_THRTL_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

//#define MOXA_MUST_RBRL_VALUE	4
#define SET_MOXA_MUST_FIFO_VALUE(info) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((info)->base+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (info)->base+UART_LCR);	\
	__efr = inb((info)->base+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK1;	\
	outb(__efr, (info)->base+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)((info)->rx_high_water), (info)->base+MOXA_MUST_RBRTH_REGISTER);	\
	outb((UCHAR)((info)->rx_trigger), (info)->base+MOXA_MUST_RBRTI_REGISTER);	\
	outb((UCHAR)((info)->rx_low_water), (info)->base+MOXA_MUST_RBRTL_REGISTER);	\
	outb((UCHAR)(0), (info)->base+MOXA_MUST_THRTL_REGISTER);	\
	outb(__oldlcr, (info)->base+UART_LCR);	\
}



#define SET_MOXA_MUST_ENUM_VALUE(baseio, Value) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK2;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb((UCHAR)(Value), (baseio)+MOXA_MUST_ENUM_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define GET_MOXA_MUST_HARDWARE_ID(baseio, pId) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_BANK_MASK;	\
	__efr |= MOXA_MUST_EFR_BANK2;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	*pId = inb((baseio)+MOXA_MUST_HWID_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_NO_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_MASK;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_JUST_TX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_MASK;	\
	__efr |= MOXA_MUST_EFR_SF_TX1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define ENABLE_MOXA_MUST_TX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_TX_MASK;	\
	__efr |= MOXA_MUST_EFR_SF_TX1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define DISABLE_MOXA_MUST_TX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_TX_MASK;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define SET_MOXA_MUST_JUST_RX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_MASK;	\
	__efr |= MOXA_MUST_EFR_SF_RX1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define ENABLE_MOXA_MUST_RX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_RX_MASK;	\
	__efr |= MOXA_MUST_EFR_SF_RX1;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define DISABLE_MOXA_MUST_RX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_RX_MASK;	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define ENABLE_MOXA_MUST_TX_RX_SOFTWARE_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldlcr, __efr;	\
	__oldlcr = inb((baseio)+UART_LCR);	\
	outb(MOXA_MUST_ENTER_ENCHANCE, (baseio)+UART_LCR);	\
	__efr = inb((baseio)+MOXA_MUST_EFR_REGISTER);	\
	__efr &= ~MOXA_MUST_EFR_SF_MASK;	\
	__efr |= (MOXA_MUST_EFR_SF_RX1|MOXA_MUST_EFR_SF_TX1);	\
	outb(__efr, (baseio)+MOXA_MUST_EFR_REGISTER);	\
	outb(__oldlcr, (baseio)+UART_LCR);	\
}

#define ENABLE_MOXA_MUST_XON_ANY_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldmcr;	\
	__oldmcr = inb((baseio)+UART_MCR);	\
	__oldmcr |= MOXA_MUST_MCR_XON_ANY;	\
	outb(__oldmcr, (baseio)+UART_MCR);	\
}

#define DISABLE_MOXA_MUST_XON_ANY_FLOW_CONTROL(baseio) {	\
	UCHAR	__oldmcr;	\
	__oldmcr = inb((baseio)+UART_MCR);	\
	__oldmcr &= ~MOXA_MUST_MCR_XON_ANY;	\
	outb(__oldmcr, (baseio)+UART_MCR);	\
}

#define READ_MOXA_MUST_GDL(baseio)	inb((baseio)+MOXA_MUST_GDL_REGISTER)


#if (LINUX_VERSION_CODE < VERSION_CODE(2,4,0))
#define CLEAR_FUNC 	cleanup_module
#define CLEAR_FUNC_RET	void
#else
#define CLEAR_FUNC 	mxser_module_exit
#define CLEAR_FUNC_RET	static void __exit
#endif


#if (LINUX_VERSION_CODE < VERSION_CODE(2,4,0))
#define INIT_FUNC 	init_module
#define INIT_FUNC_RET	int
#else
#define INIT_FUNC 	mxser_module_init
#define INIT_FUNC_RET	static int __init
#endif


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
#define DRV_VAR		(&mxvar_sdriver)
#define DRV_VAR_P(x)	mxvar_sdriver.x
#else
#define DRV_VAR		(mxvar_sdriver)
#define DRV_VAR_P(x)	mxvar_sdriver->x
#endif

#ifndef INIT_WORK
#define INIT_WORK(_work, _func, _data){	\
	_data->tqueue.routine = _func;\
	_data->tqueue.data = _data;\
	}
#endif

#ifndef set_current_state
#define	set_current_state(x) 		current->state = x
#endif


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
#define IRQ_RET void
#define IRQ_RETVAL(x)
#else
#define IRQ_RET irqreturn_t
#endif


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
#if (LINUX_VERSION_CODE >= VERSION_CODE(2,4,0))
#define	MXQ_TASK() {\
		MOD_INC_USE_COUNT;\
		if (schedule_task(&info->tqueue) == 0)\
			MOD_DEC_USE_COUNT;\
	}
#else
#define MXQ_TASK()	queue_task(&info->tqueue,&tq_scheduler)
#endif
#else
#define	MXQ_TASK()	schedule_work(&info->tqueue)
#endif

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,0))
#define MX_MOD_INC	MOD_INC_USE_COUNT
#define MX_MOD_DEC	MOD_DEC_USE_COUNT
#else
#define MX_MOD_INC	try_module_get(THIS_MODULE)
#define MX_MOD_DEC	module_put(THIS_MODULE)	
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#ifndef ASYNC_CALLOUT_ACTIVE
#define ASYNC_CALLOUT_ACTIVE 0
#endif
#endif


#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#define MX_TTY_DRV(x)	tty->driver->x
#else
#define MX_TTY_DRV(x)	tty->driver.x
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,5))
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
#define MX_SESSION()	process_session(current)
#else
#define MX_SESSION()	current->signal->session
#endif
#else
#define MX_SESSION()	current->session
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#define MX_CGRP()	process_group(current)	
#else
#define MX_CGRP()	current->pgrp	
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#define MX_ACCESS_CHK(type, addr, size)	access_ok(type, addr, size)	
#else
#define MX_ACCESS_CHK(type, addr, size)	verify_area(type, addr, size)	
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#define MX_ERR(x)	!(x)	
#else
#define MX_ERR(x)	x	
#endif

#if (LINUX_VERSION_CODE >= VERSION_CODE(2,6,0))
#define GET_FPAGE	__get_free_page	
#else
#define GET_FPAGE	get_free_page
#endif

#ifndef atomic_read
#define atomic_read(v)	v
#endif


#ifndef UCHAR
typedef unsigned char	UCHAR;
#endif


#endif
