/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    platform.h

	Platform dependancy

	2013		Tim Huang

*/


#ifndef _PLATFORM_H_
#define _PLATFORM_H_

/* We should define OS in makefile */

/* #define OS_LINUX */
/* #define OS_WINDOWS */
/* #define OS_SOLARIS */

#ifndef NULL
#define NULL 0
#endif

/* OS dependency */
#ifdef OS_LINUX
typedef struct _gpio_param {
	unsigned char *base;
} gpio_param_t;
#include <linux/delay.h>
#include <asm/io.h>
#define MXCPLD_READ_REG(ioparam, offset)			inb((unsigned long)ioparam.base+offset)
#define MXCPLD_WRITE_REG(ioparam, offset, x)   		outb(x, (unsigned long)ioparam.base+offset)
#define MXCPLD_SLEEP(x) 							mdelay(x)
// Sleep 1 microsecond

#elif defined(OS_WINDOWS)

typedef struct _gpio_param {
	unsigned char *base;
} gpio_param_t;

#define MXCPLD_READ_REG(ioparam, offset)           	// Implement here
#define MXCPLD_WRITE_REG(ioparam, offset, x)     	// Implement here
#define MXCPLD_SLEEP(x) 							// Implement here
// Sleep 1 microsecond

#elif defined(OS_SOLARIS)
#include <sys/ddi.h>
#include <sys/sunddi.h>

typedef struct _gpio_param {
	ddi_acc_handle_t handle;
	unsigned char *base;
} gpio_param_t;

#define MXCPLD_READ_REG(ioparam, offset)          	ddi_mem_get8(ioparam.handle, ioparam.base+offset)
#define MXCPLD_WRITE_REG(ioparam, offset, x)     	ddi_mem_put8(ioparam.handle, ioparam.base+offset, x)
#define MXCPLD_SLEEP(x) 							delay(drv_usectohz(x))				// Sleep 1 microsecond

#elif defined(OS_SCO6)

typedef struct _gpio_param {
	unsigned char *base;
} gpio_param_t;

#define MXCPLD_READ_REG(ioparam, offset) inb(ioparam.base+offset)
#define MXCPLD_WRITE_REG(ioparam, offset, x) outb(ioparam.base+offset, x)
#define MXCPLD_SLEEP(x) time_delay(2386*x)

#elif defined(OS_SCO5)

typedef struct _gpio_param {
	unsigned char *base;
} gpio_param_t;

#define MXCPLD_READ_REG(ioparam, offset) inb(ioparam.base+offset)
#define MXCPLD_WRITE_REG(ioparam, offset, x) outb(ioparam.base+offset, x)
#define MXCPLD_SLEEP(x) time_delay(2386*x)

#else

/* #error  need define OS in makefile. (SCO5 doesn't support #error) */
#define MXCPLD_READ_REG(ioparam, offset) __check_OS_definition__
#define MXCPLD_WRITE_REG(ioparam, offset, x) __check_OS_definition__
#define MXCPLD_SLEEP(x) __check_OS_definition__


/* OS */
#endif

#endif

