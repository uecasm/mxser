/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    cpld.c

	This header file of CPLD control defines and structures

	2013		Tim Huang

*/

#ifndef _CPLD_H_
#define _CPLD_H_

#include "platform.h"
#include "cpld_param.h"

#ifdef OS_SCO5

/* Basic CPLD access functions */
int mxCPLDInit();
int mxCPLDEnable();
int mxCPLDDisable();
int mxCPLDSetDirection();
int mxCPLDSetType();
int mxCPLDRead();
int mxCPLDWrite();

/* Advance CPLD access function */
int mxCPLDGetAlarmCode();
int mxCPLDGetTerminator();
int mxCPLDGetPullState();
int mxCPLDGetAutoMode();
int mxCPLDGetMasterSlave();

int mxCPLDSetTerminator();
int mxCPLDSetPullState();
int mxCPLDSetAutoMode();
int mxCPLDSetBaudRate();
int mxCPLDSetMasterSlave();
int mxCPLDSetDiagnose();
int mxCPLDGetVersion();

int mxCPLDTriggerRefresh();

#else

/* Basic CPLD access functions */
int mxCPLDInit(gpio_param_t io_param);
int mxCPLDEnable(gpio_param_t io_param);
int mxCPLDDisable(gpio_param_t io_param);
int mxCPLDSetDirection(gpio_param_t io_param, int direction);
int mxCPLDSetType(gpio_param_t io_param, int type);
int mxCPLDRead(gpio_param_t io_param, unsigned char addr, unsigned char *data);
int mxCPLDWrite(gpio_param_t io_param, unsigned char addr, unsigned char data);

/* Advance CPLD access function */
int mxCPLDGetAlarmCode(gpio_param_t io_param, int port);
int mxCPLDGetTerminator(gpio_param_t io_param, int port);
int mxCPLDGetPullState(gpio_param_t io_param, int port);
int mxCPLDGetAutoMode(gpio_param_t io_param, int port);
int mxCPLDGetMasterSlave(gpio_param_t io_param, int port);

int mxCPLDSetTerminator(gpio_param_t io_param, int port, int state);
int mxCPLDSetPullState(gpio_param_t io_param, int port, int state);
int mxCPLDSetAutoMode(gpio_param_t io_param, int port, int mode);
int mxCPLDSetBaudRate(gpio_param_t io_param, int port, int baud);
int mxCPLDSetMasterSlave(gpio_param_t io_param, int port, int mode);
int mxCPLDSetDiagnose(gpio_param_t io_param, int port, int state);
int mxCPLDGetVersion(gpio_param_t io_param);

int mxCPLDTriggerRefresh(gpio_param_t io_param, int port);

#endif
/* _CPLD_H_ */
#endif
