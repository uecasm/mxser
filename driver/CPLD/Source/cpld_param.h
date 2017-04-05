/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    cpld_param.c

	CPLD control parameters

	2013		Douglas Lee

*/

#ifndef _CPLD_PARAM_H_
#define _CPLD_PARAM_H_

#define MX_CPLD_DATA_PIN_0          0
#define MX_CPLD_DATA_PIN_1          1
#define MX_CPLD_DATA_PIN_2          2
#define MX_CPLD_DATA_PIN_3          3
#define MX_CPLD_DATA_PIN_4          4
#define MX_CPLD_ADDRDATA_PIN        5
#define MX_CPLD_READWRITE_PIN       6
#define MX_CPLD_CHIPSELECT_PIN      7

#define MX_CPLD_CTRL_MASK           0xE0
#define MX_CPLD_DATA_MASK           0x1F

#define MX_CPLD_STATE_MASK          0xF
#define MX_CPLD_BAUD_MASK           0x7
#define MX_CPLD_ALARM_MASK          0x7

#define MX_CPLD_INIT_STATE               0xFF

#define MX_CPLD_READ        		0
#define MX_CPLD_WRITE      			1

#define MX_CPLD_TYPE_ADDR       	0
#define MX_CPLD_TYPE_DATA       	1

#define MX_CPLD_OK          		0
#define MX_CPLD_ERR        			-1

#define MX_CPLD_MIN_PORT    		0
#define MX_CPLD_MAX_PORT    		7

#define MX_CPLD_ALARM_0     		0
#define MX_CPLD_ALARM_1     		1
#define MX_CPLD_ALARM_2     		2
#define MX_CPLD_ALARM_3     		3
#define MX_CPLD_ALARM_4     		4
#define MX_CPLD_ALARM_5     		5
#define MX_CPLD_ALARM_6     		6
#define MX_CPLD_ALARM_7     		7

#define MX_CPLD_TERMIN_FLAG 		0x2
#define MX_CPLD_PULL_FLAG 			0x1
#define MX_CPLD_AUTO_FLAG 			0x8
#define MX_CPLD_WRITE_MASTER_SLAVE_FLAG 0x4
#define MX_CPLD_READ_MASTER_SLAVE_FLAG  0x4
#define MX_CPLD_DIAGNOSE_FLAG   0x10

#define MX_CPLD_TERMIN_ON 			1
#define MX_CPLD_TERMIN_OFF  		0

#define MX_CPLD_PULL_ON 			1
#define MX_CPLD_PULL_OFF  			0

#define MX_CPLD_AUTO_ON 			1
#define MX_CPLD_AUTO_OFF 	 		0

#define MX_CPLD_BAUD_9600           0
#define MX_CPLD_BAUD_19200         	1
#define MX_CPLD_BAUD_38400         	2
#define MX_CPLD_BAUD_57600         	3
#define MX_CPLD_BAUD_115200       	4
#define MX_CPLD_BAUD_230400       	5
#define MX_CPLD_BAUD_460800       	6
#define MX_CPLD_BAUD_921600       	7

#define MX_CPLD_SLAVE_MODE      0
#define MX_CPLD_MASTER_MODE     1

#define MX_CPLD_DIAG_OFF    0
#define MX_CPLD_DIAG_ON     1

#define MX_CPLD_GET_ALARM_BASE  	0x0
#define MX_CPLD_GET_STATE_BASE  		0x10
#define MX_CPLD_SET_STATE_BASE  	0x18
#define MX_CPLD_SET_BAUD_BASE  		0x8

#define MX_CPLD_VER_MASK 0x10
#define MX_CPLD_VER_MINOR0 0x00
#define MX_CPLD_VER_MINOR1 0x01
#define MX_CPLD_VER_MINOR2 0x02
#define MX_CPLD_VER_MINOR3 0x03
#define MX_CPLD_VER_MINOR4 0x04
#define MX_CPLD_VER_MINOR5 0x05
#define MX_CPLD_VER_MAJOR0 0x06
#define MX_CPLD_VER_MAJOR1 0x07

#endif

