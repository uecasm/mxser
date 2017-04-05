/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    gpio.c

	This header file of GPIO defines and structures

	2013		Tim Huang

*/

#ifndef _GPIO_H_
#define _GPIO_H_

#include "platform.h"

#define MX_GPIO_INPUT		0x8
#define MX_GPIO_CONFIG		0x9
#define MX_GPIO_OUTPUT		0xA

#define MX_GPIO_OK              0
#define MX_GPIO_ERR             -1

#define MX_GPIO_STATE_INPUT       0
#define MX_GPIO_STATE_OUTPUT    1

#define MX_GPIO_LOW         0
#define MX_GPIO_HIGH        1

#define MX_GPIO_MIN_PIN         0
#define MX_GPIO_MAX_PIN         7

#ifdef OS_SCO5

int mxGPIOReadAll();
int mxGPIOWriteAll();
int mxGPIOCurrentWriteAll();
int mxGPIOCurrentWrite();
int mxGPIOWrite();
int mxGPIORead();
int mxGPIOGetConfig();
int mxGPIOSetConfig();
int mxGPIOInit();

#else

int mxGPIOReadAll(gpio_param_t io_param, unsigned char *data);
int mxGPIOWriteAll(gpio_param_t io_param, unsigned char data);
int mxGPIOCurrentWriteAll(gpio_param_t io_param, unsigned char *data);
int mxGPIOCurrentWrite(gpio_param_t io_param, int io);
int mxGPIOWrite(gpio_param_t io_param, int io, int input);
int mxGPIORead(gpio_param_t io_param, int io);
int mxGPIOGetConfig(gpio_param_t io_param, int io);
int mxGPIOSetConfig(gpio_param_t io_param, int io, int config);
int mxGPIOInit(gpio_param_t io_param);

#endif

/* _GPIO_H_ */
#endif

