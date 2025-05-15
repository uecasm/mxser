/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    cpld.c

	This module contains the code that handle cpld control

	2013		Tim Huang

*/

#ifdef OS_SCO6
extern void time_delay();

#elif defined(OS_SCO5)
extern void time_delay();

#endif

#include "gpio.h"
#include "cpld.h"

#define MX_RETRY_CNT 5

/** \fn mxCPLDInit
 *   \brief Initiate CPLD.
 *
 *       This function initiate CPLD, which set all CPIO into output and Pull high
 *
 *    \param[in] base The base address of GPIO address
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDInit(io_param)
	gpio_param_t io_param;
#else
int mxCPLDInit(gpio_param_t io_param)
#endif
{
    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    /* Initiate all GPIO pin into output status */
    if(mxGPIOInit(io_param) != MX_GPIO_OK)
        return MX_CPLD_ERR;

    /* Initiate all GPIO pin into PULL high state */
    mxGPIOWrite(io_param, MX_CPLD_CHIPSELECT_PIN, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_READWRITE_PIN, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_ADDRDATA_PIN, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_HIGH);
    mxGPIOWrite(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_HIGH);

    return MX_CPLD_OK;
}

/** \fn mxCPLDEnable
 *   \brief Enable CPLD.
 *
 *       This function enable CPLD through pull low of Chip select pin
 *
 *    \param[in] base The base address of GPIO address
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDEnable(io_param)
	gpio_param_t io_param;
#else
int mxCPLDEnable(gpio_param_t io_param)
#endif
{
    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    /* Pull low of chip select pin */
	if(mxGPIOWrite(io_param, MX_CPLD_CHIPSELECT_PIN, MX_GPIO_LOW) != MX_GPIO_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDDisable
 *   \brief Disable CPLD.
 *
 *       This function disable CPLD through pull high of Chip select pin
 *
 *    \param[in] base The base address of GPIO address
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDDisable(io_param)
	gpio_param_t io_param;
#else
int mxCPLDDisable(gpio_param_t io_param)
#endif
{
    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    /* Pull High of Chip select first */
    mxGPIOWrite(io_param, MX_CPLD_CHIPSELECT_PIN, MX_GPIO_HIGH);

    /* Restore All GPIO pins into Output State */
    mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_ADDRDATA_PIN, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_READWRITE_PIN, MX_GPIO_STATE_OUTPUT);
    mxGPIOSetConfig(io_param, MX_CPLD_CHIPSELECT_PIN, MX_GPIO_STATE_OUTPUT);

    /* Pull high of chip select pin and others */
    if(mxGPIOWriteAll(io_param, MX_CPLD_INIT_STATE) != MX_GPIO_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetDirection
 *   \brief Setup read/write direction of CPLD.
 *
 *       This function setup read/write dirction of CPLD through change status
 *       of Read/Write Pin
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] direction The read or write direction
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetDirection(io_param, direction)
	gpio_param_t io_param;
	int direction;
#else
int mxCPLDSetDirection(gpio_param_t io_param, int direction)
#endif
{
    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(direction != MX_CPLD_READ && direction != MX_CPLD_WRITE)
        return MX_CPLD_ERR;

    if(direction == MX_CPLD_READ) {
        /* Pull High of Read/Write pin */
        if(mxGPIOWrite(io_param, MX_CPLD_READWRITE_PIN, MX_GPIO_HIGH) != MX_GPIO_OK)
            return MX_CPLD_ERR;
    } else if (direction == MX_CPLD_WRITE) {
        /* Pull Low of Read/Write pin */
		if(mxGPIOWrite(io_param, MX_CPLD_READWRITE_PIN, MX_GPIO_LOW) != MX_GPIO_OK)
            return MX_CPLD_ERR;
    }

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetType
 *   \brief Setup data pin type of CPLD.
 *
 *       This function setup data pin type of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] type The address or data
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetType(io_param, type)
	gpio_param_t io_param;
	int type;
#else
int mxCPLDSetType(gpio_param_t io_param, int type)
#endif
{
    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(type != MX_CPLD_TYPE_ADDR && type != MX_CPLD_TYPE_DATA)
        return MX_CPLD_ERR;

    if(type == MX_CPLD_TYPE_ADDR) {
        /* Pull Low of Addr/Data pin */
        if(mxGPIOWrite(io_param, MX_CPLD_ADDRDATA_PIN, MX_GPIO_LOW) != MX_GPIO_OK)
            return MX_CPLD_ERR;
    } else if (type == MX_CPLD_TYPE_DATA) {
        /* Pull High of Addr/Data pin */
        if(mxGPIOWrite(io_param, MX_CPLD_ADDRDATA_PIN, MX_GPIO_HIGH) != MX_GPIO_OK)
            return MX_CPLD_ERR;
    }

    return MX_CPLD_OK;
}

/** \fn mxCPLDRead
 *   \brief Read data from CPLD.
 *
 *       This function read data from CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] addr The address to read
 *    \param[out] data The data
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDRead(io_param, addr, data)
	gpio_param_t io_param;
	unsigned char addr;
	unsigned char *data;
#else
int mxCPLDRead(gpio_param_t io_param, unsigned char addr, unsigned char *data)
#endif
{
    unsigned char chCurrentOutput;
    unsigned char chOutput;
	unsigned char chData[MX_RETRY_CNT], chCnt[MX_RETRY_CNT];
	int i, j;

	/* Find the value which occurs more than twice */
	for (i = 0; i < MX_RETRY_CNT; i++) {

		if(io_param.base == NULL)
			return MX_CPLD_ERR;

		if(data == NULL)
			return MX_CPLD_ERR;

		/* Setup Chip into read state */
		mxCPLDSetDirection(io_param, MX_CPLD_READ);

		/* Change Data pins status into Output for addr setting */
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_STATE_OUTPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_STATE_OUTPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_STATE_OUTPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_STATE_OUTPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_STATE_OUTPUT);

		/* Back up GPIO status */
		mxGPIOCurrentWriteAll(io_param, &chCurrentOutput);

		/* Setup address */
		chOutput = 0;
		chOutput = chCurrentOutput & MX_CPLD_CTRL_MASK;
		chOutput |= (addr & MX_CPLD_DATA_MASK);

		/* Write address */
		mxGPIOWriteAll(io_param, chOutput);

		/* Change into address type */
		mxCPLDSetType(io_param, MX_CPLD_TYPE_ADDR);

		/* Enable Chip */
		if(mxCPLDEnable(io_param) != MX_CPLD_OK)
			return MX_CPLD_ERR;

		/* Wait one CPLD clock, which about 70 ns */
		MXCPLD_SLEEP(1);

		/* Change into data type */
		mxCPLDSetType(io_param, MX_CPLD_TYPE_DATA);

		/* Change Data pins status into Input for data reading */
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_STATE_INPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_STATE_INPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_STATE_INPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_STATE_INPUT);
		mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_STATE_INPUT);

		/* Wait one CPLD clock, which about 70 ns */
		MXCPLD_SLEEP(1);

		/* Read from data pins */
		mxGPIOReadAll(io_param, data);
		(*data) = (*data) & MX_CPLD_DATA_MASK;

		/* Do not have to restore direction pin because it default setting is read,
		   so we disable chip right away */
		mxCPLDDisable(io_param);

		/* Store in array */
		chData[i] = *data;
		
		/* Choose the right value */		
		chCnt[i] = 0;
		for (j = i - 1; j >= 0; j--) {
			if (chData[j] == chData[i]) {
				chCnt[i]++;
			}
		}
		if (chCnt[i] >= (MX_RETRY_CNT/2)) {
			break;
		}
	}

    return MX_CPLD_OK;
}

/** \fn mxCPLDWrite
 *   \brief Write data to CPLD.
 *
 *       This function write data into CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] addr The address to write
 *    \param[out] data The data
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDWrite(io_param, addr, data)
	gpio_param_t io_param;
	unsigned char addr;
	unsigned char data;
#else
int mxCPLDWrite(gpio_param_t io_param, unsigned char addr, unsigned char data)
#endif
{
    unsigned char chCurrentOutput;
    unsigned char chOutput;
	unsigned char chInput;
	int retry_cnt = MX_RETRY_CNT;

	if(io_param.base == NULL)
		return MX_CPLD_ERR;

RETRY:
	/* Setup Chip into write state */
	mxCPLDSetDirection(io_param, MX_CPLD_WRITE);

	/* Change Data pins status into Output for addr setting */
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_STATE_OUTPUT);

	/* Back up GPIO status */
	mxGPIOCurrentWriteAll(io_param, &chCurrentOutput);

	/* Setup output data */
	chOutput = 0;
	chOutput = chCurrentOutput & MX_CPLD_CTRL_MASK;
	chOutput |= (addr & MX_CPLD_DATA_MASK);

	/* Write address */
	mxGPIOWriteAll(io_param, chOutput);

	/* Change into address type */
	mxCPLDSetType(io_param, MX_CPLD_TYPE_ADDR);

	/* Enable Chip */
	if(mxCPLDEnable(io_param) != MX_CPLD_OK)
		return MX_CPLD_ERR;

	/* Wait one CPLD clock, which about 70 ns */
	MXCPLD_SLEEP(1);	

	/* Change Data pins status into Output for addr setting */
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_0, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_1, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_2, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_3, MX_GPIO_STATE_OUTPUT);
	mxGPIOSetConfig(io_param, MX_CPLD_DATA_PIN_4, MX_GPIO_STATE_OUTPUT);

	/* Change into data type */
	mxCPLDSetType(io_param, MX_CPLD_TYPE_DATA);

	/* Back up GPIO status again */
	mxGPIOCurrentWriteAll(io_param, &chCurrentOutput);

	/* Prepare write data */
	chOutput = 0;
	chOutput = chCurrentOutput & MX_CPLD_CTRL_MASK;
	chOutput |= (data & MX_CPLD_DATA_MASK);

	/* Write data */
	mxGPIOWriteAll(io_param, chOutput);

	/* Wait one CPLD clock, which about 70 ns */
	MXCPLD_SLEEP(1);

	/* Disable Chip*/
	mxCPLDDisable(io_param);

    if (addr & MX_CPLD_SET_STATE_BASE) {
		mxCPLDRead(io_param, ((addr & (~MX_CPLD_SET_STATE_BASE))|MX_CPLD_GET_STATE_BASE), &chInput);
		if (chInput != data) {
			if (retry_cnt > 0) {
				retry_cnt--;
				goto RETRY;
			}
		}
	}

	return MX_CPLD_OK;
}

/** \fn mxCPLDGetAlarmCode
 *   \brief Get current alarm code of CPDL.
 *
 *       This function get current alarm code of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_ALARM_0 Alarm code 0
 *               MX_CPLD_ALARM_1 Alarm code 1
 *               MX_CPLD_ALARM_2 Alarm code 2
 *               MX_CPLD_ALARM_3 Alarm code 3
 *               MX_CPLD_ALARM_4 Alarm code 4
 *               MX_CPLD_ALARM_5 Alarm code 5
 *               MX_CPLD_ALARM_6 Alarm code 6
 *               MX_CPLD_ALARM_7 Alarm code 7
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDGetAlarmCode(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDGetAlarmCode(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;
    int ret = 0;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_GET_ALARM_BASE + port;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Acquire alarm code */
    data = data & MX_CPLD_ALARM_MASK;

    switch(data) {
        case MX_CPLD_ALARM_0:
            ret =  MX_CPLD_ALARM_0;
            break;

        case MX_CPLD_ALARM_1:
            ret =  MX_CPLD_ALARM_1;
            break;

        case MX_CPLD_ALARM_2:
            ret =  MX_CPLD_ALARM_2;
            break;

        case MX_CPLD_ALARM_3:
            ret =  MX_CPLD_ALARM_3;
            break;

        case MX_CPLD_ALARM_4:
            ret =  MX_CPLD_ALARM_4;
            break;

        case MX_CPLD_ALARM_5:
            ret =  MX_CPLD_ALARM_5;
            break;

        case MX_CPLD_ALARM_6:
            ret =  MX_CPLD_ALARM_6;
            break;

        case MX_CPLD_ALARM_7:
            ret =  MX_CPLD_ALARM_7;
            break;

        default:
            ret = MX_CPLD_ERR;
            break;
    }

    return ret;
}

/** \fn mxCPLDGetTerminator
 *   \brief Get current terminator setting of CPLD.
 *
 *       This function get current terminator setting of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_TERMIN_ON   Terminator on
 *               MX_CPLD_TERMIN_OFF  Terminator off
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDGetTerminator(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDGetTerminator(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;
    int ret = 0;


    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_TERMIN_FLAG)
        ret = MX_CPLD_TERMIN_ON;
    else
        ret = MX_CPLD_TERMIN_OFF;

    return ret;

}

/** \fn mxCPLDGetPullState
 *   \brief Get current pull high/low setting of CPLD.
 *
 *       This function get current high/low setting of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_PULL_HIGH   Pull High
 *               MX_CPLD_PULL_LOW    Pull Low
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDGetPullState(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDGetPullState(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;
    int ret = 0;


    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_PULL_FLAG)
        ret = MX_CPLD_PULL_ON;
    else
        ret = MX_CPLD_PULL_OFF;

    return ret;

}

/** \fn mxCPLDGetAutoMode
 *   \brief Get current auto mode setting of CPLD.
 *
 *       This function get current auto mode setting of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_AUTO_ON   Auto mode on
 *               MX_CPLD_AUTO_OFF  Auto mode off
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDGetAutoMode(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDGetAutoMode(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;
    int ret = 0;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_AUTO_FLAG)
        ret = MX_CPLD_AUTO_ON;
    else
        ret = MX_CPLD_AUTO_OFF;

    return ret;
}

/** \fn mxCPLDSetTermintor
 *   \brief Set terminator of CPLD.
 *
 *       This function set terminator of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] state The terminator state
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetTerminator(io_param, port, state)
	gpio_param_t io_param;
	int port;
	int state;
#else
int mxCPLDSetTerminator(gpio_param_t io_param, int port, int state)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    if(state != MX_CPLD_TERMIN_ON && state != MX_CPLD_TERMIN_OFF)
        return MX_CPLD_ERR;

    /* Set address into get state address to get current setting */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get terminator and pull high low setting */
    data = data & MX_CPLD_STATE_MASK;

    if(state == MX_CPLD_TERMIN_ON)
        data |= MX_CPLD_TERMIN_FLAG;
    else if (state == MX_CPLD_TERMIN_OFF)
        data &= ~(MX_CPLD_TERMIN_FLAG);

    /* Prepare for write address */
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetPullState
 *   \brief Set pull state of CPLD.
 *
 *       This function set pull state of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] state The pull state
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetPullState(io_param, port, state)
	gpio_param_t io_param;
	int port;
	int state;
#else
int mxCPLDSetPullState(gpio_param_t io_param, int port, int state)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    if(state != MX_CPLD_PULL_ON && state != MX_CPLD_PULL_OFF)
        return MX_CPLD_ERR;

    /* Set address into get state address to get current setting */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get terminator and pull high low setting */
    data = data & MX_CPLD_STATE_MASK;

    if(state == MX_CPLD_PULL_ON)
        data |= MX_CPLD_PULL_FLAG;
    else if (state == MX_CPLD_PULL_OFF)
        data &= ~(MX_CPLD_PULL_FLAG);

    /* Prepare for write address */
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetAutoMode
 *   \brief Set auto mode of CPLD.
 *
 *       This function set auto mode of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] mode The auto mode
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetAutoMode(io_param, port, mode)
	gpio_param_t io_param;
	int port;
	int mode;
#else
int mxCPLDSetAutoMode(gpio_param_t io_param, int port, int mode)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    if(mode != MX_CPLD_AUTO_ON && mode != MX_CPLD_AUTO_OFF)
        return MX_CPLD_ERR;

    /* Set address to to get current baud and auto setting */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get the setting */
    data = data & MX_CPLD_STATE_MASK;

    if(mode == MX_CPLD_AUTO_ON)
        data |= MX_CPLD_AUTO_FLAG;
    else if (mode == MX_CPLD_AUTO_OFF)
        data &= ~(MX_CPLD_AUTO_FLAG);

    /* Prepare for write address */
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetBaudRate
 *   \brief Set buadrate of CPLD.
 *
 *       This function set buadrate of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] baud The baudrate
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetBaudRate(io_param, port, baud)
	gpio_param_t io_param;
	int port;
	int baud;
#else
int mxCPLDSetBaudRate(gpio_param_t io_param, int port, int baud)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Prepare data */
    data = (baud & MX_CPLD_BAUD_MASK);

    /* Prepare for write address */
    address = MX_CPLD_SET_BAUD_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDGetMasterSlave
 *   \brief Get current Master/Slave setting of CPLD.
 *
 *       This function get current Master/Slave setting of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_SLAVE_MODE   Slave mode
 *               MX_CPLD_MASTER_MODE  Master Mode
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDGetMasterSlave(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDGetMasterSlave(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;
    int ret = 0;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read master/slave */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_READ_MASTER_SLAVE_FLAG)
        ret = MX_CPLD_MASTER_MODE;
    else
        ret = MX_CPLD_SLAVE_MODE;

    return ret;
}

/** \fn mxCPLDSetMasterSlave
 *   \brief Set Master/Slave of CPLD.
 *
 *       This function set Master/Slave of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] mode The Master/Save mode
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetMasterSlave(io_param, port, mode)
	gpio_param_t io_param;
	int port;
	int mode;
#else
int mxCPLDSetMasterSlave(gpio_param_t io_param, int port, int mode)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    if(mode != MX_CPLD_SLAVE_MODE && mode != MX_CPLD_MASTER_MODE)
        return MX_CPLD_ERR;

    /* Setup the address for master/slave setting */
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get the setting */
    data = data & MX_CPLD_STATE_MASK;

    if(mode == MX_CPLD_MASTER_MODE)
        data |= MX_CPLD_WRITE_MASTER_SLAVE_FLAG;
    else if (mode == MX_CPLD_SLAVE_MODE)
        data &= ~(MX_CPLD_WRITE_MASTER_SLAVE_FLAG);

    /* Prepare for write address */
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}

/** \fn mxCPLDSetDiagnose
 *   \brief Set Diagnose status of CPLD.
 *
 *       This function set Diagnose status of CPLD.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \param[in] mode The Diagnose state
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDSetDiagnose(io_param, port, state)
	gpio_param_t io_param;
	int port;
	int state;
#else
int mxCPLDSetDiagnose(gpio_param_t io_param, int port, int state)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    if(state != MX_CPLD_AUTO_ON && state != MX_CPLD_AUTO_OFF)
        return MX_CPLD_ERR;

    /* Prepare for read address for pull state and terminator state*/
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get the setting */
    data = data & MX_CPLD_STATE_MASK;

    if(state == MX_CPLD_AUTO_ON)
        data |= MX_CPLD_AUTO_FLAG;
    else if (state == MX_CPLD_AUTO_OFF)
        data &= ~(MX_CPLD_AUTO_FLAG);

    /* Prepare for write address */
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;

}

/** \fn mxCPLDTriggerRefresh
 *   \brief Trigger CPLD to refresh.
 *
 *       This function trigger CPLD to refresh.
 *
 *    \param[in] base The base address of GPIO address
 *    \param[in] port The port to be readed
 *    \return MX_CPLD_OK    Success
 *               MX_CPLD_ERR   Error
 */
#ifdef OS_SCO5
int mxCPLDTriggerRefresh(io_param, port)
	gpio_param_t io_param;
	int port;
#else
int mxCPLDTriggerRefresh(gpio_param_t io_param, int port)
#endif
{
    unsigned char data;
    unsigned char address;

    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    if(port < MX_CPLD_MIN_PORT || port > MX_CPLD_MAX_PORT)
        return MX_CPLD_ERR;

    /* Prepare for read address for master/slave*/
    address = MX_CPLD_GET_STATE_BASE + port;

    /* Read current setting into data */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Get the setting */
    data = data & MX_CPLD_STATE_MASK;

    /* If it is slave mode, return  */
    if(!(data & MX_CPLD_READ_MASTER_SLAVE_FLAG))
        return MX_CPLD_OK;

    /* Turn into slave mode first */
    data &= ~(MX_CPLD_WRITE_MASTER_SLAVE_FLAG);
    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    /* Turn on master mode to refresh */
    data |= MX_CPLD_WRITE_MASTER_SLAVE_FLAG;

    address = MX_CPLD_SET_STATE_BASE + port;

    /* Write new setting into CPLD */
    if(mxCPLDWrite(io_param, address, data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    return MX_CPLD_OK;
}


/** \fn mxCPLDGetVersion
 *   \brief Get version of CPLD.
 *
 *       This function get CPLD version. The CPLD version is in the following mapping address.
 *
 *       address  D4-bit
 *          0        m0
 *          1        m1
 *          2        m2
 *          3        m3
 *          4        m4
 *          5        m5
 *          6        M0
 *          7        M1
 *
 *        M1M0          - represents the CPLD major number
 *        m5m4m3m2m1m0  - represents the CPLD major number
 *
 *    \param[in] base The base address of GPIO address
 *    \return 0 Error
 *               Others   CPLD version
 */
#ifdef OS_SCO5
int mxCPLDGetVersion(io_param, port, state)
	gpio_param_t io_param;
#else
int mxCPLDGetVersion(gpio_param_t io_param)
#endif

{
    unsigned char data;
    unsigned char address;
    int ret = 0;


    if(io_param.base == NULL)
        return MX_CPLD_ERR;

    /* Setup address for port */
    address = MX_CPLD_VER_MAJOR1;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x80;


    /* Setup address for port */
    address = MX_CPLD_VER_MAJOR0;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x40;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR5;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x20;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR4;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x10;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR3;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x08;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR2;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x04;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR1;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x02;


    /* Setup address for port */
    address = MX_CPLD_VER_MINOR0;

    /* Read port state */
    if(mxCPLDRead(io_param, address, &data) != MX_CPLD_OK)
        return MX_CPLD_ERR;

    if(data & MX_CPLD_VER_MASK)
        ret |= 0x01;

    return ret;
}

