/*  Copyright (C) MOXA Inc. All rights reserved.

    This software is distributed under the terms of the
    MOXA License.  See the file COPYING-MOXA for details.
*/

/*
    gpio.c

	This module contains the code that handle GPIO functionality

	2013		Tim Huang

*/

#ifdef OS_SCO6
extern void time_delay();

#elif defined(OS_SCO5)
extern void time_delay();

#endif

#include "gpio.h"

/** \fn mxGPIOInit
 *   \brief Init routine of GPIO.
 *
 *       This function init the GPIO config, add GPIO pin will be reset to output
 *       after this function.
 *
 *    \param[in] addr The base address of GPIO access
 *    \return MX_GPIO_OK Success
 *               MX_GPIO_ERR Error
 */

#ifdef OS_SCO5
int mxGPIOInit(io_param)
	gpio_param_t io_param;
#else
int mxGPIOInit(gpio_param_t io_param)
#endif
{
    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    /* Init the GPIO config into output state */
    MXCPLD_WRITE_REG(io_param, MX_GPIO_CONFIG, 0xFF);

    return MX_GPIO_OK;
}

/** \fn mxGPIOConfig
 *   \brief Set GPIO input/output config.
 *
 *       This function set the directin config of GPIO, it should be called before read and write.
 *
 *    \param[in] handle some OS need this parameter
 *    \param[in] addr The base address of GPIO address
 *    \param[in] io     The target GPIO pin
 *    \param[in] config The config of GPIO
 *    \return MX_GPIO_OK Success
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOSetConfig(io_param, io, config)
	gpio_param_t io_param;
	int io;
	int config;
#else
int mxGPIOSetConfig(gpio_param_t io_param, int io, int config)
#endif
{
    unsigned char chCurrentConfig;
    unsigned char chSettingFlag;

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(io < MX_GPIO_MIN_PIN || io > MX_GPIO_MAX_PIN)
        return MX_GPIO_ERR;

    if(config != MX_GPIO_STATE_INPUT && config != MX_GPIO_STATE_OUTPUT)
        return MX_GPIO_ERR;

    /* Backup current config */
	chCurrentConfig = MXCPLD_READ_REG(io_param, MX_GPIO_CONFIG);

    /* Create flag for config */
    chSettingFlag = 0x1 << io;

    /* Change current setting according to config */
    if(config == MX_GPIO_STATE_INPUT) {
        chCurrentConfig &= ~(chSettingFlag);
    } else if (config == MX_GPIO_STATE_OUTPUT) {
        chCurrentConfig |= chSettingFlag;
    }

    MXCPLD_WRITE_REG(io_param, MX_GPIO_CONFIG, chCurrentConfig);

    return MX_GPIO_OK;
}

/** \fn mxGPIOGetConfig
 *   \brief Get GPIO input/output config.
 *
 *       This function get the directin config of GPIO, it should be called before read and write.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[in] io     The target GPIO pin
 *    \return MX_GPIO_INPUT    The pin is config as Input
 *               MX_GPIO_OUTPUT The pin is config as Output
 *               MX_GPIO_ERR Error
 */

#ifdef OS_SCO5
int mxGPIOGetConfig(io_param, io)
	gpio_param_t io_param;
	int io;
#else
int mxGPIOGetConfig(gpio_param_t io_param, int io)
#endif
{
    unsigned char chCurrentConfig;
    unsigned char chSettingFlag;

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(io < MX_GPIO_MIN_PIN || io > MX_GPIO_MAX_PIN)
        return MX_GPIO_ERR;

    /* Read current config */
	chCurrentConfig = MXCPLD_READ_REG(io_param, MX_GPIO_CONFIG);

    /* Create flag for config */
    chSettingFlag = 0x1 << io;

    /* Check config */
    if(chCurrentConfig & chSettingFlag)
        return MX_GPIO_STATE_OUTPUT;

    return MX_GPIO_STATE_INPUT;
}

/** \fn mxGPIORead
 *   \brief Read GPIO input.
 *
 *       This function read from GPIO.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[in] io     The target GPIO pin
 *    \return MX_GPIO_LOW  Pull LOW
 *               MX_GPIO_HIGH Pull High
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIORead(io_param, io)
	gpio_param_t io_param;
	int io;
#else
int mxGPIORead(gpio_param_t io_param, int io)
#endif
{
    unsigned char chCurrentData;
    unsigned char chOffset;

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(io < MX_GPIO_MIN_PIN || io > MX_GPIO_MAX_PIN)
        return MX_GPIO_ERR;

    /* Prepare offset */
    chOffset = 0x1 << io;

    /* Read GPIO data */
	chCurrentData = MXCPLD_READ_REG(io_param, MX_GPIO_INPUT);

    if(chCurrentData & chOffset)
        return MX_GPIO_HIGH;

    return MX_GPIO_LOW;
}

/** \fn mxGPIOWrite
 *   \brief write GPIO output.
 *
 *       This function write into GPIO.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[in] io     The target GPIO pin
 *    \param[in] input The output state
 *    \return MX_GPIO_OK   Successful
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOWrite(io_param, io, input)
	gpio_param_t io_param;
	int io;
	int input;
#else
int mxGPIOWrite(gpio_param_t io_param, int io, int input)
#endif
{
    unsigned char chCurrentData;
    unsigned char chOffset;

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(io < MX_GPIO_MIN_PIN || io > MX_GPIO_MAX_PIN)
        return MX_GPIO_ERR;

    if(input != MX_GPIO_LOW && input != MX_GPIO_HIGH)
        return MX_GPIO_ERR;

    /* Prepare offset */
    chOffset = 0x1 << io;

    /* Read GPIO output data */
    chCurrentData = MXCPLD_READ_REG(io_param, MX_GPIO_OUTPUT);

    if(input == MX_GPIO_HIGH)
        chCurrentData |= chOffset;
    else if( input == MX_GPIO_LOW)
        chCurrentData &= ~(chOffset);

    MXCPLD_WRITE_REG(io_param, MX_GPIO_OUTPUT, chCurrentData);

    return MX_GPIO_OK;
}

/** \fn mxGPIOWrite
 *   \brief get GPIO current output status.
 *
 *       This function get current output status.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[in] io     The target GPIO pin
 *    \return MX_GPIO_LOW  Pull LOW
 *               MX_GPIO_HIGH Pull High
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOCurrentWrite(io_param, io)
	gpio_param_t io_param;
	int io;
#else
int mxGPIOCurrentWrite(gpio_param_t io_param, int io)
#endif
{
    unsigned char chCurrentData;
    unsigned char chOffset;

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(io < MX_GPIO_MIN_PIN || io > MX_GPIO_MAX_PIN)
        return MX_GPIO_ERR;

    /* Prepare offset */
    chOffset = 0x1 << io;

    /* Read GPIO output data */
    chCurrentData = MXCPLD_READ_REG(io_param, MX_GPIO_OUTPUT);

    if(chCurrentData & chOffset)
        return MX_GPIO_HIGH;

    return MX_GPIO_LOW;
}

/** \fn mxGPIOCurrentWriteAll
 *   \brief get all GPIO current output status.
 *
 *       This function get all current output status.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[out] data  The output data buffer
 *    \return MX_GPIO_OK Successful
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOCurrentWriteAll(io_param, data)
	gpio_param_t io_param;
	unsigned char *data;
#else
int mxGPIOCurrentWriteAll(gpio_param_t io_param, unsigned char *data)
#endif
{

    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    if(data == NULL)
        return MX_GPIO_ERR;

    *data = MXCPLD_READ_REG(io_param, MX_GPIO_OUTPUT);


    return MX_GPIO_OK;
}

/** \fn mxGPIOWriteAll
 *   \brief Write all GPIO pins in a time.
 *
 *       This function write to all pins in a time.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[in] input The output data
 *    \return MX_GPIO_OK Successful
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOWriteAll(io_param, data)
	gpio_param_t io_param;
	unsigned char data;
#else
int mxGPIOWriteAll(gpio_param_t io_param, unsigned char data)
#endif
{
    if(io_param.base == NULL)
        return MX_GPIO_ERR;

    MXCPLD_WRITE_REG(io_param, MX_GPIO_OUTPUT, data);

    return MX_GPIO_OK;
}

/** \fn mxGPIOReadAll
 *   \brief Read all GPIO pins in a time.
 *
 *       This function read all pins in a time.
 *
 *    \param[in] addr The base address of GPIO address
 *    \param[out] data The output buffer
 *    \return MX_GPIO_OK Successful
 *               MX_GPIO_ERR Error
 */
#ifdef OS_SCO5
int mxGPIOReadAll(io_param, data)
	gpio_param_t io_param;
	unsigned char *data;
#else
int mxGPIOReadAll(gpio_param_t io_param, unsigned char *data)
#endif
{
     if(io_param.base == NULL)
        return MX_GPIO_ERR;

    *data = MXCPLD_READ_REG(io_param, MX_GPIO_INPUT);


     return MX_GPIO_OK;
}
