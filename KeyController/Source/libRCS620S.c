/*
 * RC-S620/S sample library for Arduino
 *
 * Copyright 2010 Sony Corporation
 */

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "libRCS620S.h"
#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"
#include "CardReader.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
#include "ToCoNet.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static uint8 rwCommand(const uint8 *command, uint16 commandLen, uint8 response[RCS620S_MAX_RW_RESPONSE_LEN], uint16 *responseLen);
static void cancel();
static uint8 calcDCS(const uint8 *data, uint16 len);
static void writeSerial(const uint8 *data, uint16 len);
static uint8 readSerial(uint8 *data, uint16 len);
static void flushSerial();
static uint32 msec();
static void delaym(uint32 time);
static uint8 checkTimeout(uint32 t0);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
tsRCS620SData sRCS620SData;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
uint8 rcsInitDevice()
{
    uint8 ret;
    uint8 response[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 responseLen;

    /*Set Timeout*/
    sRCS620SData.timeout = RCS620S_DEFAULT_TIMEOUT;

    /* RFConfiguration (various timings) */
    ret = rwCommand((const uint8 *)"\xd4\x32\x02\x00\x00\x00", 6, response, &responseLen);
    if (!ret || (responseLen != 2) || (memcmp(response, "\xd5\x33", 2) != 0)) {
        return 0;
    }

    /* RFConfiguration (max retries) */
    ret = rwCommand((const uint8 *)"\xd4\x32\x05\x00\x00\x00", 6, response, &responseLen);
    if (!ret || (responseLen != 2) || (memcmp(response, "\xd5\x33", 2) != 0)) {
        return 0;
    }

    /* RFConfiguration (additional wait time = 24ms) */
    ret = rwCommand((const uint8 *)"\xd4\x32\x81\xff", 4, response, &responseLen);
    if (!ret || (responseLen != 2) || (memcmp(response, "\xd5\x33", 2) != 0)) {
        return 0;
    }

    return 1;
}

uint8 rcsPollingFeliCa(uint16 systemCode)
{
    uint8 ret;
    uint8 buf[9];
    uint8 response[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 responseLen;

    /* InListPassiveTarget */
    memcpy(buf, "\xd4\x4a\x01\x01\x00\xff\xff\x00\x00", 9);
    buf[5] = (uint8_t)((systemCode >> 8) & 0xff);
    buf[6] = (uint8_t)((systemCode >> 0) & 0xff);

    ret = rwCommand(buf, 9, response, &responseLen);
    if (!ret || (responseLen != 22) ||
        (memcmp(response, "\xd5\x4b\x01\x01\x12\x01", 6) != 0)) {
        return 0;
    }

    memcpy(sRCS620SData.idm, response + 6, 8);
    memcpy(sRCS620SData.pmm, response + 14, 8);

    sRCS620SData.cardtype = 0x01;

    return 1;
}

uint8 rcsPollingTypeA()
{
    uint8 ret;
    uint8 buf[4];
    uint8 response[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 responseLen;

    /* InListPassiveTarget */
    memcpy(buf, "\xd4\x4a\x01\x00", 4);

    ret = rwCommand(buf, 4, response, &responseLen);
    if (!ret || (memcmp(response, "\xd5\x4b\x01\x01", 4) != 0)) {
        return 0;
    }
    sRCS620SData.nfcidlen = response[7];
    memcpy(sRCS620SData.nfcid, response + 8, sRCS620SData.nfcidlen);

    sRCS620SData.cardtype = 0x02;

    return 1;
}

uint8 rcsCardCommand(const uint8 *command, uint8 commandLen, uint8 response[RCS620S_MAX_CARD_RESPONSE_LEN], uint8 *responseLen)
{
    uint8 ret;
    uint16 commandTimeout;
    uint8 buf[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 len;

    if (sRCS620SData.timeout >= (0x10000 / 2)) {
        commandTimeout = 0xffff;
    } else {
        commandTimeout = (uint16)(sRCS620SData.timeout * 2);
    }

    /* CommunicateThruEX */
    buf[0] = 0xd4;
    buf[1] = 0xa0;
    buf[2] = (uint8)((commandTimeout >> 0) & 0xff);
    buf[3] = (uint8)((commandTimeout >> 8) & 0xff);
    buf[4] = (uint8)(commandLen + 1);
    memcpy(buf + 5, command, commandLen);

    ret = rwCommand(buf, 5 + commandLen, buf, &len);
    if (!ret || (len < 4) || (memcmp(buf, "\xd5\xa1\x00", 3) != 0) || (len != (3 + buf[3]))) {
        return 0;
    }

    *responseLen = (uint8)(buf[3] - 1);
    memcpy(response, buf + 4, *responseLen);

    return 1;
}

uint8 rcsCardDataExchange(const uint8 *command, uint8 commandLen, uint8 response[RCS620S_MAX_CARD_RESPONSE_LEN])
{
    uint8 ret;
    uint16 commandTimeout;
    uint8 buf[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 len;

    /* CardDataExchange */
    buf[0] = 0xd4;
    buf[1] = 0x40;
    buf[2] = 0x01;
    memcpy(buf + 3, command, commandLen);

    ret = rwCommand(buf, 3 + commandLen, buf, &len);
    if (!ret || (memcmp(buf, "\xd5\x41\x00", 3) != 0)) {
        return 0;
    }

    memcpy(response, buf + 3, 4);

    return 1;
}

uint8 rcsRfOff()
{
    uint8 ret;
    uint8 response[RCS620S_MAX_RW_RESPONSE_LEN];
    uint16 responseLen;

    /* RFConfiguration (RF field) */
    ret = rwCommand((const uint8 *)"\xd4\x32\x01\x00", 4,
                    response, &responseLen);
    if (!ret || (responseLen != 2) || (memcmp(response, "\xd5\x33", 2) != 0)) {
        return 0;
    }

    return 1;
}

uint8 rcsPush(const uint8 *data, uint8 dataLen)
{
    uint8 ret;
    uint8 buf[RCS620S_MAX_CARD_RESPONSE_LEN];
    uint8 responseLen;

    if (dataLen > 224) {
        return 0;
    }

    /* Push */
    buf[0] = 0xb0;
    memcpy(buf + 1, sRCS620SData.idm, 8);
    buf[9] = dataLen;
    memcpy(buf + 10, data, dataLen);

    ret = rcsCardCommand(buf, 10 + dataLen, buf, &responseLen);
    if (!ret || (responseLen != 10) || (buf[0] != 0xb1) || (memcmp(buf + 1, sRCS620SData.idm, 8) != 0) || (buf[9] != dataLen)) {
        return 0;
    }

    buf[0] = 0xa4;
    memcpy(buf + 1, sRCS620SData.idm, 8);
    buf[9] = 0x00;

    ret = rcsCardCommand(buf, 10, buf, &responseLen);
    if (!ret || (responseLen != 10) || (buf[0] != 0xa5) || (memcmp(buf + 1, sRCS620SData.idm, 8) != 0) || (buf[9] != 0x00)) {
        return 0;
    }

    delaym(1000);

    return 1;
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
static uint8 rwCommand(const uint8 *command, uint16 commandLen, uint8 response[RCS620S_MAX_RW_RESPONSE_LEN], uint16 *responseLen)
{
    uint8 ret;
    uint8 buf[9];

    flushSerial();

    uint8 dcs = calcDCS(command, commandLen);

    /* transmit the command */
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0xff;
    if (commandLen <= 255) {
        /* normal frame */
        buf[3] = commandLen;
        buf[4] = (uint8)-buf[3];
        writeSerial(buf, 5);
    } else {
        /* extended frame */
        buf[3] = 0xff;
        buf[4] = 0xff;
        buf[5] = (uint8)((commandLen >> 8) & 0xff);
        buf[6] = (uint8)((commandLen >> 0) & 0xff);
        buf[7] = (uint8)-(buf[5] + buf[6]);
        writeSerial(buf, 8);
    }
    writeSerial(command, commandLen);
    buf[0] = dcs;
    buf[1] = 0x00;
    writeSerial(buf, 2);

    /* receive an ACK */
    ret = readSerial(buf, 6);
    if (!ret || (memcmp(buf, "\x00\x00\xff\x00\xff\x00", 6) != 0)) {
        cancel();
        return 0;
    }

    /* receive a response */
    ret = readSerial(buf, 5);
    if (!ret) {
        cancel();
        return 0;
    } else if  (memcmp(buf, "\x00\x00\xff", 3) != 0) {
        return 0;
    }
    if ((buf[3] == 0xff) && (buf[4] == 0xff)) {
        ret = readSerial(buf + 5, 3);
        if (!ret || (((buf[5] + buf[6] + buf[7]) & 0xff) != 0)) {
            return 0;
        }
        *responseLen = (((uint16)buf[5] << 8) | ((uint16)buf[6] << 0));
    } else {
        if (((buf[3] + buf[4]) & 0xff) != 0) {
            return 0;
        }
        *responseLen = buf[3];
    }
    if (*responseLen > RCS620S_MAX_RW_RESPONSE_LEN) {
        return 0;
    }

    ret = readSerial(response, *responseLen);
    if (!ret) {
        cancel();
        return 0;
    }

    dcs = calcDCS(response, *responseLen);

    ret = readSerial(buf, 2);
    if (!ret || (buf[0] != dcs) || (buf[1] != 0x00)) {
        cancel();
        return 0;
    }

    return 1;
}

static void cancel()
{
    /* transmit an ACK */
    writeSerial((const uint8 *)"\x00\x00\xff\x00\xff\x00", 6);
    delaym(1);
    flushSerial();
}

static uint8 calcDCS(const uint8 *data,
                      uint16 len)
{
    uint8 sum = 0;
    uint16 i;

    for (i = 0; i < len; i++) {
        sum += data[i];
    }

    return (uint8)-(sum & 0xff);
}

static void writeSerial(const uint8 *data, uint16 len)
{
	uint16 i;

	for (i = 0; i < len; i++){
		sSerStream.bPutChar(sSerStream.u8Device, data[i]);
	}
}

static uint8 readSerial(uint8 *data, uint16 len)
{
    uint16 nread = 0;
    uint32 t0 = msec();

    while (nread < len) {
        if (checkTimeout(t0)) {
            return 0;
        }

		if (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
			data[nread] = SERIAL_i16RxChar(sSerPort.u8SerialPort);
			nread++;
		}
    }

    return 1;
}

static void flushSerial()
{
	SERIAL_vFlush(sSerStream.u8Device);
}

static uint32 msec() {
    return  u32TickCount_ms;
}

static void delaym(uint32 time) {
	vWait(time);
}


static uint8 checkTimeout(uint32 t0)
{
    uint32 t = msec();

    if (abs(t - t0) >= sRCS620SData.timeout) {
        return 1;
    }

    return 0;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
