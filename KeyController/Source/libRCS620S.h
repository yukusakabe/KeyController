/*
 * RC-S620/S sample library for Arduino
 *
 * Copyright 2010 Sony Corporation
 */

#ifndef ____libRCS620S__
#define ____libRCS620S__

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "CardReader.h"
#include <stdint.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define RCS620S_MAX_CARD_RESPONSE_LEN		254
#define RCS620S_MAX_RW_RESPONSE_LEN			265
#define RCS620S_DEFAULT_TIMEOUT					1000

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct
{
	uint32 timeout;
	uint8 cardtype;
	uint8 idm[8];
	uint8 pmm[8];
	uint8 nfcid[10];
	uint8 nfcidlen;
} tsRCS620SData;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
uint8 rcsInitDevice();
uint8 rcsPollingFeliCa(uint16);
uint8 rcsPollingTypeA();
uint8 rcsCardCommand(const uint8 *command, uint8 commandLen, uint8 response[RCS620S_MAX_CARD_RESPONSE_LEN], uint8 *responseLen);
uint8 rcsCardDataExchange(const uint8 *command, uint8 commandLen, uint8 response[RCS620S_MAX_CARD_RESPONSE_LEN]);
uint8 rcsRfOff();
uint8 rcsPush(const uint8 *data, uint8 dataLen);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern tsRCS620SData sRCS620SData;

#endif /* !RCS620S_H_ */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
