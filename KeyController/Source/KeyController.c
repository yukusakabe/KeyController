/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - 2013 all rights reserved.
 *
 * Condition to use:
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"
#include "eeprom_6x.h"

#include "KeyController.h"
#include "config.h"
#include "Version.h"

#include "SMBus.h"
#include "SMBus.c"

// DEBUG options

#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
#define ToCoNet_USE_MOD_NBSCAN_SLAVE
#define ToCoNet_USE_MOD_NWK_LAYERTREE

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct
{
    // Counter
    uint8 u8SleepCt;
    uint16 u16SleepCt;
    uint16 u16IntervalCt;
    uint16 u16PollingCt;
    uint16 u16TimerCt[3];
    uint32 u32LedCt;

    // Card Id
    uint8 u8CardNum;

    uint8 u8LockState[3];
    uint8 u8Do[4];
    uint8 u8Di[4];

    uint8 u8Lock[3];
    uint8 u8Door[3];
    uint8 u8Output[3];

    bool u8LockLogic;
    bool u8DoorLogic;
    bool u8PinLogic;
} tsAppData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vInitHardware(int f_warm_start);
static void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);
static void vPin(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vHandleSerialInput(void);
static void vSendPacket(uint8 *data, uint8 len);
static void vCheckState();
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static uint8 vReadRom(uint8 *id);
static uint8 vWriteRom(uint8 *id);
static uint8 vLockOpen(uint8 dnum);
static uint8 vLockClose(dnum);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Version/build information. This is not used in the application unless we
   are in serial debug mode. However the 'used' attribute ensures it is
   present in all binary files, allowing easy identifaction... */

// Wakeup port
static const uint32 u32DioPortWakeUp = 1UL << 7; // UART Rx Port
static bool_t bWakeupByButton;
tsToCoNet_Nwk_Context *pContextNwk = NULL;
tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig;

//
static tsAppData sAppData;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbAppColdStart(bool_t bAfterAhiInit)
{
	//static uint8 u8WkState;
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.

		// Register modules
		ToCoNet_REG_MOD_ALL();

	} else {
		// disable brown out detect
		vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
				FALSE,
				FALSE,
				FALSE,
				FALSE);

		// clear application context
		memset (&sAppData, 0x00, sizeof(sAppData));
		memset (&sNwkLayerTreeConfig, 0x00, sizeof(tsToCoNet_NwkLyTr_Config));


		// ToCoNet configuration
		sToCoNet_AppContext.u32AppId = APP_ID;
		sToCoNet_AppContext.u8Channel = CHANNEL;
		sToCoNet_AppContext.bRxOnIdle = TRUE;

		// others
		SPRINTF_vInit128();

		// Others
		vInitHardware(FALSE);

		// Register
		ToCoNet_Event_Register_State_Machine(vPin);
		ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// MAC start
		ToCoNet_vMacStart();

		sAppData.u8LockState[0] = 0;
		sAppData.u8LockState[1] = 0;
		sAppData.u8LockState[2] = 0;

		EEP_6x_bRead(1, 1, &sAppData.u8LockLogic);
		EEP_6x_bRead(2, 1, &sAppData.u8DoorLogic);
		EEP_6x_bRead(3, 1, &sAppData.u8PinLogic);

		SPRINTF_vInit128();

		vSMBusInit();
		//bSMBusWrite(0x3F, 0x00, 0, NULL);
	}
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbAppWarmStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.
		bWakeupByButton = FALSE;

		if(u8AHI_WakeTimerFiredStatus()) {
			// wake up timer
		} else
		if(u32AHI_DioWakeStatus() & u32DioPortWakeUp) {
			// woke up from DIO events
			bWakeupByButton = TRUE;
		} else {
			bWakeupByButton = FALSE;
		}
	} else {
		// Initialize hardware
		vInitHardware(TRUE);

		// MAC start
		ToCoNet_vMacStart();
	}
}

/****************************************************************************
 *
 * NAME: vMain
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vMain(void)
{
	vHandleSerialInput();
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vNwkEvent
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
	switch(eEvent) {
	default:
		break;
	}
}

/****************************************************************************
 *
 * NAME: cbvMcRxHandler
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
	int i;
	uint8 c = 0;
	uint8 x = 0;
	uint8 id[10];
	uint8 *p = pRx->auData;
	uint8 buf[256];

	if (p[0] == 0xEE) {
		switch (p[1]) {
			case 0x00:
				break;

			case 0x01:
				vLockOpen(0);
				break;

			case 0x02:
				vLockOpen(1);
				break;

			case 0x03:
				vLockOpen(2);
				break;

			case 0x11:
				vLockClose(0);
				break;

			case 0x12:
				vLockClose(1);
				break;

			case 0x13:
				vLockClose(2);
				break;

			case 0x20:
				sAppData.u8LockLogic = sAppData.u8LockLogic ^ 1;
				EEP_6x_bWrite(1, 1, &sAppData.u8LockLogic);
				break;

			case 0x21:
				sAppData.u8DoorLogic = sAppData.u8DoorLogic ^ 1;
				EEP_6x_bWrite(1, 1, &sAppData.u8DoorLogic);
				break;

			case 0x22:
				sAppData.u8PinLogic = sAppData.u8PinLogic  ^ 1;
				EEP_6x_bWrite(1, 1, &sAppData.u8PinLogic);
				break;


			default:
				break;
		}
	}


	if (p[0] == 0xff) {
		switch (p[1]) {
		case 0x00:
			if ( pRx->u8Len >=12) {
				if (vReadRom(p + 2) != 1) {
					vWriteRom(p + 2);
					EEP_6x_bRead(0, 1, &c);
					EEP_6x_bRead(c  * 10, 10, id);

					vfPrintf(&sSerStream, "R:");
					for (i = 0; i < 10; i++) {
						vfPrintf(&sSerStream, "%02x",  id[i]);
					}
				}
			}
			break;

		case 0xFF:
			vfPrintf(&sSerStream, "EEPROM Deleted"LB);
			EEP_6x_bWrite(0, 1, &x);
			break;

		case 0x01:
				if ( pRx->u8Len >=12) {
					if (vReadRom(p + 2) ) {
						vLockOpen(0);

						vfPrintf(&sSerStream, " :");
						for (i = 0; i < 10; i++) {
							vfPrintf(&sSerStream, "%02x",  p[i + 2]);
						}
					}
				}
			break;

		case 0x02:
			if ( pRx->u8Len >=12) {
				if (vReadRom(p + 2)) {
					vLockOpen(1);
					vfPrintf(&sSerStream, " :");
					for (i = 0; i < 10; i++) {
						vfPrintf(&sSerStream, "%02x",  p[i + 2]);
					}
				}
			}
			break;

		case 0x03:
			if ( pRx->u8Len >=12) {
				if (vReadRom(p + 2)) {
					vLockOpen(2);
					vfPrintf(&sSerStream, " :");
					for (i = 0; i < 10; i++) {
						vfPrintf(&sSerStream, "%02x",  p[i + 2]);
					}
				}
			}
			break;
		}
	}

	sAppData.u32LedCt = u32TickCount_ms;
}

/****************************************************************************
 *
 * NAME: cbvMcEvTxHandler
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	return;
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
   		// LED ON when receive
    	break;

    default:
    	break;
    }
}

/****************************************************************************
 *
 * NAME: cbToCoNet_u8HwInt
 *
 * DESCRIPTION:
 *   called during an interrupt
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 *                  FALSE -  interrupt is not handled, escalated to further
 *                           event call (cbToCoNet_vHwEvent).
 *                  TRUE  -  interrupt is handled, no further call.
 *
 * NOTES:
 *   Do not put a big job here.
 ****************************************************************************/
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {

	return FALSE;
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vInitHardware(int f_warm_start)
{
	// Serial Initialize
#if 0
	// UART の細かい設定テスト
	tsUartOpt sUartOpt;
	memset(&sUartOpt, 0, sizeof(tsUartOpt));
	sUartOpt.bHwFlowEnabled = FALSE;
	sUartOpt.bParityEnabled = E_AHI_UART_PARITY_ENABLE;
	sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
	sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
	sUartOpt.u8WordLen = 7;

	vSerialInit(UART_BAUD, &sUartOpt);
#else
	vSerialInit(UART_BAUD, NULL);
#endif
	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(0);
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[96];
	static uint8 au8SerialRxBuffer[32];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_SLAVE;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_SLAVE;
}

/****************************************************************************
 *
 * NAME:vInitExternalDevice
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vHandleSerialInput(void)
{
    // handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;
		uint8 c = 0;
		int i;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
	    SERIAL_vFlush(sSerStream.u8Device);

		switch(i16Char) {
		case 't':
			EEP_6x_bRead(0, 1, &c);
			vfPrintf(&sSerStream, LB "%d" LB, c);
			break;

		case 's':
			vfPrintf(&sSerStream, LB"%01x%01x%01x ", sAppData.u8DoorLogic, sAppData.u8LockLogic,  sAppData.u8PinLogic);

				for (i = 0; i < 3; i++) {
					if (sAppData.u8Door[i] == 0) {
						vfPrintf(&sSerStream, "O");
					} else {
						vfPrintf(&sSerStream, "C");
					}

					if (sAppData.u8Lock[i] == 0) {
						vfPrintf(&sSerStream, "U");
					} else {
						vfPrintf(&sSerStream, "L");
					}

					vfPrintf(&sSerStream, "%01x", sAppData.u8Output[i]);

					vfPrintf(&sSerStream, " ");
				}
			break;

		case 'l':
			sAppData.u32LedCt = u32TickCount_ms;
			break;

		case '1':
			vLockOpen(0);
			break;

		case '2':
			vLockOpen(1);
			break;

		case '3':
			vLockOpen(2);
			break;

		default:
			break;
		}

		vfPrintf(&sSerStream, LB);
	    SERIAL_vFlush(sSerStream.u8Device);
	}
}

static void vPin(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
	uint8 i;

	switch (eEvent) {
	case E_EVENT_TICK_TIMER:
		vCheckState();

		for (i = 0; i < 3; i++) {
			switch (sAppData.u8LockState[i]) {
			default :

			case 0:
				if (!sAppData.u8Lock[i] && !sAppData.u8Door[i]) {
					sAppData.u8LockState[i] = 1;
					vfPrintf(&sSerStream, "Door %d :State 1"LB, i);
					vfPrintf(&sSerStream, "Door %02X : Lock %02X"LB, sAppData.u8Door[0], sAppData.u8Lock[0]);
				} else {
					vLockClose(i);
					vWait(10);
				}
				break;

			case 1:
				if (sAppData.u8Lock[i]) {
					sAppData.u8LockState[i] = 2;
					vLockOpen(i);
					vfPrintf(&sSerStream, "Door %d :State 2"LB, i);
					vfPrintf(&sSerStream, "Door %02X : Lock %02X"LB, sAppData.u8Door[0], sAppData.u8Lock[0]);
				}
				break;

			case 2:
				if  (sAppData.u8Door[i]) {
					sAppData.u8LockState[i] = 0;
					vfPrintf(&sSerStream, "Door %d :State 0"LB, i);
					vfPrintf(&sSerStream, "Door %02X : Lock %02X"LB, sAppData.u8Door[0], sAppData.u8Lock[0]);
				} else {
					if  (sAppData.u16TimerCt[i] == 0) {
						sAppData.u8LockState[i] = 0;
						vfPrintf(&sSerStream, "Door %d :State0"LB, i);
						vfPrintf(&sSerStream, "Door %02X : Lock %02X"LB, sAppData.u8Door[0], sAppData.u8Lock[0]);
					} else {
						sAppData.u16TimerCt[i]--;
						//vfPrintf(&sSerStream, LB"%d", sAppData.u16TimerCt[i]);
					}
				}

				break;
			}
		}

		break;

	default:
		break;
	}
}

static uint8 vReadRom(uint8 *id)
{
	int i;
	uint8 buf[10];
	uint8 cardnum;

	EEP_6x_bRead(0, 1, &cardnum);

	for (i = 0; i < cardnum; i++) {
		EEP_6x_bRead((i + 1) * 10, 10, buf);

		if (memcmp(buf, id, 10) == 0) {
			return 1;
		}
	}

	return 0;
}

static uint8 vWriteRom(uint8 *id)
{
	uint8 c;
	EEP_6x_bRead(0, 1, &c);

	EEP_6x_bWrite((c + 1) * 10, 10, id);
	c++;
	EEP_6x_bWrite(0, 1, &c);
}

static uint8 vLockOpen(uint8 dnum) {
	switch (dnum) {
	case 0:
		if (sAppData.u8PinLogic == 0) {
			bSMBusWrite(0x38, 0xFF, 0, NULL);
		} else {
			bSMBusWrite(0x38, 0xDF, 0, NULL);
		}

	case 1:
		if (sAppData.u8PinLogic == 0) {
			bSMBusWrite(0x39, 0xFF, 0, NULL);
		} else {
			bSMBusWrite(0x39, 0xDF, 0, NULL);
		}

	case 2:
		if (sAppData.u8PinLogic == 0) {
			bSMBusWrite(0x3A, 0xFF, 0, NULL);
		} else {
			bSMBusWrite(0x3A, 0xDF, 0, NULL);
		}
	}
	sAppData.u8LockState[dnum] = 2;
	sAppData.u16TimerCt[dnum] = 2500;
	vfPrintf(&sSerStream, "Door No.%d Open"LB, dnum + 1);
}

static uint8 vLockClose(dnum) {
	if (!sAppData.u8Door[dnum]) {
		switch (dnum) {
		case 0:
			if (sAppData.u8PinLogic == 0) {
				bSMBusWrite(0x38, 0xDF, 0, NULL);
				vfPrintf(&sSerStream, "ASDASD");
			} else {
				bSMBusWrite(0x38, 0xFF, 0, NULL);
			}

		case 1:
			if (sAppData.u8PinLogic == 0) {
				bSMBusWrite(0x39, 0xDF, 0, NULL);
			} else {
				bSMBusWrite(0x39, 0xFF, 0, NULL);
			}

		case 2:
			if (sAppData.u8PinLogic == 0) {
				bSMBusWrite(0x3A, 0xDF, 0, NULL);
			} else {
				bSMBusWrite(0x3A, 0xFF, 0, NULL);
			}
		}
		vfPrintf(&sSerStream, "Door No.%d Close"LB, dnum + 1);
	}
}

static void vSendPacket(uint8 *data, uint8 len) {
	tsTxDataApp tsTx;
	uint8 buf[512];

	memset(&tsTx, 0, sizeof(tsTxDataApp));
	memset(&buf, 0, sizeof(buf));

	tsTx.u32SrcAddr = ToCoNet_u32GetSerial();
	tsTx.u32DstAddr = 0xFFFF;

	tsTx.bAckReq = FALSE;
	tsTx.u8Retry = 0x00;
	tsTx.u8CbId = 0x00;
	tsTx.u8Seq = 0x00 & 0xFF;
	tsTx.u8Cmd = 0;

	buf[0] = 0xEE;
	buf[1] = 0x00;

	memcpy(buf + 2, data, (int)len);

	memcpy(tsTx.auData,  buf, (int)len + 2);
	tsTx.u8Len = len + 2;

	ToCoNet_bMacTxReq(&tsTx);
}

static void vCheckState() {
	bool_t bOk = TRUE;
	uint8 state[3];
	int i;

	bOk &= bSMBusSequentialRead(0x38, 1, &state[0]);
	if (!bOk) vfPrintf(&sSerStream, LB "Read Error 1");
	vWait(50);
	bOk &= bSMBusSequentialRead(0x39, 1, &state[1]);
	if (!bOk) vfPrintf(&sSerStream, LB "Read Error 2");
	vWait(50);
	bOk &= bSMBusSequentialRead(0x3A, 1, &state[2]);
	if (!bOk) vfPrintf(&sSerStream, LB "Read Error 3");
	vWait(50);

	sAppData.u8Door[0] = (state[0] & 0x10) && 0x10 ^ sAppData.u8DoorLogic;
	sAppData.u8Door[1] = (state[1] & 0x10) && 0x10 ^ sAppData.u8DoorLogic;
	sAppData.u8Door[2] = (state[2] & 0x10) && 0x10 ^ sAppData.u8DoorLogic;

	sAppData.u8Lock[0] = (state[0] & 0x08) && 0x08 ^ sAppData.u8LockLogic;
	sAppData.u8Lock[1] = (state[1] & 0x08) && 0x08 ^ sAppData.u8LockLogic;
	sAppData.u8Lock[2] = (state[2] & 0x08) && 0x08 ^ sAppData.u8LockLogic;

	sAppData.u8Output[0] = (state[0] & 0x20) && 0x20;
	sAppData.u8Output[1] = (state[1] & 0x20) && 0x20;
	sAppData.u8Output[2] = (state[2] & 0x20) && 0x20;
}

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	if (eEvent == E_EVENT_START_UP) {
		sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_PARENT;
		//sNwkLayerTreeConfig.u8Role = TOCONET_NWK_ROLE_ROUTER;
		//sNwkLayerTreeConfig.u8Layer = 1;

		pContextNwk = ToCoNet_NwkLyTr_psConfig(&sNwkLayerTreeConfig);
		ToCoNet_Nwk_bInit(pContextNwk);
		ToCoNet_Nwk_bStart(pContextNwk);
	}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
