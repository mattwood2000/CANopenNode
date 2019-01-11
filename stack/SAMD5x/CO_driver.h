/*
  Insert License
*/

#ifndef CO_DRIVER_H
#define CO_DRIVER_H


/* For documentation see file drvTemplate/CO_driver.h */


#include <stddef.h>         /* for 'NULL' */
#include <stdint.h>         /* for 'int8_t' to 'uint64_t' */

#include <stdio.h>
#include <string.h>

#include "atmel_start.h"

/* CAN module base address */
#define ADDR_CAN1               CAN0
#define ADDR_CAN2               CAN1

/* Critical sections */
#define CO_LOCK_CAN_SEND()      //taskENTER_CRITICAL()
#define CO_UNLOCK_CAN_SEND()    //taskEXIT_CRITICAL()

#define CO_LOCK_EMCY()          //taskENTER_CRITICAL()
#define CO_UNLOCK_EMCY()        //taskEXIT_CRITICAL()

#define CO_LOCK_OD()            //taskENTER_CRITICAL()
#define CO_UNLOCK_OD()          //taskEXIT_CRITICAL()


/* Data types */
/* int8_t to uint64_t are defined in stdint.h */
typedef unsigned char           bool_t;
typedef float                   float32_t;
typedef long double             float64_t;
typedef char                    char_t;
typedef unsigned char           oChar_t;
typedef unsigned char           domain_t;


/* Return values */
typedef enum {
	CO_ERROR_NO                 = 0,
	CO_ERROR_ILLEGAL_ARGUMENT   = -1,
	CO_ERROR_OUT_OF_MEMORY      = -2,
	CO_ERROR_TIMEOUT            = -3,
	CO_ERROR_ILLEGAL_BAUDRATE   = -4,
	CO_ERROR_RX_OVERFLOW        = -5,
	CO_ERROR_RX_PDO_OVERFLOW    = -6,
	CO_ERROR_RX_MSG_LENGTH      = -7,
	CO_ERROR_RX_PDO_LENGTH      = -8,
	CO_ERROR_TX_OVERFLOW        = -9,
	CO_ERROR_TX_PDO_WINDOW      = -10,
	CO_ERROR_TX_UNCONFIGURED    = -11,
	CO_ERROR_PARAMETERS         = -12,
	CO_ERROR_DATA_CORRUPT       = -13,
	CO_ERROR_CRC                = -14
} CO_ReturnError_t;


/* CAN receive message structure as aligned in CAN module. */
typedef struct {
	/** CAN identifier. It must be read through CO_CANrxMsg_readIdent() function. */
	uint32_t            ident;
	uint8_t             DLC ;
	uint8_t             data[8];
} CO_CANrxMsg_t;


/* Received message object */
typedef struct{
	uint16_t            ident;
	uint16_t            mask;
	void               *object;
	void              (*pFunct)(void *object, const CO_CANrxMsg_t *message);
} CO_CANrx_t;


/* Transmit message object. */
typedef struct{
	uint32_t            ident;
	uint8_t             DLC;
	uint8_t             data[8];
	volatile bool_t     bufferFull;
	volatile bool_t     syncFlag;
	bool_t              rtr;
} CO_CANtx_t;


/* CAN module object. */
typedef struct{
	Can                *CANbaseAddress;
	CO_CANrx_t         *rxArray;
	uint16_t            rxSize;
	CO_CANtx_t         *txArray;
	uint16_t            txSize;
	volatile bool_t     CANnormal;
	volatile bool_t     useCANrxFilters;
	volatile bool_t     bufferInhibitFlag;
	volatile bool_t     firstCANtxMessage;
	volatile uint16_t   CANtxCount;
	uint32_t            errOld;
	void               *em;
	struct can_async_descriptor *descr;
} CO_CANmodule_t;


/* Endianes */
#define CO_LITTLE_ENDIAN


/* Request CAN configuration or normal mode */
void CO_CANsetConfigurationMode(Can *CANbaseAddress);
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule);


/* Initialize CAN module object. */
CO_ReturnError_t CO_CANmodule_init(
	CO_CANmodule_t         *CANmodule,
	Can                    *CANbaseAddress,
	CO_CANrx_t              rxArray[],
	uint16_t                rxSize,
	CO_CANtx_t              txArray[],
	uint16_t                txSize,
	uint16_t                CANbitRate);


/* Switch off CANmodule. */
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule);


/* Read CAN identifier */
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg);


/* Configure CAN message receive buffer. */
CO_ReturnError_t CO_CANrxBufferInit(
	CO_CANmodule_t         *CANmodule,
	uint16_t                index,
	uint16_t                ident,
	uint16_t                mask,
	bool_t                  rtr,
	void                   *object,
	void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message));


/* Configure CAN message transmit buffer. */
CO_CANtx_t *CO_CANtxBufferInit(
	CO_CANmodule_t         *CANmodule,
	uint16_t                index,
	uint16_t                ident,
	bool_t                  rtr,
	uint8_t                 noOfBytes,
	bool_t                  syncFlag);


/* Send CAN message. */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);


/* Clear all synchronous TPDOs from CAN module transmit buffers. */
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule);


/* Verify all errors of CAN module. */
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule);


/* CAN interrupt callback handlers. */
void CO_TxCallback(struct can_async_descriptor *descr);
void CO_RxCallback(struct can_async_descriptor *descr);
void CO_ErrCallback(struct can_async_descriptor *descr, enum can_async_interrupt_type type);


#endif
