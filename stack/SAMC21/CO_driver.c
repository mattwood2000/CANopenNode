/*

  Insert License
  CAN module driver for the SAMC21 based on AtmelStart framework

*/

#include "CANopen.h"
#include "CO_driver.h"
#include "CO_Emergency.h"
#include "driver_init.h"

#define CO_UNUSED(v)  (void)(v)

static int32_t CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);

/******************************************************************************/
void CO_CANsetConfigurationMode(Can *CANbaseAddress)
{
  CO_UNUSED(CANbaseAddress);
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
  CO_UNUSED(CANmodule->CANbaseAddress);

    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        Can                    *CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
	uint16_t i;

	/* verify arguments */
	if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure object variables */
	CANmodule->CANbaseAddress = CANbaseAddress;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false;
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;
	CANmodule->em = NULL;
	CANmodule->descr = &CAN_0;

	for(i=0U; i<rxSize; i++) {
		rxArray[i].ident = 0U;
		rxArray[i].pFunct = NULL;
	}

	for(i=0U; i<txSize; i++) {
		txArray[i].bufferFull = false;
	}

	/* Configure CAN module registers */
	can_async_register_callback(CANmodule->descr, CAN_ASYNC_RX_CB, (FUNC_PTR)CO_RxCallback);
	can_async_register_callback(CANmodule->descr, CAN_ASYNC_TX_CB, (FUNC_PTR)CO_TxCallback);
	can_async_enable(CANmodule->descr);

	return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
	can_async_disable(CANmodule->descr);
}

CO_ReturnError_t CO_CANrxBufferInit(
				    CO_CANmodule_t         *CANmodule,
                                    uint16_t                index,
                                    uint16_t                ident,
                                    uint16_t                mask,
                                    bool_t                  rtr,
                                    void                   *object,
                                    void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
	CO_ReturnError_t ret = CO_ERROR_NO;

	if ((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) && (index < CANmodule->rxSize)) {
		/* Buffer, which will be configured */
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->pFunct = pFunct;

		/*
		  CAN identifier and CAN mask, bit aligned with CAN module.
		  Different on different microcontrollers.
		*/
		buffer->ident = ident & 0x07FFU;

		if(rtr) {
			buffer->ident |= 0x0800U;
		}

		buffer->mask = (mask & 0x07FFU) | 0x0800U;

		/* Set CAN hardware module filter and mask. */
		if(CANmodule->useCANrxFilters) {

		}
	} else {
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}

CO_CANtx_t *CO_CANtxBufferInit(
				CO_CANmodule_t         *CANmodule,
				uint16_t                index,
				uint16_t                ident,
				bool_t                  rtr,
				uint8_t                 noOfBytes,
				bool_t                  syncFlag)
{
	CO_CANtx_t *buffer = NULL;

	if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
		/* Get specific buffer */
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer. */
		buffer->ident = ident;
		buffer->rtr = rtr;

		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
		buffer->DLC = noOfBytes;
	}

	return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t err = CO_ERROR_NO;
	bool remoteFrame = false;

	/* Verify overflow */
	if (buffer->bufferFull) {
		if (!CANmodule->firstCANtxMessage) {
			/* Don't set error, if bootup message is still on buffers */
			CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
		}
	err = CO_ERROR_TX_OVERFLOW;
	}

	CO_LOCK_CAN_SEND();

	// Check to see if TX buffer is free and copy message */
	if (hri_can_get_TXFQS_TFQF_bit((const void *const)&CANmodule->descr->dev) == false) {

		CANmodule->bufferInhibitFlag = buffer->syncFlag;

		//can_async_register_callback(CANmodule->descr, CAN_ASYNC_TX_CB, (FUNC_PTR)CO_TxCallback);
		//hri_can_write_IE_TCE_bit(CANmodule->descr->dev.hw, true);
		CO_CANsendToModule(CANmodule, buffer);

	} else {	// if buffer is not free, send by interrupt
		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	}

	CO_UNLOCK_CAN_SEND();

	return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND();
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality.
	*/

	if (/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag) {
		/* clear TXREQ */
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}

	/* delete also pending synchronous TPDOs in TX buffers */
	if (CANmodule->CANtxCount != 0U) {
		uint16_t i;
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		for (i = CANmodule->txSize; i > 0U; i--) {
			if (buffer->bufferFull) {
				if (buffer->syncFlag) {
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;
					tpdoDeleted = 2U;
				}
			}
			buffer++;
		}
	}

	CO_UNLOCK_CAN_SEND();

	if (tpdoDeleted != 0U) {
		CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
	}
}

/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
	uint16_t rxErrors = 0, txErrors = 0, overflow = 0;
	CO_EM_t* em = (CO_EM_t*)CANmodule->em;
	uint32_t err;

	/* get error counters from module. Id possible, function may use different way to
	 * determine errors.
	*/

	rxErrors = can_async_get_rxerr(CANmodule->descr);

	txErrors = can_async_get_txerr(CANmodule->descr);

	err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

	if (CANmodule->errOld != err) {
		CANmodule->errOld = err;

		if(txErrors >= 256U) {   /* bus off */
			CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
		} else {  /* not bus off */
			CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

			if ((rxErrors >= 96U) || (txErrors >= 96U)) {
				/* bus warning */
				CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
			}

			if (rxErrors >= 128U) { /* RX bus passive */
				CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
			} else {
				CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
			}

			if (txErrors >= 128U) { /* TX bus passive */
				if (!CANmodule->firstCANtxMessage) {
					CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
				}
			} else {
				bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
				if (isError) {
					CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
					CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
				}
			}

			if ((rxErrors < 96U) && (txErrors < 96U)) { /* no error */
				CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
			}
		}

		if (overflow != 0U) { /* CAN RX bus overflow */
			CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
		}
	}
}

/******************************************************************************/
void CO_TxCallback(struct can_async_descriptor *descr)
{
	CO_CANmodule_t *CANmodule = CO->CANmodule[0];
	struct can_message mcanTxMsg;
	CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
	bool_t msgMatched = false;
	int32_t ret;

	CANmodule->descr = descr;

	/* First CAN message (bootup) was sent successfully */
	CANmodule->firstCANtxMessage = false;
	/* Clear flag from previous message */
	CANmodule->bufferInhibitFlag = false;
	/* Are there any new messages waiting to be send */
	if (CANmodule->CANtxCount > 0U) {
		uint16_t j;    /* Index of transmitting message */

		/* First buffer */
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		/* Search through whole array of pointers to transmit message buffers. */
		for (j = CANmodule->txSize; j > 0U; j--) {
			/* If message buffer is full, send it. */
			if( buffer->bufferFull) {
				buffer->bufferFull = false;
				CANmodule->CANtxCount--;

				CO_CANsendToModule(CANmodule, buffer);

				break; /* Exit for loop */
			}
			buffer++;
		} /* End of for loop */

		/* Clear counter if no more messages */
		if (j == 0U) {
			CANmodule->CANtxCount = 0U;
		}
	} else {
		/* Nothing more to send */
		//can_disable_interrupt(CANmodule->CANbaseAddress, 0x1u << CANMB_TX);
		//can_async_register_callback(CANmodule->descr, CAN_ASYNC_TX_CB, NULL);
		//hri_can_write_IE_TCE_bit(CANmodule->descr->dev.hw, false);
		return;
	}

#if 0
    if (ul_status & CAN_SR_ERRA);   //error active
    if (ul_status & CAN_SR_WARN);   //warning limit
    //CO_EM_CAN_BUS_WARNING
    if (ul_status & CAN_SR_ERRP);   //error passive
    //CO_EM_CAN_TX_BUS_PASSIVE
    if (ul_status & CAN_SR_BOFF);   //bus off
    //CO_EM_CAN_TX_BUS_OFF
    if (ul_status & CAN_SR_SLEEP);  //controller in sleep mode
    if (ul_status & CAN_SR_WAKEUP); //controller woke up
    if (ul_status & CAN_SR_TOVF);   //timer overflow
    if (ul_status & CAN_SR_TSTP);   //timestamp - start or end of frame
    if (ul_status & CAN_SR_CERR);   //CRC error in mailbox
    if (ul_status & CAN_SR_SERR);   //stuffing error in mailbox
    if (ul_status & CAN_SR_AERR);   //ack error
    if (ul_status & CAN_SR_FERR);   //form error
    if (ul_status & CAN_SR_BERR);   //bit error
#endif
}

/******************************************************************************/
void CO_RxCallback(struct can_async_descriptor *descr)
{
	CO_CANmodule_t *CANmodule = CO->CANmodule[0];
	struct can_message mcanRxMsg;
	CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
	CO_CANrxMsg_t rcvMsgBuf;
	uint16_t index;             /* index of received message */
	uint32_t rcvMsgIdent;       /* identifier of the received message */
	CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
	bool_t msgMatched = false;
	int32_t ret;

	CANmodule->descr = descr;

	mcanRxMsg.data = rcvMsgBuf.data;
	memset(rcvMsgBuf.data, 0, 8);
	ret = can_async_read(CANmodule->descr, &mcanRxMsg);

	rcvMsgBuf.ident = mcanRxMsg.id;
	rcvMsgBuf.DLC = mcanRxMsg.len;

	rcvMsg = &rcvMsgBuf;

	rcvMsgIdent = rcvMsg->ident;

	if (CANmodule->useCANrxFilters) {
		/* CAN module filters are used. Message with known 11-bit identifier has */
		/* been received */
		index = 0;  /* get index of the received message here. Or something similar */
		if (index < CANmodule->rxSize) {
			buffer = &CANmodule->rxArray[index];
			/* verify also RTR */
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
				msgMatched = true;
			}
		}
	} else {
		/* CAN module filters are not used, message with any standard 11-bit identifier */
		/* has been received. Search rxArray from CANmodule for the same CAN-ID. */
		buffer = &CANmodule->rxArray[0];
		for (index = CANmodule->rxSize; index > 0U; index--) {
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
				msgMatched = true;
				break;
			}
			buffer++;
		}
	}

	/* Call specific function, which will process the message */
	if (msgMatched && (buffer != NULL) && (buffer->pFunct != NULL)) {
		buffer->pFunct(buffer->object, rcvMsg);
	}
}

static int32_t CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	struct can_message mcanTxMsg;
	int32_t ret;

	mcanTxMsg.id   = buffer->ident;
	mcanTxMsg.data = buffer->data;
	mcanTxMsg.len  = buffer->DLC;
	mcanTxMsg.fmt  = CAN_FMT_STDID;

	if (buffer->rtr == true) {
		mcanTxMsg.type = CAN_TYPE_REMOTE;
	} else {
		mcanTxMsg.type = CAN_TYPE_DATA;
	}

	ret = can_async_write(CANmodule->descr, &mcanTxMsg);

	return ret;
}
