#include "can_fdcan1_transport.h"

#include "fdcan.h"

static bool CanFdcan1Transport_Initialize(void *context, uint8_t node_id)
{
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)context;
	FDCAN_FilterTypeDef filter;

	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0U;
	filter.FilterType = FDCAN_FILTER_RANGE;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = ((uint32_t)node_id) << 8;
	filter.FilterID2 = filter.FilterID1 + 0xFFU;
	if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
		return false;
	if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
		FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		return false;
	if (HAL_FDCAN_Start(handle) != HAL_OK)
		return false;
	return HAL_FDCAN_ActivateNotification(handle,
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) == HAL_OK;
}

static bool CanFdcan1Transport_ConfigureBitrate(void *context,
	uint32_t bitrate_kbps)
{
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)context;

	if (bitrate_kbps == 0U || HAL_FDCAN_Stop(handle) != HAL_OK)
		return false;
	if (bitrate_kbps <= 1000U)
	{
		handle->Init.DataPrescaler = 10000U / bitrate_kbps;
		handle->Init.NominalPrescaler = 10000U / bitrate_kbps;
	}
	else
	{
		handle->Init.DataPrescaler = 10000U / bitrate_kbps;
		handle->Init.NominalPrescaler = 10U;
	}
	if (handle->Init.DataPrescaler == 0U || HAL_FDCAN_Init(handle) != HAL_OK)
		return false;
	return HAL_FDCAN_Start(handle) == HAL_OK;
}

static bool CanFdcan1Transport_Receive(void *context, CanTransportFrame *frame)
{
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)context;
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[64];
	uint8_t index;

	if (frame == 0 || HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0,
		&header, data) != HAL_OK)
		return false;
	if (header.IdType != FDCAN_STANDARD_ID ||
		header.RxFrameType != FDCAN_DATA_FRAME ||
		header.DataLength > FDCAN_DLC_BYTES_8)
		return false;
	frame->identifier = (uint16_t)header.Identifier;
	frame->length = (uint8_t)header.DataLength;
	for (index = 0U; index < frame->length; ++index)
		frame->data[index] = data[index];
	return true;
}

static bool CanFdcan1Transport_Transmit(void *context,
	const CanTransportFrame *frame)
{
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)context;
	FDCAN_TxHeaderTypeDef header;

	if (frame == 0 || frame->length > 8U)
		return false;
	header.IdType = FDCAN_STANDARD_ID;
	header.Identifier = frame->identifier;
	header.FDFormat = FDCAN_FD_CAN;
	header.DataLength = frame->length;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.BitRateSwitch = FDCAN_BRS_ON;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0U;
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	return HAL_FDCAN_AddMessageToTxFifoQ(handle, &header,
		(uint8_t *)frame->data) == HAL_OK;
}

CanTransportPort CanFdcan1Transport_CreatePort(void)
{
	CanTransportPort port;
	port.context = &hfdcan1;
	port.initialize = CanFdcan1Transport_Initialize;
	port.configure_bitrate_kbps = CanFdcan1Transport_ConfigureBitrate;
	port.receive = CanFdcan1Transport_Receive;
	port.transmit = CanFdcan1Transport_Transmit;
	return port;
}
