#include "can_fdcan1_transport.h"

#include "fdcan.h"

typedef struct
{
	FDCAN_HandleTypeDef *handle;
	uint8_t node_id;
	bool use_fd;
	bool use_brs;
} CanFdcan1TransportContext;

static CanFdcan1TransportContext CanFdcan1State;

static bool CanFdcan1Transport_ConfigureReception(
	CanFdcan1TransportContext *transport)
{
	FDCAN_HandleTypeDef *handle = transport->handle;
	FDCAN_FilterTypeDef filter;

	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0U;
	filter.FilterType = FDCAN_FILTER_RANGE;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = ((uint32_t)transport->node_id) << 8;
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

static bool CanFdcan1Transport_Initialize(void *context, uint8_t node_id)
{
	CanFdcan1TransportContext *transport =
		(CanFdcan1TransportContext *)context;
	FDCAN_HandleTypeDef *handle = transport->handle;

	transport->node_id = node_id;
	handle->Init.FrameFormat = transport->use_fd ?
		(transport->use_brs ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_FD_NO_BRS) :
		FDCAN_FRAME_CLASSIC;
	if (HAL_FDCAN_Init(handle) != HAL_OK)
		return false;
	return CanFdcan1Transport_ConfigureReception(transport);
}

static bool CanFdcan1Transport_ConfigureBitrate(void *context,
	uint32_t bitrate_kbps)
{
	CanFdcan1TransportContext *transport =
		(CanFdcan1TransportContext *)context;
	FDCAN_HandleTypeDef *handle = transport->handle;

	if (bitrate_kbps == 0U || (!transport->use_fd && bitrate_kbps > 1000U) ||
		HAL_FDCAN_Stop(handle) != HAL_OK)
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
	return CanFdcan1Transport_ConfigureReception(transport);
}

static bool CanFdcan1Transport_ConfigureNodeId(void *context, uint8_t node_id)
{
	CanFdcan1TransportContext *transport =
		(CanFdcan1TransportContext *)context;

	if (node_id > 7U || HAL_FDCAN_Stop(transport->handle) != HAL_OK)
		return false;
	transport->node_id = node_id;
	if (HAL_FDCAN_Init(transport->handle) != HAL_OK)
		return false;
	return CanFdcan1Transport_ConfigureReception(transport);
}

static bool CanFdcan1Transport_Receive(void *context, CanTransportFrame *frame)
{
	CanFdcan1TransportContext *transport =
		(CanFdcan1TransportContext *)context;
	FDCAN_HandleTypeDef *handle = transport->handle;
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
	CanFdcan1TransportContext *transport =
		(CanFdcan1TransportContext *)context;
	FDCAN_HandleTypeDef *handle = transport->handle;
	FDCAN_TxHeaderTypeDef header;

	if (frame == 0 || frame->length > 8U)
		return false;
	header.IdType = FDCAN_STANDARD_ID;
	header.Identifier = frame->identifier;
	header.FDFormat = transport->use_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	header.DataLength = frame->length;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.BitRateSwitch = transport->use_brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0U;
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	return HAL_FDCAN_AddMessageToTxFifoQ(handle, &header,
		(uint8_t *)frame->data) == HAL_OK;
}

CanTransportPort CanFdcan1Transport_CreatePort(bool use_fd, bool use_brs)
{
	CanTransportPort port;
	CanFdcan1State.handle = &hfdcan1;
	CanFdcan1State.node_id = 0U;
	CanFdcan1State.use_fd = use_fd;
	CanFdcan1State.use_brs = use_fd && use_brs;
	port.context = &CanFdcan1State;
	port.maximum_bitrate_kbps = use_fd ? 5000U : 1000U;
	port.initialize = CanFdcan1Transport_Initialize;
	port.configure_node_id = CanFdcan1Transport_ConfigureNodeId;
	port.configure_bitrate_kbps = CanFdcan1Transport_ConfigureBitrate;
	port.receive = CanFdcan1Transport_Receive;
	port.transmit = CanFdcan1Transport_Transmit;
	return port;
}
