#include "rotor_feedback_runtime.h"

#include <string.h>

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)
#define TEST_NEAR(left_, right_, tolerance_) \
	TEST_CHECK(((left_) > (right_) ? (left_) - (right_) : \
		(right_) - (left_)) <= (tolerance_))

typedef struct
{
	BspAngleSensorSample sample;
	BspResult initialize_result;
	BspResult start_result;
	BspResult request_result;
	BspResult read_result;
	uint32_t initialize_count;
	uint32_t start_count;
	uint32_t stop_count;
} RotorFeedbackFakePortContext;

static BspResult RotorFeedbackFake_Initialize(void *opaque)
{
	RotorFeedbackFakePortContext *context = opaque;
	context->initialize_count++;
	return context->initialize_result;
}

static BspResult RotorFeedbackFake_Start(void *opaque)
{
	RotorFeedbackFakePortContext *context = opaque;
	context->start_count++;
	return context->start_result;
}

static BspResult RotorFeedbackFake_Stop(void *opaque)
{
	RotorFeedbackFakePortContext *context = opaque;
	context->stop_count++;
	return BSP_RESULT_OK;
}

static BspResult RotorFeedbackFake_Request(void *opaque)
{
	RotorFeedbackFakePortContext *context = opaque;
	return context->request_result;
}

static BspResult RotorFeedbackFake_Read(void *opaque,
	BspAngleSensorSample *sample)
{
	RotorFeedbackFakePortContext *context = opaque;
	if (context->read_result == BSP_RESULT_OK)
		*sample = context->sample;
	return context->read_result;
}

static BspAngleSampleStatusSet RotorFeedbackFake_Status(void *opaque)
{
	RotorFeedbackFakePortContext *context = opaque;
	return context->sample.status;
}

static BspAngleSensorPort RotorFeedbackFake_CreatePort(
	RotorFeedbackFakePortContext *context)
{
	BspAngleSensorPort port;

	memset(context, 0, sizeof(*context));
	context->initialize_result = BSP_RESULT_OK;
	context->start_result = BSP_RESULT_OK;
	context->request_result = BSP_RESULT_OK;
	context->read_result = BSP_RESULT_OK;
	context->sample.status = BSP_ANGLE_SAMPLE_POSITION_VALID;
	context->sample.sequence = 1U;
	memset(&port, 0, sizeof(port));
	port.context = context;
	port.initialize = RotorFeedbackFake_Initialize;
	port.start_acquisition = RotorFeedbackFake_Start;
	port.stop_acquisition = RotorFeedbackFake_Stop;
	port.request_sample = RotorFeedbackFake_Request;
	port.try_read_latest = RotorFeedbackFake_Read;
	port.read_status = RotorFeedbackFake_Status;
	return port;
}

static FeedbackRouterSourceRef RotorFeedbackTest_None(void)
{
	FeedbackRouterSourceRef source = {FEEDBACK_ROUTER_SOURCE_NONE,
		FEEDBACK_ROUTER_SOURCE_INDEX_NONE};
	return source;
}

static FeedbackRouterSourceRef RotorFeedbackTest_Angle(uint8_t index)
{
	FeedbackRouterSourceRef source = {FEEDBACK_ROUTER_SOURCE_ANGLE_SENSOR,
		index};
	return source;
}

static FeedbackRouterSourceRef RotorFeedbackTest_Observer(void)
{
	FeedbackRouterSourceRef source = {
		FEEDBACK_ROUTER_SOURCE_SENSORLESS_OBSERVER, 0U};
	return source;
}

static void RotorFeedbackTest_InitConfig(RotorFeedbackRuntimeConfig *config)
{
	FeedbackRouterSourceRef none = RotorFeedbackTest_None();

	memset(config, 0, sizeof(*config));
	config->primary_encoder_index = ROTOR_FEEDBACK_PRIMARY_ENCODER_NONE;
	config->sample_period_s = 0.00005f;
	config->routing.electrical_angle = none;
	config->routing.motor_velocity = none;
	config->routing.motor_position = none;
	config->routing.output_position = none;
	config->routing.calibration_reference = none;
	config->routing.fallback_electrical_angle = none;
}

static int RotorFeedbackTest_ObserverOnly(void)
{
	RotorFeedbackRuntimeContext runtime;
	RotorFeedbackRuntimeConfig config;
	FeedbackRouterNormalizedSample observer;
	EncoderContext encoder;
	const RotorFeedbackFrame *frame;

	RotorFeedbackTest_InitConfig(&config);
	config.routing.sensorless_observer_available = true;
	config.routing.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	config.routing.electrical_angle = RotorFeedbackTest_Observer();
	config.routing.motor_velocity = RotorFeedbackTest_Observer();
	config.routing.calibration_reference = RotorFeedbackTest_Observer();
	memset(&encoder, 0, sizeof(encoder));
	TEST_CHECK(Encoder_ParamInit(&encoder, 10U, 0.0005f));
	TEST_CHECK(RotorFeedbackRuntime_Initialize(&runtime, &config, 0, 0U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	TEST_CHECK(!RotorFeedbackRuntime_HasPrimaryEncoder(&runtime));
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 14U);
	TEST_CHECK(encoder.read_error_count == 0U);
	memset(&observer, 0, sizeof(observer));
	observer.available = true;
	observer.valid_signals = FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	observer.electrical_angle_rad = 1.25f;
	observer.motor_velocity_rad_s = 2.0f;
	observer.calibration_reference_rad = 3.5f;
	TEST_CHECK(RotorFeedbackRuntime_UpdateRoutes(&runtime, &encoder,
		&observer, 14U) == ROTOR_FEEDBACK_RUNTIME_OK);
	frame = RotorFeedbackRuntime_GetFrame(&runtime);
	TEST_CHECK(frame != 0);
	TEST_NEAR(frame->electrical_angle_rad, 1.25f, 0.00001f);
	TEST_NEAR(frame->electrical_velocity_rad_s, 28.0f, 0.00001f);
	TEST_CHECK((frame->valid_signals &
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK) ==
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK);
	return 0;
}

static int RotorFeedbackTest_SinglePrimary(void)
{
	RotorFeedbackRuntimeContext runtime;
	RotorFeedbackRuntimeConfig config;
	RotorFeedbackFakePortContext fake;
	BspAngleSensorPort port = RotorFeedbackFake_CreatePort(&fake);
	EncoderContext encoder;
	FeedbackRouterSignalMask primary_caps =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE) |
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY) |
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION) |
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_CALIBRATION_REFERENCE);

	RotorFeedbackTest_InitConfig(&config);
	config.primary_encoder_index = 0U;
	config.routing.angle_sensor_count = 1U;
	config.routing.angle_sensor_capabilities[0] = primary_caps;
	config.routing.electrical_angle = RotorFeedbackTest_Angle(0U);
	config.routing.motor_velocity = RotorFeedbackTest_Angle(0U);
	config.routing.motor_position = RotorFeedbackTest_Angle(0U);
	config.routing.calibration_reference = RotorFeedbackTest_Angle(0U);
	fake.sample.single_turn_position_u32 = UINT32_C(0x40000000);
	memset(&encoder, 0, sizeof(encoder));
	TEST_CHECK(Encoder_ParamInit(&encoder, 10U, 0.0005f));
	TEST_CHECK(RotorFeedbackRuntime_Initialize(&runtime, &config, &port, 1U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 2U);
	TEST_CHECK(Encoder_IsOnline(&encoder));
	TEST_CHECK(encoder.raw_q15 == UINT16_C(0x4000));
	TEST_CHECK(RotorFeedbackRuntime_UpdateRoutes(&runtime, &encoder, 0, 2U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	TEST_CHECK((runtime.frame.valid_signals & primary_caps) == primary_caps);
	TEST_NEAR(runtime.frame.motor_position_rad, 1.5707963f, 0.0002f);
	/* A duplicate sequence follows the deployed primary-encoder tolerance. */
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 2U);
	TEST_CHECK(encoder.bad_frame_streak == 1U);
	return 0;
}

static int RotorFeedbackTest_PrimaryAndOutput(void)
{
	RotorFeedbackRuntimeContext runtime;
	RotorFeedbackRuntimeConfig config;
	RotorFeedbackFakePortContext fake[2];
	BspAngleSensorPort ports[2];
	EncoderContext encoder;
	FeedbackRouterSignalMask motor_caps =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_ELECTRICAL_ANGLE) |
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_VELOCITY) |
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_MOTOR_POSITION);
	FeedbackRouterSignalMask output_cap =
		FEEDBACK_ROUTER_SIGNAL_MASK(FEEDBACK_ROUTER_SIGNAL_OUTPUT_POSITION);

	ports[0] = RotorFeedbackFake_CreatePort(&fake[0]);
	ports[1] = RotorFeedbackFake_CreatePort(&fake[1]);
	fake[0].sample.single_turn_position_u32 = UINT32_C(0x20000000);
	fake[1].sample.single_turn_position_u32 = UINT32_C(0xF0000000);
	RotorFeedbackTest_InitConfig(&config);
	config.primary_encoder_index = 0U;
	config.use_output_position_for_position_control = true;
	config.routing.angle_sensor_count = 2U;
	config.routing.angle_sensor_capabilities[0] = motor_caps;
	config.routing.angle_sensor_capabilities[1] = output_cap;
	config.routing.electrical_angle = RotorFeedbackTest_Angle(0U);
	config.routing.motor_velocity = RotorFeedbackTest_Angle(0U);
	config.routing.motor_position = RotorFeedbackTest_Angle(0U);
	config.routing.output_position = RotorFeedbackTest_Angle(1U);
	memset(&encoder, 0, sizeof(encoder));
	TEST_CHECK(Encoder_ParamInit(&encoder, 10U, 0.0005f));
	TEST_CHECK(RotorFeedbackRuntime_Initialize(&runtime, &config, ports, 2U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 2U);
	TEST_CHECK(RotorFeedbackRuntime_UpdateRoutes(&runtime, &encoder, 0, 2U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	TEST_NEAR(runtime.frame.motor_position_rad, 0.7853982f, 0.0002f);
	TEST_NEAR(runtime.frame.output_position_rad, 5.8904862f, 0.0002f);
	TEST_NEAR(runtime.frame.control_position_rad, 5.8904862f, 0.0002f);
	TEST_CHECK(runtime.frame.uses_output_position);
	/* The secondary tracker unwraps independently across its turn boundary. */
	fake[0].sample.sequence++;
	fake[1].sample.sequence++;
	fake[1].sample.single_turn_position_u32 = UINT32_C(0x10000000);
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 2U);
	TEST_CHECK(RotorFeedbackRuntime_UpdateRoutes(&runtime, &encoder, 0, 2U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	TEST_NEAR(runtime.frame.output_position_rad, 6.6758844f, 0.0002f);

	/* Output loss is immediate and isolated: motor routes stay usable while the
	 * required output-position bit is cleared for fail-closed control gating. */
	fake[0].sample.sequence++;
	fake[1].read_result = BSP_RESULT_NOT_READY;
	RotorFeedbackRuntime_CapturePhysical(&runtime, &encoder, 2U);
	TEST_CHECK(RotorFeedbackRuntime_UpdateRoutes(&runtime, &encoder, 0, 2U) ==
		ROTOR_FEEDBACK_RUNTIME_OK);
	TEST_CHECK((runtime.frame.valid_signals & motor_caps) == motor_caps);
	TEST_CHECK((runtime.frame.valid_signals & output_cap) == 0U);
	return 0;
}

static int RotorFeedbackTest_RejectsUnqualifiedFallback(void)
{
	RotorFeedbackRuntimeContext runtime;
	RotorFeedbackRuntimeConfig config;

	RotorFeedbackTest_InitConfig(&config);
	config.routing.sensorless_observer_available = true;
	config.routing.sensorless_observer_capabilities =
		FEEDBACK_ROUTER_OBSERVER_SIGNAL_MASK;
	config.routing.electrical_angle = RotorFeedbackTest_Observer();
	config.routing.fallback_electrical_angle = RotorFeedbackTest_Observer();
	TEST_CHECK(RotorFeedbackRuntime_Initialize(&runtime, &config, 0, 0U) ==
		ROTOR_FEEDBACK_RUNTIME_FALLBACK_NOT_QUALIFIED);
	TEST_CHECK(!runtime.initialized);
	return 0;
}

int RotorFeedbackRuntime_RunHostTests(void)
{
	int result;

	TEST_CHECK(sizeof(SecondaryAngleTrackerContext) <= 40U);
	TEST_CHECK(sizeof(RotorFeedbackRuntimeContext) <= 512U);
	result = RotorFeedbackTest_ObserverOnly();
	if (result != 0)
		return result;
	result = RotorFeedbackTest_SinglePrimary();
	if (result != 0)
		return result;
	result = RotorFeedbackTest_PrimaryAndOutput();
	if (result != 0)
		return result;
	return RotorFeedbackTest_RejectsUnqualifiedFallback();
}
