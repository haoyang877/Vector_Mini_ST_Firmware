#include "parameter_transaction_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

typedef struct
{
	ParameterTransactionOperation operation;
	unsigned int begin_count;
	unsigned int restore_count;
	unsigned int save_count;
	unsigned int end_count;
	unsigned int complete_count;
	unsigned int fail_count;
	bool save_succeeds;
} FakeTransactionContext;

static ParameterTransactionOperation Fake_GetOperation(void *context)
{
	return ((FakeTransactionContext *)context)->operation;
}
static bool Fake_Begin(void *context)
{
	((FakeTransactionContext *)context)->begin_count++;
	return true;
}
static bool Fake_Restore(void *context)
{
	((FakeTransactionContext *)context)->restore_count++;
	return true;
}
static bool Fake_Save(void *context)
{
	FakeTransactionContext *fake = (FakeTransactionContext *)context;
	fake->save_count++;
	return fake->save_succeeds;
}
static void Fake_End(void *context) { ((FakeTransactionContext *)context)->end_count++; }
static void Fake_Complete(void *context) { ((FakeTransactionContext *)context)->complete_count++; }
static void Fake_Fail(void *context) { ((FakeTransactionContext *)context)->fail_count++; }

int ParameterTransactionService_RunHostTests(void)
{
	FakeTransactionContext fake = { PARAMETER_TRANSACTION_SAVE, 0U, 0U, 0U,
		0U, 0U, 0U, true };
	ParameterTransactionPort port;
	ParameterTransactionServiceContext service;

	port.context = &fake;
	port.get_operation = Fake_GetOperation;
	port.begin = Fake_Begin;
	port.restore_defaults = Fake_Restore;
	port.save = Fake_Save;
	port.end = Fake_End;
	port.complete = Fake_Complete;
	port.fail = Fake_Fail;
	TEST_CHECK(ParameterTransactionService_Initialize(&service, &port));

	ParameterTransactionService_RunBackground(&service);
	ParameterTransactionService_RunBackground(&service);
	TEST_CHECK(fake.begin_count == 1U);
	TEST_CHECK(fake.save_count == 1U);
	TEST_CHECK(fake.complete_count == 1U);

	fake.operation = PARAMETER_TRANSACTION_NONE;
	ParameterTransactionService_RunBackground(&service);
	fake.operation = PARAMETER_TRANSACTION_RESTORE_DEFAULTS;
	fake.save_succeeds = false;
	ParameterTransactionService_RunBackground(&service);
	TEST_CHECK(fake.restore_count == 1U);
	TEST_CHECK(fake.fail_count == 1U);
	TEST_CHECK(fake.end_count == 2U);
	return 0;
}
