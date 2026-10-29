#include "maintenance_workspace.h"
#include "bsp_system.h"

/* 真实类型联合体保证对齐，空间只按最大成员分配一次；借用期禁止切换成员。 */
typedef union
{
    uint64_t alignment;
    EncoderCalibrationWorkspace encoder;
    InterfaceParam parameter;
    uint8_t legacy_parameter[PARAM_RECORD_READ_BYTES];
} MaintenanceWorkspaceStorage;

typedef char MaintenanceWorkspace_FitsBudget
    [(sizeof(MaintenanceWorkspaceStorage) <= MAINTENANCE_WORKSPACE_CAPACITY_BYTES) ? 1 : -1];

typedef enum
{
    WORKSPACE_IDLE,
    WORKSPACE_PARAMETER,
    WORKSPACE_ENCODER
} WorkspaceOwner;

static MaintenanceWorkspaceStorage s_storage;
static volatile uint32_t s_owner;

InterfaceParam *MaintenanceWorkspace_BorrowParameter(void)
{
    return BspSystem_CompareExchange32(&s_owner, WORKSPACE_IDLE, WORKSPACE_PARAMETER)
               ? &s_storage.parameter
               : NULL;
}

void MaintenanceWorkspace_ReleaseParameter(InterfaceParam *record)
{
    if (record == &s_storage.parameter)
    {
        (void)BspSystem_CompareExchange32(&s_owner, WORKSPACE_PARAMETER, WORKSPACE_IDLE);
    }
}

EncoderCalibrationWorkspace *MaintenanceWorkspace_BorrowEncoder(void)
{
    bool owned = s_owner == WORKSPACE_ENCODER ||
                 BspSystem_CompareExchange32(&s_owner, WORKSPACE_IDLE, WORKSPACE_ENCODER);
    return owned ? &s_storage.encoder : NULL;
}

void MaintenanceWorkspace_ReleaseEncoder(void)
{
    (void)BspSystem_CompareExchange32(&s_owner, WORKSPACE_ENCODER, WORKSPACE_IDLE);
}

bool MaintenanceWorkspace_IsReleased(void)
{
    return s_owner == WORKSPACE_IDLE;
}
