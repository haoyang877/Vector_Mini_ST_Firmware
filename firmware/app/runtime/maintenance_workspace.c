#include "maintenance_workspace.h"
#include "critical_hw.h"

/* 真实类型联合体保证对齐，空间只按最大成员分配一次；借用期禁止切换成员。 */
typedef union
{
    uint64_t alignment;
    EncoderCalibrationWorkspace encoder;
    InterfaceParam parameter;
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
static volatile WorkspaceOwner s_owner;

InterfaceParam *MaintenanceWorkspace_BorrowParameter(void)
{
    uint32_t mask = critical_hw_enter();
    InterfaceParam *record = NULL;
    if (s_owner == WORKSPACE_IDLE)
    {
        s_owner = WORKSPACE_PARAMETER;
        record = &s_storage.parameter;
    }
    critical_hw_exit(mask);
    return record;
}

void MaintenanceWorkspace_ReleaseParameter(InterfaceParam *record)
{
    uint32_t mask = critical_hw_enter();
    if (s_owner == WORKSPACE_PARAMETER && record == &s_storage.parameter)
    {
        s_owner = WORKSPACE_IDLE;
    }
    critical_hw_exit(mask);
}

EncoderCalibrationWorkspace *MaintenanceWorkspace_BorrowEncoder(void)
{
    uint32_t mask = critical_hw_enter();
    EncoderCalibrationWorkspace *workspace = NULL;
    if (s_owner == WORKSPACE_IDLE || s_owner == WORKSPACE_ENCODER)
    {
        s_owner = WORKSPACE_ENCODER;
        workspace = &s_storage.encoder;
    }
    critical_hw_exit(mask);
    return workspace;
}

void MaintenanceWorkspace_ReleaseEncoder(void)
{
    uint32_t mask = critical_hw_enter();
    if (s_owner == WORKSPACE_ENCODER)
    {
        s_owner = WORKSPACE_IDLE;
    }
    critical_hw_exit(mask);
}

bool MaintenanceWorkspace_IsReleased(void)
{
    uint32_t mask = critical_hw_enter();
    bool released = s_owner == WORKSPACE_IDLE;
    critical_hw_exit(mask);
    return released;
}
