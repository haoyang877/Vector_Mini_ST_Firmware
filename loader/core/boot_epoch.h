#ifndef LOADER_BOOT_EPOCH_H
#define LOADER_BOOT_EPOCH_H
#include "nv_record.h"
/** 独占双擦除页启动高水位；UID绑定设备，8B编程单元，所有Flash访问由端口提供。 */
typedef struct
{
    LoaderFlashIo io;
    LoaderFlashRegion page[2];
    uint32_t magic;
    uint16_t version;
    uint64_t header_commit, entry_commit;
    uint8_t uid[12];
} LoaderBootStore;
typedef enum
{
    YGB_OK,
    YGB_ARGUMENT,
    YGB_GEOMETRY,
    YGB_IO,
    YGB_MAINTENANCE,
    YGB_EXHAUSTED
} LoaderBootResult;
/**
 * @brief 仅对两页完整空白的首次供应写入可信高水位。
 * @param store 固定双页、UID和版本，调用方独占。
 * @param trusted_floor 来自离线供应流程的可信下界，不能取旧备份或远程请求值。
 * @return 两页提交完成返回YGB_OK；失败停止，不原位重写。
 * @note 不发布启动号；下次Allocate严格大于floor。运行时不自动调用此函数。
 */
LoaderBootResult LoaderBoot_Provision(const LoaderBootStore *store, uint64_t trusted_floor);
/**
 * @brief 分配并在两页持久验证一个新的启动号。
 * @param store UID绑定的专用双页端口。
 * @param published_epoch 仅两份持久见证均验证后写入，失败保持原值。
 * @return 成功YGB_OK；未知/损坏/耗尽拒绝，不猜测启动历史。
 * @note 每次调用表示新启动尝试；APP只能消费Loader交接值，不自行分配。
 */
LoaderBootResult LoaderBoot_Allocate(const LoaderBootStore *store, uint64_t *published_epoch);
#endif
