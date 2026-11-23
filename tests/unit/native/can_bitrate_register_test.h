#ifndef CAN_BITRATE_REGISTER_TEST_H
#define CAN_BITRATE_REGISTER_TEST_H
#include <assert.h>
#include <stdlib.h>

/* 厂商 SDK 采用 SEG1-2、SEG2-1、SJW-1 编码；与独立黄金寄存器值比较。 */
static void check_can_timing(const can_bittime_type *actual, char **argv)
{
    uint32_t actime = (actual->ac_bts1_size - 2U) | ((actual->ac_bts2_size - 1U) << 16U) |
                      ((actual->ac_rsaw_size - 1U) << 24U);
    uint32_t fdtime = (actual->fd_bts1_size - 2U) | ((actual->fd_bts2_size - 1U) << 16U) |
                      ((actual->fd_rsaw_size - 1U) << 24U);
    uint32_t lbtcfg = (actual->bittime_div - 1U) | ((uint32_t)actual->fd_ssp_offset << 8U);
    assert(actime == strtoul(argv[4], NULL, 0));
    assert(fdtime == strtoul(argv[5], NULL, 0));
    assert(lbtcfg == strtoul(argv[6], NULL, 0));
    assert(180000U == strtoul(argv[2], NULL, 10) * actual->bittime_div *
                          (actual->ac_bts1_size + actual->ac_bts2_size));
    assert(180000U == strtoul(argv[3], NULL, 10) * actual->bittime_div *
                          (actual->fd_bts1_size + actual->fd_bts2_size));
}
#endif
