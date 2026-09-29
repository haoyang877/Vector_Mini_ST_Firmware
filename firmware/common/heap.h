/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HEAP_H
#define HEAP_H

#include <stddef.h>

/* 应用内存池：固定容量、纯软件实现，不依赖硬件头文件。 */

/* 三类借用互斥；最大编码器工作区6KiB，另留128B覆盖块头、尾标记和对齐。
 * 各实际调用方编译期检查payload大小，不能随记录增长静默挤占元数据余量。 */
#define HEAP_WORKSPACE_CAPACITY_BYTES ((size_t)(1024U * 6U))
#define TOTAL_HEAP_SIZE (HEAP_WORKSPACE_CAPACITY_BYTES + (size_t)128U)

/**
 * @brief  从固定容量内存池分配一块内存。
 * @param  requested_bytes 请求字节数。
 * @return 成功返回对齐后的块指针；容量不足或参数为 0 返回 NULL。
 * @note 不阻塞；分配器状态为单所有者，禁止中断与前台并发调用。
 */
void *Heap_Allocate(size_t requested_bytes);

/**
 * @brief  释放由 Heap_Allocate 返回的内存块。
 * @param  pv 待释放指针；NULL 调用无效果。
 * @note 不阻塞；重复释放或释放非本池指针属未定义行为。
 */
void Heap_Free(void *pv);

/**
 * @brief  查询内存池当前剩余可用字节数。
 * @return 当前空闲字节数（含块头开销）。
 * @note 诊断用途；结果随每次分配/释放变化。
 */
size_t Heap_GetFreeBytes(void);

/**
 * @brief  查询内存池历史最小空闲字节数。
 * @return 自启动以来的最小空闲字节数。
 * @note 用于容量回归验证（见 test_outer_loop_runtime.py）。
 */
size_t Heap_GetMinimumFreeBytes(void);

#endif
