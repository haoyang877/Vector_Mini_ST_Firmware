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

#ifndef __HEAP_H__
#define __HEAP_H__

#include <stddef.h>

/* 应用内存池：固定容量、纯软件实现，不依赖硬件头文件。 */

/* All live application allocations: encoder calibration 8192 B plus one
 * parameter record (< 5 KiB), including allocator headers/alignment. Keep the
 * capacity regression in test_outer_loop_runtime.py when adding allocations. */
#define TOTAL_HEAP_SIZE ((size_t)(1024 * 14))

/**
 * @brief  从固定容量内存池分配一块内存。
 * @param  xWantedSize 请求字节数。
 * @return 成功返回对齐后的块指针；容量不足或参数为 0 返回 NULL。
 * @note 不阻塞；分配器状态为单所有者，禁止中断与前台并发调用。
 */
void *HEAP_malloc(size_t xWantedSize);

/**
 * @brief  释放由 HEAP_malloc 返回的内存块。
 * @param  pv 待释放指针；NULL 调用无效果。
 * @note 不阻塞；重复释放或释放非本池指针属未定义行为。
 */
void HEAP_free(void *pv);

/**
 * @brief  查询内存池当前剩余可用字节数。
 * @return 当前空闲字节数（含块头开销）。
 * @note 诊断用途；结果随每次分配/释放变化。
 */
size_t HEAP_get_free_size(void);

/**
 * @brief  查询内存池历史最小空闲字节数。
 * @return 自启动以来的最小空闲字节数。
 * @note 用于容量回归验证（见 test_outer_loop_runtime.py）。
 */
size_t HEAP_get_minimumEver_free_size(void);

#endif
