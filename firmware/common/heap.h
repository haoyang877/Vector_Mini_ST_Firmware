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

#include "main.h"

/* 仅 Flash 参数读写使用一份记录（小于 3 KiB）；标定使用独立固定的 8 KiB 工作区。
 * 容量含分配器头和对齐，增加堆用户时同步更新 test_outer_loop_runtime.py 回归。 */
#define TOTAL_HEAP_SIZE ((size_t) (1024 * 4))

void  *HEAP_malloc(size_t xWantedSize);
void   HEAP_free(void *pv);
size_t HEAP_get_free_size(void);
size_t HEAP_get_minimumEver_free_size(void);

#endif
