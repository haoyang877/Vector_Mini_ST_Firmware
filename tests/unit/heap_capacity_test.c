#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_param.h"
#include "heap.h"

/* 验证实际维护借用大小、写满边界和释放后的最大连续容量，不访问硬件。 */
static int exercise_borrow(size_t bytes)
{
    uint8_t *buffer = HEAP_malloc(bytes);
    if (buffer == NULL)
    {
        return 1;
    }
    memset(buffer, 0xA5, bytes);
    for (size_t index = 0U; index < bytes; ++index)
    {
        if (buffer[index] != 0xA5U)
        {
            return 2;
        }
    }
    HEAP_free(buffer);
    return 0;
}

/**
 * @brief 验证三类互斥借用及碎片合并后的最大编码器工作区。
 * @return 全部符合容量要求返回0；否则返回失败编号。
 */
int main(void)
{
    const size_t sizes[] = {
        1024U * (sizeof(int32_t) + sizeof(uint16_t)),
        sizeof(InterfaceParam_TypeDef),
        sizeof(CoggingMapRecord),
    };
    for (unsigned pass = 0U; pass < 4U; ++pass)
    {
        for (unsigned index = 0U; index < sizeof(sizes) / sizeof(sizes[0]); ++index)
        {
            if (sizes[index] > HEAP_WORKSPACE_CAPACITY_BYTES || exercise_borrow(sizes[index]))
            {
                fprintf(stderr, "borrow failed: size=%zu pass=%u\n", sizes[index], pass);
                return 1;
            }
        }
    }
    size_t free_bytes = HEAP_get_free_size();
    void *first = HEAP_malloc(512U), *middle = HEAP_malloc(1024U), *last = HEAP_malloc(512U);
    if (first == NULL || middle == NULL || last == NULL)
    {
        return 2;
    }
    HEAP_free(middle);
    HEAP_free(first);
    HEAP_free(last);
    if (HEAP_get_free_size() != free_bytes || exercise_borrow(sizes[0]) ||
        HEAP_malloc(TOTAL_HEAP_SIZE) != NULL)
    {
        return 3;
    }
    printf("heap capacity OK: pool=%zu encoder=%zu param=%zu cogging=%zu free=%zu\n",
           TOTAL_HEAP_SIZE,
           sizes[0],
           sizes[1],
           sizes[2],
           free_bytes);
    return 0;
}
