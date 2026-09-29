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

#include "heap.h"

#include <stdint.h>
#include <stdlib.h>

/* 应用内存池实现：固定容量、纯软件，不依赖硬件头文件。 */

#define HEAP_ALIGNMENT_BYTES 8
#define HEAP_ALIGNMENT_MASK (0x0007)

/* Block sizes must not get too small. */
#define HEAP_MINIMUM_BLOCK_BYTES ((size_t)(s_block_header_bytes << 1))

/* Assumes 8bit bytes! */
#define HEAP_BITS_PER_BYTE ((size_t)8)

static uint8_t s_workspace_storage[TOTAL_HEAP_SIZE];

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct HeapBlockTag
{
    struct HeapBlockTag *next_free_block; /*<< The next free block in the list. */
    size_t block_size_bytes;              /*<< The size of the free block. */
} HeapBlock;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void insert_block_into_free_list(HeapBlock *block_to_insert);

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void initialize_heap(void);

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const size_t s_block_header_bytes =
    (sizeof(HeapBlock) + ((size_t)(HEAP_ALIGNMENT_BYTES - 1))) & ~((size_t)HEAP_ALIGNMENT_MASK);

/* Create a couple of list links to mark the start and end of the list. */
static HeapBlock s_free_list_start, *s_free_list_end = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t s_free_bytes_remaining = 0U;
static size_t s_minimum_free_bytes = 0U;

/* Gets set to the top bit of an size_t type.  When this bit in the block_size_bytes
member of an HeapBlock structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static size_t s_allocated_block_bit = 0;

/*-----------------------------------------------------------*/

/**
    * @brief  Allocate memory from internal heap
    * @param  requested_bytes: requested allocation size in bytes
    * @retval allocated memory pointer
 **/
void *Heap_Allocate(size_t requested_bytes)
{
    HeapBlock *block, *previous_block, *new_block;
    void *result = NULL;

    /* If this is the first call to malloc then the heap will require
    initialisation to setup the list of free blocks. */
    if (s_free_list_end == NULL)
    {
        initialize_heap();
    }

    /* Check the requested block size is not so large that the top bit is
    set.  The top bit of the block size member of the HeapBlock structure
    is used to determine who owns the block - the application or the
    kernel, so it must be free. */
    if ((requested_bytes & s_allocated_block_bit) == 0)
    {
        /* The wanted size is increased so it can contain a HeapBlock
        structure in addition to the requested amount of bytes. */
        if (requested_bytes > 0)
        {
            requested_bytes += s_block_header_bytes;

            /* Ensure that blocks are always aligned to the required number
            of bytes. */
            if ((requested_bytes & HEAP_ALIGNMENT_MASK) != 0x00)
            {
                /* Byte alignment required. */
                requested_bytes += (HEAP_ALIGNMENT_BYTES - (requested_bytes & HEAP_ALIGNMENT_MASK));
            }
        }

        if ((requested_bytes > 0) && (requested_bytes <= s_free_bytes_remaining))
        {
            /* Traverse the list from the start    (lowest address) block until
            one    of adequate size is found. */
            previous_block = &s_free_list_start;
            block = s_free_list_start.next_free_block;
            while ((block->block_size_bytes < requested_bytes) && (block->next_free_block != NULL))
            {
                previous_block = block;
                block = block->next_free_block;
            }

            /* If the end marker was reached then a block of adequate size
            was    not found. */
            if (block != s_free_list_end)
            {
                /* Return the memory space pointed to - jumping over the
                HeapBlock structure at its start. */
                result =
                    (void *)(((uint8_t *)previous_block->next_free_block) + s_block_header_bytes);

                /* This block is being returned for use so must be taken out
                of the list of free blocks. */
                previous_block->next_free_block = block->next_free_block;

                /* If the block is larger than required it can be split into
                two. */
                if ((block->block_size_bytes - requested_bytes) > HEAP_MINIMUM_BLOCK_BYTES)
                {
                    /* This block is to be split into two.  Create a new
                    block following the number of bytes requested. The void
                    cast is used to prevent byte alignment warnings from the
                    compiler. */
                    new_block = (void *)(((uint8_t *)block) + requested_bytes);

                    /* Calculate the sizes of two blocks split from the
                    single block. */
                    new_block->block_size_bytes = block->block_size_bytes - requested_bytes;
                    block->block_size_bytes = requested_bytes;

                    /* Insert the new block into the list of free blocks. */
                    insert_block_into_free_list(new_block);
                }

                s_free_bytes_remaining -= block->block_size_bytes;

                if (s_free_bytes_remaining < s_minimum_free_bytes)
                {
                    s_minimum_free_bytes = s_free_bytes_remaining;
                }

                /* The block is being returned - it is allocated and owned
                by the application and has no "next" block. */
                block->block_size_bytes |= s_allocated_block_bit;
                block->next_free_block = NULL;
            }
        }
    }

    return result;
}
/*-----------------------------------------------------------*/

/**
    * @brief  Free memory back to internal heap
    * @param  *pv: allocated memory pointer
 **/
void Heap_Free(void *pv)
{
    uint8_t *puc = (uint8_t *)pv;
    HeapBlock *link;

    if (pv != NULL)
    {
        /* The memory being freed will have an HeapBlock structure immediately
        before it. */
        puc -= s_block_header_bytes;

        /* This casting is to keep the compiler from issuing warnings. */
        link = (void *)puc;

        if ((link->block_size_bytes & s_allocated_block_bit) != 0)
        {
            if (link->next_free_block == NULL)
            {
                /* The block is being returned to the heap - it is no longer
                allocated. */
                link->block_size_bytes &= ~s_allocated_block_bit;

                /* Add this block to the list of free blocks. */
                s_free_bytes_remaining += link->block_size_bytes;
                insert_block_into_free_list(((HeapBlock *)link));
            }
        }
    }
}
/*-----------------------------------------------------------*/

/**
    * @brief  Get current free heap size
    * @retval free heap size in bytes
 **/
size_t Heap_GetFreeBytes(void)
{
    return s_free_bytes_remaining;
}
/*-----------------------------------------------------------*/

/**
    * @brief  Get minimum ever free heap size
    * @retval minimum ever free heap size in bytes
 **/
size_t Heap_GetMinimumFreeBytes(void)
{
    return s_minimum_free_bytes;
}
/*-----------------------------------------------------------*/

/**
    * @brief  Initialize internal heap structures
 **/
static void initialize_heap(void)
{
    HeapBlock *first_free_block;
    uint8_t *aligned_storage;
    size_t address;
    size_t total_heap_bytes = TOTAL_HEAP_SIZE;

    /* Ensure the heap starts on a correctly aligned boundary. */
    address = (size_t)s_workspace_storage;

    if ((address & HEAP_ALIGNMENT_MASK) != 0)
    {
        address += (HEAP_ALIGNMENT_BYTES - 1);
        address &= ~((size_t)HEAP_ALIGNMENT_MASK);
        total_heap_bytes -= address - (size_t)s_workspace_storage;
    }

    aligned_storage = (uint8_t *)address;

    /* s_free_list_start is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    s_free_list_start.next_free_block = (void *)aligned_storage;
    s_free_list_start.block_size_bytes = (size_t)0;

    /* s_free_list_end is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
    address = ((size_t)aligned_storage) + total_heap_bytes;
    address -= s_block_header_bytes;
    address &= ~((size_t)HEAP_ALIGNMENT_MASK);
    s_free_list_end = (void *)address;
    s_free_list_end->block_size_bytes = 0;
    s_free_list_end->next_free_block = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by s_free_list_end. */
    first_free_block = (void *)aligned_storage;
    first_free_block->block_size_bytes = address - (size_t)first_free_block;
    first_free_block->next_free_block = s_free_list_end;

    /* Only one block exists - and it covers the entire usable heap space. */
    s_minimum_free_bytes = first_free_block->block_size_bytes;
    s_free_bytes_remaining = first_free_block->block_size_bytes;

    /* Work out the position of the top bit in a size_t variable. */
    s_allocated_block_bit = ((size_t)1) << ((sizeof(size_t) * HEAP_BITS_PER_BYTE) - 1);
}
/*-----------------------------------------------------------*/

/**
    * @brief  Insert a free block into heap free list
    * @param  *block_to_insert: free block pointer
 **/
static void insert_block_into_free_list(HeapBlock *block_to_insert)
{
    HeapBlock *iterator;
    uint8_t *puc;

    /* Iterate through the list until a block is found that has a higher address
    than the block being inserted. */
    for (iterator = &s_free_list_start; iterator->next_free_block < block_to_insert;
         iterator = iterator->next_free_block)
    {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
    make a contiguous block of memory? */
    puc = (uint8_t *)iterator;
    if ((puc + iterator->block_size_bytes) == (uint8_t *)block_to_insert)
    {
        iterator->block_size_bytes += block_to_insert->block_size_bytes;
        block_to_insert = iterator;
    }

    /* Do the block being inserted, and the block it is being inserted before
    make a contiguous block of memory? */
    puc = (uint8_t *)block_to_insert;
    if ((puc + block_to_insert->block_size_bytes) == (uint8_t *)iterator->next_free_block)
    {
        if (iterator->next_free_block != s_free_list_end)
        {
            /* Form one big block from the two blocks. */
            block_to_insert->block_size_bytes += iterator->next_free_block->block_size_bytes;
            block_to_insert->next_free_block = iterator->next_free_block->next_free_block;
        }
        else
        {
            block_to_insert->next_free_block = s_free_list_end;
        }
    }
    else
    {
        block_to_insert->next_free_block = iterator->next_free_block;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
    before and the block after, then it's next_free_block pointer will have
    already been set, and should not be set here as that would make it point
    to itself. */
    if (iterator != block_to_insert)
    {
        iterator->next_free_block = block_to_insert;
    }
}
