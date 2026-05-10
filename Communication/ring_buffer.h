#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "main.h"
#include <string.h>

#define BUFFER_SIZE		256

typedef struct
{
	uint16_t readindex;
	uint16_t writeindex;
}Cyclic_TypeDef;

void Cyclic_AddReadIndex(uint16_t length);
uint8_t Cyclic_Read(uint16_t index);
uint16_t Cyclic_GetLength(void);
uint16_t Cyclic_GetRemain(void);
uint16_t Cyclic_Write(uint8_t *data, uint16_t length);

#endif