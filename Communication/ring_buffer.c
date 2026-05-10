
/**********************************************************/
//Ring buffer is transplanted from the code offered by bilibili uploader keysking
//For more details, check the link below
//https://www.bilibili.com/video/BV1p75yzSEt9/
/**********************************************************/

#include "ring_buffer.h"

Cyclic_TypeDef Cyclic;

/*ring buffer array*/
uint8_t Cyclic_buffer[BUFFER_SIZE];

/**
	* @brief  Add read index of ring buffer
    * @param  length: index length to be added (byte)
 **/
void Cyclic_AddReadIndex(uint16_t length)
{
	Cyclic.readindex += length;
	/*avoid index out of range*/
	Cyclic.readindex %= BUFFER_SIZE;
}

/**
	* @brief  Read ring buffer
    * @param  index
    * @retval byte of this index
 **/
uint8_t Cyclic_Read(uint16_t index)
{
	return Cyclic_buffer[index % BUFFER_SIZE];
}

/**
	* @brief  Get covered length of ring buffer
    * @retval covered length (byte)
 **/
uint16_t Cyclic_GetLength(void)
{
	return (Cyclic.writeindex - Cyclic.readindex + BUFFER_SIZE) % BUFFER_SIZE;
}

/**
	* @brief  Get remained length of ring buffer
    * @retval remained length (byte)
 **/
uint16_t Cyclic_GetRemain(void)
{
	return BUFFER_SIZE - Cyclic_GetLength();
}

/**
	* @brief  Write data to ring buffer
	* @param  *data: data pointer
	* @param  length: length of data (byte)
    * @retval 0 failed, drop data
    * @retval 1-BUFFER_SIZE succeeded, return written data length
 **/
uint16_t Cyclic_Write(uint8_t *data, uint16_t length)
{
	uint16_t write_length;
	
	/*drop data*/
	if(Cyclic_GetRemain() < length)
		write_length = 0;
	else
	{
		/*length of buffer tail greater than length to be written*/
		if(Cyclic.writeindex + length < BUFFER_SIZE)
		{
			memcpy(Cyclic_buffer + Cyclic.writeindex, data, length);
			Cyclic.writeindex += length;
		}
		/*exceed remained length of buffer tail, allocate part of data to buffer head*/
		else
		{
			uint16_t first_length = BUFFER_SIZE - Cyclic.writeindex;
			memcpy(Cyclic_buffer + Cyclic.writeindex, data, first_length);
			memcpy(Cyclic_buffer, data + first_length, length - first_length);
			Cyclic.writeindex = length - first_length;
		}
		write_length = length;
	}
	
	return write_length;
}

