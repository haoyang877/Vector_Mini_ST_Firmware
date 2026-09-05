#ifndef BOOTLOADER_IMAGE_CONTRACT_H
#define BOOTLOADER_IMAGE_CONTRACT_H

#include <stdint.h>

#define BOOT_IMAGE_MANIFEST_MAGIC 0x56494D47UL
#define BOOT_IMAGE_CONTRACT_VERSION 1U
#define BOOT_REQUEST_MAILBOX_MAGIC 0x42525154UL

typedef enum
{
	BOOT_REQUEST_NONE = 0,
	BOOT_REQUEST_INSTALL_CANDIDATE = 1,
	BOOT_REQUEST_ENTER_RECOVERY = 2,
	BOOT_REQUEST_CONFIRM_RUNNING_IMAGE = 3
} BootRequestType;

typedef struct
{
	uint32_t magic;
	uint16_t contract_version;
	uint16_t header_size;
	uint32_t product_id;
	uint32_t compatible_hardware_mask;
	uint32_t mcu_id;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint8_t release_channel;
	uint32_t build_number;
	uint32_t image_size_bytes;
	uint32_t vector_table_address;
	uint32_t parameter_schema_min;
	uint32_t parameter_schema_max;
	uint32_t rollback_counter;
	uint32_t hash_algorithm;
	uint32_t hash_offset;
	uint32_t signature_algorithm;
	uint32_t signature_offset;
} BootImageManifest;

typedef struct
{
	uint32_t magic;
	uint32_t request;
	uint32_t candidate_address;
	uint32_t candidate_size_bytes;
	uint32_t request_crc32;
} BootRequestMailbox;

#endif
