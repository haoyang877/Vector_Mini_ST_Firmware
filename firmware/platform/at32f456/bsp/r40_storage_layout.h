#ifndef R40_STORAGE_LAYOUT_H
#define R40_STORAGE_LAYOUT_H

/* AT32F456CCU7 256KiB主Flash，末16KiB仅供参数。与旧板物理地址隔离。
 * 当前RAW记录2288字节，擦除只覆盖前两个2KiB扇区，后续扩展区保持原样。
 * 完整APP身份/Loader布局仍由后续目标打包定义，不在这里伪造产品描述符。 */
#define R40_FLASH_BASE 0x08000000U
#define R40_FLASH_BYTES 0x00040000U
#define R40_FLASH_SECTOR_BYTES 0x800U
#define R40_PARAM_BASE 0x0803C000U
#define R40_PARAM_BYTES 0x4000U
#define R40_PARAM_RAW_BYTES 2288U

#if R40_PARAM_BASE + R40_PARAM_BYTES != R40_FLASH_BASE + R40_FLASH_BYTES
#error "R40 parameter region must end at the physical flash limit"
#endif
#if R40_PARAM_BASE % R40_FLASH_SECTOR_BYTES != 0U
#error "R40 parameter region must be sector aligned"
#endif

#endif
