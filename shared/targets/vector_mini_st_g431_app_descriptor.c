/* 仅Loader布局APP目标编译；版本/角色来自构建输入，不把旧独立APP伪装成可升级镜像。 */
#include "image_format.h"
#include "vector_mini_st_g431.h"
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
#ifndef YG_APP_IMAGE_VERSION
#error Loader APP target requires explicit YG_APP_IMAGE_VERSION
#endif
#ifndef YG_APP_IMAGE_ROLE
#error Loader APP target requires explicit YG_APP_IMAGE_ROLE
#endif
#if YG_APP_IMAGE_ROLE != YGI_ROLE_NORMAL
#error CRC32 profile supports normal APP only
#endif
#define BE32_BYTES(v) (uint8_t)((v) >> 24), (uint8_t)((v) >> 16), (uint8_t)((v) >> 8), (uint8_t)(v)
#define BE16_BYTES(v) (uint8_t)((v) >> 8), (uint8_t)(v)
__attribute__((used, section(".app_descriptor")))
const uint8_t AppImage_Descriptor[YGI_DESCRIPTOR_BYTES] = {
    BE32_BYTES(YGI_DESCRIPTOR_MAGIC),
    BE32_BYTES(YGI_DESCRIPTOR_VERSION),
    BE32_BYTES(YGI_PROFILE),
    BE32_BYTES(TARGET_LAYOUT),
    BE32_BYTES(TARGET_MCU),
    BE32_BYTES(TARGET_PRODUCT),
    BE32_BYTES(TARGET_HARDWARE),
    BE32_BYTES(YG_APP_IMAGE_VERSION),
    BE32_BYTES(YG_APP_IMAGE_ROLE),
    BE32_BYTES(TARGET_PARAM_ABI),
    BE16_BYTES(TARGET_SCHEMA),
    BE16_BYTES(TARGET_SCHEMA),
    /* 44..47 镜像长度、48..51 整包 CRC32 由打包工具回填；构建时占位为零。 */
    BE32_BYTES(0U),
    BE32_BYTES(0U),
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    /* 三个提交双字仅由Loader/APP各写一次；下载时保持擦除态。 */
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0};
#endif
