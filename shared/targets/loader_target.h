/* 哨兵：配套APP必须将目标选择目录放在共享目录之前，禁止偶然编到别的板卡。 */
#error "Select the target-specific loader_target.h include directory before shared/targets"
