# Loader可用性查询接口评审提案

状态：用户已通过新增只读查询，已实现。日期：2026-10-26。

## 触发原因

修订5要求Bsp_Init在HAL和开放中断前调用BootHw_Init一次。现有应用的bind_boot_entry又调用
BootHw_Init取得s_boot_supported；该函数实际还检查门极状态，并可能设置VTOR、试运行起点。
将初始化留给BSP后，应用不能再次依靠初始化副作用来查询能力。

BootHw_IsConfirmed不能替代：false既可能是有效试运行镜像尚未确认，也可能是不支持Loader。
BootHw_ImageCrc32也不能替代：CRC为0不构成“不支持”的可靠哨兵。不能只根据编译宏伪造运行结果。

## 建议变更

保留原六个签名不变，在boot_hw.h增加一个只读查询：

```c
/**
 * @brief 查询本次启动是否已完成有效的Loader APP交接。
 * @return BootHw_Init成功建立交接后为true；未初始化、独立APP或不支持为false。
 * @note 前台有界只读，不访问SPI、不写VTOR/邮箱/Flash，不重置试运行时限，不授权运动。
 */
bool BootHw_IsAvailable(void);
```

STM32 Loader APP返回初始化时缓存的真实结果；STM32独立APP和AT32当前实现返回false。
应用bind_boot_entry用该查询替代第二次BootHw_Init。Bsp_Init仍只调用一次BootHw_Init。
不改变Loader产品、邮箱/镜像/参数ABI、确认条件、继承看门狗或试运行时限。

## 备选方案及取舍

- 把BootHw_Init改成可重复的查询/初始化混合接口：签名不变，但扩大副作用与上下文合同，
  不利于职责分开，也容易再次重置时限，不采用。
- Bsp_Init返回Loader状态结构：扩大所有板启动接口，对非Loader目标也引入额外耦合，不采用。
- 编译宏或CRC猜测能力：无法表达实际交接检查失败，不采用。

## 评审与验证范围

仅新增一个只读BSP查询和其调用替换；通过后接口包总数由56变为57。
验证未初始化、独立APP、有效试运行、已确认、交接失败，以及反复查询不改变VTOR和试运行时限。
用户已同意本项查询，随后取消后续接口逐项评审要求；今后的接口调整直接实现并同步文档、回归与硬件证据。
