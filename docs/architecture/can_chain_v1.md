# 双板CAN链路与职责 v1

更新：2026-10-26。公开契约见[bsp_can.h](../../firmware/platform/bsp/include/bsp_can.h)。

公共bsp_can.c拥有64位票据、100ms截止、一次性终态、取消隔离和1秒恢复节流。
各板can.c拥有控制器配置、原生帧转换、收发、IRQ及恢复。
can_frame_codec.c只校验元数据和换算DLC。协议CRC、地址、队列和业务属于communication。
BSP通过BspApp_OnCanReceive通知应用，不依赖具体协议处理器。

## 生命周期和结果

Bsp_Init配置控制器；应用绑定端点后启动收发。公共代码确认启动后才开放接收事件。
AT32保留PB8/PB9、原位时序、SSP、ISO FD、过滤器和4096拍配置读回。
STM32保留原CubeMX/HAL初始化顺序。IRQ确认硬件事件后有界通知，协议接收预算保持。

TrySend/TrySendTracked/Receive/TakeTxCompletion/GetState/Recover统一返回BspResult，区分忙、未就绪、坏帧和硬件错误。
失败读取保持输出。元数据支持合法标准/扩展、经典/FD组合，拒绝非法RTR/FD/BRS组合。
板上过滤器和业务协议仍保持原EXT FD+BRS范围。
终态区分成功、超时、取消、错误；已确认成功不被后续截止或bus-off覆盖。
两板捕获接收溢出，先报告再消费幸存帧，不在IRQ无限排空。
Recover受公共1秒节流，应用保留原调度。禁用构建返回NOT_SUPPORTED，不访问外设。

## 验证

原生测试覆盖帧组合、DLC、票据、截止、迟到完成、成功后bus-off、溢出和恢复；协议/参数使用原黄金向量。
AT32实测仅身份、健康、参数读取，检查坏CRC丢弃、有效请求恢复、连续收发和参数保持，不发送控制/写参数。

## 快环抢占约束

TrySend由唯一前台发送者串行调用，公共层拥有票据，但不以全局中断锁包围组帧、
复制及硬件写入。后端负责实际控制器访问的互斥。AT32使用BASEPRI=优先级2隔离
CAN错误/接收中断，ADC优先级0与载波优先级1保持可抢占；已有更严格屏蔽原样恢复。
STM32保留HAL硬件调用的原短临界区，头部准备和接收解码在其外执行。
接收仍只有目标选定的一位ISR/前台所有者；报文校验、溢出、100ms票据期限、
完成身份及bus-off恢复语义不变。
