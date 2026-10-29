/**
 * @file bsp_can.h
 * @brief CAN帧收发、物理完成和控制器恢复。
 * @note 公共帧保持identifier字段及原协议数值；MCU实现持有硬件状态。
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief CAN帧逻辑表示，不直接序列化该结构。 */
    typedef struct
    {
        uint32_t identifier; /**< 标准ID最多11位，扩展ID最多29位。 */
        bool extended;       /**< 扩展ID标志；可用帧格式由硬件能力决定。 */
        bool remote;         /**< 远程帧标志；FD帧不能为远程帧。 */
        bool fd;             /**< CAN FD标志；硬件不支持时拒绝。 */
        bool bitrate_switch; /**< BRS标志，仅FD帧可用。 */
        uint8_t length;      /**< 实际字节数；FD为0..8/12/16/20/24/32/48/64。 */
        uint8_t data[64];    /**< 仅length以内有效，不保留调用方指针。 */
    } BspCanFrame;

    /** @brief 控制器状态，不代表主机在线或线路畅通。 */
    typedef enum
    {
        BSP_CAN_DISABLED = 0, /**< 未启用或未Start。 */
        BSP_CAN_READY,        /**< 已启动且无已知硬件错误。 */
        BSP_CAN_WARNING,      /**< 错误警告。 */
        BSP_CAN_BUS_OFF,      /**< bus-off。 */
        BSP_CAN_ERROR         /**< 错误被动或其它硬件故障。 */
    } BspCanState;

    /** @brief 唯一跟踪帧的终态；入队不是发送完成。 */
    typedef enum
    {
        BSP_CAN_TX_OK = 0,
        BSP_CAN_TX_TIMEOUT,
        BSP_CAN_TX_CANCELLED,
        BSP_CAN_TX_ERROR
    } BspCanTxResult;

    /** @brief 一次可消费的物理发送结果。 */
    typedef struct
    {
        uint64_t ticket;       /**< 调用方非零身份，不截断为硬件标记。 */
        BspCanTxResult result; /**< 物理完成、超时、取消或故障。 */
    } BspCanCompletion;

    /**
     * @brief 启动已配置CAN并在原阶段开放应用接收。
     * @return 成功为BSP_OK；未初始化或已启动为NOT_READY；未提供为NOT_SUPPORTED；
     *   硬件失败为IO_ERROR或既有致命路径。
     * @note 前台一次，接收对象已就绪；不更改原过滤器、位时序或业务队列。
     */
    BspResult BspCan_Start(void);

    /**
     * @brief 一次非阻塞提交符合当前目标格式的CAN帧。
     * @param frame 调用期间稳定的完整帧，不保存指针。
     * @return 接受为BSP_OK；参数/格式非法为INVALID_ARGUMENT；槽或跟踪占用为BUSY；
     *   未启动为NOT_READY；未提供为NOT_SUPPORTED；硬件失败为IO_ERROR。
     * @note 前台发送所有者调用；共享层不包围全局中断锁，MCU层隔离自身硬件访问。
     * @note 不排业务队列、不补零或截断、不重试；AT32硬件访问只屏蔽CAN优先级，允许快环抢占。
     * @note OK不等于物理完成；
     *   BSP验证ID、长度及FD/远程/BRS合法组合，硬件不支持的格式返回NOT_SUPPORTED。
     * @note 当前扩展FD+BRS协议限制由上层及目标配置保持；首轮迁移不增加新帧型的实际发送。
     */
    BspResult BspCan_TrySend(const BspCanFrame *frame);

    /**
     * @brief 预约唯一跟踪槽并提交需要物理完成确认的帧。
     * @param frame 与TrySend相同的完整帧，不保存指针。
     * @param ticket 非零64位身份；旧终态及取消隔离结束前不得复用。
     * @return 接受为BSP_OK；参数非法为INVALID_ARGUMENT；跟踪/硬件占用为BUSY；
     *   未启动为NOT_READY；未提供为NOT_SUPPORTED；硬件失败为IO_ERROR。
     * @note 仅前台，接受才产生一次终态；拒绝不产生票据，不覆盖旧票据。
     * @note 保留100ms期限及提交隔离；持续TakeCompletion，迟到完成不得匹配新票据。
     */
    BspResult BspCan_TrySendTracked(const BspCanFrame *frame, uint64_t ticket);

    /**
     * @brief 非阻塞取出一帧完整接收报文。
     * @param frame 仅成功时整体更新的输出，不保存指针。
     * @return 取得为BSP_OK；无帧为NOT_READY；空指针为INVALID_ARGUMENT；未提供为NOT_SUPPORTED；
     *   接收溢出为OVERFLOW；已取出的损坏帧为FRAME_ERROR；硬件访问失败为IO_ERROR。
     * @note 按目标选择唯一ISR或前台接收所有者，有界消费；二者不得并发读取同一接收源。
     * @note 读取会消费帧；不解析协议、不自动应答、不改变过滤配置，失败输出不变。
     * @note 溢出在BSP锁存，下一次有效Receive先报告一次OVERFLOW且不弹出幸存帧；
     *   清除的是已报告事件，之后的新溢出仍须锁存，多次尚未消费的溢出可合并一次。
     * @note 不清空幸存硬件帧或应用队列；仅被硬件覆盖的帧无法恢复，不保证溢出后的业务连续性。
     * @note 损坏帧只丢弃本帧；目标过滤规则排除的帧不冒充硬件错误。
     *   ISR的每次尝试均计入原预算，错误后有幸存帧仍需后续消费，见EXECUTION.md。
     */
    BspResult BspCan_Receive(BspCanFrame *frame);

    /**
     * @brief 推进跟踪完成检查并消费一次终态。
     * @param completion 只在取得记录时整体更新，不保存指针。
     * @return 取得记录为BSP_OK，发送成败看result；无终态为NOT_READY；空指针为INVALID_ARGUMENT；
     *   未提供为NOT_SUPPORTED。
     * @note 前台发送所有者持续有界调用；按确切硬件完成位检查，不能用空队列代替成功。
     * @note 超时或故障时可请求取消；终态消费后仍隔离未释放槽，不自动复位或业务重发。
     */
    BspResult BspCan_TakeCompletion(BspCanCompletion *completion);

    /**
     * @brief 只读查询CAN硬件状态。
     * @return 未启用/未启动为DISABLED；其它优先级为BUS_OFF、ERROR、WARNING、READY。
     * @note 前台有界只读，不恢复硬件或清跟踪结果，不证明远端在线。
     */
    BspCanState BspCan_GetState(void);

    /**
     * @brief 对当前bus-off发出一次原有恢复请求。
     * @return 请求执行成功为BSP_OK；无bus-off/未启动/未到限频间隔为NOT_READY；
     *   未提供为NOT_SUPPORTED；失败为IO_ERROR。
     * @note 仅前台，保留每秒限频；不改位时序、过滤、业务目标或自动重发。
     * @note 保留活动票据失败及取消隔离；OK仅表示恢复请求完成，不保证总线已恢复。
     */
    BspResult BspCan_Recover(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
