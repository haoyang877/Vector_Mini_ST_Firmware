# 统一电机测试协议 v1

2026-10-31。此扩展仅在`APP_ENABLE_MOTOR_BENCH=1`的AT32 Commissioning镜像登记。
普通APP不登记0x7F00，原命令、能力字段、Flash及参数ABI保持原样。
本协议用于待执行的台架用例，不表示已测；主机默认不连接硬件。

## 外层与身份

使用原CAN FD/CRC/ACK帧，CMD为`0x7F00`，只接受主站2单播，单帧载荷最多46字节，不分片。
请求开头`operation:u16, version:u16=1`；应答开头`result:u16, operation:u16`，全部小端。
float为IEEE754 binary32，禁止用C结构体直接当线路布局。
result采用原结果码：0成功、1受理、4字段错误、5拒绝、6忙/数据未就绪、11越界、17故障。

LIMITS、RUN、CAPTURE、STOP为有副作用命令。最近同序号同载荷返回原回复，不再次执行；
同序号不同载荷及旧序号拒绝。序号按16位半范围比较，相邻有副作用请求跨度须小于32768。
重连后先INFO，取其last_sequence初始化后续序号；主机不自动重发动力请求。

## 操作

下表长度不包含请求4字节公共头。

| op | 输入长度/布局 | 行为 |
| --- | --- | --- |
| 0 INFO | 0 | 46字节应答，见下表 |
| 1 LIMITS | 20：current_a:f32、speed_rad_s:f32、travel_rad:f32、duration_ms:u32、lease_ms:u32 | 仅停止时设置；三项正有限值，电流不得超过活动配置；速度≤500rad/s，累计行程≤10000rad，时限1..120000ms，租期100..1000ms |
| 2 RUN | 16：kind:u16、reserved:u16=0、reference:f32、duration_us:u32、warmup_us:u32 | kind1为正Id、2为有符号Iq短脉冲、3为速度。均从零参考启动，预热最多100ms；Iq非零段最多5000µs，随后零电流闭环5ms再停止 |
| 3 CAPTURE | 16：group:u16、stride:u16、trigger:u16、pre:u16、delay_ms:u32、phase:u16、reserved:u16=0 | 128点定长缓冲；stride1..65535，pre0..32，delay≤120000ms。受理返回capture_id:u32 |
| 4 READ | 8：capture_id:u32、index:u16、reserved:u16=0 | 仅冻结且生命周期确认关相后返回单样本；不允许读活动缓冲或旧代际 |
| 5 KEEPALIVE | 4：run_id:u32 | 必须匹配当前运行身份；只更新时间，不启动或恢复动作 |
| 6 STOP | 0 | 取消测试/标定、冻结已触发记录并撤销配置许可；调用原生命周期停止，不绕过功率所有者 |
| 7 LUT | 4：start:u16、count:u16 | 仅停止时读RAM LUT；count1..16且不越过1024项，回复公共头＋start/count＋int16数组 |
| 8 SNAPSHOT | 0 | 公共头＋32字节最近样本＋group/run_state/fault/calibration_flags四个u16；不承诺均匀的主机接收间隔 |

RUN受理回复公共头＋run_id:u32；READ回复公共头＋capture_id:u32＋index/count:u16＋32字节样本。
32字节样本为`tick:u32 + values[7]:f32`，tick单位为20kHz快拍，自然回绕。
采集冻结后允许在关相状态重复读数据；重读不重新激励。

触发：0立即，1实际测试参考首次更新，2首次更新后delay_ms，3标定阶段等于phase。
相位触发使用现有诊断编号；1/2也可观察原0xA0标定。sin/cos要求stride≥10，读取原辅助ADC最新完成数据，
没有额外的辅助ADC时间戳，不据此声称两个模拟通道同时采样。
停止时布置的1/2触发仅绑定下一次运行，不读取上次参考改变时刻；运行中布置则绑定当前运行。

## INFO应答布局

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0/2 | u16/u16 | result/op |
| 4 | u32 | magic=0x3154424D，字节`MBT1` |
| 8/12/16 | u32 | 快拍tick、run_id、capture_id |
| 20/22 | u16 | run_state、capture_state |
| 24/26 | u16 | run_kind、reason |
| 28/30/32 | u16 | 有效记录数量、group、采集期间非有限样本计数 |
| 34/36/38/40 | u16 | 标定标志、电零位Q15、极对数、原电机故障 |
| 42 | u16 | bit0限值已配置，bit1编码器反向，bit2数字输出已关，bit3已有编码器样本，bit4已有有副作用序号；高8位为标定阶段 |
| 44 | u16 | 最近有副作用请求的序号 |

run_state：0空闲、1等待原生命周期启动、2活动、3结束、4失败。
capture_state：0空闲、1等待触发、2记录中、3冻结。
run_kind：0无、1Id、2Iq脉冲、3速度、4原LUT、5原电零位、6原机械零位。
reason：0无、1电流、2速度、3累计行程、4主机租期、5正常时限结束、6启动超时、
7状态/代际变化、8编码器、9原故障、10显式停止、11非有限测量。
正常结束与失败均不自动重新启动；独立状态判断区分STOP受理、数字关相和机械静止。

## 通道

| group | values[0..6] |
| --- | --- |
| 1 phase | raw_a/raw_b/raw_c等效12位count、Ia/Ib/Ic A、编码器电角rad |
| 2 dq | Id给定/Iq给定/Id反馈/Iq反馈A、Vd/Vq V、机械速度rad/s |
| 3 speed | 请求速度/斜坡速度/反馈速度rad/s、Iq给定/反馈A、母线V、多圈机械角rad |
| 4 sincos | sin/cos原始count、机械速度rad/s、编码器线性化Q15、母线V、两个保留0 |
| 5 angle | 原始/方向修正/线性化Q15、观测器机械位置rad、编码器电角rad、Id/Iq反馈A |

## 运行约束

默认限值无效，原普通启动也不能绕过测试守卫。每拍在原生命周期BeginFastTick前检查电流、速度、
累计编码器行程、非有限值、故障、代际和租期；失败或期满登记原STOP，当拍BeginFastTick关闭输出。
RUN复用原RequestEnable、FOC和速度环；原0xA0标定复用原RequestCalibration，仅在限值有效且租期新鲜时运行。
LUT成功以关相前台实际发布结果确认，不把FINALIZE或回到IDLE直接当作成功。
配置只约束测试，不写参数/Flash，不清除真实故障；新连接不继承旧动作。
RUN时限包括预热、非零段及Iq的5ms回零尾段，不得超过配置的duration_ms。

速度运行参考仍受活动配置的speed_limit_rad_s约束。主机单列test_speed_rad_s用于工况幅值，
不能把用于标定的较高速度截止值直接当成速度闭环给定。

黄金向量见[测试协议向量](unified_motor_bench_v1_vectors.json)。请求字节、固件内存行为及主机报告已有离线回归，
关相硬件验证范围见[测试记录](../reports/2026-10/unified_motor_validation_20261031.md)，不代表带电用例已经通过。
