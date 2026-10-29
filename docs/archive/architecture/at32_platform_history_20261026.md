# AT32平台历史实施笔记

截至2026-10-26的旧平台README存档。以下是历史过程，路径、入口和未验收状态不代表当前实现。
现行职责见[当前BSP架构](../../architecture/bsp_architecture_v1.md)。

# YG-ESC_V1.0 AT32F456 平台

板卡原名R40；现有文件名、工程名及宏暂保留R40。适配电机型号为YQ-5708，灯的使用逻辑后续适配。

ports现与旧板统一为board/boot/comm/motor/time五类目录；仅调整文件路径和工程引用，
公共接口与真实实现位置见双板对应表（历史路径：`../../../docs/architecture/platform_interface_implementations_v1.md`）。

2026-10-24采样职责已拆分：共同契约为`platform/bsp/include/bsp_sampling.h`，
采集运行、外设配置、存储协调和诊断分别归BSP模块，ports只保留原码兼容适配。
新旧板完整初始化/IRQ/温度链路与验证边界见完整采样链路（历史路径：`../../../docs/architecture/sampling_chain_v1.md`）。

2026-10-24 PWM职责也已统一：两板均由`bsp/bsp_pwm.c`拥有占空比、六路启停和状态读取，
`bsp/pwm_timer_setup.c`承接原定时器配置。目标许可与限时状态机移到共用的
`platform/bsp/`，公共契约为`platform/bsp/include/bsp_pwm.h`；旧PWM/功率端口已移除。
行为差异、保护语义与验证边界见PWM与功率输出链路（历史路径：`../../../docs/architecture/pwm_power_chain_v1.md`）。

2026-10-24 CAN职责已统一：`bsp/bsp_can.c`管理帧与发送票据，`can_controller.c`管理配置/启动/恢复，
`can_irq.c`负责硬件中断接线，帧格式校验复用公共codec。详见双板CAN链路（历史路径：`../../../docs/architecture/can_chain_v1.md`）。

新旧板的引脚、定时器、ADC/DMA、存储、功率及电流限值统一见
资源管理表（历史路径：`../../../docs/hardware/board_resource_registry_v1.md`）。
本页包含分阶段实现记录，最新实测状态以资源管理表和适配计划的日期条目为准。

2026-10-24配置整理：完整APP两个工程的C/C++ Misc Controls分别预包含
`../config/targets/r40_app_off.h`和`../config/targets/r40_app_commissioning.h`
（以上路径从本平台目录看；工程实际使用`../../config/targets/`）。
板型/电机选择、测试保护模式、速度整定、外设选项与临时热值集中在目标头，
Keil Define只保留`AT32F456CCU7,USE_STDPERIPH_DRIVER`。
旧板与这两个目标使用相同的49个APP/电机/参数/通信C文件；没有新建一份新板中间层。
限时目标仍使用已授权的3A保护、2047看门狗重载及800Hz PLL试验值，
永久关相目标仍为90Hz PLL；整理不提高限值、不补发运行命令或自动烧录。

当前底层验证目标已接入启动、时钟、关相、PWM占空比、ADC/DMA、CAN、MT6835、
电机NTC诊断及参数存储端口。电机NTC标称规格为R25=10kΩ、B=3950K，
已接入私有采样与换算；MCU内部测温继续禁用，电机温度未接入完整APP保护或遥测。
尚未接入完整APP、新电机数据或输出轴反馈。
不提供未实现BSP能力的成功占位，不将此镜像作为电机控制APP发布。

### 独立开环验证目标

`R40_F456_OpenLoop_Dry.uvprojx`和`R40_F456_OpenLoop_Power.uvprojx`提供独立目标，显式构建选择
`r40-openloop-dry`或`r40-openloop-power`，不改变默认APP或原Foundation的含义。
Dry移除启相调用；Power也不自动开始，须通过私有调试内存接口显式请求一次脉冲。
两种目标均不访问CAN或Flash参数，使用既有数学函数计算旋转电压，反馈只做故障检查。

当前固定试验为0.20V相电压幅值、2Hz电角频率、1秒总时长、首尾200ms斜坡，
相电流超过0.60A或母线越出20–30V停止。先关相等待1秒稳定，再完成2048拍零偏学习，
检查量程及各相40累加计数峰峰值；显式请求后等待连续10ms各相零流偏差不超过0.10A，
500ms仍不能满足则锁定故障，等待期间输出关闭；
ADC、编码器错误或250ms主机心跳过期均锁存停机。
计时和20000拍样本数任一到期即停止，不能再次开始，须显式复位。

`g_openloop_command`包含magic=0x4F4C3430、version=1、command/inverse、nonce/
completed_nonce/result和heartbeat/heartbeat_inverse。主机先清command，写完整请求再提交command。
命令1开始、2停止、3保持快照、4恢复发布、5停止喂WDT；命令5只在Dry且未运行时可用。
result=2表示请求接受，4表示拒绝；必须独立核对快照state/fault和实际输出。
快照为`g_openloop_snapshot`，只接受begin=end且非零偶数的版本。
WDT复位后记录复位原因并锁定故障，不自启。实际看门狗超时由LICK决定，不能视为精密定时器。
本目标是短时验收入口，不是完成热保护、运动范围管理或闭环控制的可发布APP。

## 已实现边界

- 时间与临界区沿用既有`time_hw`、`critical_hw`契约；PWM按上文迁入统一`BspPwm`契约，控制行为保持。
- 上电先将PA9/PA10/PA11、PB0/PB1/PB4置低；PB12编码器CS先置高。
- 8MHz HEXT、PLL 90/1/4，得到180MHz。VCO720MHz，APB1/2为180MHz、APB3为45MHz。
- CAN时钟为PCLK1；仲裁135+45、数据27+9，分频1，对应1M/5M。
  数据段TDC偏移28；外部收发已通过2026-10-23关相回显验收，详见后文。
- TMR1中央对齐PR=4500，对应20kHz；TMR6预分频179、PR=499，对应2kHz。
  ADC完整序列完成后可向后续APP提供快环样本；本验证目标不调用`App_FastTick`。
- 关相先关闭总输出，再关闭三相主/互补通道；保留比较值和CH1采样资源。
- 200ns死区是待测配置，当前验证镜像不启相，不能据此批准运动或10A测试。
- 时钟等待有界，失败不切换门极引脚复用；定时器配置完成且关相检查通过后才复用引脚。

`foundation/main.c`仅运行底层时基，2kHz中断计数用于基础验证，
不替代将来APP的监督任务。ADC采样、MT6835读数与CAN接收计数在此镜像运行，
不发送电机控制响应或启相。未来完整APP由独立构建目标接入同一平台实现。

## ADC与CAN实现边界

ADC1为PA0/1/2三相电流、PA3母线电压的四rank抢占序列；每rank硬件4次过采样，
不右移，最大16380，沿用公共原始计数域。ADC时钟30MHz，采样12.5周期、
12位转换12.5周期，四rank预算约13.34us，位于50us载波周期内。
相电流顺序采集而非同时采集，通道间偏移和模拟建立时间仍需实测，
不能用此预算代替实际噪声与控制稳定性验收。

当前`BSP_ENABLED_MOTOR_TEMPERATURE=1`、`BSP_ENABLED_MCU_TEMPERATURE=0`：
ADC2采PA4母线电流和PA7电机NTC，DMA1通道2使用两个双样本半缓冲。
完整App_Off现通过顶层装配将PA4实测电流用于CAN状态反馈；每次上电关相采128个
稳定新样本，独立记录零点后换算。`g_r40_bus_current_diagnostic`保留原码、偏置及资格，
未校零/过期/异常时反馈无效，不切回估算。中间层FOC估算对象保持原语义，
旧板继续使用估算；详见接入计划（历史路径：`../../../docs/plans/active/2026-10-23-pa4-bus-current-app-v1.md`）。
ADC1普通温度序列、内部温度/VINTRV使能和DMA1通道3均不启动。
两个采样开关独立；禁用电机NTC时ADC2退回只采PA4，仍保留双半缓冲。
MCU公共接口继续报告无有效样本，不用电机NTC替代芯片结温。
App_Off现在显式配置`BSP_REQUIRED_MCU_TEMPERATURE=0`：经用户授权，
缺失MCU温度不阻塞该关相调试目标的自检，温度遥测仍保持无效。
该开关必须与编译期功率锁同时使用，其他目标默认仍要求有效MCU温度；
详见温度可选策略（历史路径：`../../../docs/plans/active/2026-10-23-mcu-temperature-optional-v1.md`）。
本验证镜像仍保持关相，NTC诊断不构成完整APP的运动或热保护验收。

ADC2由2kHz TMR6触发。以下内部温度采样保留供离线回归，当前目标未启用：
内部温度/VREF只能由ADC1的IN16/IN17取得，
另用ADC1普通序列和DMA1通道3写入两组2项缓冲，同样由2kHz TMR6触发。
这是P4对P2内部通道映射的修正：ADC2_IN16/17接VSSA，不能用于内部温度/VREF。
每次只发布完成的一半；DMA丢半帧、传输错误、ADC触发失败或快环缺帧
均锁存故障并关相，不自动恢复。序列号回绕不等于未采样。
辅助通道使用640.5周期采样，每组两路预算约43.53us；ADC1普通序列可能
被相电流抢占并重做被打断通道，完成延迟须实测，不按无抢占预算证明时序。
使能内部传感器后至少等待18000个内核周期再启动，满足100us建立要求。
普通目标不采PA5/PA6；关相Dry诊断目标可启用下述输出轴原始观测。
电机NTC的R25=10kΩ、B=3950K记录于硬件输入（历史路径：`../../../docs/hardware/r40_at32f456_configuration_v1.md`），
不可填入下方MCU内部温度校准项。
`r40_analog.h`记录模拟链路数据；完整APP的目标选择和保护限值仍需后续定版。

### 输出轴sin/cos原始观测

`BSP_ENABLED_OUTPUT_SINCOS`默认0，仅`R40_F456_OpenLoop_Dry`显式设为1。
此时ADC2在原有母线电流/可选NTC之后追加PA5、PA6，不修改公共BSP函数、类型或计数域。
四路顺序采样约87.1us，仍小于2kHz触发的500us周期；PA5与PA6相隔约21.8us，
只能用于当前慢速手动信号检查，不宣称两路同步或完成输出轴角度解算。

板内私有`g_r40_sincos_snapshot`为6个32位字：begin、sequence、milliseconds、
pa5_counts、pa6_counts、end。DMA每40帧发布一次（50Hz），原始计数0..4095，
电压仅可按名义VDDA=3.3V估算。读取须满足begin=end、非零偶数，并确认时间与sequence前进。
CPU与ADC须持续运行；若ADC故障或时间不前进，旧快照不可当作有效新信号。
不根据静态非零值判定传感器有效，也不预设PA5/PA6的sin/cos方向。
先观测静态电平，再手动转动并保存两路变化与轨迹；相位、增益、零偏和角度标定另行验收。

Dry链接不含功率启动函数，不初始化CAN、不读写参数Flash。
普通Power目标与sin/cos诊断同时使能仍会编译失败。新增的显式
`r40-openloop-combined`目标另外要求`R40_OPENLOOP_SINCOS_TEST=1`，
仅用于用户授权的1秒开环与两路原始采样联合测试；不属于产品APP，不连接输出轴闭环。
三个目标默认都不启动；Dry/Power默认0.20V和1秒，Combined经用户追加授权为1.0V和10秒；
均保留2Hz、毫秒和采样步数双重时限；Dry/Power相电流跳闸默认0.60A，
Combined经用户追加授权为2.0A。其余保留
心跳与WDT保护，不改变中间层或公共BSP。每个实际试转都须明确授权；不自动恢复先前试转。
总采样步数由该目标时长乘以20kHz计算，编译时长只允许400..10000ms。
1.0V是相电压幅值而非母线设定值，2.0A是相电流幅值门限；
0.7A电源限流是母线输入电流，三者不可混用。过流保护可以在10秒之前结束试验。

下载前不仅要暂停CPU，还必须先停止TMR1/TMR6及ADC对应DMA1通道2/3。
CPU暂停不会自动停止DMA；其缓冲区与J-Link在SRAM内加载的Flash算法地址可能重叠，
本轮已观察到0x200000E0处的采样数据覆盖算法并导致RAMCode校验失败。
同一调试会话完成暂停、停采样、设置下载期WDT和加载，镜像完整读回一致后再复位运行。
运行期必须核对WDT已恢复4分频/重装511、调试不冻结，不能沿用下载期设置。

CAN只使用PTB单个硬件槽，忙时通过既有API返回false，不增加协议队列。
跟踪发送以TPIF、对应handle的TSTAT终态及硬件槽释放共同判断成功；
取消未完成前不复用，100ms超时与bus-off返回一次失败，原样保留64位票据。
错误ISR仅锁存bus-off，恢复由既有前台接口执行，保持过滤与1M/5M时序。
硬件接受所有扩展数据帧，FD/BRS在接收边界校验。

## PWM、温度和参数存储

### 电机NTC私有诊断

原理图第4页：CN5的2脚接3V3、1脚为TEMP_MOT_IN，外部NTC跨接两脚，R92为1kΩ下拉，
R91为22Ω ADC支路串阻、C119为100nF。R91不在直流分压下臂，不把它加到1kΩ中。
电机NTC和ADC参考同用3V3，采用比例换算：
`Rntc = 1000 * (4095 - raw) / raw`，
`T°C = 1 / (1/298.15 + ln(Rntc/10000)/3950) - 273.15`。
温度升高时NTC阻值下降、ADC计数上升；标称25°C约0.300V/372计数。
此处只使用12位单次ADC计数，不套用相电流的四次累加缩放。

`bsp/motor_temperature_at32f456.h`是AT32平台私有诊断头，公共BSP没有新增或修改。
`R40_MotorTemperaturePoll`只在验证工程前台消费辅助快照并执行`logf`，
不在20kHz采样或2kHz监督中断计算对数。输出独立的原始值、序列号、阻值、温度和状态，
不写入`MotorHwTemperaturePoll`、`foc.temperature_c`或既有遥测字段。
重复帧不重复计数；辅助数据超过2ms、未采到或ADC故障时立即无效，温度设NaN，
不会继续显示之前的有效温度。序列号及毫秒计时跨越UINT32_MAX仍正常处理。

0..2计数报告疑似开路，4093..4095报告疑似短路，其余在-40..150°C之外报告超出换算窗口；
这些是诊断窗口和近电源轨检查，不是电机过温阈值或器件额定范围。
电源/接线问题也可能导致同样的近轨读数；无效数据不夹紧为正常温度。
NTC故障状态不代替完整APP的故障管理，现有驱动级ADC丢帧/错误仍锁存关相。
阻值和B值公差、B值参考温区及实板误差未标定，后续需用标准电阻与参考温度验证。
电机温度进入保护/遥测属于后续温度来源架构调整，不能借此解除MCU温度准入条件。

后续实机验收（本轮未执行）：取得验证镜像加载授权、确认功率输出关闭后，
在CN5的2/1脚间接10kΩ标准电阻，观察`s_motor_temperature`：原始值约372、
阻值约10kΩ、温度约25°C，`status`应为VALID且`s_motor_temperature_samples`递增。
再使用较小电阻或给NTC缓慢升温，应看到计数和温度同时上升；
断开或以低阻替代传感器应分别出现OPEN或SHORT且温度为NaN。
误差按标准电阻实测值、ADC误差与电源比例核对，不把理想值当作实测精度保证。
同时确认MCU温度仍无有效样本、CAN配置仍为1M/5M；不发送运行命令。

### 保留的MCU测温和存储

`bsp/bsp_pwm.c`实现统一PWM与输出状态接口，A/B/C分别写TMR1的C2DT/C3DT/C4DT。
占空比按实际PR=4500截断为计数，保持0和1端点；逐相写不改变其他相，
三相写以短临界区防止软件调用交错。所有写入都不触碰CH1、计数器、通道使能或总输出。
依靠初始化启用的预装载，在定时器更新事件生效；不人工产生更新事件。
临界区不停止硬件计数，未来完整快环要验证三次写入不跨越硬件更新边界。

`bsp/bsp_temperature.c`保留原有`BspTemperature_PollMcu`实现；禁用采样时始终返回
`sample_ready=false`、`conversion_ok=false`，不覆盖原始值和温度，也不伪造健康状态。
启用该保留路径后，只消费新的ADC1普通序列快照，
重复帧、过期帧不刷新上层有效期。ADC值0/4095、供电超出2.4~3.6V、
温度超出-40~150°C或校准参数无效时返回换算失败。
原来的温度滤波、90°C保护、100ms失效窗口及故障锁存都留在未修改的中间层。
未校准时仍返回新原始样本，但`conversion_ok=false`；完整APP不能以此取得运动资格。

实板校准数据入口是`platform/config/boards/at32f456.h`的五项目标配置：

| 配置 | 含义 |
| --- | --- |
| `R40_TEMPERATURE_CALIBRATED` | 取得且核对实测数据后才设为1，默认0 |
| `R40_TEMPERATURE_CAL_C` | 第一个稳定校准温度，°C |
| `R40_TEMPERATURE_CAL_MV` | 该温度下内部VTS实测电压，mV |
| `R40_TEMPERATURE_SLOPE_MV_PER_C` | 两点实测有符号dVTS/dT，mV/°C |
| `R40_INTERNAL_REFERENCE_MV` | 本芯片VINTRV实测值，mV |

在两个已知稳定温度T1/T2记录ADC1内部温度/VREF的平均原始值和实测VDDA：
`VTS=raw_ts*VDDA/4095`，`VINTRV=raw_vref*VDDA/4095`，
`slope=(VTS2-VTS1)/(T2-T1)`。换算采用
`VDDA=VINTRV*4095/raw_vref`、`T=T1+(raw_ts*VINTRV/raw_vref-VTS1)/slope`。
这个斜率是实测电压导数，不能直接套用手册公式中另一符号定义的Avg_Slope。
未取得这些数据前不以典型V25/VINTRV或旧板地址冒充逐颗标定。

`param_store_at32f456.c`实现冻结的字节读写接口，地址由私有`r40_storage_layout.h`定义：
256KiB Flash末16KiB（0x0803C000至0x0803FFFF）从链接区排除；当前RAW记录2288字节，
最多擦写前两个2KiB扇区，剩余扩展空间不被擦除。读操作允许完整保留区范围，
写操作拒绝超容量、空指针、源缓冲与保留区重叠、运行态、ISR及未屏蔽中断的调用。
相同内容不擦写；任一步错误立即停止，回读不一致返回失败；无完成标志不能当作成功。
忙超时后保留控制器现场并禁止再次读写，等待复位，不猜测控制器已恢复。
参数schema、CRC和业务校验仍由原有参数层负责，不自动迁移或写默认参数。
该实现保持原接口“失败效果未知”的语义，不提供断电原子更新保证。
验证镜像上电只读参数首字；写接口仅由后文规定的显式关相诊断命令调用。

依据用户提供的RM v2.01第82、85、88、365页，以及
[官方数据手册v2.00](https://www.arterychip.com/download/DS/DS_AT32F455_456_457_V2.00_EN.pdf)
第27、91、92页；温度/VINTRV的典型值不是逐颗出厂校准数据。

## MT6835实现边界

新目标链接公共器件目录的`drivers/encoder/encoder_mt6835.c`，旧板链接同目录的TLE5012B协议；
两板各自的`bsp/encoder_spi.c`实现同一私有SPI契约，公共`bsp_encoder.h`统一公共接口。SPI2使用PB12软件CS、
PB13/14/15的MUX5，模式3、MSB优先、8位全双工，180MHz/32=5.625MHz。
不占用新的DMA通道。MISO保留弱上拉，CS在初始化和每帧结束时置高。

每帧发送`A0 03 00 00 00 00`，一次CS周期内返回锁存角度、状态和CRC。
`begin`只启动首字节，`begin_readback`推进两个命令字节并启动数据阶段，
`complete`取完四字节并释放CS。保留同步回退；所有等待共用从CS拉低前起算的
20us DWT预算，另有4096次轮询上限，防止周期计数停止导致死等。
该预算包含阶段间计算；未来接入完整APP时须测量其余快环工作量，
不能把48个SCK的8.53us线速时间当作整条快环执行时间。
CS建立/保持各至少36个CPU周期（180MHz下200ns），真实波形仍待测。

CRC覆盖完整21位角度与3位状态，初值0、多项式0x07、无反射/末尾异或。
任意状态位（超速、弱磁、欠压）置位映射为既有传感器故障；
CRC错误映射帧错误，SPI错误/超时映射总线超时。失败不覆盖输出角度/原始诊断字，
丢弃残帧、恢复SPI配置，下一拍可重新读；这不代表解除应用层锁存故障。
只有有效帧将21位角度右移5位，保持每圈65536计数及原有方向/零位/多圈逻辑。
`frame_word`为角度高16位，`safety_word`为剩余角度/状态字节与CRC拼接的16位值。

6个配置诊断字依次只读寄存器0x007、0x008、0x009、0x00A、0x00D、0x011，
对应ABZ分辨率、零位、方向/迟滞及带宽。读取失败保留旧字；
诊断与快环各自保有事务所有权，互不截断对方事务。不写零位、EEPROM或校准命令。
初始化只核对总线与周期时基，不将只读配置成功当作传感器健康证明；
健康角度需要完整帧的CRC与状态验证。底层验证目标在ADC完整帧中断中采样，
仅保存最后有效角和成功/失败计数，不执行控制或启相。

协议参考用户提供的MT6835 Rev.1.3第20–26、32页；CRC初值与帧构造交叉检查了
[SimpleFOC](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6835/MT6835.cpp)、
[VESC](https://github.com/vedderb/bldc/blob/master/encoder/enc_mt6835.c)和
[Hotakus](https://github.com/Hotakus/mt6835/blob/master/src/mt6835.c)。
本端口根据既有接口独立实现，未导入这些框架或其写寄存器功能。

## 只构建与离线检查

从仓库根目录运行，使用本机实际的Keil路径：

```text
python tools/run.py build_firmware --uv4 <UV4.exe路径> --target r40-foundation
python tools/run.py verify --profile pr
```

显式`r40-foundation`只构建、不下载，产物在`outputs/build/at32_foundation/`。
旧`normal`及`all`的含义不变，默认发布流程不会收录本验证镜像。
工程关闭构建前后命令和调试前自动下载；构建命令本身不连接调试器。

离线测试编译真实端口，用寄存器替身覆盖时钟源/PLL/切换/频率失败、SysTick失败、
引脚复用前关相、通道映射、启停保留比较值、单个输出残留、临界区嵌套及时基。
这些结果不证明实物门极波形、ADC时序或温升。

## 后续顺序与验收

### 关相外设验收入口

验证工程的`r40_bench.c`提供独立调试内存接口，未接入完整APP或产品协议。
`g_r40_snapshot`只在`begin=end`、非零且为偶数时可作为一致快照读取，包含时基计数、
ADC与编码器结果、采样中断最大周期数、NTC状态和关相寄存器证据。
中断用时包含此次诊断计数开销，不能直接当作最终完整APP快环预算。

`g_r40_bench`的magic为0x52343042、version为1。主机先写command=0，
再写nonce（非零且不同于上一请求）、command_inverse及arguments，最后发布command。
结果0/1/2/3/4分别为空闲/执行中/通过/失败/拒绝，completed_nonce用于对应请求。
全部测试入口都先检查六路输出使能、总输出使能及驱动输入电平；失败停止，不能自动继续运行。
命令5保持最近发布快照供慢速SWD读取，命令6释放；两者不停止采样，也不暂停CPU。
命令7仅在无待发送帧时向诊断响应ID发送一帧`R40BTX!!`，独立检查板端发送方向，
不会增加接收计数；命令成功仅表示入队，物理完成须核对`can_tx_ok`及主机实际接收。

- command=1：调用真实PWM端口检查0/12.5/50/87.5/100%与单相25%比较值，最后恢复50%。
  不使能通道或总输出，因此不提供PWM引脚波形、死区或MOS驱动能力证明。
- command=2且arguments[0]=0x53544F52：仅接受原始两扇区全FF时写入2288字节测试图样。
- command=3且同一确认字：仅接受完整匹配的测试图样及FF尾部，核对重启保持后擦回全FF。
  两个存储命令都会停止采样定时器/DMA并屏蔽中断，结束后保持停止，下一步必须显式复位。
  实机执行前必须由主机备份并核对参数区，执行后再全区核对恢复结果。

专用CAN诊断只接受扩展FD+BRS帧，ID=0x1FFF0001、长度至少8且前四字节为`R40B`，
以ID=0x1FFF0002原样回显，通过真实64位票据端口确认发送完成。
此回显不是产品协议，不解释电机运行、参数或Flash命令；未匹配帧只统计接收次数。
波特率固定1M/5M。外部回显通过才能证明CAN收发器和主机链路，不以内环测试代替。

2026-10-23调整CAN_H/CAN_L外部接线后，正常通信模式下128帧回显全部通过，
覆盖8/12/16/20/24/32/48/64字节，另一次板端主动发送也正确收到。
新增发送失败为0，始终关相；本次未改固件或位时序。底层外部通信通过，完整APP协议另行验收。

AT32接收过滤器使用“掩码0比较、1忽略”（RM 21.3.1和21.7.17–18）。
IDE和远程帧位必须比较；ID、DLC、FD/BRS/ESI在硬件过滤时忽略，交由冻结的接收边界校验。
实机首轮暴露的掩码反向问题已加入独立过滤语义测试，不能只断言配置字段等于实现值。
RETLIM/REALIM受软件复位影响，在退出RESET后设置并有界读回，避免初始化值被复位覆盖。

1. P2已完成ADC四次累加及同步触发、CAN FD收发和故障处理。
   P4已补PWM占空比、可校准的MCU温度轮询与参数存储；内部温度禁用，电机NTC仅做私有诊断。
   通过实际端口的离线边界测试；新旧目标分别编译，功能层和公共接口保持冻结。
2. P3底层已实现：MT6835四线SPI、帧解码/CRC/故障与采样生命周期，保持原角度单位和接口。
   离线覆盖三种调用路径、全部65536输出桶及低5位边界、32个单比特错误、
   全部状态组合、各字节超时/错误、计数回绕/停走、事务互斥与未修改角度层的组合。
   实机已验证20kHz读取和转动角度，闭环台架继续记录最大中断时间；尚无逻辑分析仪波形验收。
3. P4：新电机目标数据、硬件身份与完整APP打包；上层只在目标组装处作必要选择。
4. P5：最后设计输出轴sin/cos的观测、闭环、标定及故障职责，再接入。

实机操作需要具体任务的明确授权。首先在功率级禁用条件下验证上电/异常关相、
1ms/2kHz/20kHz真实时序，再验证电压电流静态采样、固定1M/5M通信、手动转动编码器。
10A幅值/10秒属于更后的功率热验收，不作为驱动接入的默认测试。

## 独立闭环台架工程

`foundation/R40_F456_ClosedLoop.uvprojx`通过`r40-closedloop`显式构建。它是台架装配入口，
直接复用未修改的FOC、PI和速度算法；不是完整产品APP。BSP/ports仍不允许依赖motor，
只有foundation装配层允许连接算法和硬件。默认启动关相，不通过CAN接受功率命令。

闭环目标沿用原始采样保护与SWD命令握手，但拒绝旧START=1；命令10、11、12、13分别
请求定向电流、编码器整圈对齐、Vq开环配合Id闭环、速度阶段。必须完成且通过前序才能进入下一阶段。
`g_closedloop_snapshot`记录同一发布序号下的参考、电流、PI电压、速度、方向、零位和验收统计。
闭环台架直接调用原角度层的20kHz PLL、方向和电零位处理；不修改原观测器或标定表。

此目标的相电流瞬时停机门限为3A，预定位电流参考1.6A，Vq阶段矢量额度1.5A，其余0.8A；
预定位/校准电压向量超过2V停机，其余阶段超过1.5V停机，
转子闭环速度超过60rpm停机。每段最长10s，250ms主机心跳、采样计数与独立WDT均保留。
各段只显式开始一次，故障后不能直接继续。输出轴sin/cos只采样，不参与控制。
运行结果、已知限制与版本演进见适配计划（历史路径：`../../../docs/plans/active/2026-10-22-at32f456-port-v1.md`）。

整圈校准先预定位，再以14个电周期扫过一机械圈并返回，比较三段保持位置；
整圈行程误差须小于1%，回零误差须小于150计数。零位取正反接近位置的等权平均，
结果仅存在RAM中。Vq阶段使用原`FOC_Vq_Mode`，q轴向0.55V爬升、d轴闭环0A；
实测正向运动达到5°且已有1000个Id质量样本后立即结束，最长10s。
这是验证电零位后的短行程响应，不用恒定电压维持低速；长时间观察由后续速度环承担。
本阶段不保证速度跟踪，也不构成整圈传感器精度校准。

闭环台架另有2048点全程记录：4秒动态电流阶段500Hz，其余阶段200Hz，
保存实测编码器位置、PLL速度、Id/Iq、Iq参考和
Vd/Vq；固定占用64KiB，仅本台架链接，不进入产品APP。20kHz末段波形记录仍保留。
两份记录均在关相后读取，故障后停止更新；不能以参考角替代实测运动。

本轮状态：定向Id、2/10/30Hz动态电流、整圈临时电角度校准和Vq短行程已有通过记录；
速度仍未达标，ADC载波快照竞态修复尚需实机回归，
不能将本台架镜像作为已完成的闭环APP。CAN保持1M/5M，底层外部通信已通过上述关相回显测试。
设备镜像的具体哈希、运行终态以最近一次台架证据为准。

后续20Hz开环检查将Combined目标设为1V、20Hz电频率和10秒，其他目标默认频率仍为2Hz。
仅Combined启用80KiB的1kHz原始角度/三相电流缓冲，另统计每个20kHz角度增量，
结束后关相读取。该记录不是产品遥测接口，也不增加中间层或BSP接口。
20Hz实机角度连续、平均速度约85.72rpm，但角度差分速度有周期纹波，不能认定恒速精度通过。
设备本轮已改为此默认关相的Combined镜像并完成单次试转；最新状态与证据见适配计划。

随后按用户要求完成20Hz/2V、同样10秒及3A保护的对照；Combined当前电压参数为2V。
两轮角度均连续，平均速度约85.72rpm；2V的10/20ms差分速度纹波减小，1ms差分未改善。
相电流RMS明显增加（约0.48A到1.63A），最大瞬时相电流2.403A，结束后保持关相。

## Keil直接控制与RTT

仅`R40_F456_OpenLoop_Combined`启用`R40_KEIL_OPENLOOP=1`，固定20Hz电频率、2V相电压幅值、
3A瞬时相电流保护和10秒上限。Keil选J-Link、SWD，当前台架采用4MHz；禁止冻结WDT。
进入调试后运行CPU，等待`g_openloop_debug.state=1`及`fault=0`，再在Watch写
`g_openloop_enable=1`，不需要PC持续刷新心跳。`g_openloop_enable=0`立即请求关相。
本入口每次复位只允许一次：完成、提前停止或故障后均保持关相并将开关清零，
再写1不会自动恢复；重新测试需先复位并等READY。设置1时CPU须正在运行。
暂停CPU、单步或前台停止可能触发WDT复位，不应在运行中设置断点。

Watch添加`g_openloop_enable`、`g_openloop_debug`、`g_openloop_rejected_starts`：

| 字段 | 含义 |
| --- | --- |
| `state` | 0校零、1就绪、2运行、3完成、4故障、5等待静流 |
| `fault` | 0无故障；4过流、6租期过期、7输出异常、8时序异常、9用户停止 |
| `ia_a/ib_a/ic_a` | 三相实测电流，A |
| `angle_counts/angle_deg` | MT6835原始单圈计数/角度，0..65535 / 0..360° |
| `sin_raw/cos_raw` | PA5/PA6的12位原始ADC数，不作零点或幅值校准 |
| `sincos_valid/sincos_age_ms` | 版本一致且年龄不超过2ms才有效 |
| `speed_rpm` | 由实测角度约10ms差分得到的电机轴速度 |
| `bus_v/elapsed_ms` | 实测母线电压/本段采样时长 |
| `rtt_sent/rtt_dropped` | 已发送/因缓冲满丢弃的完整二进制帧累计数 |

`g_openloop_debug.begin/end`是版本号；读取时须相同且为偶数。高速SWD多字段读取可能跨帧，
波形记录优先使用RTT完整帧或原1kHz板内缓存。开关由前台处理并受原ADC/相电流保护约束。

RTT采用未修改的[SEGGER官方源码](https://github.com/SEGGERMicro/RTT)，固定提交与哈希见
`firmware/third_party/segger_rtt/manifest.json`。只有前台写RTT，满缓冲整帧丢弃且不等待PC，
不在ADC中断打印、不读取RTT下行控制命令，不使用半主机printf。
通道0为20Hz可读CSV，字段见首次输出标题，电流为mA、角度为毫度、速度为千分之一rpm。
通道1的有效负载约96KB/s，使用2kHz、48字节J-Scope帧，通道名为`JScope_t4f4f4f4f4u4u4f4u4u4f4u4`：

| J-Scope列 | 数据 | 单位 |
| --- | --- | --- |
| 时间戳 | `uint32_t` | 微秒 |
| 0、1、2 | Ia、Ib、Ic | float，A |
| 3 | MT6835角度 | float，° |
| 4、5 | PA5、PA6 | uint32，原始ADC计数 |
| 6 | 电机轴速度 | float，rpm |
| 7、8 | state、fault | uint32 |
| 9 | 母线电压 | float，V |
| 10 | 采样序号 | uint32，名义0.5ms一次，可用来识别丢帧 |

在Keil已连接J-Link的情况下，RTT Viewer选择`Existing Session`，查看Terminal 0；
需要波形时用J-Scope的RTT模式，按上表命名数据列。连接说明见
[RTT Viewer文档](https://kb.segger.com/J-Link_RTT_Viewer)和[J-Scope文档](https://kb.segger.com/J-Scope)。
同一上行通道只应有一个读取者；RTT Viewer/Logger/J-Scope同时消费通道1会互相抢数据。
二进制通道按固定48字节解释。中途断开读取可能遗留半帧，重新记录前先停止并复位MCU，
建立新的J-Scope记录，确认READY后才写使能；不能沿用上次半帧或在电机运行中为此复位。
若自动搜索不到控制块，从本次MAP读取`_SEGGER_RTT`地址，不能照抄另一镜像的地址。


闭环台架当前电流PI为1kHz候选（Kp=0.89535、Ki=3455.752），速度候选Kp=0.1、
Ki=10、目标正反20rpm、加减速2.5rad/s²。这些参数尚非产品验收参数。
载波漏采检查已改为短临界区快照，防止被ADC抢占后混用不同采样时刻；
真实50us漏采、计数停走、ADC/DMA异常及故障关相门限均保留。


## 共享APP的关相集成目标

完整APP唯一`main()`在现有`app/composition/app_startup.c`，旧板、新板和Loader配套APP共用。
本目录`application/main.c`定义真实入口，模板与旧板一致；
`application/platform_app.c`实现`AppPlatform_Init/Start/Process`及平台中断/保护；
初始化阶段、APP绑定后的接收/看门狗启动、业务后台后的平台维护顺序不变。
已删除退役的角度缓存源文件及无消费者的临界区性能统计，现有2kHz RTT和保护不变。

### 完整APP标定诊断准备

2026-10-23增加完整APP的2kHz只读RTT，仍未解除App_Off功率锁。
YQ-5708标定速度选择改为30rad/s机械、420rad/s电角，适配14对极，
避免旧21对极配置的机械目标落入观测器下限以下；旧型号配置不变。
具体流程及未验收边界见完整APP标定与闭环计划（历史路径：`../../../docs/plans/active/2026-10-23-r40-app-commissioning-v1.md`）。

Keil可观察`g_r40_app_telemetry`及其`sample`；采样每10个快拍产生一次，
RTT1输出88字节J-Scope帧。电流单位A，角速度为电机轴rad/s，角度为rad或Q15，
具体字段名带单位。满队列/满RTT丢整帧并计数，不等待主机，不含RTT控制入口。
2kHz是采集频率，主机实际收到的连续性须用sequence和丢帧计数确认。

SEGGER第三方实现仅保留在third_party/segger_rtt，唯一项目配置为
platform/config/SEGGER_RTT_Conf.h；完整APP和开环联调工程引用同一配置。
锁策略、通道上限和默认缓冲区是编译期配置；通道名与专用缓冲区在ConfigUpBuffer调用时传入。
app_telemetry只承担本项目的采样封帧及有界非阻塞发送，不属于SEGGER库。
drivers/measurement/bus_current_feedback承担PA4零偏/换算/滤波/有效性，APP状态适配器选择实测来源，
是当前功能代码；其测量职责向统一采样层收敛属于BSP职责整理，尚未在本次配置去重中迁移。

入口精简后不再接入临时5kHz角度缓存、Iq脉冲和分阶段调度探针。
保留现有2kHz RTT及一次功率窗口；控制启动和目标只能走共享APP命令。

`application/ESC-APP-AT32F456.uvprojx`通过`build_firmware --target r40-app-off`显式构建。
它链接原有APP、状态机、FOC、参数管理和CAN协议，使用AT32启动入口及平台端口；
不链接closedloop_bench或openloop_model。旧板normal和已有台架目标保持独立，all仍只构建旧板。

本目标固定`R40_APP_PHASE_LOCK=1`。原有start接口的本目标实现始终保持六路关闭并计数，
没有RAM/CAN解锁入口。ADC 20kHz和监督2kHz调用原App_FastTick/App_SupervisorTick，
前台执行App_Process；独立WDT只在前台、采样和监督都有进展时刷新。
这是完整共享软件的关相验证镜像，不是可驱动电机的正式固件。

完整APP通过CAN1_RX中断转发既有协议收包入口；端点绑定完成后才启用接收中断。
单次最多接收三帧，积压再次挂起，CRC和业务解析仍在前台；独立台架继续前台轮询。
接收优先级低于电流采样和电机监督，位时序始终为1M/5M。

完整APP使用本目录的独立启动文件，栈为8KiB且不分配堆；SDK原启动文件不改。
早期台架用的`g_r40_reset_diagnostic`跨复位记录、异常批量快照和快环历史峰值已删除。
故障入口只执行禁中断、关相、屏障和停止喂狗，不再维护另一份运行状态或复位历史。
原UNINIT记录及其链接选择已退役，其他RAM区域和Flash/参数布局不变。
保留50us单拍超时关断、2kHz RTT和限时目标一次功率许可；SCB故障寄存器可直接由调试器读取。
旧台架脚本若依赖已删的符号，必须按当前MAP调整；不得从固定旧地址推断仍有有效诊断对象。

J-Link会话必须加载`application/r40_debug.JLinkScript`，或显式执行
`SetSkipDebugDeInit = 1`及`SetDbgPowerDownOnClose = 0`。
固件以DWT监测采样时序，探针退出时清除DWT会触发正常的时基失效关断和WDT复位。
这是探针退出配置，不可通过删除ADC保护绕过；详见
[SEGGER周期计数器说明](https://kb.segger.com/J-Link_Cortex-M_application_uses_cycle_counter)和
[调试退出命令](https://kb.segger.com/J-Link_Command_Strings#SetSkipDebugDeInit)。
Keil需在J-Link设置中指定该脚本；若使用其他调试驱动，不能假定脚本自动生效。

MT6835在完整APP中使用40us整帧预算，包含两段SPI之间的原有热模型与观测器计算；
独立台架保持20us。全部目标仍检查CRC、传感器状态和有界轮询，APP整拍50us超时关断不变。
`g_r40_mt6835_frames`、`g_r40_mt6835_transfer_errors`和`g_r40_mt6835_max_cycles`
用于只读核对采样进展、总线错误及实际整帧周期；后者除以180得到us。

板级配置选择`FOC_ACTIVE_HW_PROFILE=2`，电机复用YQ-5708的配置选择，不再复制台架参数。
用户授权只调整配置入口；原控制算法、状态机、协议和BSP函数/类型/时基契约保持。
YQ-5708磁链采用已确认的电机轴Kt/Iq口径，数值为0.003814286Wb。
用户指定关相目标暂用3600s热时间常数，持续/峰值电流仍为3A；这不是实测热参数，
仅用于APP集成检查，不代表热保护验收。缺磁链的其他型号仍使用无效占位。
除显式限时验证目标外，没有关相锁的完整APP仍禁止编译不完整电机默认配置。
MCU温度未校准时仍无效，不能用电机NTC替代。

目标身份通过app/composition/app_target.h选择：旧板继续使用原G431清单，
新目标使用r40_f456_bringup.h；产品、硬件修订、布局和发布版本均为未分配0，
不冒充G431或可在线升级目标。不提供Loader及生产身份能力；UID读取真实96位硬件值。
APP从0x08000000启动，程序上界0x0803C000，末16KiB参数区保持。

PC13红灯、PC14蓝灯均低有效，上电先熄灭再设输出。PB5/PB6不初始化。
现有indicator_hw接口保持：非零闪烁码优先，故障使用红灯、模式使用蓝灯；
无闪烁码时，RED/YELLOW降级红，GREEN/BLUE/CYAN降级蓝，PURPLE/WHITE为红蓝同时，
NULL熄灭；保留原20Hz呼吸包络并由1ms软件调光执行。这里不声称双色灯能还原RGB颜色。

入口精简后删除`g_r40_app_led_test`和重复的`g_r40_app_snapshot`；
LED由正常APP指示驱动，状态/身份/电流/角度改用既有CAN读取和2kHz RTT查看。
采样失败、快环超过50us、主循环检测进展丢失或功率输出意外开启均立即关断。

首次实机启动中故障12为原MOTOR_FAULT_INVALID_PARAMETERS，来自当时未完成的电机配置。
后续磁链及临时热参数更新只改变编译默认值，不自动覆盖Flash中的参数；需以新镜像的
实际读回确认故障状态，不能承诺红灯立即消失。平台关相启动通过不代表电机、温度保护
或正式发布验收完成，MCU温度无效及其他启动条件仍须单独处理。

### 完整APP的Keil配置

Off与Commissioning沿用旧板的七组工程树、ARMCC 5.06工具链、C99、公共编译选项和
逐文件优化。三个完整APP中的49个公共C文件及6个Shared文件归组和顺序一致。
平台入口、采样和限时观测位于`40_Platform`，RTT与芯片SDK位于`90_Vendor`。
新板仍保留自己的设备包、启动文件、内存/Flash分区、SDK、目标配置及禁止自动下载设置。
目标配置通过ARMCC的`--preinclude`导入，不能直接使用ARMClang的`-include`。

新板平台仅作工具链兼容调整：C99编译期大小检查、ARMCC屏障/调用地址内建、
UNINIT对象的`zero_init`属性，以及散装文件的`i.<函数名>`选择规则。
诊断区地址、参数区、3A保护和控制时基保持不变；不修改公共控制实现或BSP契约。
工程对齐阶段未烧录；后续ARMCC关相实测发现PIO快环偶发超时，由公共目标配置
统一选择已有SPI DMA路径修复。Off已通过关相启动、静态ADC/MT6835及CAN 1M/5M读取，
96.67s无新增复位，快环峰值42.21us；公共实现和BSP接口不变。
ARMCC限时版控制仍未实测，既有ARMClang镜像的速度/时序数据属于历史结果。
当前结果见关相实测（历史路径：`../../../docs/plans/active/2026-10-24-armcc-off-hil-v1.md`）。
详见对齐计划及证据（历史路径：`../../../docs/plans/active/2026-10-24-keil-project-alignment-v1.md`）。

### MCU内部温度采样与校准

完整关相APP目标显式定义`BSP_ENABLED_MCU_TEMPERATURE=1`，采集ADC1_IN16温度及
ADC1_IN17内部参考电压；独立台架的默认开关与旧板实现不改。沿用ADC1普通序列、
DMA1通道3和2kHz监督轮询，不改变相电流抢占序列或公共`BspTemperature_PollMcu`契约。

Keil Watch可添加`g_r40_mcu_temperature_diagnostic`。读取时要求`begin=end`且为非零偶数。
`raw_ts/raw_vref`为12位单次转换计数；`sequence/sample_count`用于确认新样本持续推进。

| status | 含义 | temperature_c / vdda_mv | protection_valid |
| --- | --- | --- | --- |
| 0 | 采样禁用 | NaN | 0 |
| 1 | 无新鲜数据；raw字段保留最后一帧 | NaN | 0 |
| 2 | 未校准，使用典型参数估算 | 仅供诊断 | 0 |
| 3 | 已配置有效实板校准 | 校准换算值 | 1 |
| 4 | ADC样本或校准配置无效 | NaN | 0 |

未校准估算采用参考电压1200mV、25°C温度电压1280mV及有符号dVts/dT=+4.27mV/°C。
来源为[官方数据手册V2.00表68/69](https://www.arterychip.com/download/DS/DS_AT32F455_456_457_V2.00_EN.pdf)。
这些典型值不等于逐颗标定，不写入公共接口的有效温度；未校准时`conversion_ok=false`，
共享APP继续报告温度失效，不能据此宣称故障17解除。典型估算也不代替MOS或电机温度。

后续在两个已知且均温稳定的工况下，记录参考温度、TS/VINTRV ADC及实测VDDA。
在板级配置或目标Define中填入`R40_TEMPERATURE_CAL_C`、`R40_TEMPERATURE_CAL_MV`、
`R40_TEMPERATURE_SLOPE_MV_PER_C`及`R40_INTERNAL_REFERENCE_MV`，验收后再设
`R40_TEMPERATURE_CALIBRATED=1`；斜率为实测(V2−V1)/(T2−T1)，不能直接照搬
手册公式中带相反符号的Avg_Slope。校准值非法时不回退成典型有效值。

验证顺序：先离线校验禁用/未校准/已校准/坏校准和原保护，再关相实测样本推进、
温升方向、VDDA稳定性及ADC时序共存。修改源代码不等于已经下载或通过实机验收。


### 限时APP：保存恢复与SPI DMA验证

完整APP的Off和限时目标共用`R40_MT6835_DMA_TRANSFER=1`：
SPI2以11.25MHz/16位传输，三半字收发由DMA2通道1/2完成，ADC仍使用DMA1通道2/3。
基础目标默认保留PIO；BSP函数/类型/单位契约不变。
参数保存由AT32端口内部协调ADC暂停、DMA边界重建和新鲜数据恢复，仍须关相前台调用。

LUT在关相后台发布后，同一候选连续5次LUT生成、保存、主动复位加载一致；
实测快环峰值43.65..44.67us，达到本轮小于45us的目标。
MT6835片选等待按DWT周期计时，保留至少200ns及迭代上界。
这只通过当前限时台架流程，并非最坏情况时序证明；WDT重载2047没有恢复默认511，
后续v26已修正保存后的测速假尖峰，并完成3次电零位保存/复位加载验证；
动态电流与速度闭环尚未验收。证据与失败边界见
完整APP计划（历史路径：`../../../docs/plans/active/2026-10-23-r40-app-commissioning-v1.md`）。

平台归属已收拢：application只保留main和platform_app；原ports并入bsp；
母线电流测量归共享drivers/measurement，2kHz只读RTT归communication/telemetry。
本轮目录整理不代表新增实机验收，现有保护、目标许可与输出轴反馈范围保持。
