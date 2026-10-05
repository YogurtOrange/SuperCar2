#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * SuperCar2 集中配置。修改这里的宏后必须重新编译、烧录才能生效。
 * 调参步骤见 docs/参数说明与调参指南.md；运行时用 VEL/WHEELS/POS 的参数调速。
 * 本工程使用驱动器内部闭环，STM32 不实现速度 PID；不要在此寻找 PID 增益。
 */

/* ==================== 电机数量、地址与方向 ==================== */
/* 固定双轮结构，保持 2；单轮测试用 CAR_SINGLE_MOTOR_TEST，不要改为 1。 */
#define CAR_MOTOR_COUNT              2U
/* 当前实际接线映射：左=1，右=2；旧巡线工程曾使用相反映射，不沿用。
 * 驱动器自身地址须一致且左右不同；0 为广播，本调度器不使用广播。 */
#define CAR_LEFT_ADDR                1U
#define CAR_RIGHT_ADDR               2U
/* 只能填 +1 或 -1；同时修正运动指令方向和速度/位置反馈方向。
 * 目标正数应对应车体前进。哪一轮反向就修改哪一轮，不同时改上位机符号。 */
#define CAR_LEFT_SIGN                1
#define CAR_RIGHT_SIGN              -1

/* ==================== 运动参数 ==================== */
/* 单位 RPM（电机轴转/分）：速度绝对值上限，也是位置命令的速度上限。
 * 超限请求直接拒绝，不自动截断；先以 10~30 RPM 空载测方向，再按负载验证上限。 */
#define CAR_MAX_RPM                  60
/* 初始化反馈结构的加速度档位，0~255；不是 STM32 PID 增益，也不是 m/s²。
 * VEL/WHEELS/POS 每条命令都带 acc，会覆盖此初值；实际调加速度要改命令参数。 */
#define CAR_DEFAULT_ACC              10U

/* ==================== 通信与保护周期（以下单位均为 ms） ==================== */
/* 已请求使能/运动期间，无有效保活达到此值即锁存故障；STOP 后仍需心跳。
 * 上位机默认每 100 ms 发一次 HEARTBEAT；PING/STATUS/错误或重复二进制包不保活。 */
#define CAR_HOST_TIMEOUT_MS          300U
/* 普通电机事务等待应答的期限，从提交 UART 发送时起算；超时锁存通信故障。
 * 与 car_uart.c 中固定 20 ms 的 UART TX 卡死检测是两条独立保护。 */
#define CAR_BUS_TIMEOUT_MS           20U
/* 任意两条电机 TX 完成到下一帧开始的最小间隔；参考旧 Drive 的 5 ms。
 * 在 UART 发送入口按时间戳实现，不阻塞主循环；普通命令和停止帧均遵守。 */
#define CAR_MOTOR_FRAME_GAP_MS        5U
/* 事务超时、优先停止后的总线安静时间，让迟到回包消退；增大将延后查询/恢复。 */
#define CAR_BUS_QUIET_MS             25U
/* 电机回包及上位机二进制半帧的字节间超时；慢速人工输入 ASCII 不受此限制。 */
#define CAR_RX_FRAME_TIMEOUT_MS      20U
/* 速度模式重复下发当前目标的间隔；新目标标记 motion_due，可提前下发。 */
#define CAR_SPEED_REFRESH_MS         50U
/* 每台电机状态 3A、速度 35 的查询间隔门槛；共享总线排队使实际周期可能更长。 */
#define CAR_STATUS_PERIOD_MS         50U
/* 每台电机位置 36 的查询间隔门槛，优先级低于状态/速度/待执行运动。 */
#define CAR_POSITION_PERIOD_MS       100U
/* 运动/使能活动时，状态或速度反馈超过此年龄则停机；恢复也要求反馈足够新鲜。
 * 应大于实测的最坏查询周期并留余量；不能靠放大此值掩盖丢包。 */
#define CAR_FEEDBACK_STALE_MS         250U
/* 上位机二进制状态/反馈的自动回传间隔；ASCII 模式仅自动回报故障变化。 */
#define CAR_TELEMETRY_PERIOD_MS       100U

/* ==================== 联调与保护开关（0=关闭，1=开启） ==================== */
/* 1=只联调 CAR_LEFT_ADDR，忽略右轮；装车双轮运行前改回 0 并重新烧录。 */
#define CAR_SINGLE_MOTOR_TEST         0
/* PB12 上拉、低有效：接 GND 触发急停；恢复前必须释放。 */
#define CAR_USE_ESTOP_PIN             1
/* PB13/PB14 上拉、低有效报警，接线/电平确认后才设 1；默认只读 UART 保护状态。
 * 电机 T/B/L 已用于 UART 回包，不能同时当作独立报警线。 */
#define CAR_USE_FAULT_PINS            0
/* 主循环喂独立看门狗，默认启用；具体 PR/RLR 在 car_control.c 的 board_init。
 * 关闭此开关会失去主循环卡死复位保护，常规调参保持 1。 */
#define CAR_USE_IWDG                  1

/* ==================== 缓冲与协议容量（常规调参保持默认） ==================== */
/* 每路 RX 环形缓冲字节数；空出一格区分满/空，默认最多存 255 字节。 */
#define CAR_RX_RING_SIZE              256U
/* 主机 TX 队列槽数；空槽及 HAL 正在发送的槽需保留，不代表可排 8 个新帧。
 * 变大占用 RAM，不能增加串口带宽；队列满时拒绝入队，控制循环继续运行。 */
#define CAR_HOST_TX_DEPTH             8U
/* v1 线协议最大 payload 字节数；不是速度参数，改变需同步协议/上位机/测试。 */
#define CAR_HOST_MAX_PAYLOAD          64U

#endif
