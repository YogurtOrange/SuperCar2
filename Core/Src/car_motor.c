/*
 * 电机驱动模块：构造帧 -> 解析回包 -> 双轮请求 -> 共享总线调度。
 * 使用 Emm 固件的帧布局、配置门禁和反馈单位。
 * 对外正方向统一为车体前进；发送与反馈均按各轮 sign 映射。
 * 普通事务单个在途，停止抢占等待；任何位置命令只发一次，ACK 丢失进入故障。
 */
#include "car_motor.h"
#include "car_uart.h"
#include "car_control.h"
#include <string.h>

#ifdef CAR_TEST
#define CAR_MOTOR_INTERNAL
#else
#define CAR_MOTOR_INTERNAL static
typedef struct {
    uint8_t data[48];
    uint8_t used;
    uint32_t last_byte;
} X42S_Parser;
#endif

/* ==================== 协议帧：Emm 控制与查询 ==================== */
CAR_MOTOR_INTERNAL uint8_t X42S_Enable(uint8_t *f, uint8_t a, uint8_t en)
{
    f[0]=a; f[1]=0xF3; f[2]=0xAB; f[3]=en; f[4]=0; f[5]=0x6B;
    return 6;
}
/* Emm 速度单位 RPM：符号单独编码，数值大端；acc 为驱动器档位，snF=0 立即执行。 */
CAR_MOTOR_INTERNAL uint8_t X42S_EmmVelocity(uint8_t *f, uint8_t a, int16_t rpm, uint8_t acc)
{
    uint16_t magnitude = (uint16_t)(rpm < 0 ? -(int32_t)rpm : rpm);
    f[0]=a; f[1]=0xF6; f[2]=(rpm<0); f[3]=(uint8_t)(magnitude>>8);
    f[4]=(uint8_t)magnitude; f[5]=acc; f[6]=0; f[7]=0x6B;
    return 8;
}
/* 位置 p 为无符号细分脉冲幅值，dir 是方向；mode 保留 0/1/2，末尾 snF=0。 */
CAR_MOTOR_INTERNAL uint8_t X42S_EmmPosition(uint8_t *f, uint8_t a, uint8_t dir,
                       uint32_t p, uint16_t rpm, uint8_t acc, uint8_t mode)
{
    f[0]=a; f[1]=0xFD; f[2]=dir; f[3]=(uint8_t)(rpm>>8); f[4]=(uint8_t)rpm;
    f[5]=acc; f[6]=(uint8_t)(p>>24); f[7]=(uint8_t)(p>>16);
    f[8]=(uint8_t)(p>>8); f[9]=(uint8_t)p; f[10]=mode; f[11]=0; f[12]=0x6B;
    return 13;
}
CAR_MOTOR_INTERNAL uint8_t X42S_Stop(uint8_t *f, uint8_t a)
{
    f[0]=a; f[1]=0xFE; f[2]=0x98; f[3]=0; f[4]=0x6B; return 5;
}
CAR_MOTOR_INTERNAL uint8_t X42S_Query(uint8_t *f, uint8_t a, uint8_t c)
{
    f[0]=a; f[1]=c;
    if(c==0x42) { f[2]=0x6C; f[3]=0x6B; return 4; }
    f[2]=0x6B; return 3;
}
CAR_MOTOR_INTERNAL uint8_t X42S_ClearProtection(uint8_t *f, uint8_t a)
{
    f[0]=a; f[1]=0x0E; f[2]=0x52; f[3]=0x6B; return 4;
}
/* ==================== 电机回包：按已知长度解析，载荷中的 6B 不截断帧 ==================== */
CAR_MOTOR_INTERNAL void X42S_ParserReset(X42S_Parser *p) { p->used=0; }
static uint8_t response_length(const X42S_Parser *p)
{
    if(p->used<2) return 0;
    switch(p->data[1]) {
    case 0x1A: case 0x3A: case 0xF3: case 0xF6: case 0xFD:
    case 0xFE: case 0xFF: case 0x0E: return 4;
    case 0x35: return 6;
    case 0x36: return 8;
    case 0x42:
        if(p->used<3) return 0;
        /* 仅接受 Emm 的 33 字节配置回包；长度包含地址与末尾 6B。
           其他长度按错误帧丢弃，完整回包随后还要检查 21 参数和配置内容。 */
        if(p->data[2]==0x21) return p->data[2];
        return 255;
    default: return 255;
    }
}
CAR_MOTOR_INTERNAL uint8_t X42S_ParserFeed(X42S_Parser *p, uint8_t b, uint32_t now)
{
    uint8_t len;
    if((uint32_t)(now-p->last_byte)>CAR_RX_FRAME_TIMEOUT_MS) p->used=0;
    p->last_byte=now;
    p->data[p->used++]=b;
    while(p->used) {
        if(p->data[0]!=CAR_LEFT_ADDR && p->data[0]!=CAR_RIGHT_ADDR) len=255;
        else len=response_length(p);
        if(len==0) return 0;
        if(len!=255 && p->used<len) return 0;
        if(len!=255 && p->data[len-1]==0x6B) {
            /* 只在已知帧长末尾检查 6B；位置数值中出现 6B 不能提前截帧。 */
            p->used=0;
            return len;
        }
        --p->used;
        memmove(p->data,p->data+1,p->used);
    }
    return 0;
}
/* Emm 位置反馈方向字节+32位幅值合成为 int64，每电机轴圈 65536 ticks。 */
CAR_MOTOR_INTERNAL int64_t X42S_SignedPosition(const uint8_t *f)
{
    uint32_t v=((uint32_t)f[3]<<24)|((uint32_t)f[4]<<16)|((uint32_t)f[5]<<8)|f[6];
    return f[2] ? -(int64_t)v : (int64_t)v;
}

/* ==================== 双轮内部状态：目标、反馈与共享事务 ==================== */
/* init_step=0查选项、1查配置、2查状态、3查速度、4失能、5初始化完成；desired_enable 是请求值。
 * position_pending 表示位置尚待接收 ACK，motion_due 表示新速度待发。 */
typedef struct {
    MotorFeedback fb;
    int8_t sign;
    uint8_t init_step, desired_enable, position_pending, position_mode;
    uint8_t motion_due, position_dir;
    uint32_t position_pulses, last_velocity, position_started, retry_at;
    uint16_t position_rpm;
} Wheel;
static Wheel wheels[CAR_MOTOR_COUNT];
static X42S_Parser parser;
/* pending 表示唯一普通事务；三个 mask 按轮索引置位，分别排队停止、解除保护、失能。 */
static uint8_t pending, pending_wheel, pending_code, stop_mask, zero_stop_mask, clear_mask, disable_mask;
static uint8_t recovering, fault_seen, round_robin;
static uint8_t pending_enable;
static uint32_t pending_time, quiet_until, next_stop_retry;

static uint8_t count(void) { return CAR_SINGLE_MOTOR_TEST ? 1U : CAR_MOTOR_COUNT; }
static uint8_t all_mask(void) { return (uint8_t)((1U<<count())-1U); }
static uint8_t due(uint32_t now, uint32_t deadline) { return (int32_t)(now-deadline)>=0; }
static Wheel *by_addr(uint8_t a)
{
    uint8_t i;
    for(i=0;i<count();i++) if(wheels[i].fb.addr==a) return &wheels[i];
    return 0;
}

/* ==================== 电机控制接口：配置门禁、目标、停止及异步恢复 ==================== */
/* i 填0读取左轮、1读取右轮；没有对应电机时返回空指针。 */
const MotorFeedback *CarMotor_Get(uint8_t i) { return i<count() ? &wheels[i].fb : 0; }

static uint8_t fresh(uint32_t now, uint32_t t)
{ return (uint32_t)(now-t)<=CAR_FEEDBACK_STALE_MS; }
/* 恢复完成条件：无待办事务、配置有效且在线、实际失能、零速、无保护、新鲜速度/状态。 */
static uint8_t safe_to_clear(uint32_t now)
{
    uint8_t i;
    if(pending || stop_mask || zero_stop_mask || clear_mask || disable_mask) return 0;
    for(i=0;i<count();i++) {
        const MotorFeedback *f=&wheels[i].fb;
        if(!f->config_ok || !f->online || f->enabled || f->speed_rpm || (f->flags&0x0C) ||
           !fresh(now,f->speed_time) || !fresh(now,f->status_time)) return 0;
    }
    return 1;
}
void CarMotor_StopAll(void)
{
    uint8_t i;
    /* 取消普通事务等待和全部待执行运动；不截断 UART 已经开始发送的帧。
       普通 STOP 保留 desired_enable，故障首次处理才清使能请求。 */
    pending=0; stop_mask=zero_stop_mask=all_mask();
    X42S_ParserReset(&parser);
    CarUart_FlushRx(UART_MOTOR);
    for(i=0;i<count();i++) {
        wheels[i].fb.target_rpm=0; wheels[i].position_pending=0;
        wheels[i].motion_due=0;
        wheels[i].fb.state=CarControl_Faults() ? MOTOR_FAULT : MOTOR_STOPPING;
    }
}
/* 启动先排停止，再逐台查询配置和反馈，最后失能；不会自动使能或运动。 */
void CarMotor_Init(uint32_t now)
{
    uint8_t i;
    memset(wheels,0,sizeof(wheels)); memset(&parser,0,sizeof(parser));
    wheels[0].fb.addr=CAR_LEFT_ADDR; wheels[0].sign=CAR_LEFT_SIGN;
    wheels[1].fb.addr=CAR_RIGHT_ADDR; wheels[1].sign=CAR_RIGHT_SIGN;
    for(i=0;i<count();i++) { wheels[i].fb.acc=CAR_DEFAULT_ACC; wheels[i].retry_at=now; }
    pending=clear_mask=disable_mask=recovering=fault_seen=round_robin=0;
    quiet_until=now; next_stop_retry=now;
    CarMotor_StopAll();
}
uint8_t CarMotor_Active(void)
{
    uint8_t i;
    for(i=0;i<count();i++) if(wheels[i].desired_enable || wheels[i].position_pending ||
       wheels[i].fb.state==MOTOR_SPEED || wheels[i].fb.state==MOTOR_POSITION) return 1;
    return 0;
}
/* 运动门禁检查实际使能和使能请求；未完成初始化、故障、停止确认中均拒绝运动。 */
static MotorResult can_move(Wheel *w)
{
    if(!w) return MOTOR_BAD_ARGUMENT;
    if(CarControl_Faults()) return MOTOR_SAFETY_LOCK;
    if(w->init_step!=5 || !w->fb.config_ok || !w->fb.online || !w->fb.enabled || !w->desired_enable)
        return MOTOR_NOT_READY;
    if(w->fb.state==MOTOR_STOPPING || w->position_pending) return MOTOR_BUSY;
    return MOTOR_OK;
}
/* addr 填电机地址，默认左1、右2；enable 填1使能、0失能。 */
MotorResult CarMotor_Enable(uint8_t addr, uint8_t enable)
{
    Wheel *w=by_addr(addr);
    if(!w || enable>1) return MOTOR_BAD_ARGUMENT;
    if(enable && CarControl_Faults()) return MOTOR_SAFETY_LOCK;
    if(!w->fb.config_ok || !w->fb.online) return MOTOR_NOT_READY;
    if(enable && (w->init_step!=5 || w->fb.state==MOTOR_STOPPING)) return MOTOR_NOT_READY;
    if(!enable) CarMotor_StopAll();
    w->desired_enable=enable;
    if(!enable) disable_mask|=(uint8_t)(1U<<(w-wheels));
    return MOTOR_OK;
}
/* addr 填电机地址；rpm 填整数RPM，默认 -60~60，正数前进、负数后退、0为零速。
 * acc 填加速度档位0~255；例如左轮前进可填1、30、10。 */
MotorResult CarMotor_Velocity(uint8_t addr, int16_t rpm, uint8_t acc)
{
    Wheel *w=by_addr(addr);
    MotorResult r=can_move(w);
    if(rpm>CAR_MAX_RPM || rpm < -CAR_MAX_RPM) return MOTOR_BAD_ARGUMENT;
    if(r!=MOTOR_OK) return r;
    w->fb.target_rpm=rpm; w->fb.acc=acc; w->motion_due=1;
    w->fb.state=MOTOR_SPEED;
    return MOTOR_OK;
}
/* l、r 填左右电机的整数RPM，默认 -60~60，正前进、负后退、0零速；acc 填档位0~255。
 * 例如 CarMotor_Wheels(30, 30, 10)；先检查两轮，再逐台发送。 */
MotorResult CarMotor_Wheels(int16_t l, int16_t r, uint8_t acc)
{
    MotorResult a=can_move(&wheels[0]), b;
    if(l>CAR_MAX_RPM || l < -CAR_MAX_RPM || r>CAR_MAX_RPM || r < -CAR_MAX_RPM)
        return MOTOR_BAD_ARGUMENT;
    if(a!=MOTOR_OK) return a;
    if(count()==2) { b=can_move(&wheels[1]); if(b!=MOTOR_OK) return b; }
    CarMotor_Velocity(CAR_LEFT_ADDR,l,acc);
    if(count()==2) CarMotor_Velocity(CAR_RIGHT_ADDR,r,acc);
    return MOTOR_OK;
}
/* addr 填电机地址；pulses 填整数细分脉冲数；rpm 填正整数RPM，默认1~60；acc 填档位0~255。
 * mode 填0相对上一目标、1绝对坐标、2相对当前位置。
 * 相对模式正数向前、负数向后；绝对模式填目标坐标；位置只发送一次。 */
MotorResult CarMotor_Position(uint8_t addr, int32_t pulses, uint16_t rpm, uint8_t acc, uint8_t mode)
{
    Wheel *w=by_addr(addr);
    MotorResult r=can_move(w);
    int64_t signed_pulses;
    if(!rpm || rpm>CAR_MAX_RPM || mode>2) return MOTOR_BAD_ARGUMENT;
    if(r!=MOTOR_OK) return r;
    signed_pulses=(int64_t)pulses*w->sign;
    w->position_dir=(signed_pulses<0);
    w->position_pulses=(uint32_t)(signed_pulses<0 ? -signed_pulses : signed_pulses);
    w->position_rpm=rpm; w->fb.acc=acc; w->position_mode=mode;
    w->position_pending=1; w->motion_due=0; w->fb.target_rpm=0;
    w->fb.state=MOTOR_POSITION;
    return MOTOR_OK;
}
/* l、r 填左右轮的整数细分脉冲数；rpm、acc、mode 的填法与上面的单轮位置函数相同。 */
MotorResult CarMotor_Positions(int32_t l, int32_t r, uint16_t rpm, uint8_t acc, uint8_t mode)
{
    MotorResult a=can_move(&wheels[0]), b;
    if(!rpm || rpm>CAR_MAX_RPM || mode>2) return MOTOR_BAD_ARGUMENT;
    if(a!=MOTOR_OK) return a;
    if(count()==2) { b=can_move(&wheels[1]); if(b!=MOTOR_OK) return b; }
    CarMotor_Position(CAR_LEFT_ADDR,l,rpm,acc,mode);
    if(count()==2) CarMotor_Position(CAR_RIGHT_ADDR,r,rpm,acc,mode);
    return MOTOR_OK;
}
/* 只排入停止->解除保护->失能流程，返回 1 不是已经清故障；后续 Poll 核验安全再清锁存。 */
uint8_t CarMotor_ClearFault(uint32_t now)
{
    (void)now;
    if(!CarControl_InputsReleased()) return 0;
    if(recovering) return 1;
    if(!CarControl_Faults()) return 1;
    CarMotor_StopAll();
    clear_mask=all_mask(); disable_mask=all_mask(); recovering=1;
    return 1;
}

/* ==================== 共享总线调度：单个在途事务、回包匹配与停止优先 ==================== */
/* 应答超时锁存通信故障；初始化失败可延后重新核验，但运动目标不会因此自动重放。 */
static void transaction_failed(uint32_t now)
{
    Wheel *w=&wheels[pending_wheel];
    w->fb.online=0;
    if(w->init_step<5) { w->init_step=0; w->fb.config_ok=0; w->retry_at=now+500U; }
    pending=0; recovering=0; clear_mask=0;
    quiet_until=now+CAR_BUS_QUIET_MS;
    CarControl_Trip(FAULT_MOTOR_COMM);
}
static void handle_frame(const uint8_t *f, uint8_t n, uint32_t now)
{
    Wheel *w=by_addr(f[0]);
    uint16_t speed;
    if(!w) return;
    /* 地址与功能码都匹配当前事务才接收；主动/迟到回包不能完成其他事务。 */
    if(!pending || f[0]!=wheels[pending_wheel].fb.addr || f[1]!=pending_code) return;
    if(f[1]==0xFD && f[2]==0x9F) return; /* 到位通知不是 Receive ACK，仍要等接收应答。 */
    pending=0; w->fb.online=1;
    switch(f[1]) {
    case 0x1A:
        w->fb.option=f[2]; w->init_step=1; break;
    case 0x42:
        /* 配置门禁：Emm 33 字节/21 参数，UART、115200、地址一致、固定 6B、Receive。
           f[6]=接口，f[18]=波特率，f[20]=地址，f[21]=校验，f[22]=应答。
           option bit1 必须支持 Emm，bit7 不得开启输入缩小；不符合时锁存而非自动改驱动参数。 */
        w->fb.config_ok=(n==33 && f[3]==0x15 && f[6]==2 && f[18]==5 &&
                         f[20]==w->fb.addr && f[21]==0 && f[22]==1 &&
                         (w->fb.option&0x02)!=0 && (w->fb.option&0x80)==0);
        if(!w->fb.config_ok) {
            CarControl_Trip(FAULT_CONFIG); w->init_step=0; w->retry_at=now+500U;
        } else w->init_step=2;
        break;
    case 0x35:
        if(f[2]>1) { CarControl_Trip(FAULT_COMMAND); break; }
        speed=(uint16_t)(((uint16_t)f[3]<<8)|f[4]);
        if(speed>5000) { CarControl_Trip(FAULT_COMMAND); break; }
        w->fb.speed_rpm=(int16_t)((f[2] ? -(int32_t)speed : speed)*w->sign);
        w->fb.speed_time=now;
        if(w->init_step==3) w->init_step=4;
        if(w->fb.state==MOTOR_STOPPING && !speed) w->fb.state=w->fb.enabled ? MOTOR_READY : MOTOR_DISABLED;
        break;
    case 0x36:
        if(f[2]>1) { CarControl_Trip(FAULT_COMMAND); break; }
        w->fb.position_ticks=X42S_SignedPosition(f)*w->sign; w->fb.position_time=now; break;
    case 0x3A:
        w->fb.flags=f[2]; w->fb.enabled=(f[2]&1U); w->fb.status_time=now;
        if(w->init_step==2) w->init_step=3;
        if(f[2]&0x0C) CarControl_Trip(FAULT_DRIVER);
        if(w->fb.state==MOTOR_POSITION && !w->position_pending &&
           (uint32_t)(now-w->position_started)>CAR_STATUS_PERIOD_MS && (f[2]&2))
            w->fb.state=MOTOR_READY;
        if(w->desired_enable && !(f[2]&1) &&
           (w->fb.state==MOTOR_SPEED || w->fb.state==MOTOR_POSITION)) CarControl_Trip(FAULT_DRIVER);
        break;
    default:
        if(f[2]!=0x02) { CarControl_Trip(FAULT_COMMAND); recovering=0; clear_mask=0; break; }
        if(f[1]==0xF3) {
            /* ACK 对应的是实际发出的使能值；等待期间 desired_enable 可能已改变。 */
            uint8_t en=pending_enable;
            w->fb.enabled=en;
            if(!en) disable_mask&=(uint8_t)~(1U<<pending_wheel);
            if(w->init_step==4) { w->init_step=5; w->desired_enable=0; }
            w->fb.state=CarControl_Faults() ? MOTOR_FAULT : (en ? MOTOR_READY : MOTOR_DISABLED);
        }
        if(f[1]==0xFD) { w->position_pending=0; w->position_started=now; }
        if(f[1]==0x0E) clear_mask&=(uint8_t)~(1U<<pending_wheel);
        break;
    }
    quiet_until=now+1U;
}
/* 成功提交后才登记在途事务；帧已复制进串口持久缓冲，函数返回后栈内 frame 可复用。 */
static uint8_t send_frame(uint8_t i, const uint8_t *f, uint8_t n, uint32_t now)
{
    if(!CarUart_SendMotor(f,n)) return 0;
    if(f[1]==0xF3) pending_enable=f[3];
    pending=1; pending_wheel=i; pending_code=f[1]; pending_time=now; return 1;
}
/* 调度阶段：消费回包->检查超时/过期->处理故障/恢复->优先停止->逐轮选择一个普通事务。 */
void CarMotor_Poll(uint32_t now)
{
    uint8_t b, n, i, step, frame[16];
    uint16_t budget=CAR_RX_RING_SIZE;
    if(CarUart_TakeRxFault(UART_MOTOR)) {
        CarUart_FlushRx(UART_MOTOR); X42S_ParserReset(&parser);
        CarControl_Trip(FAULT_UART);
    }
    while(budget-- && CarUart_Read(UART_MOTOR,&b)) {
        n=X42S_ParserFeed(&parser,b,now);
        if(n) handle_frame(parser.data,n,now);
    }
    if(pending && (uint32_t)(now-pending_time)>=CAR_BUS_TIMEOUT_MS) transaction_failed(now);
    if(CarMotor_Active()) for(i=0;i<count();i++) {
        if(!fresh(now,wheels[i].fb.status_time) || !fresh(now,wheels[i].fb.speed_time))
            CarControl_Trip(FAULT_MOTOR_COMM);
    }
/* 故障首次进入时清使能请求；故障未解除且速度非零/反馈过期时约每 100 ms 再发停止。 */
    if(CarControl_Faults() && !fault_seen) {
        fault_seen=1; CarMotor_StopAll(); next_stop_retry=now+100U;
        for(i=0;i<count();i++) wheels[i].desired_enable=0;
    }
    if(CarControl_Faults() && due(now,next_stop_retry) && !recovering) {
        next_stop_retry=now+100U;
        for(i=0;i<count();i++) if(wheels[i].fb.speed_rpm || !fresh(now,wheels[i].fb.speed_time)) {
            stop_mask|=(uint8_t)(1U<<i); zero_stop_mask|=(uint8_t)(1U<<i);
        }
    }
    if(recovering && safe_to_clear(now) && CarControl_Clear(1,now)) {
        recovering=0; fault_seen=0;
        for(i=0;i<count();i++) wheels[i].fb.state=MOTOR_DISABLED;
    }
    if(CarUart_MotorTxBusy()) return;
    /* 先对两轮发 FE 98 专用停止（保留位置模式停止路径），随后补发实测 F6 零速帧。
       四帧均不等 ACK，遵守非阻塞帧间隔；最后留安静窗口再查询零速确认。
       F6 零速的 dir=0、rpm=0、acc=0、snF=0 与旧 Motor_Emergency_Stop/test.py 一致。 */
    if(stop_mask || zero_stop_mask) {
        pending=0;
        for(i=0;i<count();i++) if(stop_mask&(1U<<i)) {
            n=X42S_Stop(frame,wheels[i].fb.addr);
            if(CarUart_SendMotor(frame,n)) {
                stop_mask&=(uint8_t)~(1U<<i); quiet_until=now+CAR_BUS_QUIET_MS;
            }
            return;
        }
        for(i=0;i<count();i++) if(zero_stop_mask&(1U<<i)) {
            n=X42S_EmmVelocity(frame,wheels[i].fb.addr,0,0);
            if(CarUart_SendMotor(frame,n)) {
                zero_stop_mask&=(uint8_t)~(1U<<i); quiet_until=now+CAR_BUS_QUIET_MS;
            }
            return;
        }
    }
    if(pending || !due(now,quiet_until)) return;
/* 轮间轮询保持公平；解除保护/失能/初始化优先。
 * 新速度目标在反馈年龄小于两倍查询周期时先于常规查询，避免每次更新排在诊断后。
 * 反馈较旧时让状态/速度查询先走，持续新目标不能饿死保护反馈；位置仍只发一次。
 * 配置中的周期是发送资格门槛，不保证固定周期；所有请求争用同一条总线。 */
    for(step=0;step<count();step++) {
        Wheel *w;
        i=(uint8_t)((round_robin+step)%count()); w=&wheels[i]; n=0;
        if(clear_mask&(1U<<i)) n=X42S_ClearProtection(frame,w->fb.addr);
        else if(disable_mask&(1U<<i)) n=X42S_Enable(frame,w->fb.addr,0);
        else if(w->init_step<5 && due(now,w->retry_at)) {
            switch(w->init_step) {
            case 0: n=X42S_Query(frame,w->fb.addr,0x1A); break;
            case 1: n=X42S_Query(frame,w->fb.addr,0x42); break;
            case 2: n=X42S_Query(frame,w->fb.addr,0x3A); break;
            case 3: n=X42S_Query(frame,w->fb.addr,0x35); break;
            default: n=X42S_Enable(frame,w->fb.addr,0); break;
            }
        }
        else if(w->init_step==5) {
            if(!CarControl_Faults() && w->desired_enable!=w->fb.enabled)
                n=X42S_Enable(frame,w->fb.addr,w->desired_enable);
            else if(!CarControl_Faults() && w->fb.state==MOTOR_SPEED && w->motion_due &&
                    (uint32_t)(now-w->fb.status_time)<2U*CAR_STATUS_PERIOD_MS &&
                    (uint32_t)(now-w->fb.speed_time)<2U*CAR_STATUS_PERIOD_MS)
                n=X42S_EmmVelocity(frame,w->fb.addr,(int16_t)(w->fb.target_rpm*w->sign),w->fb.acc);
            else if((uint32_t)(now-w->fb.status_time)>=CAR_STATUS_PERIOD_MS)
                n=X42S_Query(frame,w->fb.addr,0x3A);
            else if((uint32_t)(now-w->fb.speed_time)>=CAR_STATUS_PERIOD_MS)
                n=X42S_Query(frame,w->fb.addr,0x35);
            else if(!CarControl_Faults() && w->position_pending)
                n=X42S_EmmPosition(frame,w->fb.addr,w->position_dir,w->position_pulses,
                                   w->position_rpm,w->fb.acc,w->position_mode);
            else if(!CarControl_Faults() && w->fb.state==MOTOR_SPEED &&
                    (w->motion_due || (uint32_t)(now-w->last_velocity)>=CAR_SPEED_REFRESH_MS))
                n=X42S_EmmVelocity(frame,w->fb.addr,(int16_t)(w->fb.target_rpm*w->sign),w->fb.acc);
            else if((uint32_t)(now-w->fb.position_time)>=CAR_POSITION_PERIOD_MS)
                n=X42S_Query(frame,w->fb.addr,0x36);
        }
        if(n && send_frame(i,frame,n,now)) {
            if(frame[1]==0xF6) { w->motion_due=0; w->last_velocity=now; }
            round_robin=(uint8_t)((i+1)%count()); return;
        }
    }
}
