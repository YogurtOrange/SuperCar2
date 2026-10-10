/*
 * 串口通信模块：底层收发 -> 上位机编解码 -> 命令处理 -> 反馈发送。
 * USART1 为电机共享总线，USART2 为香橙派/调试入口，均由 CubeMX 初始化为 115200/8N1。
 * ISR 只搬字节和维护收发标志；命令在 CarUart_HostPoll 中由主循环执行。
 * 调参入口：car_config.h 的周期/容量、主机命令的 RPM/acc；波特率还涉及 main/.ioc 与驱动器配置门禁。
 */
#include "car_uart.h"
#include "car_motor.h"
#include "car_control.h"
#include <string.h>
#include <stdio.h>
#include <limits.h>

#ifdef CAR_TEST
#define CAR_UART_INTERNAL
#else
#define CAR_UART_INTERNAL static
typedef struct {
    uint8_t data[CAR_HOST_MAX_PAYLOAD+8];
    uint8_t used;
    uint32_t last_byte;
} HostCodec;
#endif

/* ==================== 底层收发：环形接收、持久发送缓冲与唯一 HAL 回调 ==================== */
/* 每路 UART 状态：head 由 ISR 推进，tail 由主循环消费；volatile 标志跨 ISR/主循环共享。 */
typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t rx_byte, rx[CAR_RX_RING_SIZE];
    volatile uint16_t head, tail;
    volatile uint8_t fault, restart, tx_busy;
    uint32_t tx_started;
} Port;
static Port ports[2];
/* 电机总线按发送完成时刻留帧间隔，不能用 HAL_Delay 阻塞急停检查。 */
static volatile uint32_t motor_tx_ended;
static volatile uint8_t motor_tx_has_ended;
/* HAL_IT 不复制输入数据：这里的电机帧和下面的主机队列必须存活至发送完成。 */
static uint8_t motor_tx[16];
static uint8_t host_tx[CAR_HOST_TX_DEPTH][CAR_HOST_MAX_PAYLOAD+8];
static uint8_t host_len[CAR_HOST_TX_DEPTH], tx_head, tx_tail;

static Port *find_port(UART_HandleTypeDef *uart)
{
    if(uart==ports[0].uart) return &ports[0];
    if(uart==ports[1].uart) return &ports[1];
    return 0;
}
void CarUart_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host)
{
    uint8_t i;
    memset(ports,0,sizeof(ports));
    ports[0].uart=motor; ports[1].uart=host;
    motor_tx_ended=0; motor_tx_has_ended=0;
    for(i=0;i<2;i++) ports[i].restart=1;
    tx_head=tx_tail=0;
    CarUart_Poll();
}
/* 每次收一个字节；队列满仅标错，不覆盖未消费数据，然后立即重挂下一字节接收。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    Port *p=find_port(uart);
    uint16_t next;
    if(!p) return;
    next=(uint16_t)((p->head+1)%CAR_RX_RING_SIZE);
    if(next==p->tail) p->fault=1;
    else { p->rx[p->head]=p->rx_byte; __DMB(); p->head=next; }
    if(HAL_UART_Receive_IT(uart,&p->rx_byte,1)!=HAL_OK) p->restart=1;
}
/* 中断里只标错/请求重挂；主循环清坏数据并锁存 FAULT_UART。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    Port *p=find_port(uart);
    if(p) { p->fault=1; p->restart=1; }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    Port *p=find_port(uart);
    if(p) {
        if(p==&ports[0]) { motor_tx_ended=HAL_GetTick(); motor_tx_has_ended=1; }
        p->tx_busy=0;
    }
}
/* 先检测 TX 卡死，再恢复 RX，最后启动主机队列下一帧；20U 是固定 TX 阈值，独立于 CAR_BUS_TIMEOUT_MS。 */
void CarUart_Poll(void)
{
    uint8_t i;
    for(i=0;i<2;i++) if(ports[i].tx_busy &&
       (uint32_t)(HAL_GetTick()-ports[i].tx_started)>20U) {
        HAL_UART_AbortTransmit(ports[i].uart);
        if(i==0) { motor_tx_ended=HAL_GetTick(); motor_tx_has_ended=1; }
        ports[i].tx_busy=0; ports[i].fault=1;
    }
    for(i=0;i<2;i++) if(ports[i].restart) {
        HAL_UART_AbortReceive(ports[i].uart);
        if(HAL_UART_Receive_IT(ports[i].uart,&ports[i].rx_byte,1)==HAL_OK)
            ports[i].restart=0;
    }
    if(!ports[1].tx_busy && tx_tail!=tx_head) {
        /* 先置忙再调用 HAL，避免发送完成中断先到，造成完成事件被覆盖。 */
        ports[1].tx_busy=1;
        ports[1].tx_started=HAL_GetTick();
        if(HAL_UART_Transmit_IT(ports[1].uart,host_tx[tx_tail],host_len[tx_tail])==HAL_OK)
            tx_tail=(uint8_t)((tx_tail+1)%CAR_HOST_TX_DEPTH);
        else { ports[1].tx_busy=0; ports[1].fault=1; }
    }
}
uint8_t CarUart_Read(UartPort port, uint8_t *b)
{
    Port *p=&ports[port];
    if(p->head==p->tail) return 0;
    *b=p->rx[p->tail]; p->tail=(uint16_t)((p->tail+1)%CAR_RX_RING_SIZE); return 1;
}
/* 短临界区丢弃旧字节，保存并恢复进入前的中断状态，不能无条件重新开中断。 */
void CarUart_FlushRx(UartPort port)
{
    uint32_t saved=__get_PRIMASK();
    __disable_irq(); ports[port].tail=ports[port].head; __set_PRIMASK(saved);
}
uint8_t CarUart_TakeRxFault(UartPort port)
{
    uint32_t saved=__get_PRIMASK();
    uint8_t f;
    __disable_irq(); f=ports[port].fault; ports[port].fault=0; __set_PRIMASK(saved);
    return f;
}
uint8_t CarUart_MotorTxBusy(void) { return ports[0].tx_busy; }
/* 把栈上帧复制到持久缓冲；忙时返回 0，不等待、不覆盖。事务 ACK 在 car_motor 处理。 */
uint8_t CarUart_SendMotor(const uint8_t *d, uint8_t n)
{
    uint32_t now=HAL_GetTick();
    if(ports[0].tx_busy || n>sizeof(motor_tx)) return 0;
    if(motor_tx_has_ended && (uint32_t)(now-motor_tx_ended)<CAR_MOTOR_FRAME_GAP_MS) return 0;
    memcpy(motor_tx,d,n); ports[0].tx_busy=1;
    ports[0].tx_started=HAL_GetTick();
    if(HAL_UART_Transmit_IT(ports[0].uart,motor_tx,n)!=HAL_OK) {
        ports[0].tx_busy=0; ports[0].fault=1; return 0;
    }
    return 1;
}
static uint8_t CarUart_SendHost(const uint8_t *d, uint8_t n)
{
    uint8_t next=(uint8_t)((tx_head+1)%CAR_HOST_TX_DEPTH);
    /* tx_tail 已前移，但 HAL 仍可能读上一槽；该槽必须保留到 TX 完成。 */
    uint8_t reserved=(uint8_t)((tx_tail+CAR_HOST_TX_DEPTH-1)%CAR_HOST_TX_DEPTH);
    if(n>sizeof(host_tx[0]) || next==tx_tail ||
       (ports[1].tx_busy && tx_head==reserved)) return 0;
    memcpy(host_tx[tx_head],d,n); host_len[tx_head]=n; tx_head=next; return 1;
}

/* ==================== 上位机编解码：CRC16、长度检查、噪声与半帧恢复 ==================== */
/* CRC16-Modbus：FFFF 初值、A001 多项式；校验向量 123456789 -> 4B37。 */
CAR_UART_INTERNAL uint16_t HostCodec_Crc16(const uint8_t *d, uint8_t n)
{
    uint16_t crc=0xFFFF;
    uint8_t i, bit;
    for(i=0;i<n;i++) {
        crc^=d[i];
        for(bit=0;bit<8;bit++) crc=(uint16_t)((crc>>1)^((crc&1)?0xA001:0));
    }
    return crc;
}
/* 主机帧 AA 55 / 版本 / TYPE / LEN / SEQ / payload / CRC低高字节；CRC 不含 AA 55。 */
CAR_UART_INTERNAL uint8_t HostCodec_Encode(uint8_t *f, uint8_t type, uint8_t seq, const uint8_t *p, uint8_t n)
{
    uint16_t crc;
    if(n>CAR_HOST_MAX_PAYLOAD) return 0;
    f[0]=0xAA; f[1]=0x55; f[2]=1; f[3]=type; f[4]=n; f[5]=seq;
    if(n) memcpy(f+6,p,n);
    crc=HostCodec_Crc16(f+2,(uint8_t)(n+4));
    f[n+6]=(uint8_t)crc; f[n+7]=(uint8_t)(crc>>8); return (uint8_t)(n+8);
}
/* 按长度收齐并校验，错误时逐字节滑窗找帧头；成功后 used 清零，但 data 保留完整帧供命令读取。 */
CAR_UART_INTERNAL int HostCodec_Feed(HostCodec *p, uint8_t b, uint32_t now)
{
    uint8_t len;
    uint16_t crc;
    int error=0;
    if((uint32_t)(now-p->last_byte)>CAR_RX_FRAME_TIMEOUT_MS) p->used=0;
    p->last_byte=now;
    p->data[p->used++]=b;
    while(p->used) {
        if(p->data[0]!=0xAA || (p->used>=2 && p->data[1]!=0x55) ||
           (p->used>=3 && p->data[2]!=1) ||
           (p->used>=5 && p->data[4]>CAR_HOST_MAX_PAYLOAD)) goto discard;
        if(p->used<6) return error;
        len=(uint8_t)(p->data[4]+8);
        if(p->used<len) return error;
        crc=HostCodec_Crc16(p->data+2,(uint8_t)(p->data[4]+4));
        if(p->data[len-2]==(uint8_t)crc && p->data[len-1]==(uint8_t)(crc>>8)) {
            p->used=0; return 1;
        }
discard:
        --p->used; memmove(p->data,p->data+1,p->used); error=-1;
    }
    return error;
}

/* ==================== 命令处理与反馈：ASCII、二进制、序号门禁及遥测 ==================== */
static HostCodec codec;
static char line[96];
/* binary_mode 决定回报格式；have_seq/last_seq 属于主机请求窗口，telemetry_seq 独立递增。 */
static uint8_t line_used, line_discard, binary_mode, have_seq, last_seq, telemetry_seq;
static uint16_t reported_fault;
static uint32_t line_time;
/* 主机 payload 整数为小端；电机帧为大端，由本模块和电机帧构造器分别转换。 */
static uint16_t le16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static int32_t le32(const uint8_t *p)
{
    uint32_t u=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
    return u<=INT32_MAX ? (int32_t)u : (int32_t)(-1-(int64_t)(UINT32_MAX-u));
}
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v)
{ uint8_t i; for(i=0;i<4;i++) p[i]=(uint8_t)(v>>(i*8)); }
static void put64(uint8_t *p, int64_t v)
{ uint8_t i; for(i=0;i<8;i++) p[i]=(uint8_t)((uint64_t)v>>(i*8)); }
static void send_binary(uint8_t type, uint8_t seq, const uint8_t *p, uint8_t n)
{
    uint8_t frame[CAR_HOST_MAX_PAYLOAD+8];
    uint8_t len=HostCodec_Encode(frame,type,seq,p,n);
    if(len) (void)CarUart_SendHost(frame,len);
}
static void send_text(const char *s)
{ (void)CarUart_SendHost((const uint8_t *)s,(uint8_t)strlen(s)); }
static void text_result(MotorResult r)
{
    static const char *const replies[]={"OK QUEUED\r\n","ERR ARGUMENT\r\n",
        "ERR NOT_READY\r\n","ERR SAFETY_LOCK\r\n","ERR BUSY\r\n"};
    send_text(replies[r<=MOTOR_BUSY ? r : MOTOR_BAD_ARGUMENT]);
}
static void text_status(uint8_t index, uint32_t now)
{
    const MotorFeedback *f=CarMotor_Get(index);
    char s[72];
    if(!f) { send_text("ERR ARGUMENT\r\n"); return; }
    /* 每行控制在单帧容量内；ASCII 不打印 64 位位置，完整位置用二进制反馈。 */
    (void)snprintf(s,sizeof(s),"DATA MOTOR %u STATE %u RPM %d FLAGS %u FAULT %u\r\n",
                  f->addr,(unsigned)f->state,f->speed_rpm,f->flags,CarControl_Faults());
    send_text(s);
    (void)snprintf(s,sizeof(s),"DATA FEEDBACK %u ONLINE %u CONFIG %u AGE %lu\r\n",
                  f->addr,f->online,f->config_ok,(unsigned long)(now-f->speed_time));
    send_text(s);
}
static void text_diag_rx(const char *kind, uint8_t addr, const uint8_t *rx, uint8_t n)
{
    char s[72];
    uint8_t i;
    int used=snprintf(s,sizeof(s),"DATA %s %u RX",kind,addr);
    if(!n) used+=snprintf(s+used,sizeof(s)-used," NONE");
    for(i=0;i<n;i++) used+=snprintf(s+used,sizeof(s)-used," %02X",(unsigned)rx[i]);
    (void)snprintf(s+used,sizeof(s)-used,"\r\n");
    send_text(s);
}
static void text_diagnostic(uint8_t index)
{
    const MotorFeedback *f=CarMotor_Get(index);
    const MotorDiagnostic *d=CarMotor_GetDiagnostic(index);
    char s[72];
    if(!f || !d) { send_text("ERR ARGUMENT\r\n"); return; }
    (void)snprintf(s,sizeof(s),"DATA DIAG %u INIT %u TIMEOUT 0x%02X REJECT 0x%02X\r\n",
                  (unsigned)f->addr,(unsigned)d->init_step,
                  (unsigned)d->timeout_code,(unsigned)d->reject_code);
    send_text(s);
    text_diag_rx("TIMEOUT",f->addr,d->timeout_rx,d->timeout_len);
    text_diag_rx("REJECT",f->addr,d->reject_rx,d->reject_len);
}
/* 严格十进制转换：拒绝多余字符、溢出和越界，避免截断后接受错误目标。 */
static uint8_t number(const char *s, int32_t min, int32_t max, int32_t *result)
{
    uint64_t v=0;
    uint8_t neg=0, digits=0;
    int64_t signed_value;
    if(*s=='-' || *s=='+') { neg=(*s=='-'); ++s; }
    while(*s) {
        if(*s<'0' || *s>'9') return 0;
        v=v*10U+(uint8_t)(*s-'0');
        if(v>2147483648ULL) return 0;
        ++digits; ++s;
    }
    if(!digits) return 0;
    signed_value=neg ? -(int64_t)v : (int64_t)v;
    if(signed_value<min || signed_value>max) return 0;
    *result=(int32_t)signed_value; return 1;
}
/* 接收大写文本命令，数字填十进制整数并以换行结束，如 WHEELS 30 30 10\r\n；now 填 HAL_GetTick()。 */
static void ascii_command(uint32_t now)
{
    char *words[9], *p=line;
    uint8_t n=0;
    int32_t a=0, b=0, c=0, d=0, e=2;
    MotorResult r=MOTOR_BAD_ARGUMENT;
    while(*p) {
        while(*p==' ' || *p=='\t') ++p;
        if(!*p) break;
        if(n==9) { send_text("ERR ARGUMENT\r\n"); return; }
        words[n++]=p;
        while(*p && *p!=' ' && *p!='\t') ++p;
        if(*p) *p++=0;
    }
    if(!n) return;
    if(n==1 && !strcmp(words[0],"PING")) { send_text("OK PONG\r\n"); return; }
    if(n==1 && !strcmp(words[0],"HEARTBEAT")) {
        CarControl_Heartbeat(now); send_text("OK HEARTBEAT\r\n"); return;
    }
    if(n==1 && !strcmp(words[0],"ESTOP")) {
        CarControl_Trip(FAULT_ESTOP); CarMotor_StopAll(); send_text("OK ESTOP\r\n"); return;
    }
    if(n==1 && !strcmp(words[0],"STOP")) { CarMotor_StopAll(); send_text("OK STOP\r\n"); return; }
    if(n==1 && !strcmp(words[0],"CLEAR_FAULT")) {
        send_text(CarMotor_ClearFault(now) ? "OK RECOVERY\r\n" : "ERR ESTOP_ACTIVE\r\n"); return;
    }
    if(n==1 && !strcmp(words[0],"STATUS")) {
        if(CarMotor_Get(0)) text_status(0,now);
        if(CarMotor_Get(1)) text_status(1,now);
        return;
    }
    if(n==4 && !strcmp(words[0],"WHEELS") &&
       number(words[1],-CAR_MAX_RPM,CAR_MAX_RPM,&a) &&
       number(words[2],-CAR_MAX_RPM,CAR_MAX_RPM,&b) && number(words[3],0,255,&c))
        r=CarMotor_Wheels((int16_t)a,(int16_t)b,(uint8_t)c);
    else if(n>=3 && !strcmp(words[0],"MOTOR") && number(words[2],1,255,&a)) {
        if((n==3 || n==4) && !strcmp(words[1],"ENABLE")) {
            b=1;
            if(n==3 || number(words[3],0,1,&b)) r=CarMotor_Enable((uint8_t)a,(uint8_t)b);
        } else if(n==5 && !strcmp(words[1],"VEL") &&
                  number(words[3],-CAR_MAX_RPM,CAR_MAX_RPM,&b) && number(words[4],0,255,&c))
            r=CarMotor_Velocity((uint8_t)a,(int16_t)b,(uint8_t)c);
        else if((n==6 || n==7) && !strcmp(words[1],"POS") &&
                number(words[3],INT32_MIN,INT32_MAX,&b) && number(words[4],1,CAR_MAX_RPM,&c) &&
                number(words[5],0,255,&d) && (n==6 || number(words[6],0,2,&e)))
            r=CarMotor_Position((uint8_t)a,b,(uint16_t)c,(uint8_t)d,(uint8_t)e);
        else if(n==3 && !strcmp(words[1],"STOP") &&
                ((a==CAR_LEFT_ADDR && CarMotor_Get(0)) ||
                 (a==CAR_RIGHT_ADDR && CarMotor_Get(1)))) {
            CarMotor_StopAll(); r=MOTOR_OK;
        } else if(n==3 && !strcmp(words[1],"DIAG")) {
            if(a==CAR_LEFT_ADDR) text_diagnostic(0);
            else if(a==CAR_RIGHT_ADDR) text_diagnostic(1);
            else send_text("ERR ARGUMENT\r\n");
            return;
        } else if(n==3 && !strcmp(words[1],"STATUS")) {
            if(a==CAR_LEFT_ADDR) text_status(0,now);
            else if(a==CAR_RIGHT_ADDR) text_status(1,now);
            else send_text("ERR ARGUMENT\r\n");
            return;
        }
    }
    if(r==MOTOR_OK) CarControl_Heartbeat(now);
    text_result(r);
}
/* 二进制速度数据：左右RPM各2字节，低字节在前，加速度占1字节；now 填 HAL_GetTick()。
 * 完整格式见 docs/上位机协议_v1.md；停止和急停指令优先处理。 */
static void binary_command(uint32_t now)
{
    const uint8_t *f=codec.data, *p=f+6;
    uint8_t type=f[3], len=f[4], seq=f[5], reply[2]={type,MOTOR_BAD_ARGUMENT};
    MotorResult result=MOTOR_BAD_ARGUMENT;
    /* 模 256 序号只接受前进 1~127（包括 255->0）；重复/倒序返回 result=5。
       STOP/ESTOP/HELLO 例外；重复普通命令不能刷新保活，位置也不能被重复执行。 */
    if(type!=3 && type!=4 && type!=0x12 && have_seq && (uint8_t)(seq-last_seq)>=128U) {
        reply[1]=5; send_binary(0x83,seq,reply,2); return;
    }
    if(type!=3 && type!=4 && type!=0x12 && have_seq && seq==last_seq) {
        reply[1]=5; send_binary(0x83,seq,reply,2); return;
    }
    switch(type) {
    case 0x01:
        if(len==5) result=CarMotor_Wheels((int16_t)le16(p),(int16_t)le16(p+2),p[4]);
        break;
    case 0x02:
        if(len==12) result=CarMotor_Positions(le32(p),le32(p+4),le16(p+8),p[10],p[11]);
        break;
    case 0x03:
        if(!len) { CarMotor_StopAll(); result=MOTOR_OK; }
        break;
    case 0x04:
        if(!len) { CarControl_Trip(FAULT_ESTOP); CarMotor_StopAll(); result=MOTOR_OK; }
        break;
    case 0x05:
        if(len==2) result=CarMotor_Enable(p[0],p[1]);
        break;
    case 0x06:
        if(!len) result=CarMotor_ClearFault(now) ? MOTOR_OK : MOTOR_SAFETY_LOCK;
        break;
    case 0x10:
        if(!len) { CarControl_Heartbeat(now); result=MOTOR_OK; }
        break;
    case 0x11:
        if(!len) { CarUart_HostReport(now); result=MOTOR_OK; }
        break;
    case 0x12:
        if(!len) {
            /* 新会话：先停双轮、请求失能，再重设序号窗口；保留故障锁存。
               HELLO 成功后仍要核验状态，重新使能并下发新目标。 */
            CarMotor_StopAll();
            if(CarMotor_Get(0)) (void)CarMotor_Enable(CAR_LEFT_ADDR,0);
            if(CarMotor_Get(1)) (void)CarMotor_Enable(CAR_RIGHT_ADDR,0);
            have_seq=0; result=MOTOR_OK;
        }
        break;
    default: break;
    }
    if(result==MOTOR_OK) {
        if(type==1 || type==2 || type==5) CarControl_Heartbeat(now);
        /* STOP/ESTOP 不把普通命令的序号窗口向后移。 */
        if(type!=3 && type!=4) { last_seq=seq; have_seq=1; }
    }
    reply[1]=(uint8_t)result; send_binary(0x83,seq,reply,2);
}
void CarUart_HostInit(void)
{
    memset(&codec,0,sizeof(codec));
    line_used=line_discard=binary_mode=have_seq=telemetry_seq=0; reported_fault=0; line_time=0;
}
/* now 填 HAL_GetTick()；从 USART2 接收上位机文本或 AA 55 开头的二进制指令。
 * 每轮只处理有限字节，错误文本丢弃到换行。 */
void CarUart_HostPoll(uint32_t now)
{
    uint8_t b;
    uint16_t budget=CAR_RX_RING_SIZE;
    int r;
    if(CarUart_TakeRxFault(UART_HOST)) {
        CarControl_Trip(FAULT_UART); line_used=0; line_discard=1; codec.used=0;
        CarUart_FlushRx(UART_HOST);
    }
    while(budget-- && CarUart_Read(UART_HOST,&b)) {
        if((uint32_t)(now-line_time)>CAR_RX_FRAME_TIMEOUT_MS) {
            /* 人工慢速输入 ASCII 允许保留；只丢弃超时二进制半帧。 */
            if(codec.used) codec.used=0;
        }
        line_time=now;
        if(codec.used || b==0xAA) {
            line_used=0;
            r=HostCodec_Feed(&codec,b,now);
            if(r==1) { binary_mode=1; line_discard=0; binary_command(now); }
            else if(r<0 && !codec.used) line_discard=1;
            continue;
        }
        if(b=='\r' || b=='\n') {
            if(!line_discard && line_used) {
                line[line_used]=0; binary_mode=0; ascii_command(now);
            } else if(line_discard && line_used) send_text("ERR FRAME\r\n");
            line_used=line_discard=0;
        } else if(b>=32 && b<=126) {
            if(line_used<sizeof(line)-1) line[line_used++]=(char)b;
            else line_discard=1;
        } else if(b!='\t') line_discard=1;
        else if(line_used<sizeof(line)-1) line[line_used++]=(char)b;
    }
}
/* ==================== 反馈发送：状态、真实采样值与故障变化 ==================== */
void CarUart_HostReport(uint32_t now)
{
    uint8_t p[48], n=0, i;
    uint16_t fault=CarControl_Faults();
    if(!binary_mode) {
        if(fault!=reported_fault) {
            char msg[32]; (void)snprintf(msg,sizeof(msg),"EVENT FAULT %u\r\n",fault); send_text(msg);
        }
        reported_fault=fault; return;
    }
/* 80 状态 -> 81 反馈 -> 82 故障变化；反馈取缓存和独立采样时刻，不把回报时刻当采样时刻。 */
    put32(p,now); put16(p+4,fault); n=6;
    for(i=0;i<CAR_MOTOR_COUNT;i++) {
        const MotorFeedback *m=CarMotor_Get(i);
        p[n++]=m ? (uint8_t)m->state : MOTOR_DISABLED;
        p[n++]=m ? m->flags : 0;
        p[n++]=m ? m->online : 0;
        p[n++]=m ? m->config_ok : 0;
    }
    send_binary(0x80,telemetry_seq++,p,n);
    n=0; put32(p,now); n=4;
    for(i=0;i<CAR_MOTOR_COUNT;i++) {
        const MotorFeedback *m=CarMotor_Get(i);
        put16(p+n,m ? (uint16_t)m->speed_rpm : 0); n+=2;
        put64(p+n,m ? m->position_ticks : 0); n+=8;
        put32(p+n,m ? m->speed_time : 0); n+=4;
        put32(p+n,m ? m->position_time : 0); n+=4;
    }
    send_binary(0x81,telemetry_seq++,p,n);
    if(fault!=reported_fault) { put16(p,fault); send_binary(0x82,telemetry_seq++,p,2); }
    reported_fault=fault;
}
