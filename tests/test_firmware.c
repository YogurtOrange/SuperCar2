#include "car_control.h"
#include "car_uart.h"
#include "car_motor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef uart[2]={{0},{1}};
static uint32_t tick;
static uint8_t *rx[2], *tx_ptr[2], tx_copy[2][80];
static uint16_t tx_length[2];
static uint32_t tx_done[2];
static uint8_t tx_active[2], estop, connected[2], driver_enabled[2], driver_fault[2];
static uint8_t reply[48], reply_len;
static uint32_t reply_at;
static int16_t driver_speed[2];
static uint32_t position_commands, velocity_commands, stop_commands;
static uint32_t zero_stop_commands, last_motor_end;
static uint8_t have_motor_end;
static uint8_t motor_frames[1024][16], motor_lengths[1024];
static uint32_t motor_times[1024], motor_frame_count;
static uint8_t bad_config, drop_position_ack;
static char host_output[16384];
static uint32_t host_output_len;
static uint32_t invalid_buffer_mutations;
TestIwdg test_iwdg;
TestDbgMcu test_dbgmcu;
uint8_t test_gpio_b_clock, test_gpio_c_clock;
static GPIO_InitTypeDef input_gpio, led_gpio;
static GPIO_PinState led_state;

void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    if(port==GPIOB) { assert(test_gpio_b_clock); input_gpio=*gpio; }
    else { assert(port==GPIOC && test_gpio_c_clock); led_gpio=*gpio; }
}
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
    assert(port==GPIOC && pin==GPIO_PIN_13 && test_gpio_c_clock);
    led_state=state;
}

uint32_t HAL_GetTick(void) { return tick; }
void Error_Handler(void) { assert(0); }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{ (void)port; return pin==GPIO_PIN_12 && estop ? GPIO_PIN_RESET : GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ assert(n==1); rx[u->id]=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ rx[u->id]=0; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ tx_active[u->id]=0; return HAL_OK; }
static void inject(uint8_t port, const uint8_t *p, uint32_t n)
{
    uint32_t i;
    for(i=0;i<n;i++) { assert(rx[port]); *rx[port]=p[i]; HAL_UART_RxCpltCallback(&uart[port]); }
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *f, uint16_t n)
{
    uint8_t i, code;
    uint16_t magnitude;
    if(tx_active[u->id]) return HAL_BUSY;
    tx_ptr[u->id]=f; memcpy(tx_copy[u->id],f,n); tx_length[u->id]=n;
    tx_done[u->id]=tick+(n+10U)/11U; tx_active[u->id]=1;
    if(u->id) return HAL_OK;
    assert(f[0]==CAR_LEFT_ADDR || f[0]==CAR_RIGHT_ADDR);
    i=(uint8_t)(f[0]==CAR_LEFT_ADDR ? 0 : 1); code=f[1];
    if(have_motor_end) assert((uint32_t)(tick-last_motor_end)>=CAR_MOTOR_FRAME_GAP_MS);
    if(motor_frame_count<1024) {
        memcpy(motor_frames[motor_frame_count],f,n);
        motor_lengths[motor_frame_count]=(uint8_t)n;
        motor_times[motor_frame_count++]=tick;
    }
    if(code==0xFE) stop_commands++;
    if(code==0xFD) position_commands++;
    if(code==0xF6) {
        if(f[3]==0 && f[4]==0 && f[5]==0) zero_stop_commands++;
        else velocity_commands++;
    }
    if(!connected[i]) return HAL_OK;
    reply[0]=f[0]; reply[1]=code; reply[2]=2; reply[3]=0x6B; reply_len=4;
    if(code==0x1A) reply[2]=6;
    if(code==0x42) {
        memset(reply+2,0,31); reply[2]=0x21;
        /* 仍是 Emm 回包，错误接口参数用于验证配置门禁拒绝使能。 */
        reply[3]=0x15; reply[6]=bad_config ? 3 : 2;
        reply[18]=5; reply[20]=f[0]; reply[22]=1;
        reply_len=33; reply[reply_len-1]=0x6B;
    }
    if(code==0xF3) driver_enabled[i]=f[3];
    if(code==0xFE) driver_speed[i]=0;
    if(code==0x0E) driver_fault[i]=0;
    if(code==0xF6) {
        magnitude=(uint16_t)(((uint16_t)f[3]<<8)|f[4]);
        driver_speed[i]=(int16_t)(f[2] ? -(int32_t)magnitude : magnitude);
    }
    if(code==0x3A) reply[2]=(uint8_t)(driver_enabled[i]|2U|driver_fault[i]);
    if(code==0x35) {
        magnitude=(uint16_t)(driver_speed[i]<0 ? -driver_speed[i] : driver_speed[i]);
        reply[2]=(driver_speed[i]<0); reply[3]=(uint8_t)(magnitude>>8);
        reply[4]=(uint8_t)magnitude; reply[5]=0x6B; reply_len=6;
    }
    if(code==0x36) {
        reply[2]=i; reply[3]=0; reply[4]=1; reply[5]=0; reply[6]=0;
        reply[7]=0x6B; reply_len=8;
    }
    if(code==0xFD && drop_position_ack) reply_len=0;
    reply_at=tx_done[0]+1;
    return HAL_OK;
}
static void advance(uint32_t n)
{
    uint32_t end=tick+n;
    uint8_t i;
    while(tick<end) {
        ++tick;
        for(i=0;i<2;i++) if(tx_active[i] && tick>=tx_done[i]) {
            if(memcmp(tx_ptr[i],tx_copy[i],tx_length[i])) invalid_buffer_mutations++;
            if(i==1 && host_output_len+tx_length[i]<sizeof(host_output)) {
                memcpy(host_output+host_output_len,tx_ptr[i],tx_length[i]);
                host_output_len+=tx_length[i]; host_output[host_output_len]=0;
            }
            if(i==0) { last_motor_end=tick; have_motor_end=1; }
            tx_active[i]=0; HAL_UART_TxCpltCallback(&uart[i]);
        }
        if(reply_len && tick>=reply_at) { inject(0,reply,reply_len); reply_len=0; }
        CarControl_Poll();
    }
}
static void ascii(const char *s)
{ inject(1,(const uint8_t *)s,(uint32_t)strlen(s)); CarControl_Poll(); }
static void binary(uint8_t type, uint8_t seq, const uint8_t *p, uint8_t n, uint8_t corrupt)
{
    uint8_t f[80], len=HostCodec_Encode(f,type,seq,p,n);
    if(corrupt) f[len-1]^=1;
    inject(1,f,len); CarControl_Poll();
}
static void reset(void)
{
    tick=0; memset(rx,0,sizeof(rx)); memset(tx_active,0,sizeof(tx_active));
    memset(driver_enabled,0,sizeof(driver_enabled)); memset(driver_speed,0,sizeof(driver_speed));
    memset(driver_fault,0,sizeof(driver_fault));
    connected[0]=connected[1]=1; estop=bad_config=drop_position_ack=reply_len=0;
    position_commands=velocity_commands=stop_commands=zero_stop_commands=host_output_len=0;
    motor_frame_count=0; have_motor_end=0; last_motor_end=0;
    invalid_buffer_mutations=0; host_output[0]=0;
    memset(&test_iwdg,0,sizeof(test_iwdg));
    memset(&test_dbgmcu,0,sizeof(test_dbgmcu));
    test_gpio_b_clock=test_gpio_c_clock=0;
    CarControl_Init(&uart[0],&uart[1]); advance(150);
}
static void test_board_control(void)
{
    reset();
    assert(input_gpio.Pin==(GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14));
    assert(input_gpio.Mode==GPIO_MODE_INPUT && input_gpio.Pull==GPIO_PULLUP);
    assert(led_gpio.Pin==GPIO_PIN_13 && led_gpio.Mode==GPIO_MODE_OUTPUT_PP);
    assert(led_gpio.Pull==GPIO_NOPULL && led_gpio.Speed==GPIO_SPEED_FREQ_LOW);
    assert((test_dbgmcu.CR&DBGMCU_CR_DBG_IWDG_STOP)!=0);
    assert(test_iwdg.PR==4 && test_iwdg.RLR==624 && test_iwdg.KR==0xAAAA);
    assert(led_state==GPIO_PIN_RESET);
    test_iwdg.KR=0; advance(350);
    assert(test_iwdg.KR==0xAAAA && led_state==GPIO_PIN_SET);
    estop=1; CarControl_Poll();
    assert(CarControl_Faults()&FAULT_ESTOP);
    advance(100); assert(led_state==GPIO_PIN_RESET);
}
static void enabled(void)
{
    reset(); assert(!CarControl_Faults());
    assert(CarMotor_Get(0)->config_ok && CarMotor_Get(1)->config_ok);
    assert(!driver_enabled[0] && !driver_enabled[1]);
    ascii("MOTOR ENABLE 1\r\n"); advance(30);
    ascii("MOTOR ENABLE 2\r\n"); advance(30);
    assert(driver_enabled[0] && driver_enabled[1]);
}
static void test_vectors(void)
{
    const uint8_t vel[]={1,0xF6,0,0,30,10,0,0x6B};
    const uint8_t pos[]={1,0x36,0,0,1,0x6B,0,0x6B};
    uint8_t f[80], n, i;
    X42S_Parser xp={0}; HostCodec hp={0};
    assert(HostCodec_Crc16((const uint8_t *)"123456789",9)==0x4B37);
    {
        const uint8_t payload[]={30,0,30,0,10};
        const uint8_t expected[]={0xAA,0x55,1,1,5,1,30,0,30,0,10,0xDA,0x7E};
        assert(HostCodec_Encode(f,1,1,payload,5)==13 && !memcmp(f,expected,13));
    }
    assert(X42S_EmmVelocity(f,1,30,10)==8 && !memcmp(f,vel,8));
    assert(X42S_EmmPosition(f,2,1,3200,30,10,2)==13);
    assert(f[8]==0x0C && f[9]==0x80 && f[10]==2);
    {
        uint8_t config[33]={1,0x42,0x21,0x15};
        uint8_t malformed[34]={1,0x42,0x22};
        config[10]=0x6B; config[32]=0x6B; malformed[33]=0x6B;
        /* 非 Emm 长度不能组成配置帧；丢弃后应能解析下一条合法 Emm 回包。 */
        for(i=0;i<sizeof(malformed);i++) assert(!X42S_ParserFeed(&xp,malformed[i],1));
        for(i=0;i<sizeof(config);i++)
            assert(X42S_ParserFeed(&xp,config[i],2)==(i==32 ? 33 : 0));
        assert(!memcmp(xp.data,config,sizeof(config)));
    }
    for(i=0;i<8;i++) assert(X42S_ParserFeed(&xp,pos[i],1)==(i==7 ? 8 : 0));
    assert(X42S_SignedPosition(pos)==92928); /* Interior 6B isn't a delimiter. */
    n=HostCodec_Encode(f,1,20,vel,8);
    for(i=0;i<n;i++) assert(HostCodec_Feed(&hp,f[i],1)==(i==n-1 ? 1 : 0));
    f[n-1]^=1;
    for(i=0;i<n;i++) assert(HostCodec_Feed(&hp,f[i],2)!=1);
    f[n-1]^=1;
    for(i=0;i<n;i++) if(i==n-1) assert(HostCodec_Feed(&hp,f[i],3)==1);
    else (void)HostCodec_Feed(&hp,f[i],3);
    memset(&hp,0,sizeof(hp));
    assert(HostCodec_Feed(&hp,0xAA,1)==0);
    assert(HostCodec_Feed(&hp,0x55,2)==0);
    assert(HostCodec_Feed(&hp,0xAA,40)==0); /* stale prefix discarded */
    for(i=1;i<n;i++) if(i==n-1) assert(HostCodec_Feed(&hp,f[i],40)==1);
    else (void)HostCodec_Feed(&hp,f[i],40);
}
static void test_watchdog_and_recovery(void)
{
    uint32_t moves, j;
    enabled(); ascii("WHEELS 30 30 10\n"); advance(100);
    assert(driver_speed[0]==30 && driver_speed[1]==-30);
    assert(CarMotor_Get(0)->speed_rpm==30 && CarMotor_Get(1)->speed_rpm==30);
    {
        uint8_t saw_left=0, saw_right=0;
        for(j=0;j<motor_frame_count;j++) {
            if(motor_lengths[j]>=8 && motor_frames[j][1]==0xF6 &&
               motor_frames[j][3]==0 && motor_frames[j][4]==30) {
                if(motor_frames[j][0]==CAR_LEFT_ADDR && motor_frames[j][2]==0) saw_left=1;
                if(motor_frames[j][0]==CAR_RIGHT_ADDR && motor_frames[j][2]==1) saw_right=1;
            }
            if(j>0 && motor_times[j]>motor_times[j-1])
                assert(motor_times[j]-motor_times[j-1]>=CAR_MOTOR_FRAME_GAP_MS);
        }
        assert(saw_left && saw_right);
    }
    advance(150); ascii("PING\n"); advance(80);
    assert(CarControl_Faults()&FAULT_HOST_TIMEOUT);
    assert(driver_speed[0]==0 && driver_speed[1]==0);
    moves=velocity_commands; ascii("HEARTBEAT\nWHEELS 30 30 10\n"); advance(50);
    assert(velocity_commands==moves);
    ascii("CLEAR_FAULT\n"); advance(150);
    assert(!CarControl_Faults()); assert(!driver_enabled[0] && !driver_enabled[1]);
    ascii("WHEELS 30 30 10\n"); advance(20); assert(velocity_commands==moves);
    assert(!invalid_buffer_mutations);
}
static void test_preempt_and_faults(void)
{
    uint32_t moves;
    enabled(); moves=velocity_commands;
    ascii("WHEELS 30 30 10\nESTOP\n"); advance(50);
    assert(CarControl_Faults()&FAULT_ESTOP); assert(velocity_commands==moves);
    assert(driver_speed[0]==0 && driver_speed[1]==0);
    estop=1; ascii("CLEAR_FAULT\n"); advance(50); assert(CarControl_Faults());
    estop=0; ascii("CLEAR_FAULT\n"); advance(150); assert(!CarControl_Faults());
    enabled(); ascii("WHEELS 30 30 10\n"); advance(50);
    driver_fault[1]=8; advance(100); assert(CarControl_Faults()&FAULT_DRIVER);
    assert(!driver_speed[0] && !driver_speed[1]);
    ascii("CLEAR_FAULT\n"); advance(150); assert(!CarControl_Faults());
    enabled(); connected[1]=0; ascii("WHEELS 30 30 10\n"); advance(100);
    assert(CarControl_Faults()&FAULT_MOTOR_COMM); assert(!driver_speed[0]);
    enabled(); HAL_UART_ErrorCallback(&uart[1]); advance(20);
    assert(CarControl_Faults()&FAULT_UART);
    enabled();
    { uint8_t spam[300]; memset(spam,'A',sizeof(spam)); inject(1,spam,sizeof(spam)); }
    advance(20); assert(CarControl_Faults()&FAULT_UART);
}
static void test_positions_and_binary(void)
{
    uint8_t p[12]={0x80,0x0C,0,0,0x80,0x0C,0,0,30,0,10,2};
    uint32_t count_before;
    enabled(); binary(2,250,p,12,0); advance(50); assert(position_commands==2);
    count_before=position_commands;
    binary(2,250,p,12,0); advance(50); assert(position_commands==count_before);
    binary(0x10,251,0,0,0); binary(0x10,252,0,0,0);
    binary(0x10,253,0,0,0); binary(0x10,254,0,0,0);
    binary(0x10,255,0,0,0); binary(0x10,0,0,0,0);
    assert(!CarControl_Faults());
    binary(2,1,p,12,1); advance(30); assert(position_commands==count_before);
    binary(2,1,p,11,0); advance(20); assert(position_commands==count_before);
    binary(4,0,0,0,0); advance(20); assert(CarControl_Faults()&FAULT_ESTOP);
    assert(!invalid_buffer_mutations);
    enabled(); drop_position_ack=1; ascii("MOTOR POS 1 3200 30 10\n"); advance(100);
    assert(position_commands==1 && CarControl_Faults());
    enabled(); ascii("MOTOR POS 1 99999999999999999 30 10\n"); advance(20);
    assert(!position_commands);
    enabled(); ascii("WHEELS 61 30 10\n"); advance(20); assert(!velocity_commands);
    /* Flood host TX while HAL owns a prior buffer; queued data must not overwrite it. */
    reset();
    for(count_before=0;count_before<100;count_before++) {
        ascii("STATUS\n"); advance(1);
    }
    advance(100); assert(!invalid_buffer_mutations);
}
static void test_configuration(void)
{
    reset(); bad_config=1;
    /* 重新启动，用错误的 Emm 接口配置确认门禁仍锁存 FAULT_CONFIG。 */
    CarControl_Init(&uart[0],&uart[1]); advance(100);
    assert(CarControl_Faults()&FAULT_CONFIG);
    ascii("MOTOR ENABLE 1\n"); advance(20); assert(!driver_enabled[0]);
}
static void test_session_and_tx_timeout(void)
{
    uint8_t p[5]={30,0,30,0,10};
    uint32_t moves, i;
    enabled(); binary(1,100,p,5,0); advance(50);
    moves=velocity_commands;
    binary(0x12,0,0,0,0); advance(100);
    assert(!driver_enabled[0] && !driver_enabled[1]);
    binary(1,1,p,5,0); advance(20); assert(velocity_commands==moves);
    enabled(); ascii("WHEELS 30 30 10\n");
    for(i=0;i<100 && !tx_active[0];i++) advance(1);
    assert(tx_active[0]); tx_done[0]=tick+1000; reply_len=0;
    advance(50); assert(CarControl_Faults()&FAULT_UART);
    assert(stop_commands>=4 && zero_stop_commands>=4);
}
int main(void)
{
    test_board_control(); puts("PASS real board GPIO, LED and watchdog register setup/feed");
    test_vectors(); puts("PASS frame vectors, CRC and parser recovery");
    test_watchdog_and_recovery(); puts("PASS host watchdog, stop and explicit recovery");
    test_preempt_and_faults(); puts("PASS ESTOP preemption, physical input, driver/communication/UART faults");
    test_positions_and_binary(); puts("PASS position exactly once, binary replay/CRC/length, bounds, TX buffer lifetime");
    test_configuration(); puts("PASS firmware configuration gate");
    test_session_and_tx_timeout(); puts("PASS session reset and stalled UART TX watchdog");
    return 0;
}
