#include <iocc2530.h>
#include "sys_init.h"
#include "uart.h"
#include "info.h"
#include "hal_mcu.h"
#include "hal_assert.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include "hal_int.h"      // P0-5：halIntLock/halIntUnlock 保护 cmdBuf 复制
#include <stdio.h>
#include "string.h"

#define RF_CHANNEL            24                                // 2.4 GHz RF channel 
#define PAN_ID                0x1410                            // PAN ID
#define MY_ADDR               0x4206                            // 本机短地址（固定，传感器通过信标发现）

static basicRfCfg_t basicRfConfig;

// === RF 接收缓冲（全局，避免 128 字节数组占用 xdata 栈导致栈溢出） ===
char gRxData[128];

// === ISR 专用全局变量（避免 __interrupt 函数局部变量在 xdata 栈上分配/释放不对称导致栈泄漏） ===
volatile char gUartRxBuf;    // UART0_ISR 专用

// === 串口指令环形缓冲 ===
#define CMD_BUF_SIZE 64
char cmdBuf[CMD_BUF_SIZE];
volatile uint8 cmdIdx = 0;
volatile uint8 cmdReady = 0;
volatile uint8 cmdOverflow = 0;   // P1-10：溢出标志，主循环检测后打印提示

// === 广播发送（关 ACK，发完恢复） ===
// P0-5：入口关中断复制 cmdBuf 到本地缓冲区，避免 ISR 在 strlen/发送期间覆盖
void broadcastCmd(char* cmd) {
    char localBuf[CMD_BUF_SIZE];
    uint16 intState;
    uint8 len;

    intState = halIntLock();                 // 关中断
    len = strlen(cmd);
    if (len >= CMD_BUF_SIZE) len = CMD_BUF_SIZE - 1;
    memcpy(localBuf, cmd, len);
    localBuf[len] = '\0';
    halIntUnlock(intState);                  // 恢复中断

    basicRfReceiveOff();
    basicRfConfig.ackRequest = FALSE;
    basicRfSendPacket(0xFFFF, (uint8*)localBuf, strlen(localBuf) + 1);
    basicRfConfig.ackRequest = TRUE;
    basicRfReceiveOn();
    D6 = 0;                          // D6 短闪 50ms 表示正在广播数据
    halMcuWaitMs(50);
    D6 = 1;
    // 静默广播：不打印确认，等待传感器 P2P 回传 {CMD=SUCCESS,...}
}

// === 串口中断：@/! 本地控制，{...} 指令累积到 } 后设标志 ===
#pragma vector = URX0_VECTOR
__interrupt void UART0_ISR(void)
{
    URX0IF = 0;
    gUartRxBuf = U0DBUF;
    if (gUartRxBuf == '@') { D7 = 0; return; }
    if (gUartRxBuf == '!') { D7 = 1; return; }
    if (gUartRxBuf == '{' || cmdIdx > 0) {
        if (cmdIdx < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdIdx++] = gUartRxBuf;
            if (gUartRxBuf == '}') { cmdBuf[cmdIdx] = '\0'; cmdReady = 1; cmdIdx = 0; }
        } else {
            cmdIdx = 0;            // 溢出保护
            cmdOverflow = 1;      // P1-10：置溢出标志，主循环打印提示
        }
    }
}

/* 射频模块接收数据函数 — 纯透传 + 信标广播 */
void rfRecvData(void)
{
    int rlen;
    unsigned int loopCounter = 0;   // P1-12：基于循环计数，容忍 broadcastCmd 50ms 等耗时漂移
    
    basicRfReceiveOn();
    printf("{data=recv node start up...}\r\n");
    
    while (TRUE) {
        // 喂狗：先写 0xA 到 WDCTL[7:4]，再写 0x5，重置倒计时
        // CPU 跑飞后不再喂狗 → 1 秒后硬件自动复位
        WDCTL = (WDCTL & 0x0F) | 0xA0;
        WDCTL = (WDCTL & 0x0F) | 0x50;
        
        // 防御：周期性恢复全局中断 + UART RX 中断使能
        // 防止 idata 栈溢出破坏 HAL_INT_LOCK 保存值导致 EA/URX0IE 被意外关闭后 @ 指令无响应
        EA = 1;
        URX0IE = 1;
        // 处理串口指令（广播透传）
        if (cmdReady) { cmdReady = 0; broadcastCmd(cmdBuf); }
        // P1-10：溢出提示（ISR 检测到缓冲区溢出时置标志）
        if (cmdOverflow) { cmdOverflow = 0; printf("{err=cmd overflow}\r\n"); }
        
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive((uint8*)gRxData, sizeof(gRxData), NULL);
            if (rlen > 0 && rlen < sizeof(gRxData)) {
                gRxData[rlen] = 0;
                // 用 Uart_Send_String 替代 printf，避免 printf 深层调用链消耗 idata 栈
                Uart_Send_String(gRxData);
                Uart_Send_String("\r\n");
            }
        }
        
        // 信标广播（约每 2 秒，~200 轮 × 10ms；broadcastCmd 50ms 额外开销使实际周期略长，可接受）
        loopCounter++;
        if (loopCounter >= 200) {
            loopCounter = 0;
            broadcastCmd("{TYPE=RECV}");
        }
        
        halMcuWaitMs(10);
    }
}

void main(void)
{
    xtal_init();
    led_init();
    uart0_init(0x00, 0x00);
    lcd_dis();
    
    // 看门狗初始化：1 秒超时，CPU 跑飞后自动复位自愈
    // WDCTL[3:2]=00→1s, [1]=1→启用, [0]=0→复位模式
    WDCTL = 0x02;       // 启用看门狗，1 秒超时，超时后硬件复位
    
    if (FAILED == halRfInit()) {
        HAL_ASSERT(FALSE);
    }
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;
    basicRfConfig.ackRequest = TRUE;
#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;
#endif

    basicRfConfig.myAddr = MY_ADDR;
    
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    
    rfRecvData();
}

/*系统时钟初始化
-------------------------------------------------------*/
void xtal_init(void)
{
  SLEEPCMD &= ~0x04;
  while(!(CLKCONSTA & 0x40));
  CLKCONCMD &= ~0x47;
  SLEEPCMD |= 0x04;
}

/*led初始化
-------------------------------------------------------*/
void led_init(void)
{
  P1SEL &= ~0x03;
  P1DIR |= 0x03;
  
  D7 = 1;
  D6 = 1;
}

/*uart0初始化
-------------------------------------------------------*/
void uart0_init(unsigned char StopBits,unsigned char Parity)
{
  P0SEL |=  0x0C;
  PERCFG&= ~0x01;
  P2DIR &= ~0xC0;
  U0CSR = 0xC0;
   
  U0GCR = 0x0A;
  U0BAUD = 0x3B;
  
  U0UCR |= StopBits|Parity;
  
  URX0IF = 0;
  URX0IE = 1;
  EA = 1;
}

/*串口发送字节函数（含超时保护）
-------------------------------------------------------*/
void Uart_Send_char(char ch)
{
  uint16 to = 0;
  U0DBUF = ch;
  while(UTX0IF == 0 && ++to < 5000);   // 超时保护：UART TX 挂起时退出，避免死机
  if (to >= 5000) { UTX0IF = 0; return; }   // 自愈：丢弃当前字节
  UTX0IF = 0;
}

/*串口发送字符串函数
-------------------------------------------------------*/
void Uart_Send_String(char *Data)
{
  while (*Data != '\0')
  {
    Uart_Send_char(*Data++);
  }
}