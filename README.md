# CC2530 无线传感器网络 v2.0

基于TI CC2530芯片的P2P双向通信与无线控制项目，实现温湿度、光照采集 + LED远程控制 + 数据采集开关。

---

## 项目结构

```
├── 4.1-P2P-DHT11/          # DHT11温湿度传感器节点（短地址 0x4103）
├── 4.1-P2P-Photoresistance/ # 光敏电阻传感器节点（短地址 0x4101）
└── 4.1-P2P-Recevier/        # 数据接收+无线控制节点（短地址 0x4102）
```

---

## 硬件平台

| 项目 | 参数 |
|------|------|
| 芯片 | TI CC2530 (8051内核, 2.4GHz RF) |
| 开发环境 | IAR Embedded Workbench for 8051 v10.30+ |
| 通信协议 | BasicRF (IEEE 802.15.4 物理层) |
| 信道 | 24 (2.470 GHz) |
| PAN ID | 0x1410 |
| 串口波特率 | 38400 |

---

## 节点地址表

| 节点 | 短地址 | MAC地址 | 功能 |
|------|--------|---------|------|
| DHT11传感器 | 0x4103 | 芯片唯一(8字节) | 采集温湿度，无线发送 + 接收LED/采集控制 |
| 光敏传感器 | 0x4101 | 芯片唯一(8字节) | 采集光照，无线发送 + 接收LED/采集控制 |
| 接收节点 | 0x4102 | 芯片唯一(8字节) | 接收数据 + 串口指令转发 |

---

## 串口控制指令

通过接收节点（0x4102）串口发送：

| 指令 | 功能 | 方向 |
|------|------|------|
| `@` | 接收节点 D7 亮 | 本地 |
| `!` | 接收节点 D7 灭 | 本地 |
| `1` | DHT11 节点 LED 亮 | 无线 → 0x4103 |
| `2` | DHT11 节点 LED 灭 | 无线 → 0x4103 |
| `3` | 光敏节点 LED 亮 | 无线 → 0x4101 |
| `4` | 光敏节点 LED 灭 | 无线 → 0x4101 |
| `5` | DHT11 恢复数据发送 | 无线 → 0x4103 |
| `6` | DHT11 暂停数据发送 | 无线 → 0x4103 |
| `7` | 光敏节点恢复数据发送 | 无线 → 0x4101 |
| `8` | 光敏节点暂停数据发送 | 无线 → 0x4101 |

---

## 数据包格式

### 传感器 → 接收节点（无线）

```
DHT11:  {Humidity=14, Temp=23, D=1, MAC=00:12:4B:00:0D:C8:A8:CE}
光敏:   {Light=704, D=0, MAC=00:12:4B:00:0D:C6:88:62}
```

| 字段 | 说明 |
|------|------|
| Humidity/Temp/Light | 传感器数据 |
| D | LED 状态: 1=亮, 0=灭 |
| MAC | 8字节完整 IEEE 地址，格式 XX:XX:XX:XX:XX:XX:XX:XX |

### 接收节点 → 传感器（无线控制）

```
{LED=1}   / {LED=0}     # LED 开关
{SEND=1}  / {SEND=0}    # 数据采集开关
```

---

## 接收节点串口输出

```
{Light=704, D=0, MAC=00:12:4B:00:0D:C6:88:62}     ← 光敏数据
{Humidity=14, Temp=23, D=1, MAC=00:12:4B:00:0D:C8:A8:CE}  ← DHT11数据
{D=0, MAC=00:12:4B:00:0D:C6:8E:6D}                 ← 本机状态（双节点收齐后打印）
{cmd=LED_ON->0x4103}                                 ← LED控制成功
{cmd=FAIL->0x4103}                                   ← 无线发送失败
```

---

## 通信机制

### 时分复用双工

CC2530 射频为半双工，采用时分复用实现双向通信：

```
传感器节点 1秒周期:
  TX(10ms) + D6闪烁(100ms) + RX窗口(200ms) + 等待(700ms) ≈ 1s

接收节点:
  持续 RX 监听 → 串口指令触发时短暂切换 TX
```

### 防冲突

- CSMA-CA 指数退避：传感器 5 次重试（10ms×2^n），接收节点 12 次重试（160ms封顶）
- BasicRF 内置硬件 CCA + ACK 确认
- 接收节点 ISR 仅设标志位（不阻塞），主循环处理发送

### MAC 地址

从 CC2530 Flash 信息页偏移 0x0C 处（地址 0x780C~0x7813）读取 8 字节 IEEE 地址，小端序。

---

## LED 引脚

| 丝印 | 引脚 | 用途 |
|------|------|------|
| D7 | P1_0 | 无线控制目标 LED |
| D6 | P1_1 | TX 发送成功闪烁指示 |

注意：本系统使用 `sys_init.h` 的 `D6`/`D7` 宏（非 HAL 的 `hal_led_on()`）直接操作 P1_0/P1_1。

---

## 编译烧录

1. IAR 打开对应目录下的 `p2p.eww`
2. 确认 `NODE_TYPE = 1`（传感器节点）或依赖默认设置（接收节点）
3. 编译 → Download and Debug 烧录到 CC2530 板
4. 串口工具 38400-8-N-1 查看输出

---

## 编译警告说明

`basic_rf.c` 第 517/544 行 "pointless integer comparison" 为 TI 库固有警告，不影响功能，可忽略。

---

## v3.0 规划：广播自组网

### 目标

去掉硬编码短地址，传感器节点通过广播方式自动上报数据，实现即插即用。

### 方案设计

参考实验例程 4.2-BroadcastCommunication（广播）、4.4-ChannelScan（信道扫描）：

#### 1. 传感器节点自组网流程

```
上电 → 扫描信道(11-26)找到接收节点 → 注册自身MAC → 广播数据
```

**地址方案**：短地址由 MAC 后 2 字节派生（全局唯一，无需硬编码）：
```c
myAddr = (macAddr[1] << 8) | macAddr[0];
```

**发现机制**：
- 接收节点在固定信道广播 `{TYPE=RECV}` 信标（每 2 秒）
- 传感器节点上电后扫描 11-26 信道，收到信标后锁定信道
- 传感器向接收节点发送 `{REG=MAC:XX:XX:XX:XX:XX:XX:XX:XX}` 注册包
- 接收节点建立 MAC→短地址映射表

#### 2. 数据上报（广播模式）

传感器节点将数据广播到 `0xFFFF`：
```c
csmaCaSendPacket(0xFFFF, pTxData, len);
```

所有接收节点均可收到，实现多接收节点冗余。

#### 3. 无线控制（MAC 直控 + 广播透传）

**核心思路**：接收节点不解析指令、不查表——只负责把 PC 串口指令**透传广播**到 `0xFFFF`。每个传感器自比对 MAC，匹配则执行。

```
PC 串口 → {LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}
              ↓
接收节点 → 广播 0xFFFF → {LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}
              ↓                ↓              ↓
          DHT11              光敏           新节点
      MAC 不匹配→忽略   MAC 匹配→执行   MAC 不匹配→忽略
```

**PC 串口指令格式（统一）**：

```
{LED=1,  MAC=XX:XX:XX:XX:XX:XX:XX:XX}   # 指定节点灯亮
{LED=0,  MAC=XX:XX:XX:XX:XX:XX:XX:XX}   # 指定节点灯灭
{SEND=1, MAC=XX:XX:XX:XX:XX:XX:XX:XX}   # 指定节点恢复发送
{SEND=0, MAC=XX:XX:XX:XX:XX:XX:XX:XX}   # 指定节点暂停发送
@                                        # 接收节点本地 D7 亮（保留）
!                                        # 接收节点本地 D7 灭（保留）
```

**接收节点改动**：
- `@` / `!` → 本地 D7 控制，**不变**
- `1`~`8` 单字符指令 → **删除**，替换为字符串缓冲
- ISR 改为累积接收字节到缓冲区，收到 `}` 结尾后广播 `0xFFFF`
- 删掉 `cmdPending` / `cmdTarget` / `cmdOn` 标志位

**传感器节点改动**：
- 上电时除读取 MAC 字节外，额外生成 MAC 字符串 `myMacStr`
- RX 窗口收到包后：`if (strstr(pRxData, myMacStr))` → MAC 匹配才执行
- 不再需要固定短地址

#### 4. 对比：当前 vs MAC 直控

| | v2.0（当前） | v3.0 MAC 直控 |
|---|---|---|
| 指令数量 | 8 个（`1`~`8`） | 4 种 × N 节点（统一格式） |
| 寻址方式 | 接收节点查表映射短地址 | 传感器自比对 MAC |
| 新增传感器 | 需修改接收节点代码 | 零配置，即插即用 |
| PC 脚本 | 逐一发送单字符 | 自动拼接 MAC 指令包 |
| 接收节点职责 | 解析指令 + 查表 + 定向发送 | **仅透传广播** |

#### 5. 信道冲突避免

| 场景 | 解决方案 |
|------|----------|
| 多传感器同时注册 | 随机退避 + 重试 |
| 接收节点信标碰撞 | 接收节点独占信道 24 发信标 |
| 数据广播碰撞 | CSMA-CA + ACK（广播模式可关闭 ACK） |
| 新节点加入 | 持续监听信标，收到后注册 |

### 关键改动

| 当前 v2.0 | v3.0 |
|-----------|------|
| `#define SEND_ADDR 0x4103` 硬编码 | `myAddr = MAC[1:0]` 自动派生 |
| P2P 定向发送 | 广播 `0xFFFF` + 定向控制 |
| 编译时区分节点 | 运行时自动发现 |
| 单一接收节点 | 支持多接收节点 |
| 手动配置 PAN/信道 | 自动扫描锁定 |

---

## v3.0 升级路线

### 决策点

| 问题 | 决定 |
|------|------|
| 传感器数据走广播还是 P2P？ | **保持 P2P → 0x4102**（可靠 ACK，不丢数据包） |
| 控制指令走什么？ | **广播 0xFFFF，传感器自比对 MAC** |
| 短地址怎么设？ | 传感器从 MAC 末 2 字节派生（占位用），接收节点固定 0x4102 |
| 广播需要 ACK 吗？ | 不需要。802.15.4 禁止广播 ACK，强行开 ACK 会导致超时 FAILED |

### 待解决问题

| # | 问题 | 严重 | 方案 |
|---|------|------|------|
| A | 广播 `0xFFFF` + `ackRequest=TRUE` → basicRfSendPacket 永远返回 FAILED | 🔴 | 广播前临时 `basicRfConfig.ackRequest = FALSE`，发完恢复 TRUE |
| B | 串口指令 `{LED=1, MAC=...}` 长达 ~44 字符，CC2530 硬件 FIFO 仅 1 字节 | 🟡 | ISR 中环形缓冲区累积字节，收到 `}` 后设 `cmdReady` 标志 |
| C | 主循环 csmaCaSendPacket 阻塞时后续串口字节丢失 | 🟡 | 环形缓冲解耦 ISR 和主循环；广播发送不重试（无 ACK，重试无意义） |
| D | MAC 比对用 `strstr` 是否可靠 | 🟢 | 802.15.4 保证全球唯一，`strstr("00:12:4B:...")` 足够，无需额外处理 |

### Phase 1：接收节点 — 串口缓冲 + 广播透传

**文件**：`4.1-P2P-Recevier/main.c`

```c
// === 新增：环形缓冲区 ===
#define CMD_BUF_SIZE 64
char cmdBuf[CMD_BUF_SIZE];
volatile uint8 cmdIdx = 0;
volatile uint8 cmdReady = 0;

// === ISR 改为累积字节 ===
__interrupt void UART0_ISR(void) {
    char c;
    URX0IF = 0;
    c = U0DBUF;
    if (c == '@') { D7 = 0; return; }   // 本地指令立即执行
    if (c == '!') { D7 = 1; return; }
    if (cmdIdx < CMD_BUF_SIZE - 1) {
        cmdBuf[cmdIdx++] = c;
        if (c == '}') {                  // 收到结束符
            cmdBuf[cmdIdx] = '\0';
            cmdReady = 1;
            cmdIdx = 0;
        }
    }
}

// === 主循环新增广播发送 ===
void broadcastCmd(char* cmd) {
    basicRfReceiveOff();
    basicRfConfig.ackRequest = FALSE;          // 广播关 ACK
    basicRfSendPacket(0xFFFF, (uint8*)cmd, strlen(cmd) + 1);
    basicRfConfig.ackRequest = TRUE;           // 恢复
    basicRfReceiveOn();
    printf("{cmd=%s->BROADCAST}\r\n", cmd);
}

// === 主循环轮询 cmdReady ===
if (cmdReady) {
    cmdReady = 0;
    broadcastCmd(cmdBuf);
}
```

**删除**：`cmdPending`/`cmdTarget`/`cmdOn`、`sendLedCmd`/`sendDataCmd`/`sendWirelessCmd`、`csmaCaSendPacket`（接收节点不再需要定向发送）

### Phase 2：传感器节点 — MAC 字符串比对

**文件**：`4.1-P2P-DHT11/main.c`、`4.1-P2P-Photoresistance/main.c`

```c
// === 上电时生成 MAC 字符串 ===
char myMacStr[24];              // "00:12:4B:00:0D:C8:A8:CE"
sprintf(myMacStr, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
        macAddr[7], macAddr[6], macAddr[5], macAddr[4],
        macAddr[3], macAddr[2], macAddr[1], macAddr[0]);

// === 短地址从 MAC 末 2 字节派生 ===
basicRfConfig.myAddr = (macAddr[1] << 8) | macAddr[0];
```

```c
// === RX 窗口：MAC 比对后执行 ===
if (strstr((char*)pRxData, myMacStr)) {
    if (strstr((char*)pRxData, "LED=1"))  { D7 = 0; printf("{cmd=LED ON}\r\n"); }
    if (strstr((char*)pRxData, "LED=0"))  { D7 = 1; printf("{cmd=LED OFF}\r\n"); }
    if (strstr((char*)pRxData, "SEND=1")) { sendEnable = 1; printf("{cmd=SEND ON}\r\n"); }
    if (strstr((char*)pRxData, "SEND=0")) { sendEnable = 0; printf("{cmd=SEND OFF}\r\n"); }
}
```

**删除**：`#define SEND_ADDR 0x4103` 等硬编码地址。`RECV_ADDR = 0x4102` 保留（传感器数据仍定向发送到接收节点）。

### Phase 3：清理旧代码

| 文件 | 删除 |
|------|------|
| 接收节点 | `sendLedCmd`、`sendDataCmd`、`sendWirelessCmd`、`cmdPending/cmdTarget/cmdOn`、ISR 中 `1`~`8`、`csmaCaSendPacket` |
| 传感器 | `SEND_ADDR` 硬编码（改为 MAC 派生） |

### 升级后串口指令

| 指令 | 功能 |
|------|------|
| `@` | 接收节点 D7 亮（本地，瞬时） |
| `!` | 接收节点 D7 灭（本地，瞬时） |
| `{LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}` | DHT11 灯亮 |
| `{LED=0, MAC=00:12:4B:00:0D:C8:A8:CE}` | DHT11 灯灭 |
| `{SEND=1, MAC=00:12:4B:00:0D:C6:88:62}` | 光敏恢复发送 |
| `{SEND=0, MAC=00:12:4B:00:0D:C6:88:62}` | 光敏暂停发送 |
| 其他格式 | 直接广播，传感器自匹配 |

### 升级后不变量

| 功能 | v2.0 → v3.0 |
|------|-------------|
| 传感器数据上报 | 不变（P2P → 0x4102，ACK） |
| 数据包格式 | 不变（`{Humidity, Temp, D, MAC}`） |
| 时分复用周期 | 不变（TX + RX200ms + 等待700ms） |
| LCD 显示 | 不变（`{A0=,A1=}` / `{A0=}`） |
| `@`/`!` 本地 LED | 不变 |

---

## 许可

MIT License
