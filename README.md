# CC2530 无线传感器网络 v3.0

基于 TI CC2530 的广播自组网 + P2P 双向通信 + 无线控制系统。

温湿度/光照采集 + LED 远程控制 + 数据采集开关 + 传感器 P2P 执行确认。

> 升级计划及实现状态见 [UPGRADE-v3.0.md](UPGRADE-v3.0.md)

---

## 项目结构

```
├── 4.1-P2P-DHT11/          # DHT11 温湿度传感器节点
├── 4.1-P2P-Photoresistance/ # 光敏电阻传感器节点
└── 4.1-P2P-Recevier/        # 接收节点（信标 + 数据汇聚 + 指令广播）
```

---

## 硬件平台

| 项目 | 参数 |
|------|------|
| 芯片 | TI CC2530 (8051 内核, 2.4GHz RF) |
| 开发环境 | IAR Embedded Workbench for 8051 |
| 通信协议 | BasicRF (IEEE 802.15.4 物理层) |
| 信道 | CH24 (单信道，数据 + 控制共用) |
| PAN ID | 0x1410 |
| 串口 | 38400-8-N-1 |

---

## 系统架构

```
                        ┌─────────────────────┐
                        │     PC 网关 / 云端    │
                        │   串口 ←→ 接收节点     │
                        └──────────┬──────────┘
                                   │ 串口 UART
                        ┌──────────▼──────────┐
                        │     接收节点 0x4102   │
                        │  ├ 信标广播 {TYPE=RECV}│
                        │  ├ P2P 收数据 → 串口透传│
                        │  └ 串口指令 → 广播转发  │
                        └────┬──────────┬─────┘
                   P2P + ACK  │          │ 广播 0xFFFF
              ┌───────────────┘          └───────────────┐
              ▼ CH24                                     ▼ CH24
   ┌──────────────────┐                      ┌──────────────────┐
   │  DHT11 传感器节点  │                      │  光敏 传感器节点   │
   │  ├ 注册 + 数据P2P  │                      │  ├ 注册 + 数据P2P  │
   │  ├ RX窗收指令       │                      │  ├ RX窗收指令       │
   │  └ 执行后 P2P ACK   │                      │  └ 执行后 P2P ACK   │
   └──────────────────┘                      └──────────────────┘
```

**单信道设计**：所有通信（信标、注册、数据、指令、ACK）在同一信道 CH24 上时分复用，CSMA-CA 处理冲突。

---

## 地址方案

| 角色 | 短地址 | 识别方式 |
|------|--------|----------|
| 接收节点 | `0x4102`（固定） | 传感器从信标 `basicRfReceiveAddress()` 获取 |
| 传感器 | `0x0001`（统一占位） | payload 中 `MAC=00:12:4B:...` 唯一识别 |
| 广播 | `0xFFFF` | IEEE 802.15.4 广播地址 |

传感器短地址统一为 `0x0001`，节点识别完全依赖 payload 中的 8 字节完整 IEEE MAC 地址。传感器**无任何硬编码地址**。

---

## 通信机制

### 接收节点（持续监听）

```
主循环 (10ms 周期):
  ├─ cmdReady → broadcastCmd() 广播指令 → 0xFFFF
  ├─ basicRfPacketIsReady() → 收包 → printf 透传到串口
  ├─ 每 2 秒 → broadcastCmd("{TYPE=RECV}") 信标
  └─ halMcuWaitMs(10)
```

### 传感器节点（两阶段周期，~1 秒）

```
阶段1: 采集 + P2P 发送（光敏 ~60ms / DHT11 ~80ms 含 18ms 拉低，sendEnable=1 且注册后）
  ├─ 读传感器 → csmaCaSendPacket(myRecvAddr, ...) → D6 短闪 50ms
  └─ 暂停时跳过（sendEnable=0）

阶段2: RX 监听窗口 (900ms)
  ├─ basicRfReceiveOn() → 等 900ms → 收包
  ├─ MAC 自比对 → 匹配则执行
  └─ 执行后 → csmaCaSendPacket(myRecvAddr, {CMD=SUCCESS,...}) ACK回传

→ 指令命中率 ≈ 93–95%（阶段2 900ms 占比，DHT11 因 18ms 拉低略低）
```

---

## 串口控制指令

通过接收节点串口发送，格式 `{KEY=VALUE, MAC=...}`：

| 指令 | 功能 | 方向 |
|------|------|------|
| `@` | 接收节点 D7 亮 | 本地 |
| `!` | 接收节点 D7 灭 | 本地 |
| `{LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}` | 指定节点 LED 亮 | 广播 → 传感器 |
| `{LED=0, MAC=00:12:4B:00:0D:C8:A8:CE}` | 指定节点 LED 灭 | 广播 → 传感器 |
| `{STATUS=1, MAC=00:12:4B:00:0D:C8:A8:CE}` | 指定节点恢复数据发送 | 广播 → 传感器 |
| `{STATUS=0, MAC=00:12:4B:00:0D:C8:A8:CE}` | 指定节点暂停数据发送 | 广播 → 传感器 |

> ⚠️ MAC 地址必须**大写**（与传感器 `myMacStr` 一致），`strstr` 大小写敏感。

---

## 数据包结构（不可更改）

以下为底层硬件协议格式，修改会影响所有上层系统。

### 信标包（接收节点 → 广播）

```
{TYPE=RECV}
```

| 方式 | 地址 | ACK | 周期 |
|------|------|-----|------|
| 广播 | 0xFFFF | FALSE | 每 2 秒 |

### 注册包（传感器 → 接收节点，P2P）

```
{REG=MAC:00:12:4B:00:0D:C8:A8:CE}
```

| 方式 | 目标 | ACK | 重试 | 时机 |
|------|------|-----|------|------|
| P2P | myRecvAddr | TRUE（硬件） | 5 次 CSMA-CA | 上电发现信标后 |

### DHT11 数据包（传感器 → 接收节点，P2P）

```
{Humidity=45, Temp=26, D=1, MAC=00:12:4B:00:0D:C8:A8:CE}
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| Humidity | unsigned | 0–255 | 湿度百分比 |
| Temp | unsigned | 0–255 | 温度（℃） |
| D | decimal | 0/1 | LED 逻辑状态：1=亮, 0=灭 |
| MAC | hex string | 23 字符 | 完整 IEEE 地址，冒号分隔，大写 |

### 光敏数据包（传感器 → 接收节点，P2P）

```
{Light=512, D=0, MAC=00:12:4B:00:0D:3F:2A:B1}
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| Light | int | 0–1023 | ADC 光强值 |
| D | decimal | 0/1 | LED 逻辑状态 |
| MAC | hex string | 23 字符 | 完整 IEEE 地址，冒号分隔，大写 |

### 指令包（接收节点 → 广播）

```
{LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{LED=0, MAC=00:12:4B:00:0D:C8:A8:CE}
{STATUS=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{STATUS=0, MAC=00:12:4B:00:0D:C8:A8:CE}
```

| 方式 | 地址 | ACK | 说明 |
|------|------|-----|------|
| 广播 | 0xFFFF | FALSE | 接收节点不解析，纯透传 |

### 指令 ACK 包（传感器 → 接收节点，P2P）

```
{CMD=SUCCESS, LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, LED=0, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, STATUS=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, STATUS=0, MAC=00:12:4B:00:0D:C8:A8:CE}
```

| 方式 | 目标 | ACK | 重试 | 说明 |
|------|------|-----|------|------|
| P2P | myRecvAddr | TRUE（硬件） | 5 次 CSMA-CA | 传感器执行后回传，确认真正执行成功 |

---

## 接收节点串口输出（完整清单）

接收节点串口输出格式：`printf("%s\r\n", pRxData)` —— 所有射频收到的 P2P 包裸透传（接收节点 `pRxData[128]`，传感器 `pRxData[64]`）。

### 启动
```
{data=recv node start up...}
```

### 注册阶段
```
{REG=MAC:00:12:4B:00:0D:C8:A8:CE}
{REG=MAC:00:12:4B:00:0D:3F:2A:B1}
```

### 传感器周期数据
```
{Humidity=45, Temp=26, D=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{Light=512, D=0, MAC=00:12:4B:00:0D:3F:2A:B1}
```

### 指令执行确认（传感器 P2P 回传，非假确认）
```
{CMD=SUCCESS, LED=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, LED=0, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, STATUS=1, MAC=00:12:4B:00:0D:C8:A8:CE}
{CMD=SUCCESS, STATUS=0, MAC=00:12:4B:00:0D:C8:A8:CE}
```

> 信标 `{TYPE=RECV}` 是接收节点自己广播的，不会出现在串口输出中。

---

## 关键设计决策

### 防冲突

- P2P 通信：CSMA-CA 指数退避 + 硬件 CCA + 5 次重试 + BasicRF 硬件 ACK
- 广播：无 ACK，`ackRequest = FALSE`
- 单信道时分复用：传感器 ~1s 周期，接收节点持续监听

### 静默广播 + 传感器 ACK

接收节点广播指令后**不打印任何确认**，仅 D6 闪烁表示存活。传感器执行指令后 P2P 回传 `{CMD=SUCCESS,...}`，接收节点收到后串口输出。

| 方案 | 旧设计 | 新设计 |
|------|--------|--------|
| 确认内容 | "我发出去了" | **"传感器执行了"** |
| 传感器离线 | 仍打印 SUCCESS（假阳性） | 无输出（真正无人执行） |

### sendEnable=0 行为

暂停数据采集后：阶段1 跳过（不读传感器、不发送数据），阶段2 RX 窗口正常（仍可接收 `STATUS=1` 恢复指令），P2P ACK 正常回传。

### D6/D7 指示灯

| 引脚 | 节点 | 功能 |
|------|------|------|
| D6 | 接收节点 | 存活心跳：每次广播闪 50ms（信标 2s + 指令触发） |
| D6 | 传感器 | 数据发送成功短闪 50ms |
| D7 | 接收节点 | `@` / `!` 本地控制 |
| D7 | 传感器 | 远程 LED 控制目标（LED=1/0 指令） |

---

## 稳定性与防御性设计

为解决长时间运行后节点死机、`@` 指令无响应、重启采集节点后无法注册等问题，v3.0 在原有业务修复基础上补充以下防御性修复（commit `887a881` + `9db3a34`）。

### RF 状态机超时与自愈（`hal_rf.c`，三个项目）

`halRfTransmit` 的 `while(!TXDONE)` 与 `halRfWaitTransceiverReady` 的 `while(FSMSTAT1 & ...)` 原本无超时，RF 状态机偶发挂起时进入永久死循环，导致信标/数据包发不出，传感器表现为"重启采集节点后 reg=FAIL，重启接收节点才能注册"。

修复策略：
- `while` 循环加 `++to < 20000` 超时退出
- 超时后执行 `ISFLUSHTX + ISRFOFF + ISFLUSHRX + ISRXON` 重置 RF 状态机并恢复 RX，避免 RF 一直卡死后信标永久发不出

### doRegister RF 状态保护（DHT11 + 光敏 `main.c`）

阶段1 用 `halRfReceiveOn/Off`（HAL 层）不会设置 `txState.receiveOn`，导致 `basicRfSendPacket` 发完 REG 包后调 `halRfReceiveOff`，重新 `halRfReceiveOn` 时 `ISFLUSHRX` 清空 RX FIFO，可能丢掉紧随其后的下一个信标。

修复：改用 `basicRfReceiveOn/Off`（BASIC_RF 层），设置 `txState.receiveOn=TRUE`，`basicRfSendPacket` 发完不再关 RX，避免 `ISFLUSHRX` 丢包。同时 REG 失败后不 `return`，继续循环等下一个信标重试。

### DHT11 通信超时保护（DHT11 `main.c`）

`dht11_read_bit` 的 `while(!COM_R)` 无超时，传感器异常（数据线卡低）时永久死循环死机。

修复：
- `while` 加 `++to < 500` 超时退出
- 全局 `dht11TimedOut` 标志传播到 `dht11_read_byte` 与 `dht11_update`，超时时中断读取并保留上次有效数据，不更新 `sTemp/sHumidity`

### `tried` 循环计数器溢出修复（DHT11 + 光敏 `main.c`）

`doRegister` 阶段1 用 `uint8 tried < 350`，但 `uint8` 范围 0–255，比较永远为 `true` → 阶段1 永久死循环 → 节点死机。光敏节点 IAR 编译警告 `Warning[Pa084]: pointless integer comparison, the result is always true` 即此问题。

修复：`uint8 tried` → `uint16 tried`

### `Uart_Recv_char` 死代码清理（三个项目）

`Uart_Recv_char` 从未被调用，但 IAR banked code model 会为所有函数（含死代码）生成 `?relay` 中转入口，与 `main.c` 中的实体定义冲突，导致链接错误：

```
Error[e27]: Entry "Uart_Recv_char::?relay" in module basic_rf redefined in module main
```

修复：删除 `Uart_Recv_char` 的定义（`main.c` / `uart.c`）和声明（`uart.h`）。已通过 grep 确认无任何调用、函数指针引用或 `extern` 声明。

### idata 栈扩容（三个项目 `p2p.ewp`）

IAR 默认 idata 栈仅 64 字节（`0x40`），中断嵌套（`rfIsr` + `basicRfRxFrmDoneIsr` + `UART0_ISR`）溢出，烧录时 IAR 调试器警告 `Possible IDATA stack overflow detected`。

修复：Idata Stack Size `0x40` → `0x80`（128 字节）。

> Extended Stack（xdata 511 字节）无法启用：勾选后 IAR 7.0 报错 `Error[e12]: Unable to open cl-ple-blpd-1e16x01.r51`，本机安装缺少对应 CLIB 变体。改用 idata 栈扩容 + 代码层防御。

### 接收节点中断状态防御性恢复（接收 `main.c`）

idata 栈溢出会破坏 `HAL_INT_LOCK(x)` 保存的 `EA` 值，`HAL_INT_UNLOCK(x)` 恢复时 `EA=0` → 全局中断永久关闭 → `@` 指令无响应（UART RX 中断不触发，但 `rx` 计数器仍跳动能收到字节）。

修复：主循环每轮强制 `EA = 1; URX0IE = 1;` 防御性恢复中断使能，即使保存值被破坏也能自愈。

---

## LED 引脚

| 丝印 | 引脚 | 逻辑 | 用途 |
|------|------|------|------|
| D7 | P1_0 | 0=亮, 1=灭 | 无线控制目标 / 本地测试 |
| D6 | P1_1 | 0=亮, 1=灭 | 存活心跳 / 发送指示 |

---

## 编译

1. IAR 打开 `p2p.eww`
2. 传感器节点 `NODE_TYPE = 1`（条件编译切换 `rfSendData`/`rfRecvData`）；接收节点 `main.c` 直接调用 `rfRecvData()`，无需设置 `NODE_TYPE`
3. 编译 → Download and Debug → 烧录
4. 串口 38400 查看输出

编译警告 `basic_rf.c:517/544` 为 TI 库固有，可忽略。

### idata 栈大小设置（IAR GUI）

三个项目的 `p2p.ewp` 已配置 `Idata Stack Size = 0x80`（128 字节，默认仅 0x40=64 字节会触发 `Possible IDATA stack overflow detected` 警告）。如需在 IAR GUI 修改：`Project → Options... → General Options → Stack/Heap → Idata stack size`，填 `0x80`。

> Extended Stack 选项（`Project → Options... → General Options → Stack/Heap → Enable extended stack`）**不要勾选**：本机 IAR 7.0 缺少对应 CLIB 库（`cl-ple-blpd-1e16x01.r51`），勾选后链接报错 `Error[e12]`。

### 死代码与 `?relay` 链接错误

IAR banked code model 会为所有函数（含从未被调用的死代码）生成 `?relay` 中转入口。若 `uart.c` 与 `main.c` 同时定义同名函数，链接时冲突报 `Error[e27]: ...?relay redefined`。已删除三个项目中的 `Uart_Recv_char` 死代码（定义 + 声明）。新增 UART 函数时，**只在一个 `.c` 中定义，其余文件通过 `extern` 或头文件声明**，避免重复定义。

---

## 许可

MIT License
