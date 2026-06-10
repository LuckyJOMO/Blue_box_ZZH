# PAN1080 `ble_hid_uart` 工程：时间基准不准问题说明与解决方案

本文档用于记录当前交付版本中“Zephyr 定时/超时与真实时间不一致”的原因与处理方案，方便后续新工程或返工时快速恢复**真实时间语义**（10s 就是 10s、2s 就是 2s）。

> 结论先行：当前工程中 `k_timer` / `K_MSEC()` / `k_uptime_get_32()` 的“毫秒语义”会被整体拉慢（实测约 \(10\times\)），根因是应用层在 `k_idle_thread_hook()` 里动态改系统时钟/分频，导致 SysTick 仍按旧频率配置运行，从而使内核时间不等于真实时间。

---

## 1. 现象（Symptoms）

在当前交付版本（以 `src/main.c` 为例）中存在如下“实测换算”：

- `k_timer_start(..., K_MSEC(1000))` **实测约等于 10 秒**
- `K_MSEC(200)` / 数值阈值 `198` **实测约等于 2 秒**

示例代码片段（交付版本）：

- `src/main.c` 末尾定时器启动：
  - `handshake_timer`：`K_MSEC(1000)` 注释为“10秒”
  - `check_timeout_timer`：`K_MSEC(200)` 注释为“2秒”

---

## 2. 影响面（What is affected）

凡是依赖 Zephyr 内核时间的逻辑都会受影响，包括但不限于：

- `k_timer` 周期
- `k_msleep()`/`k_sleep()` 延时
- `k_uptime_get_32()` 差值判超时（如 198 代表“约 2s”）
- 基于时间的重发窗口、超时窗口、状态机等待窗口
- LED 闪烁节奏（若用 Zephyr 时间）

因此当前工程里存在“代码值 ≠ 真实时间”的现象，必须以**实测或注释语义**理解交付版本。

---

## 3. 根因分析（Root Cause）

### 3.1 Zephyr system timer 在 PAN1080 上的实现要点

PAN1080 的系统时钟驱动在：

- `01_SDK/zephyr/drivers/timer/sys_clock_pan108x.c`

该驱动使用 Cortex-M **SysTick** 作为系统 tick。每个 tick 需要的硬件周期数由下式决定：

- `CYC_PER_TICK = sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC`

系统初始化时将 `SysTick->LOAD` 配置为 `CYC_PER_TICK - 1`（或 tickless 情况下动态配置）。

### 3.2 工程中覆盖 `k_idle_thread_hook()` 并手动改时钟/分频

当前工程在：

- `01_SDK/zephyr/samples_panchip/solutions/ble_hid_uart/src/main.c`

实现了 `k_idle_thread_hook()`，并在 idle 时：

- 改 AHB/APB 分频（`CLK_TOP_CTRL`）
- 可能切系统时钟到 `RCH` 并关闭 `XTH`（32MHz 外部晶振）
- 执行 `__WFI()`，再切回

这会导致：

- SysTick 仍按“初始化时假设的系统频率”倒计数
- 但 CPU/总线实际频率在 idle 阶段被降低或切源
- 从而造成 **内核时间整体变慢**（例如约 \(10\times\)）

> 一句话：**内核计时用 SysTick，但应用私自改变了 SysTick 的时钟环境且未同步内核，所以 `K_MSEC()` 不再代表真实毫秒。**

---

## 4. 当前版本的“时间语义约定”（Delivery-time convention）

由于上述根因，交付版本中出现了“实测换算”：

- `K_MSEC(1000)` ≈ 10s（真实时间）
- `198` ≈ 2s（真实时间）

交付阶段的要求是**结果正确**，因此当前版本以“实测语义”可工作，但维护风险较高：

- 后续若移除降频/切时钟逻辑，所有周期/超时会瞬间变为原来的 \(1/10\)
- 团队成员若按字面理解 `K_MSEC(1000)=1s` 会误改超时/节奏

---

## 5. 解决方案总览（Solutions）

### 方案 A（推荐，根治）：恢复 Zephyr 时间语义 + 使用真实时间常量

目标：

- `K_SECONDS(10)` 就是**真实 10 秒**
- `k_uptime_get_32()` 的差值按毫秒理解即可

原则：

- 不要在 `k_idle_thread_hook()` 里手动改系统时钟源/分频来影响 SysTick
- 若需要低功耗，启用并使用 PAN1080 的 SoC PM 正路（由 `soc_power.c` 管理睡眠/唤醒），并通过 LPTMR 对系统时间进行补偿

相关实现（SoC PM）：

- `01_SDK/zephyr/soc/arm/panchip/pan1080/soc_power.c`
  - 其中在深睡唤醒后调用 `lptmr_announce_sleep_cycle_to_system()` 来补偿系统时间

**采用方案 A 后必须同步修改：**

把交付版本中所有“按实测换算”的时间数值统一改回真实毫秒/秒：

- `handshake_timer`：真实 10s → `K_SECONDS(10)`（或 `K_MSEC(10000)`）
- `check_timeout_timer`：真实 2s → `K_SECONDS(2)`（或 `K_MSEC(2000)`）
- 所有 `198 ≈ 2s` 的阈值 → `2000ms` 级别的阈值（建议宏化统一）

优点：

- 语义清晰、可维护
- 新工程/量产最推荐

风险点（需要验证）：

- 更深的睡眠态可能影响 UART/BLE/LED 外设响应，需要按产品需求选择 PM state 并做回归测试

### 方案 B（止血）：保留现状但工程化“时间缩放”

适用场景：已交付版本不希望大改时钟/PM，只做最小变更保持行为一致。

做法：

- 把所有“实测换算值”集中成宏（避免散落魔法数）
- 文档化当前版本的时间缩放因子（例如约 \(10\times\)）
- 若必须保证跨温度/电压更稳定的真实时间，可增加一次“运行时标定”并换算（复杂度较高）

优点：

- 对交付版本影响最小

缺点：

- 维护成本高、跨环境一致性差
- 一旦时钟/idle 行为变化，所有时间都会漂

---

## 6. 后续改造建议（Recommended path for a new project）

如果后续启动新工程，建议路线：

1. **不要**在 `k_idle_thread_hook()` 中手动切系统时钟/分频（尤其不要在不通知内核的情况下做）
2. 若要低功耗，采用 PAN1080 的 PM 正路（`soc_power.c`）
3. 所有超时/周期统一按真实时间写（`K_SECONDS()` / `K_MSEC()`），并集中在一个头文件/宏区管理
4. 用逻分/抓包对“真实时间”验收，而不是看代码数字

---

## 7. 验证方法（How to verify real-time correctness）

建议的验收标准（只看真实结果）：

- **握手定时发送间隔**：抓 UART 或逻分，连续两次握手帧间隔应为 **10s ± 允许误差**
- **主控超时窗口**：让主控不回包，APP 侧从发起到收到“超时失败”应为 **2s ± 允许误差**
- **高负载/弱信号场景**：验证不会因睡眠/唤醒导致误超时或丢包

---

## 8. 相关文件索引（Where to look）

系统时钟驱动（SysTick/LPTMR 补偿）：

- `01_SDK/zephyr/drivers/timer/sys_clock_pan108x.c`

SoC 低功耗/深睡流程（包含时间补偿调用）：

- `01_SDK/zephyr/soc/arm/panchip/pan1080/soc_power.c`

本工程应用层（含 `k_idle_thread_hook()` 与定时器配置）：

- `01_SDK/zephyr/samples_panchip/solutions/ble_hid_uart/src/main.c`

协议层中与超时相关的逻辑（示例：`198` 判定、重发窗口等）：

- `01_SDK/zephyr/samples_panchip/solutions/ble_hid_uart/src/mcu_to_ble_protocol_v2.0.c`
- `01_SDK/zephyr/samples_panchip/solutions/ble_hid_uart/src/mcu_to_ble_protocol_v2.0.h`

