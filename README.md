# 旋转倒立摆（Furuta 摆）— 高云 GW1N-9C FPGA 实现

用一片 FPGA 独立完成「传感器解码 + 串级 PID + 自动起摆 + 电机 PWM + 数码管/串口交互」的
旋转倒立摆控制器，不含任何 MCU。

- **目标板**：Pocket Lab-F1 核心板，GW1N-UV9EQ144C6/I5（eLQFP144）
- **工具链**：Gowin EDA V1.9.12.04
- **时钟**：50 MHz 板载晶振
- **语言**：Verilog-2001

## 硬件构成

| 部件 | 接口 | 说明 |
|---|---|---|
| SA100 角位移传感器 | 单路 PWM（pin 45） | 摆杆倾角 θ，PWM 周期 1024 µs，占空比编码 0~360° |
| MG513 霍尔编码器 | A/B（pin 43/44） | 摆臂转角 φ，4 倍频后 1040 计数/圈 |
| TB6612 电机驱动 | PWM=41, AIN1=42, AIN2=46 | 20 kHz 单极性 PWM + 方向 |
| 4 位数码管 | 共阳，动态扫描 | 显示臂角/预设/摆角/占空比 |
| 串口 CH340 | RX=49, TX=48, 115200 8N1 | 在线调参 |
| 拨码 | SW4=79 起摆+平衡, SW2=76 只平衡 | SW1(75) 已损坏未使用 |

## 目录结构

```
inverted pendulum/
  src/top.v        顶层: 复位、按键消抖、拨码、编码器定标、显示、模块例化
  src/ctrl.v       控制核心: 1 kHz 节拍、状态机、角度环/位置环、启摆、20 kHz PWM
  src/sense.v      编码器 4 倍频 + SA100 PWM→角度 (ton/per 归一化 + 32/24 除法器)
  src/uart.v       8N1 收发 + 调参包解析 + 参数读回
  src/ui.v         20 ms 消抖、4 位扫描数码管、bin2bcd
  src/params.vh    全部可调常量
  src/top.cst      管脚约束
  src/top.sdc      50 MHz 时钟约束
  build.tcl        无头编译脚本 (gw_sh build.tcl)
  版本记录.md      接线、调参、显示模式速查
model/
  pendulum_design.py   LQR + 非线性仿真 + 定点验证 (另一条设计路线, 未用于 RTL)
  diag.py / diag2.py   摩擦死区与起摆参数扫描
  check_*.py           与本工程 RTL 对应的位精确校验脚本
  设计说明.md           建模、动力学、起摆算法论证
```

## 控制结构

状态机（`ctrl.v`）：

| mode | 名称 | 动作 |
|---|---|---|
| 0 | 停止 | PWM = 0；显示模式 3 下可用 K5/K6 开环点动 |
| 1 | 起摆 | 每 40 ms 采一次角度，三点判极值；到该侧最高点就 ±35 % 踢 100 ms、反向回位 100 ms |
| 2 | 平衡 | 10 ms 角度环 + 50 ms 位置环 |

**自动交棒**：`mode 1` 时每 1 ms 判断一次，连续 5 ms 落在显示角 **150°~210°**
就自动切到 `mode 2`（`CAP_RANGE` / `CAP_CONFIRM_MS`）。

**直立环**（10 ms，`u% = e·Kp/9362`）：

```
e_cnt  = 角度给定 - 角度实际          (计数, 1 计数 = 0.088°)
给定    = 位置环输出 loc_out          (同号, 见 ctrl.v 注释)
u      = Kp·e + Ki·∫e + Kd·Δe
```

**积分门限**：只有在 `mode 2` + 保护窗内 + `|θ偏差| < 10°` + `Ki ≠ 0` 时才累积，
其余情况一律清零；幅值限 `ANG_INT_MAX`，且每次推上起摆开关（`ev_start`）立即清零。

**保护**：`|θ偏差| > 65°` 时 PWM 直接归零，回到窗内自动恢复，不停机。

**位置环**（50 ms）：把摆臂位置误差转成摆杆目标角度偏移，默认 `W_KP = W_KD = 0`（关闭）。
摆臂转角超过 `POS_LIM` 时输出限幅 ±100 计数（≈ ±8.8° 摆杆角度）。

## 串口调参（115200 8N1）

```
[s,Z_KP,140]    直立环 Kp      [s,Z_KI,5]   直立环 Ki     [s,Z_KD,26]  直立环 Kd
[s,target,180]  直立中心角度 (度)
[s,Z_SGN,1]     翻转平衡输出方向
[s,W_KP,1638]   位置环 Kp2 (Q12, 1638 = 0.40)
[s,W_KD,16384]  位置环 Kd2 (Q12, 16384 = 4.00)
[s,GET,1]       读回参数, 回一行: A008C B0005 C001A D0000 E0712
```

- 增益填**整数**，和 `params.vh` 里的 `KP_ANG/KI_ANG/KD_ANG` 是同一个数；
  内部统一 ×7，`实际增益 = 数字 / 9362`（P 项：`Kp=140` → 0.17 %占空比/度）。
- 每个被接受的包原样回显 + CRLF；**看到回显才算生效**。
- `Z_SGN` 确认好方向后，把 `params.vh` 里的 `BAL_SGN` 改成 1 固定下来。

## 编译

```
cd "inverted pendulum"
& "D:\Gowin\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe" build.tcl
```

`build.tcl` 已固定 `set_option -top_module top`。产物在 `impl/pnr/`（`.fs` 可直接下载）。

## 模型与仿真

`model/` 下是独立的 Python 设计与验证环境（需要 numpy / matplotlib）：

```
cd model
python pendulum_design.py     # LQR 设计 + 非线性仿真 + 定点/浮点对比
python diag2.py               # 摩擦死区、起摆参数扫描
python check_uart_chain.py    # 位精确复刻串口解析 + 角度链 (对照 RTL)
python check_pid_gain.py      # 直立环增益与物理需求核算
```

注意：`model/` 里的 LQR 级联架构**不是**当前 RTL 实现（RTL 是串级 PID），
两者共用同一套动力学参数。

## 已知限制

- **静态时序未收敛**：`angle_q → u_q` 是一条约 92 ns 的组合链（46 级逻辑），
  静态 Fmax 仅 ~11.4 MHz（要求 50 MHz）。目前能工作是因为角度只以 ~1 kHz 更新、
  链路有毫秒级时间稳定；但如果角度变化恰好落在采样沿前 ~92 ns 内，会采到中间值。
  彻底的修法是在角度链中间插入流水线寄存器。
- 摆杆转轴静摩擦是平衡质量的主要瓶颈（见 `model/设计说明.md` §5）。
- 起摆用的是"最高点踢一脚"的开环策略，能量泵效率低于文档中论证的振幅判据法。

## 许可

个人学习/竞赛项目，未附许可证。引用请注明出处。
