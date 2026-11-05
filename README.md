# embedded_systems_design

基于 STM32F746G-DISCO 的双板声学测距工程。两块板识别同一次 50 ms chirp，交换到达时刻（TOA），用到达时间差和温度对应的声速计算声程差。

| 工程 | 角色 | 默认 IP | IAR 工作区 |
| --- | --- | --- | --- |
| [Board_A](Board_A/) | 主板、近板、同步基准，采集 DHT11 温度并向 B 板发送温度和声速 | `192.168.1.10` | [Board_A/EWARM/Project.eww](Board_A/EWARM/Project.eww) |
| [Board_B](Board_B/) | 从板、远板，根据同步结果修正时钟，接收 A 板温度和声速 | `192.168.1.11` | [Board_B/EWARM/Project.eww](Board_B/EWARM/Project.eww) |

## 编译和连接

1. 用 IAR Embedded Workbench for Arm **9.40.1** 分别打开两套工作区，选择各自的项目配置 `LT_KeShe` 或 `LT_KeShe_B`，编译后烧录到对应板卡。
2. 两板通过以太网互通。默认子网掩码为 `255.255.255.0`，同步和测距报文使用 UDP `5000`。
3. 两板麦克风模块的模拟输出接各自的 **PA0（ADC3_IN0）**，模块与本板共地。A 板 DHT11 的 DATA 接 **PB4**，与 A 板共地。
4. 等两板时钟收敛后，播放与内置模板匹配的 chirp。测两麦克风间距时，将声源放在两麦克风连线的外侧，按“声源 → A 板 → B 板”共线摆放。屏幕的 `PATH DIFF cm` 是声程差，在这种布置下等于两麦克风间距。

## 声音识别参数

| 参数 | A 板 | B 板 | 作用 |
| --- | --- | --- | --- |
| `MIC_CHIRP_MIN_R` | `0.26f` | `0.26f` | chirp 模板的归一化相关门限 |
| 包络触发下限 | 10 个 ADC 码 | 6 个 ADC 码 | 自适应触发门限的最低值 |
| `MIC_TRIG_CONFIRM` | 64 点 | 16 点 | 包络必须连续越限的点数 |

两板都使用 48 点滑动平均和 `max(nfar + 4, npk + 2, 本板下限)`。先通过连续越限确认候选，再做 chirp 模板匹配。

参数位于 [A 板 mic.c](Board_A/Core/Src/mic.c) 和 [B 板 mic.c](Board_B/Core/Src/mic.c)。`Core/Src/freertos.c` 负责时钟同步和 TOA 报文，`Core/Src/disp.c` 负责触摸菜单、声速选择和测量结果显示，`LWIP/App/lwip.c` 设置本板地址。串口 USART1 为 115200 baud，`tg` 是包络触发次数，`ev` 是通过 chirp 识别的事件数，`rx` 是收到的对端 TOA 报文数，`pr` 是本板成功配对并接受的次数。

## 温度模式与距离记录

点击屏幕左上角温度区域，进入 `TEMP MODE`。

- `DHT11 AUTO` 使用 A 板 DHT11 的有效读数，A 板通过 PB4 采集，B 板通过网络接收。
- `SET TEMP` 打开手动档位，从 −20、0、20、40、60 °C 中选择。两板独立设置，对比读数时选同一档。

包络触发时记录当时所选声速。声纹识别和 TOA 配对成功后，用该声速与校准时差计算并保存距离，最近十条记录和累计 SD 使用各次测量值。若自动模式在触发时没有有效温度，声音识别和配对仍正常进行，该次距离显示为 `--`，不计入距离 SD。温度刷新或切换模式只影响之后触发的事件。
