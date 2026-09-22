# 巡航配平最小运行包（Coupling）

植物线：**`aerodynamic_model_V1_1_Coupling.slx`**。  
工作区由 **`initial_main_V1_1.m`** 写入。刚体用 Newton–Euler；结构只有 15 个弹性模态（`n_rigid = 0`，`As` 为 30×30）。

配平分两步，**同一 MATLAB 会话内顺序执行**，中途不要再跑 `initial_main_V1_1`（会把 Wagner / η 初值盖掉）：

1. `trim_flexible_direct_findop`：准定常下联合配平刚体 + 柔性  
2. `restore_wagner_restored`：在该点上恢复非定常 Wagner 状态

两步都只改内存中的模型，**不保存 `.slx`**。`Throttle` 端口名字是油门，**实际单位是 RPM**。

本包不包含 TRIM / RESTORED，也不跑旧的 `trim_cruise_rigid` 求解。`trim_cruise_rigid.m` 只给 `initial_main_V1_1` 提供刚体初值结构。

## 运行

需要 MATLAB + Simulink + Simulink Control Design（`findop` / `operspec`）。工作目录必须是本文件夹根。

```matlab
cd('把这里换成本文件夹的绝对路径')
run_cruise_trim
```

或者手动两步：

```matlab
cd('把这里换成本文件夹的绝对路径')
addpath(fullfile(pwd, 'trim'));

% 1. 准定常柔性联合配平（内部会先跑 initial_main_V1_1）
aeroelastic_direct_result = trim_flexible_direct_findop(0.3);

% 2. 在柔性点上恢复 Wagner（结构 eta 保持不动）
wagner_result = restore_wagner_restored(0.3);
```

默认工况：`U = 50 m/s`，桨距 34°，倾转 0，悬停桨 RPM = 0。  
`Ready to simulate : 1` 表示该步残差和短时 hold 都过了。第 2 步过了才是「准定常柔性点 + 非定常气动状态」齐备。

### 成功后直接仿真

不要再跑初始化。打开已加载的 Coupling 模型，确认：

| 位置 | 填什么 |
|---|---|
| Model Settings → Data Import/Export → Input | 勾选，变量名 `trim_input_dataset` |
| `Flexible Aircraft Dynamics/Integrator` | `trim_Velocity_b0` |
| `Flexible Aircraft Dynamics/Integrator1` | `trim_Omega_b0` |
| `Flexible Aircraft Dynamics/Integrator2` | `trim_Position0` |
| `Flexible Aircraft Dynamics/Integrator3` | `trim_Attitude_rad0` |
| `Flexible Aircraft Dynamics/Structure_SS` | `x0_structure` |

气动块已经引用 `x0_wagner_*` 和 `alpha_du0_*`。点 **Run**。

## 改工况

| 改什么 | 改哪里 |
|---|---|
| 空速 | **两处一起改**：`initial_main_V1_1.m` 的 `U` **和** `trim/trim_flexible_direct_findop.m` 的 `condition.U_mps` |
| 桨距 / 倾转 / 悬停 RPM | `trim_flexible_direct_findop.m` 顶部 `condition.*` |
| 升降舵 / 迎角初值 | 同上 `initial.*` |
| 密度、极曲线、结构路径 | `initial_main_V1_1.m` 自定义块 |

Coupling 里动压按 `(V/U)^2` 缩放冻结的 `qS_strip`。`U` 是 `initial_main_V1_1` 生成 Wagner / `qS` 时的参考空速，必须等于配平目标空速，否则升力会偏。

高空速时注意桨表：`J = U/(nD)`，`D = 3 m`，`J_max = 2.4`。脚本会按这个自动抬巡航桨 RPM 下限。

## 两步分别做什么

### 1. `trim_flexible_direct_findop`

Wagner / 下洗旁路，气动力按准定常极曲线。刚体、控制、弹性 η、η̇ **一次 `findop` 联立**。

| 量 | 处理 |
|---|---|
| `u,v` | 已知（`u=U`，`v=0`），稳态 |
| `w`（迎角） | 自由，稳态 |
| `Ω_b` | 已知 0，稳态 |
| 位置 `z` | 已知高度，稳态（平飞） |
| `θ` | 自由，稳态；靠 `ż=0` 锁平飞 |
| 结构 `η,η̇` | 未知 + 稳态 |
| 升降舵、巡航桨 E2/E3/E6/E7 | 自由 |
| 悬停桨、副翼、方向舵、倾转 | 默认固定 |

### 2. `restore_wagner_restored`

冻结已配平的 η，采样变形后的准定常 `CL_qs`，代数恢复  
`x0 = (I-A)\(B*CL_qs)`，再打开四个面的 `Wagner_SS`。结构状态不重解。

## 目录

```
Coupling_Cruise_Trim/
  run_cruise_trim.m
  initial_main_V1_1.m
  aeroelastic.m
  aerodynamic_model_V1_1_Coupling.slx
  Library/Prop_gryo.slx
  lib/                  桨表 + 片条辅助函数
  thrust/               推进 MATLAB Function
  trim/                 配平与 Wagner 恢复
  polars/               NACA 2412 + 带舵 ΔC
  struct/results/       节点、单元、15 阶模态
```

不要把 `slprj/`、`*.autosave` 提交到 git。
