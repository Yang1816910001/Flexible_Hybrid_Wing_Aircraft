# Prop_gryo 原理验证

用 `test.slx` 对照 MATLAB 回放，检查 `Library/Prop_gryo.slx` 的入流、查表、\(C^\top\) 和陀螺。表来自 `../Prop_dimentionless`（先在那边跑过 `main`）。

## 文件

| 路径 | 作用 |
|---|---|
| `../test.slx` | 手改 Constant、看 Display |
| `../Library/Prop_gryo.slx` | 推进库。Prop1 举升，Prop2 倾转+陀螺 |
| `make_verify_cases.m` | 打印隔离工况的期望 `Va/Vl/Wp/Wb` |
| `verify_prop_gryo_test.m` | 把一组 Display 与回放写入 `Prop_gryo_verify.xlsx` |
| `Prop_gryo_verify.xlsx` | 2026-09-07 快照（改 `C^\top` 之前的 Display 也在表里） |

```matlab
cd('D:\Load_Estimation_Case\Coupling_Model_V1.0')
addpath('tests'); addpath('lib'); addpath('Library')
load_prop_tables          % cfd2d_* / cfd3d_* 进 base
make_verify_cases         % 打印期望值
open_system('test')
```

`verify_prop_gryo_test` 里的 Display 数组要改成你当前窗口的数再跑，否则 Excel 是旧快照。

## 轴与口

桨表机体系：**x 前、y 右、z 上**（和耦合结构系 z 向下不是同一套）。

Aerospace **XYZ**，口令 `[安装角, 倾转角, 0]`（弧度）：

\[
C = R_y(\theta)\,R_x(\Gamma),\qquad
\mathbf{v}_{\mathrm{eng}}=C\mathbf{v}_b,\qquad
\mathbf{F}_b=C^\top\mathbf{F}_e
\]

\(\Gamma\) 是绕机体 **x** 的外倾。\(\alpha=\beta=0\) 且 \(\mathbf{V}\) 沿 x 时，\(V_a=V\cos\theta\)、\(V_l=V|\sin\theta|\)，**与 \(\Gamma\) 无关**。

| 口 | 单位 | 写法 |
|---|---|---|
| `Vb`, `Omegab`, `r_arm`, `r_cg2k` | m, rad/s | **3×1** `[10; 0; 0]`，不要 `[10 0 0]` |
| `phi_cant`, `eta`, `dot_eta` | rad | 如 `-7/180*pi` |
| `RPS` | **rps** | `500/60`，不要直接填 RPM |
| `BladePitchAngle_DEG` | deg | 倾转表桨距，常用 `11` |

Display 总线顺序：`Wp`(6) → `Tip_mach` → `Va` → `Vl` → `Wb`(6)。

- **`Wp`**：桨轴系 \([F_x,F_y,F_z,M_x,M_y,M_z]\)，气动，不含陀螺  
- **`Wb`**：\(C^\top W_e\)，再在后三个力矩上加陀螺  

`n=\mathrm{RPS}\)，\(M_\mathrm{tip}=\pi n D/a_\infty\)。`RPS=500/60` 时 `Tip_mach=0.2308`。对不上就是转速口当了 RPM。

量纲还原（LUT 给的是剥马赫后的 \(C_\mathrm{inc}\)）：

\[
C=C_{\mathrm{inc}}(1+k M_{\mathrm{tip}}^2),\quad
F=C_{ef}\rho n^2 D^4,\quad
M=C_{em}\rho n^2 D^5
\]

| | \(k_T\) | \(k_Q\) | 工作区 |
|---|---|---|---|
| 举升 2D | 0.16732 | 0.22154 | `cfd2d_kT/kQ` |
| 倾转 3D | **0.170294** | **0.217845** | `cfd3d_kT/kQ` |

倾转块若仍乘 `cfd2d_kT`，本工况相对差约 \(1.6\times10^{-4}\)，查表对错看不出来。

## 仿真共用

每组都先固定这些，再只改表里那一两格：

```matlab
RPS    = 500/60;
pitch  = 11;
Omegab = [0; 0; 0];
dot_eta = 0;
r_arm  = [0; 0; 0];   % 先排除力臂串入流
```

建议顺序：**0 → C → D → A → H1 → H3 → B → F → I**。

## 气动工况

期望值由 `lift_inflow` / `engine_inflow` + 查表得到。`Wb` 一律按 **\(C^\top W_e\)**。

### 0 — `r_arm` 是否漏进速度

工况 **D**，只改 `r_arm`：`[0;0;0]` vs 标量 `3`。\(\Omega=0\) 时 `Va/Vl/Wp` 应完全一样。若标量 `3` 变成 `7.282/6.861`，力臂在改入流。

### A — Prop1 悬停（LUT + \(C^\top\)）

`Vb=[10;0;0]`，`cant=-7°`，Prop1（倾转锁 90°）

| | 期望 |
|---|---|
| `Va`, `Vl` | `0`, `10` |
| `Wp` 力 | `[734.84, -2.151, -49.70]` |
| `Wp` 矩 | `[-94.10, 146.84, -133.32]` |
| `Wb` 力 \(C^\top\) | `[-49.70, -91.69, -729.10]` |
| `Wb` 矩 \(C^\top\) | `[-133.32, 157.21, 75.51]` |

力在 \(+z\)（`[49.6, 3.9, 734.8]`）说明还在用 \(C W_e\)，没有转置。

### B — 倾转 0°

Prop2：`eta=0`，`cant=-7°`，`Vb=[10;0;0]`

| | 期望 |
|---|---|
| `Va`, `Vl` | `10`, `0` |
| `Wp` 力 | `[587.94, -6.646, -24.38]` |
| `Wb` 力 \(C^\top\) | `[587.94, -9.567, -23.39]` |

### C / D / E — 45° 与外倾

都是 Prop2，`eta=45°`，`Vb=[10;0;0]`，只改 cant。

| | C `cant=0` | D `cant=-7°` | E `cant=+7°` |
|---|---|---|---|
| `Va`, `Vl` | **7.071, 7.071** | **同样** | **同样** |
| `Wp` 力 | `[550.72, -10.67, -50.04]` | 与 C 相同 | 与 C 相同 |
| `Wb` 力 \(C^\top\) | `[354.03, -10.67, -424.80]` | `[354.03, -62.37, -420.33]` | `[354.03, 41.18, -422.94]` |

来流沿 x 时，绕 x 的 7° 不改 `Va/Vl` 和桨轴系 `Wp`，只改 `Wb` 的侧向。D 若出现 `7.282/6.861`，不是安装角几何。

### F / G — 来流沿 +z（外倾才进 Ja）

| | F Prop1 悬停 | G Prop2 45° |
|---|---|---|
| `Vb` | `[0; 0; 10]` | `[0; 0; 10]` |
| `cant` | `-7°` | `-7°` |
| `Va`, `Vl` | **-9.926**, **1.219** | **-7.018**, **7.123** |
| `Wp` 力 | `[611.93, 0.833, -2.447]` | `[668.37, -0.964, -34.59]` |
| `Wb` 力 \(C^\top\) | `[-2.45, -73.75, -607.47]` | `[448.15, -61.53, -493.24]` |

F 的 \(V_l\approx 10|\sin 7^\circ|\)。若仍是 `Va≈0, Vl≈10`，z 没转到桨轴。

## 陀螺

用 **Prop2 − 无陀螺块** 的差。`Wp` 必须相同，只比 `Wb` 后三个力矩。

库里（倾转块）：

\[
\mathbf{T}_A=C^\top[1,0,0]^\top,\quad
\mathbf{H}=I(-2\pi n)\mathbf{T}_A,\quad
\mathbf{M}= \mathbf{H}\times[0,\dot\eta,0]^\top
\]

\(I=1\) 占位。叉乘只用 **`dot_eta`**，**不用 `Omegab`**。完整物理应是 \(\boldsymbol{\Omega}_{\mathrm{nac}}=\boldsymbol{\Omega}_b+\dot\eta\mathbf{e}_{\mathrm{hinge}}\)，那是补口的问题，不是 H3 的判据。

气动用 **D**，`r_arm=[0;0;0]`。此组 \(\mathbf{H}=[-37.02,\ 4.51,\ 36.75]\)。

| 组 | 改什么 | 期望 `dM` |
|---|---|---|
| H1 | `Omegab=0`, `dot_eta=0` | `[0; 0; 0]`（若 `[3.7; 0; -3.7]` 即零速率残差） |
| H2 | `Omegab=[0;0.2;0]`, `dot_eta=0` | 与 H1 相同（库没叉 \(\boldsymbol{\Omega}_b\)） |
| H3 | `Omegab=0`, **`dot_eta=0.2`** | **`[-7.35, 0, -7.40]`**（\(\mathbf{H}\times\hat y\)）；反序叉乘则反号 |
| H4 | H3 的 \(\dot\eta\) 或 `RPS` 加倍 | `\|dM\|` 加倍 |

H3 的 `My` 应为 0。不要在 `Wp` 里找陀螺。

## 未覆盖

- 右发镜像 `diag([1,-1,1,-1,1,-1])`（`test.slx` 接的是左发 Prop1/2）
- `lift_inflow` 与 Aerospace 的 Rx 外倾符号（α=β=0 时 Ja/Jl 看不出来，用 F）
- 耦合体轴 z 向下 vs 桨表 z 向上（验证口按桨表 z 上）
- \(I=1\) 不是真实惯量；比例关系仍可验
- 点速度 \(\boldsymbol{\Omega}\times r\)（上述组把 `r_arm` 置零了）
