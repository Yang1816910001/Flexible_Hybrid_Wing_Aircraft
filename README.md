# 片条气弹模型

开环气弹：结构模态 + 片条气动 + 八发倾转推力。根目录只留入口和模型，其余按用途分到子目录。

```
strip_theories_verify/
├── initial_main.m          ← 唯一入口
├── aerodynamic_model.slx   ← 整机 Simulink
├── aeroelastic.m           ← 片条 / 振型 / Wagner / 结构 SS
├── README.md
├── docs/                   定义与端口表
├── polars/                 NACA 2412 极曲线
├── thrust/                 发动机 MATLAB Function（Simulink 按文件名调用）
├── trim/                   配平、IC、开环作图
├── tests/                  逻辑用例 A–F
├── viz/                    结构 / 片条 / 振型图
└── demos/                  独立推力演示（一般不用）
```

## 怎么跑

MATLAB 里 `cd` 到本目录：

```matlab
initial_main                      % 写工作区，再跑悬停配平开环 1.5 s
do_verify = false; initial_main   % 只初始化，不仿真
```

需要结构包导出（路径写在 `initial_main` 里）：

`Aircraft_structure_redefine/results/` → `nodes.csv`、`elements.csv`、`modal_matrix_15.csv`、`frequencies.csv`

`initial_main` 会调用 `aeroelastic.setup_paths`，把 `thrust` / `trim` / `tests` / `viz` 加进路径。

开环悬停没有姿态回路，大约 1–2 s 姿态还能看。图 21–25：速度、角速度、姿态、弹性模态、前后发推力。

## 根目录（只这三项干活）

| 文件 | 作用 |
| --- | --- |
| `initial_main.m` | 写工作区变量，默认接着开环冒烟 |
| `aerodynamic_model.slx` | 整机：结构 / 片条 / 推进 |
| `aeroelastic.m` | `load_strips`、`load_maps`、Wagner、结构状态空间、`setup_paths` |

不要对现成的 `aerodynamic_model.slx` 再跑 `aeroelastic.wire()`。

## `docs/` 说明写在哪

| 文件 | 内容 |
| --- | --- |
| `气弹模型总览.md` | 整机回路总览（幻灯片） |
| `转换矩阵说明.md` | 轴系、$R_{sb}$、$\Phi$、$T_{m2\mathrm{body}}$、$T_q$ |
| `节点质量与梁属性定义.md` | 结构节点、梁、质量从哪来 |
| `气动片条定义.md` | 50 片条、迎角、Wagner |
| `推进载荷定义.md` | 八发、短梁、扳手 |
| `feishu/` | 模块端口表（给接线核对） |

公式请用 Markdown Preview 看，编辑器源码里 `$...$` 不会渲染。

## `thrust/` 发动机函数（Simulink 按文件名调用，勿合并）

| 文件 | 作用 |
| --- | --- |
| `engine_stub_pose.m` | 短梁绕折点转，给出推力轴与发动机点 |
| `engine_pack_wrenches.m` | 推力 → 结构系节点扳手 `W_prop` |
| `engine_struct_to_body.m` | 结构系 → 机体系 |
| `engine_point_velocity.m` | 发动机点速度 |
| `engine_dcm_body_to_eng.m` | 机体 → 发动机轴 DCM |
| `engine_alpha_beta.m` | 发动机迎角 / 侧滑（观测） |
| `engine_body_triad.m` | 机体轴三连 |
| `engine_rodrigues.m` | 绕轴旋转 |

## `trim/` 配平与开环

悬停（`initial_main` 默认走这条）：

| 文件 | 作用 |
| --- | --- |
| `trim_hover_static.m` | 解前后推力 |
| `trim_build_hover_context.m` | 悬停配平用的几何 / 质量 |
| `hover_static_Q.m` / `hover_static_residual.m` | 悬停广义力与残差 |
| `apply_hover_trim_ic.m` | 写入 IC / 油门 / Wagner / qS |
| `verify_aerodynamic_model.m` | 开环 1.5 s 并出图 |

巡航：

| 文件 | 作用 |
| --- | --- |
| `trim_cruise_static.m` | 巡航静态配平 |
| `trim_build_context.m` | 巡航配平上下文 |
| `cruise_static_Q.m` / `cruise_static_residual.m` | 巡航广义力与残差 |
| `trim_cruise_sim.m` / `cruise_sim_residual.m` | 短时仿真配平 |
| `apply_cruise_trim_ic.m` | 巡航 IC |
| `verify_cruise_trim.m` / `plot_cruise_response.m` | 巡航开环检查 |

其它：`strip_local_alphas.m` 主翼/V 尾当地迎角；`wire_response_outs.m` 根层速度、姿态、推力出口。`last_*.mat` 是上次配平缓存，可删，下次会重算。

## `tests/` / `polars/` / `viz/` / `demos/`

| 路径 | 作用 |
| --- | --- |
| `tests/run_all_logic.m` | 跑 Case A–F |
| `tests/caseA_…F_*.m` | 几何、重力、推进、气动、冒烟、量级 |
| `tests/ensure_aero_workspace.m` | 没有工作区就调 `initial_main` |
| `tests/logic_check.m` | PASS/FAIL 计数 |
| `polars/` | 片条 CD/CM 用的 NACA 2412 极曲线 |
| `viz/plot_structure_aero_modes.m` | 结构 / 片条 / 振型 |
| `demos/thrust_demo.slx` | 只含推进的独立图；整机里已有推进 |

## 其它命令

```matlab
trim_hover_static('mode','pitch2')              % 只做悬停配平
trim_cruise_static('mode','pitch3')             % 巡航静态配平
trim_cruise_sim                                 % 巡航短时仿真配平
verify_cruise_trim                              % 巡航开环
run_all_logic                                   % A–F
plot_structure_aero_modes                       % 几何 / 振型图
```
