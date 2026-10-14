%% V1.1 植物线工作区 → aerodynamic_model_V1_1_RESTORED.slx
%  仓库根目录运行:  initial_main_V1_1
%  空速、8 路 RPM、桨距、迎角、舵面、倾转、求解开关：只改下面自定义块。

% 配平求解器会设 trim_mode_override='rigid' 再调本脚本
mode_keep = [];
if exist('trim_mode_override', 'var')
    mode_keep = trim_mode_override;
end
condition_keep = struct();
if exist('trim_condition_override', 'var')
    condition_keep = trim_condition_override;
end
clearvars -except mode_keep condition_keep;
clc;

%% ========== 自定义：配平方案、工况、待求量与初值 ==========
%% 0. 配平方案选择
% 这只是算法开关，不属于飞行工况，也不是配平待求量。
% 'rigid'：零弹性变形下的刚体纵向配平
% 'aeroelastic'：刚体平衡与静态弹性变形同时收敛
%  只有保存结果的完整工况签名与本文件一致时，才会读取 last_*_trim.mat。
%  任一固定工况、自由量分组或模型文件改变后，都必须重新配平。
trim_mode = 'rigid';

%% 1. 设定工况与固定构型（配平过程中保持不变）
% 飞行环境 / 目标速度
U = 55;               % [m/s] 真空速；当前配平保持该速度量级
altitude_m = 0;       % [m] 高度；当前只记录，尚未自动修改 rho/声速
rho = 1.225;          % [kg/m^3] 空气密度，应与高度工况一致
g = 9.80665;          % [m/s^2] 重力加速度

% 推进与倾转构型：目前都是固定量，不进入 fmincon 待求向量
BladePitch_deg = 34;          % [deg] 桨距，表节点 [9.9 13 22 31 34 40]
tilt_deg = zeros(8, 1);       % [deg] 8 路倾转角
dot_tilt_degps = zeros(8, 1); % [deg/s] 静态配平通常必须为 0

%% 2. 配平待求量及其初值
% 规则：free_*=true 时该值是 fmincon 初值；false 时它是固定输入。
% alpha 是待求状态量；RPM和三个舵面是配平控制输入。
alpha0_deg = 5;               % [deg] 迎角状态初值/固定值
elevator0_deg = -2.5870;      % [deg] 升降舵输入初值/固定值
aileron0_deg = 0;             % [deg] 副翼输入初值/固定值
rudder0_deg = 0;              % [deg] 方向舵输入初值/固定值

% 8 路转速 [RPM]，顺序 E1..E8（升力 1/4/5/8，倾转 2/3/6/7）
rpm_cmd = [ ...
    0; ...   % E1 升力
    400; ... % E2 倾转
    400; ... % E3 倾转
    0; ...   % E4 升力
    0; ...   % E5 升力
    400; ... % E6 倾转
    400; ... % E7 倾转
    0];      % E8 升力
% 每组共用一个 RPM。[] = 8 路全部固定。数值向量 = 一组；cell = 多组。
% 左右对称：14 前升力，23 前倾转，58 后升力，67 后倾转。
rpm_trim_index = {[1 4], [2 3], [5 8], [6 7]};
% 与上面对应：true=fmincon 可调，false=冻结为上面的 rpm_cmd（组内仍按你写的值）。
rpm_group_free = [false, true, false, true];

% 哪些状态/控制输入由求解器调整；RPM 由 rpm_group_free 逐组控制。
free_alpha = true;
free_elevator = true;
free_aileron = false;
free_rudder = false;

%% 3. 非配平初始状态与姿态约束
% 这些量用于仿真初始化，当前纵向求解器不调整它们。
phi0_deg = 0;
psi0_deg = 0;
theta_equals_alpha = true;     % false 时用 theta0_deg
theta0_deg = -5.1333;
Omega_b0 = zeros(3, 1);        % [p;q;r] rad/s
Position0 = zeros(3, 1);       % [x;y;z] m
Velocity_b0 = [];              % [] = U*[cos a; 0; sin a]

%% 4. 数值求解设置（不是物理工况，也不是配平输入）
% 仿真时钟（图 FixedStep=Ts，StopTime=sim_time）
Ts = 0.001;                     % [s] 离散步长
sim_time = 15;                   % [s] 常规仿真时长

% 限幅（只约束 free / rpm_trim_index 的量）
alpha_limit_deg = [-10 18];   % 与当前无舵翼型极曲线覆盖范围一致
rpm_limit = [0 2000];
elevator_limit_deg = [-20 20];
aileron_limit_deg = [-20 20];
rudder_limit_deg = [-20 20];

% fmincon 有限差分步长（变量会先按上下限归一化）
trim_fmincon_algorithm = 'interior-point'; % 主用；改成 'sqp' 即切换到备选 SQP
dx_alpha_deg = 0.05;
dx_rpm = 10;
dx_surface_deg = 0.05;

% 配平短仿真 / 迭代
trim_quasi_time_s = 0.01;      % [s] 准定常优化，只取初始状态附近的瞬时力/力矩
trim_simulation_time_s = 0.60; % [s] 恢复 Wagner 后的最终复核
trim_tail_fraction = 0.25;
trim_tolerance = 2e-3;
trim_preference_weight = 1e-4; % 欠定解唯一化，抑制气弹外环中舵面/RPM跳动
elastic_tolerance = 2e-3;
max_outer_iterations = 30;
max_inner_iterations = 15;
max_iterations = 30;
elastic_relaxation = 0.15;     % 气弹外环固定小步长，避免舵面随模态变形大幅摆动

%% 5. 模型、结构、气动和桨数据（模型配置，不属于单次配平待求量）
% 植物线
model = 'aerodynamic_model_V1_2_Wrench';
FullPath = mfilename('fullpath');
% 如果是在命令行直接运行，mfilename 返回空，则用当前工作目录
if isempty(FullPath)
    currentDir = pwd;
else
    % 提取所在目录（去掉脚本文件名）
    currentDir = fileparts(FullPath);
end
% 结构结果 + 几何
struct_dir = fullfile(currentDir, 'Library');
mac_m           = 2.56;   % [m]  主翼平均气动弦长（保留旧变量名供现有模块使用）
v_tail_mac_m    = 0.80;   % [m]  V尾平均气动弦长；有实测值时在这里替换
v_tail_cant_deg = 42;     % [deg] V 尾外倾（当地迎角 / load_strips）
span_m          = [];     % [m]  参考展长；[] = 2*max|y_strip|
zeta            = 0.02;   % [-]  弹性 Rayleigh 阻尼比

% 气动表 / Wagner
polar_file      = '';     % 无舵 polar；'' = polars/naca2412_Re6p81e6_Ncrit9.pol
flap_polar_file = '';     % 带舵 ΔC 表；'' = polars/flap_polars.mat
wagner_a0       = 1;      % Wagner 升力斜率增益
ctrl_flap_kappa = [];     % 舵效折减；[] = 表里的 kappa，没有则 1

% 桨表路径
prop_table_dir  = '';   % CFD_HOVER_2D / CFD_DATA_3D；'' = load_prop_tables 默认搜索
%% =================================================

if ~isempty(mode_keep)
    trim_mode = mode_keep;
end

% 函数化配平入口可通过该结构覆盖工况。覆盖发生在任何片条气动量
% （qS/qSc、Wagner 时间尺度等）生成之前，确保求解状态与气动参数
% 使用同一个速度、密度和构型。
if ~isempty(fieldnames(condition_keep))
    if isfield(condition_keep, 'model'),             model = char(string(condition_keep.model)); end
    if isfield(condition_keep, 'U_mps'),              U = condition_keep.U_mps; end
    if isfield(condition_keep, 'altitude_m'),         altitude_m = condition_keep.altitude_m; end
    if isfield(condition_keep, 'rho_kgpm3'),          rho = condition_keep.rho_kgpm3; end
    if isfield(condition_keep, 'blade_pitch_deg'),    BladePitch_deg = condition_keep.blade_pitch_deg; end
    if isfield(condition_keep, 'tilt_deg'),           tilt_deg = condition_keep.tilt_deg(:); end
    if isfield(condition_keep, 'dot_tilt_degps'),     dot_tilt_degps = condition_keep.dot_tilt_degps(:); end
end

trim_case = struct();
trim_case.model = model;
trim_case.U_mps = U;
trim_case.altitude_m = altitude_m;
trim_case.rho = rho;
trim_case.g = g;
trim_case.blade_pitch_deg = BladePitch_deg;
trim_case.rpm_cmd = rpm_cmd(:);
if iscell(rpm_trim_index)
    trim_case.rpm_trim_index = rpm_trim_index;
else
    trim_case.rpm_trim_index = rpm_trim_index(:).';
end
trim_case.rpm_group_free = logical(rpm_group_free(:)).';
trim_case.tilt_deg = tilt_deg(:);
trim_case.dot_tilt_degps = dot_tilt_degps(:);
trim_case.tilt_cmd_rad = deg2rad(trim_case.tilt_deg);
trim_case.dot_tilt_cmd_radps = deg2rad(trim_case.dot_tilt_degps);
trim_case.alpha0_deg = alpha0_deg;
trim_case.elevator0_deg = elevator0_deg;
trim_case.aileron0_deg = aileron0_deg;
trim_case.rudder0_deg = rudder0_deg;
trim_case.phi0_deg = phi0_deg;
trim_case.psi0_deg = psi0_deg;
trim_case.theta_equals_alpha = theta_equals_alpha;
trim_case.theta0_deg = theta0_deg;
trim_case.Omega_b0 = Omega_b0(:);
trim_case.Position0 = Position0(:);
trim_case.Velocity_b0 = Velocity_b0;
trim_case.free_alpha = free_alpha;
trim_case.free_elevator = free_elevator;
trim_case.free_aileron = free_aileron;
trim_case.free_rudder = free_rudder;
trim_case.alpha_limit_deg = alpha_limit_deg;
trim_case.rpm_limit = rpm_limit;
trim_case.elevator_limit_deg = elevator_limit_deg;
trim_case.aileron_limit_deg = aileron_limit_deg;
trim_case.rudder_limit_deg = rudder_limit_deg;
trim_case.dx_alpha_deg = dx_alpha_deg;
trim_case.dx_rpm = dx_rpm;
trim_case.dx_surface_deg = dx_surface_deg;
trim_case.fmincon_algorithm = trim_fmincon_algorithm;
trim_case.quasi_simulation_time_s = trim_quasi_time_s;
trim_case.simulation_time_s = trim_simulation_time_s;
trim_case.tail_fraction = trim_tail_fraction;
trim_case.trim_tolerance = trim_tolerance;
trim_case.trim_preference_weight = trim_preference_weight;
trim_case.elastic_tolerance = elastic_tolerance;
trim_case.max_outer_iterations = max_outer_iterations;
trim_case.max_inner_iterations = max_inner_iterations;
trim_case.max_iterations = max_iterations;
trim_case.elastic_relaxation = elastic_relaxation;
trim_case.Ts = Ts;
trim_case.mac_m = mac_m;
trim_case.v_tail_mac_m = v_tail_mac_m;
trim_case.v_tail_cant_deg = v_tail_cant_deg;
trim_case.zeta = zeta;
trim_case.struct_dir = struct_dir;
trim_case.polar_file = polar_file;
trim_case.flap_polar_file = flap_polar_file;
trim_case.wagner_a0 = wagner_a0;
trim_case.ctrl_flap_kappa = ctrl_flap_kappa;
trim_case.prop_table_dir = prop_table_dir;
clear altitude_m rpm_cmd rpm_trim_index rpm_group_free tilt_deg dot_tilt_degps ...
    alpha0_deg elevator0_deg aileron0_deg rudder0_deg ...
    phi0_deg psi0_deg theta_equals_alpha theta0_deg Omega_b0 Position0 Velocity_b0 ...
    free_alpha free_elevator free_aileron free_rudder ...
    alpha_limit_deg rpm_limit elevator_limit_deg aileron_limit_deg rudder_limit_deg ...
    trim_fmincon_algorithm dx_alpha_deg dx_rpm dx_surface_deg ...
    trim_quasi_time_s trim_simulation_time_s trim_tail_fraction trim_tolerance ...
    trim_preference_weight elastic_tolerance ...
    max_outer_iterations max_inner_iterations max_iterations elastic_relaxation;

aeroelastic.setup_paths();
addpath(fullfile(fileparts(mfilename('fullpath')), 'trim'));
if isempty(prop_table_dir)
    load_prop_tables();
else
    load_prop_tables(prop_table_dir);
end
v_tail_cant_rad = deg2rad(v_tail_cant_deg);

%% 结构片条 + 振型
strips = aeroelastic.load_strips(struct_dir, ...
    'mac_m', mac_m, 'v_tail_mac_m', v_tail_mac_m, ...
    'v_tail_cant_deg', v_tail_cant_deg, ...
    'keep_rigid', false);  % 模态阵 / Tq / T_m2* 不含前 6 刚体阶
if isempty(span_m)
    span = 2 * max(abs(strips.y_strip(:)));
else
    span = span_m;
end
n_rigid = strips.n_rigid;
n_elastic = strips.n_elastic;
n_modes = strips.n_modes;
Phi = strips.Phi;
nodes = strips.nodes;
aero = strips.by_surface;

%% 图里用到的映射 / 惯量 / 力臂（机体系）
load_maps = aeroelastic.load_maps(Phi, nodes);
T_prop2W = load_maps.T_prop2W;
Tq_mass = load_maps.Tq_mass;
T_aero2W = aeroelastic.wing_aileron_to_W(strips).T;
eng = load_maps.engine_geom;
R_sb = eng.R_sb;
r_cg2kink_b = eng.r_cg2kink_b;

grav = aeroelastic.gravity(nodes, 'g', g);
W_mass_g = grav.W_mass;
m_total_kg = grav.m_total_kg;
nodal = aeroelastic.nodal_mass(nodes);
M_node = nodal.M;       % 6×6×89，(:,:,node_id)；惯性载荷 W_in = -M*a
m_node = nodal.m_node;  % 89×1，与 node_id 对齐
% 翼/尾载荷节点：梁站 + 该侧中央翼盒 + 该侧发动机臂；尾翼只有 V 尾站
surface_nodes = aeroelastic.surface_load_nodes(nodes);
I_cg_b = grav.I_cg_b;
I_cg_b_inv = grav.I_cg_b_inv;
T_m2body = aeroelastic.node_kinematics(Phi, nodes).T_m2body;
r_cg2node_b = aeroelastic.node_arms(nodes, 'R_sb', R_sb).r_cg2node_b;

%% 极曲线 + V1.1 occupancy 掩膜 + ΔC LUT
if isempty(polar_file)
    polar = aeroelastic.polar();
else
    polar = aeroelastic.polar(polar_file);
end
polar_alpha_deg = polar.alpha_deg(:).';
polar_CL  = polar.CL(:).';
polar_CD  = polar.CD(:).';
polar_CDp = polar.CDp(:).';
polar_CM  = polar.CM(:).';
if isempty(flap_polar_file)
flap_tables = control_surfaces.load_tables();
else
    flap_tables = control_surfaces.load_tables(flap_polar_file);
end
control_surfaces.export_polar_luts(flap_tables);
ctrl_map_aileron_R = aero.wing_R.map_aileron(:);
ctrl_map_aileron_L = aero.wing_L.map_aileron(:);
ctrl_map_tail_R = aero.v_tail_R.map_tail(:);
ctrl_map_tail_L = aero.v_tail_L.map_tail(:);
ctrl_Mix_wing_R = aero.wing_R.Mix;
ctrl_Mix_wing_L = aero.wing_L.Mix;
ctrl_Mix_v_tail_R = aero.v_tail_R.Mix;
ctrl_Mix_v_tail_L = aero.v_tail_L.Mix;
ctrl_family_wing_R = aero.wing_R.flap_family;
ctrl_family_wing_L = aero.wing_L.flap_family;
ctrl_family_v_tail_R = aero.v_tail_R.flap_family;
ctrl_family_v_tail_L = aero.v_tail_L.flap_family;
ctrl_delta_lim_deg = strips.delta_lim_deg;
if isempty(ctrl_flap_kappa)
    ctrl_flap_kappa = 1;
    if isstruct(flap_tables) && isfield(flap_tables, 'kappa')
        ctrl_flap_kappa = flap_tables.kappa;
    end
end

%% Wagner（整机建再按面切开；IC 用 α=0 准定常 CL）
[~, sys_d, info] = aeroelastic.wagner_ss( ...
    'a0', wagner_a0, 'c', strips.c_strip, 'span', span, 'n_strip', strips.n_strip, ...
    'y_strip', strips.y_strip, 'ds', strips.ds, ...
    'U', U, 'rho', rho, 'Ts', Ts, 'output', 'CL');
qS_strip = info.qbar * info.S_strip;
qSc_strip = qS_strip .* info.c_strip;
c_over_2U = (info.c_strip / 2) / U;
CL_qs0 = interp1(polar.alpha_deg, polar.CL, 0, 'linear');
u_qs0 = CL_qs0 * ones(strips.n_strip, 1);
x0_wagner = (eye(size(sys_d.A, 1)) - sys_d.A) \ (sys_d.B * u_qs0);
aeroelastic.split_wagner(sys_d.A, sys_d.B, sys_d.C, sys_d.D, x0_wagner, ...
    qS_strip, qSc_strip, c_over_2U, u_qs0, aero);

%% CG → 片条 AC（机体系；列数 = 该面片条数）
r_cg2ac_b_wing_L = R_sb * aero.wing_L.r_ac_s;
r_cg2ac_b_wing_R = R_sb * aero.wing_R.r_ac_s;
r_cg2ac_b_v_tail_L = R_sb * aero.v_tail_L.r_ac_s;
r_cg2ac_b_v_tail_R = R_sb * aero.v_tail_R.r_ac_s;
ones_1_wing = ones(1, size(r_cg2ac_b_wing_L, 2));
ones_1_tail = ones(1, size(r_cg2ac_b_v_tail_L, 2));

%% 结构离散 SS + 总线 IC（维数跟 n_rigid / n_elastic）
[~, sys_sd] = aeroelastic.structure_ss(strips.omega, ...
    'zeta', zeta, 'Ts', Ts, 'damping', 'rayleigh', 'n_rigid', n_rigid);
As = sys_sd.A;
Bs = sys_sd.B;
Cs = sys_sd.C;
Ds = sys_sd.D;
% Out_Dynamics shape is owned by the diagram (Out Bus Elements).
% Delay blocks use IC=0 (scalar) so they do NOT freeze the bus field list.
% Optional struct kept only for trim helpers that still assign fields by name.
DynamicsAndFlexibleStates_IC = local_dynamics_ic_(n_rigid, n_elastic);

%% 丢掉图不读的中间量（含 split_wagner 多写的 r_ac_s_ / n_hat_s_）
keep_exact = { ...
    'trim_mode', 'trim_case', 'model', 'Ts', 'U', 'sim_time', 'rho', 'g', ...
    'mac_m', 'v_tail_mac_m', 'v_tail_cant_deg', 'v_tail_cant_rad', 'span', 'zeta', ...
    'wagner_a0', 'struct_dir', ...
    'n_modes', 'n_rigid', 'n_elastic', ...
    'ctrl_delta_lim_deg', 'ctrl_flap_kappa', ...
    'BladePitch_deg', ...
    'm_total_kg', 'm_node', 'M_node', 'Phi', 'surface_nodes', ...
    'I_cg_b', 'I_cg_b_inv', 'W_mass_g', ...
    'Tq_mass', 'T_prop2W', 'T_aero2W', 'T_m2body', ...
    'r_cg2kink_b', 'r_cg2node_b', ...
    'r_cg2ac_b_wing_L', 'r_cg2ac_b_wing_R', ...
    'r_cg2ac_b_v_tail_L', 'r_cg2ac_b_v_tail_R', ...
    'ones_1_wing', 'ones_1_tail', ...
    'As', 'Bs', 'Cs', 'Ds', 'x0_structure', ...
    'DynamicsAndFlexibleStates_IC', ...
    'polar_alpha_deg', 'polar_CL', 'polar_CD', 'polar_CDp', 'polar_CM', ...
    'ctrl_map_aileron_L', 'ctrl_map_aileron_R', ...
    'ctrl_map_tail_L', 'ctrl_map_tail_R', ...
    'cfd2d', 'cfd3d'};
keep_prefix = { ...
    'cfd2d_', 'cfd3d_', ...
    'Ad_', 'Bd_', 'Cd_', 'Dd_', 'x0_wagner_', ...
    'qS_strip_', 'qSc_strip_', 'c_over_2U_', ...
    'alpha_du0_', 'G_du_', ...
    'T_m2b_', 'T_m2h_', ...
    'Tq_', 'Tq_D_', 'Tq_CM_', ...
    'ctrl_Mix_', 'ctrl_family_', ...
    'polar_aileron_', 'polar_ruddervator_'};
local_keep_base_(keep_exact, keep_prefix);

%% 巡航配平初值（供模型积分器和输入端使用）
switch lower(trim_mode)
    case 'rigid'
        trim_default = trim_cruise_rigid('init');
        x0_structure = zeros(size(As, 1), 1);
    case 'aeroelastic'
        trim_default = trim_cruise_aeroelastic('init');
        x0_structure = trim_default.x0_structure;
        if numel(x0_structure) ~= size(As, 1)
            error('弹性配平的 x0_structure 维度与当前结构模型不一致。');
        end
    otherwise
        error('trim_mode 只能是 ''rigid'' 或 ''aeroelastic''。');
end
trim_Velocity_b0 = trim_default.Velocity_b0;
trim_Omega_b0 = trim_default.Omega_b0;
trim_Position0 = trim_default.Position0;
trim_Attitude_rad0 = trim_default.Attitude_rad0;

trim_RPM_cmd = trim_default.rpm_cmd;
trim_Tilt_cmd_rad = trim_default.tilt_cmd_rad;
trim_dotTilt_cmd_radps = trim_default.dot_tilt_cmd_radps;
trim_Aileron_deg = trim_default.aileron_deg;
trim_Elevator_deg = trim_default.elevator_deg;
trim_Rudder_deg = trim_default.rudder_deg;

% 四个气动面的 Wagner 状态和诱导迎角延迟初值。
trim_aero_ic = trim_default.aero_ic;
x0_wagner_wing_L = trim_aero_ic.wing_L.x0_wagner;
x0_wagner_wing_R = trim_aero_ic.wing_R.x0_wagner;
x0_wagner_v_tail_L = trim_aero_ic.v_tail_L.x0_wagner;
x0_wagner_v_tail_R = trim_aero_ic.v_tail_R.x0_wagner;
alpha_du0_wing_L = trim_aero_ic.wing_L.alpha_du0;
alpha_du0_wing_R = trim_aero_ic.wing_R.alpha_du0;
alpha_du0_v_tail_L = trim_aero_ic.v_tail_L.alpha_du0;
alpha_du0_v_tail_R = trim_aero_ic.v_tail_R.alpha_du0;
clear trim_aero_ic;

% 顶层六个 Inport 的默认配平输入；模型配置 ExternalInput 填此变量名。
trim_time = [0; sim_time];
trim_input_dataset = Simulink.SimulationData.Dataset;
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries([trim_RPM_cmd.'; trim_RPM_cmd.'], trim_time), 'Throttle');
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries([trim_Tilt_cmd_rad.'; trim_Tilt_cmd_rad.'], trim_time), 'Tilt_cmd');
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries([trim_dotTilt_cmd_radps.'; trim_dotTilt_cmd_radps.'], trim_time), 'dotTilt_cmd');
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries(trim_Aileron_deg * ones(2, 1), trim_time), 'Aileron_deg');
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries(trim_Elevator_deg * ones(2, 1), trim_time), 'Elevator_deg');
trim_input_dataset = trim_input_dataset.addElement( ...
    timeseries(trim_Rudder_deg * ones(2, 1), trim_time), 'Rudder_deg');
clear trim_time;

fprintf(['V1_1 init  U=%.3g m/s  Ts=%g  sim_time=%g s  m=%.4g kg  BladePitch=%.3g deg\n' ...
    '  modes=%d (rigid=%d elastic=%d)  As %s  T_m2body %s  T_aero2W %s  T_prop2W %s\n' ...
    '  r_cg2ac_b wing_L %s  tail_L %s  ones_1_wing %s  ones_1_tail %s\n' ...
    '  r_cg2node_b %s  r_cg2kink_b %s  I_cg_b_inv %s\n'], ...
    U, Ts, sim_time, m_total_kg, BladePitch_deg, ...
    n_modes, n_rigid, n_elastic, ...
    mat2str(size(As)), mat2str(size(T_m2body)), ...
    mat2str(size(T_aero2W)), mat2str(size(T_prop2W)), ...
    mat2str(size(r_cg2ac_b_wing_L)), mat2str(size(r_cg2ac_b_v_tail_L)), ...
    mat2str(size(ones_1_wing)), mat2str(size(ones_1_tail)), ...
    mat2str(size(r_cg2node_b)), mat2str(size(r_cg2kink_b)), ...
    mat2str(size(I_cg_b_inv)));
fprintf('  load nodes  wing_R=%d  wing_L=%d  v_tail_R=%d  v_tail_L=%d\n', ...
    surface_nodes.wing_R.n, surface_nodes.wing_L.n, ...
    surface_nodes.v_tail_R.n, surface_nodes.v_tail_L.n);
fprintf('  section nodes  wing_joint(F07)=%d  v_tail_joint(F14)=%d  tail_bay(F13)=%d\n', ...
    surface_nodes.wing_joint.n, surface_nodes.v_tail_joint.n, ...
    surface_nodes.tail_bay.n);
faces = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
for i = 1:numel(faces)
    nm = faces{i};
    fprintf('  %-10s  Tq %s  G_du %s  map=%s\n', nm, ...
        mat2str(size(eval([ 'Tq_' nm ]))), ...
        mat2str(size(eval([ 'G_du_' nm ]))), ...
        mat2str(size(eval(map_name_(nm)))));
end
fprintf('  polar_aileron_dCL %s  (empty if flap polars missing)\n', ...
    local_sz_('polar_aileron_dCL'));
fprintf('  %s trim: alpha=%.4f deg  elevator=%.4f  aileron=%.4f  rudder=%.4f deg\n', ...
    trim_mode, trim_default.alpha_deg, trim_default.elevator_deg, ...
    trim_default.aileron_deg, trim_default.rudder_deg);
fprintf('  RPM=%s  BladePitch=%.3g deg\n', ...
    mat2str(trim_default.rpm_cmd.', 4), BladePitch_deg);
fprintf('Ready: open_system(''%s'')\n', model);

function ic = local_dynamics_ic_(n_rigid, n_elastic)
    % Trim helper only — NOT wired to Delay. Add fields in trim scripts as needed.
    % n_rigid is typically 0 (elastic-only modal maps).
    n_modes = n_rigid + n_elastic;
    ic = struct( ...
        'rigid_m_displacement', zeros(max(n_rigid, 0), 1), ...
        'rigid_m_velocity', zeros(max(n_rigid, 0), 1), ...
        'elastic_m_velocity', zeros(n_elastic, 1), ...
        'Velocity_b', zeros(3, 1), ...
        'Omega_b', zeros(3, 1), ...
        'Alpha', 0, ...
        'Beta', 0, ...
        'mach', 0, ...
        'Q', zeros(n_modes, 1), ...
        'elastic_displacement', zeros(n_elastic, 1), ...
        'Attitude_rad', zeros(3, 1));
end

function local_keep_base_(exact, prefixes)
    S = evalin('base', 'whos');
    names = {S.name};
    keep = false(size(names));
    for i = 1:numel(names)
        n = names{i};
        if any(strcmp(n, exact))
            keep(i) = true;
            continue
        end
        for k = 1:numel(prefixes)
            if strncmp(n, prefixes{k}, numel(prefixes{k}))
                keep(i) = true;
                break
            end
        end
    end
    drop = names(~keep);
    if ~isempty(drop)
        evalin('base', ['clear ' strjoin(drop, ' ')]);
    end
end

function s = local_sz_(nm)
    if evalin('base', sprintf('exist(''%s'',''var'')', nm))
        s = mat2str(size(evalin('base', nm)));
    else
        s = 'missing';
    end
end

function nm = map_name_(face)
    switch face
        case 'wing_L', nm = 'ctrl_map_aileron_L';
        case 'wing_R', nm = 'ctrl_map_aileron_R';
        case 'v_tail_L', nm = 'ctrl_map_tail_L';
        case 'v_tail_R', nm = 'ctrl_map_tail_R';
        otherwise, nm = '';
    end
end
