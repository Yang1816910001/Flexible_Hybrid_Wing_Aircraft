%% 气弹工作区初始化（入口）
%  在仓库根目录运行:
%    initial_main                 % 写工作区 + 悬停配平开环 1.5 s
%    do_verify=false; initial_main
%
% 桨表：lib/load_prop_tables 读 CFD_HOVER_2D.mat / CFD_DATA_3D.mat
% （默认 ../Prop_dimentionless/results）。详见 README.md。

clearvars -except do_verify;
clc;
if ~exist('do_verify', 'var') || isempty(do_verify)
    do_verify = true;
end
aeroelastic.setup_paths();
load_prop_tables();   % cfd2d_* / cfd3d_*；表在 Prop_dimentionless/results 或本目录 lib

%% 来流 / 时间 / 气压高度（涡扇，非模态 Position）
rho = 1.225;
dt = 0.001;
Ts = dt;
zeta = 0.02;
g = 9.80665;
U = hypot(38.69221536838454, 3.1651981735079);  % Gain 1/U
Altitude_m = 0;   % 真实气压高度 [m]，进 Propulsion m2ft
prop_Mach = 0;    % 涡扇查表马赫；悬停钉 0，巡航配平会改成 U/a
sim_time = 5;     % 模型 StopTime 表达式
v_tail_cant_rad = deg2rad(42);  % V 尾外倾，当地迎角用

%% 结构片条 + 振型
struct_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
strips = aeroelastic.load_strips(struct_dir, 'mac_m', 2.56, 'v_tail_cant_deg', 42);
span = 18.0;
ds_strip = strips.ds;
c_strip = strips.c_strip;
n_strip = strips.n_strip;
n_modes = strips.n_modes;
n_rigid = strips.n_rigid;
n_elastic = strips.n_elastic;
Phi = strips.Phi;
nodes = strips.nodes;
omega = strips.omega;
y_strip = strips.y_strip;
aero = strips.by_surface;

%% 推进 / 重力 / 体轴运动学
load_maps = aeroelastic.load_maps(Phi, nodes);
Tq_prop = load_maps.Tq_prop;    % n_modes×48  → Q
T_prop2W = load_maps.T_prop2W;  % ndof×48     → 全节点 W
Tq_mass = load_maps.Tq_mass;
ndof = load_maps.ndof;
eng = load_maps.engine_geom;
e_thrust_cruise = eng.e_cruise;
e_thrust_hover = eng.e_hover;
eng_is_tilt = double(eng.is_tilt);
p_kink_m = eng.p_kink_m;
p_eng0_m = eng.p_eng_m;
cg_m = eng.cg_m;
r_cg2kink_m = eng.r_cg2kink_m;  % 3×8 结构系，列序 E1..E8
r_cg2kink_b = eng.r_cg2kink_b;  % 3×8 机体系，R_sb * r_cg2kink_m
R_sb = eng.R_sb;

W_mass_g = aeroelastic.gravity(nodes, 'g', g).W_mass;
T_m2body = aeroelastic.node_kinematics(Phi, nodes).T_m2body;

%% 极曲线 → LUT
polar = aeroelastic.polar();
polar_alpha_deg = polar.alpha_deg(:).';
polar_CL  = polar.CL(:).';
polar_CD  = polar.CD(:).';
polar_CDp = polar.CDp(:).';
polar_CM  = polar.CM(:).';

%% Wagner（整机建、再按面切开并 assignin）
[~, sys_d, info] = aeroelastic.wagner_ss( ...
    'a0', 1, 'c', c_strip, 'span', span, 'n_strip', n_strip, ...
    'y_strip', y_strip, 'ds', ds_strip, ...
    'U', U, 'rho', rho, 'Ts', Ts, 'output', 'CL');
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;
S_strip = info.S_strip;
qS_strip = info.qbar * S_strip;
qSc_strip = info.qbar * S_strip .* info.c_strip;
c_over_2U = (info.c_strip / 2) / U;
CL_qs0 = interp1(polar.alpha_deg, polar.CL, 0, 'linear');
u_qs0 = CL_qs0 * ones(n_strip, 1);
x0_wagner = (eye(size(Ad, 1)) - Ad) \ (Bd * u_qs0);
sz_Ad = size(Ad);
S_tot = sum(S_strip);

aeroelastic.split_wagner(Ad, Bd, Cd, Dd, x0_wagner, ...
    qS_strip, qSc_strip, c_over_2U, u_qs0, aero);

%% 结构模态 SS
[~, sys_sd] = aeroelastic.structure_ss(omega, ...
    'zeta', zeta, 'Ts', Ts, 'damping', 'rayleigh', 'n_rigid', n_rigid);
As = sys_sd.A;
Bs = sys_sd.B;
Cs = sys_sd.C;
Ds = sys_sd.D;
x0_structure = zeros(size(As, 1), 1);

%% 总线对象（无：无 η̈ / DCM / 惯性诊断量）
DynamicsAndFlexibleStates = local_dynamics_bus_();
DynamicsAndFlexibleStates_IC = Simulink.Bus.createMATLABStruct('DynamicsAndFlexibleStates');

fprintf(['strips=%d  modes=%d (%d+%d)  S=%.4g m^2  U=%.3g m/s  Altitude_m=%.3g m\n' ...
    'Wagner Ad %s  As %s  T_m2body %s  Tq_prop %s  r_cg2kink %s\n'], ...
    n_strip, n_modes, n_rigid, n_elastic, S_tot, U, Altitude_m, ...
    mat2str(sz_Ad), mat2str(size(As)), mat2str(size(T_m2body)), ...
    mat2str(size(Tq_prop)), mat2str(size(r_cg2kink_m)));
fprintf('  T_prop2W %s  Tq_prop %s  (W=T_prop2W*W_prop, Qp=Tq_prop*W_prop)\n', ...
    mat2str(size(T_prop2W)), mat2str(size(Tq_prop)));
disp('r_cg2kink_m (struct) / r_cg2kink_b (body), CG→kink [m], cols E1..E8:');
disp(r_cg2kink_m);
disp(r_cg2kink_b);
for nm = aero.surface_names
    p = aero.(nm);
    fprintf('  %-10s  n=%2d  T_m2b %s  Tq %s  G_du %s\n', ...
        char(nm), p.n_strip, mat2str(size(p.T_m2b)), mat2str(size(p.Tq)), ...
        mat2str(size(p.G_du)));
end

clear Ad Bd Cd Dd qS_strip qSc_strip c_over_2U CL_qs0 u_qs0 x0_wagner ...
    sys_d info S_strip sys_sd load_maps eng polar ...
    Phi nodes omega y_strip ds_strip c_strip span rho dt zeta g ...
    struct_dir strips aero sz_Ad S_tot n_strip n_rigid n_elastic nm p ndof;
% 保留 n_modes、Ts、U、Altitude_m、sim_time、v_tail_cant_rad、x0_structure

if do_verify
    verify_aerodynamic_model();
end

function bus = local_dynamics_bus_()
    names = {'rigid_m_displacement', ...
        'rigid_m_velocity','elastic_m_velocity', ...
        'Velocity_b','Omega_b','Alpha','Beta','mach', ...
        'Q','elastic_displacement'};
    dims = [6 6 15 3 3 1 1 1 21 15];
    elems = Simulink.BusElement;
    for i = 1:numel(names)
        e = Simulink.BusElement;
        e.Name = names{i};
        e.Dimensions = dims(i);
        e.DataType = 'double';
        e.Complexity = 'real';
        e.DimensionsMode = 'Fixed';
        elems(i) = e;
    end
    bus = Simulink.Bus;
    bus.Elements = elems;
    bus.Description = 'Strip/Propulsion states (no eta_ddot diagnostics)';
end
