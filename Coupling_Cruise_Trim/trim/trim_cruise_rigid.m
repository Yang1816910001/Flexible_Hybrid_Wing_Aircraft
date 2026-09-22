function result = trim_cruise_rigid(action, U_mps)
%TRIM_CRUISE_RIGID  刚体纵向配平（求解 + 仿真初值）。
%  工况只改 initial_main_V1_1.m 开头自定义块。不修改、不保存模型。
%
%  result = trim_cruise_rigid            % 短仿真 + fmincon
%  result = trim_cruise_rigid('init')    % initial_main_V1_1 刚体初值
%  result = trim_cruise_rigid('init', U)

if nargin < 1 || isempty(action)
    action = 'solve';
end
if nargin < 2
    U_mps = [];
end
switch lower(char(string(action)))
    case 'solve'
        result = solve_rigid_();
    case 'init'
        result = init_from_case_(U_mps);
    otherwise
        error('trim_cruise_rigid: action 只能是 ''solve'' 或 ''init''。');
end
end


function result = init_from_case_(U_mps)
c = case_from_base_();
if ~isempty(U_mps)
    c.U_mps = U_mps;
end

file = fullfile(fileparts(mfilename('fullpath')), 'last_rigid_trim.mat');
if isfile(file)
    data = load(file, 'result');
    trim = data.result;
    [same_case, reason] = case_matches_(trim, c, 'rigid');
    if isfield(trim, 'ok') && trim.ok && same_case
        result = trim;
        result.rpm_cmd = result.rpm_cmd(:);
        if ~isfield(result, 'aero_ic')
            result.aero_ic = aero_states_(result.alpha_deg, result.aileron_deg, ...
                result.elevator_deg, result.rudder_deg);
        end
        fprintf('rigid init: 使用完整工况匹配的 last_rigid_trim.mat\n');
        return
    end
    fprintf('rigid init: 忽略 last_rigid_trim.mat（%s）\n', reason);
end

result.alpha_deg = c.alpha0_deg;
result.rpm_cmd = c.rpm_cmd(:);
result.rpm_value = rpm_group_values_(result.rpm_cmd, c.rpm_trim_index);
result.rpm_trim_index = c.rpm_trim_index;
result.elevator_deg = c.elevator0_deg;
result.aileron_deg = c.aileron0_deg;
result.rudder_deg = c.rudder0_deg;
result.blade_pitch_deg = c.blade_pitch_deg;
result.tilt_deg = c.tilt_deg;
result.tilt_cmd_rad = c.tilt_cmd_rad;
result.dot_tilt_cmd_radps = c.dot_tilt_cmd_radps;
result.U_mps = c.U_mps;

alpha_rad = deg2rad(c.alpha0_deg);
if isempty(c.Velocity_b0)
    result.Velocity_b0 = c.U_mps * [cos(alpha_rad); 0; sin(alpha_rad)];
else
    result.Velocity_b0 = c.Velocity_b0(:);
end
result.Omega_b0 = c.Omega_b0(:);
result.Position0 = c.Position0(:);
if c.theta_equals_alpha
    theta = alpha_rad;
else
    theta = deg2rad(c.theta0_deg);
end
result.Attitude_rad0 = [deg2rad(c.phi0_deg); theta; deg2rad(c.psi0_deg)];
result.aero_ic = aero_states_(c.alpha0_deg, c.aileron0_deg, ...
    c.elevator0_deg, c.rudder0_deg);
end


function result = solve_rigid_()
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'lib'), fullfile(root, 'trim'));
cd(root);
assignin('base', 'trim_mode_override', 'rigid');
evalin('base', 'initial_main_V1_1');
evalin('base', 'clear trim_mode_override');
c = case_from_base_();
model = c.model;

loaded_here = ~bdIsLoaded(model);
if loaded_here
    load_system(model);
end

cfg = c;
cfg.U = c.U_mps;
cfg.T = c.simulation_time_s;
cfg.As = evalin('base', 'As');
cfg.Bs = evalin('base', 'Bs');
cfg.wagner_mode = 'quasi';
cfg.freeze_rigid = true;
[x0, xmin, xmax, dx] = pack_x_(cfg);
[res0, accel0, CL0] = run_case_(x0, cfg);
u0 = unpack_x_(cfg, x0);

fprintf('\nfmincon start: alpha=%.4f deg, RPM groups=%s, elevator=%.4f deg, |res|=%.3e\n', ...
    u0.alpha_deg, rpm_print_(u0, cfg), u0.elevator_deg, norm(res0));
if max(abs(res0)) < c.trim_tolerance
    x = x0;
    res_qs = res0;
    accel_qs = accel0;
    CL_qs = CL0;
    exitflag = 1;
    output = struct('message', '初值已经满足刚体配平容差，跳过 fmincon。');
    fval = sum(res0 .^ 2);
    fprintf('initial guess already meets tolerance; fmincon skipped\n');
else
    [x, res_qs, accel_qs, CL_qs, exitflag, output, fval] = solve_fmincon_( ...
        x0, xmin, xmax, dx, cfg);
end

u = unpack_x_(cfg, x);
aero_ic = steady_aero_states_(CL_qs);
cfg_full = cfg;
cfg_full.wagner_mode = 'full';
cfg_full.T = field_or_(cfg, 'quasi_simulation_time_s', 0.01);
cfg_full.aero_ic = aero_ic;
cfg_initial = cfg_full;
cfg_initial.sample_mode = 'first';
[res_wagner, ~, CL_wagner_qs, CL_wagner] = run_case_(x, cfg_initial);
wagner_output_error = 0;
for name = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'}
    nm = name{1};
    wagner_output_error = wagner_output_error + ...
        norm(CL_wagner.(nm) - CL_wagner_qs.(nm)) ^ 2;
end
wagner_output_error = sqrt(wagner_output_error);
% 初值一致后再做长时保持性检查；这一步不再反向修改初值。
cfg_hold = cfg_full;
cfg_hold.T = c.simulation_time_s;
cfg_hold.freeze_rigid = false;
[res, accel] = run_case_(x, cfg_hold);
result.static_ok = max(abs(res_qs)) < c.trim_tolerance ...
    && max(abs(res_wagner)) < c.trim_tolerance;
result.hold_ok = max(abs(res)) < c.trim_tolerance;
result.ok = result.static_ok;
result.mode = 'rigid';
result.alpha_deg = u.alpha_deg;
result.rpm = u.rpm;
result.rpm_cmd = u.rpm;
result.rpm_value = rpm_group_values_(u.rpm, c.rpm_trim_index);
result.rpm_trim_index = c.rpm_trim_index;
result.rpm_group_free = field_or_(c, 'rpm_group_free', []);
result.elevator_deg = u.elevator_deg;
result.aileron_deg = u.aileron_deg;
result.rudder_deg = u.rudder_deg;
result.blade_pitch_deg = c.blade_pitch_deg;
result.tilt_deg = c.tilt_deg;
result.tilt_cmd_rad = c.tilt_cmd_rad;
result.dot_tilt_cmd_radps = c.dot_tilt_cmd_radps;
result.acceleration = accel;
result.residual = res;
result.quasi_steady_acceleration = accel_qs;
result.quasi_steady_residual = res_qs;
result.wagner_initial_residual = res_wagner;
result.wagner_output_error = wagner_output_error;
result.quasi_CL = CL_qs;
result.wagner_initial_CL_qs = CL_wagner_qs;
result.wagner_initial_CL = CL_wagner;
result.optimizer = ['fmincon-' fmincon_algorithm_(cfg)];
result.optimizer_exitflag = exitflag;
result.optimizer_output = output;
result.preference_cost = fval;
result.case_signature = case_signature_(c, 'rigid');
result.U_mps = c.U_mps;
result.altitude_m = c.altitude_m;
alpha = deg2rad(u.alpha_deg);
if isempty(c.Velocity_b0)
    result.Velocity_b0 = c.U_mps * [cos(alpha); 0; sin(alpha)];
else
    result.Velocity_b0 = c.Velocity_b0(:);
end
result.Omega_b0 = c.Omega_b0(:);
result.Position0 = c.Position0(:);
if c.theta_equals_alpha
    theta = alpha;
else
    theta = deg2rad(c.theta0_deg);
end
result.Attitude_rad0 = [deg2rad(c.phi0_deg); theta; deg2rad(c.psi0_deg)];
result.aero_ic = aero_ic;

fprintf('\nalpha = %.4f deg, elevator = %.4f deg\n', u.alpha_deg, u.elevator_deg);
fprintf('RPM = %s\n', mat2str(u.rpm.', 4));
fprintf('blade pitch = %.3g deg\n', c.blade_pitch_deg);
fprintf('du/dt = %.4g m/s^2, dw/dt = %.4g m/s^2, dq/dt = %.4g rad/s^2\n', ...
    accel(1), accel(2), accel(3));
fprintf('quasi-steady |res| = %.3e, Wagner initial |res| = %.3e, hold |res| = %.3e\n', ...
    norm(res_qs), norm(res_wagner), norm(res));
fprintf('Wagner output - quasi-steady CL = %.3e\n', wagner_output_error);
fprintf('static trim ok = %d, 0.6 s hold ok = %d\n', ...
    result.static_ok, result.hold_ok);

if result.ok
    save(fullfile(fileparts(mfilename('fullpath')), 'last_rigid_trim.mat'), 'result');
    fprintf('saved trim/last_rigid_trim.mat\n');
    apply_result_to_workspace_(result);
else
    fprintf('not saved: current rigid case did not converge\n');
end

if loaded_here
    close_system(model, 0);
end
end


function [x, res, accel, CL_qs, exitflag, output, fval] = solve_fmincon_( ...
        x0, xmin, xmax, dx, cfg)
if exist('fmincon', 'file') ~= 2
    error('刚体配平需要 Optimization Toolbox 的 fmincon。');
end
if numel(x0) < 3
    error('纵向配平有 3 个平衡方程，至少需要 3 个自由配平量。');
end

center = (xmin + xmax) / 2;
scale = (xmax - xmin) / 2;
z0 = (x0 - center) ./ scale;
dz = max(dx ./ scale, 1e-5);

cache_z = [];
cache_res = [];
cache_accel = [];
cache_CL = [];
options = optimoptions('fmincon', ...
    'Algorithm', fmincon_algorithm_(cfg), ...
    'Display', 'iter', ...
    'FiniteDifferenceType', 'central', ...
    'FiniteDifferenceStepSize', dz, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8, ...
    'FunctionTolerance', 1e-8, ...
    'MaxIterations', cfg.max_iterations, ...
    'MaxFunctionEvaluations', max(100, 20 * (numel(z0) + 1) * cfg.max_iterations));

[z, fval, exitflag, output] = fmincon(@objective_, z0, [], [], [], [], ...
    -ones(size(z0)), ones(size(z0)), [], options);
x = center + scale .* z;
[res, accel, CL_qs] = evaluate_(z);

    function value = objective_(z_now)
        % 残差是主目标；小权重偏好项为欠定问题选择唯一、单位无关的解。
        [r, ~, ~] = evaluate_(z_now);
        preference = field_or_(cfg, 'trim_preference_weight', 1e-6);
        value = sum(r.^2) + preference * sum((z_now - z0).^2);
    end

    function [r, a, cl] = evaluate_(z_now)
        if isempty(cache_z) || ~isequal(z_now, cache_z)
            x_now = center + scale .* z_now;
            [cache_res, cache_accel, cache_CL] = run_case_(x_now, cfg);
            cache_z = z_now;
        end
        r = cache_res;
        a = cache_accel;
        cl = cache_CL;
    end
end


function [res, accel, CL_qs, CL_wagner, alpha_base] = run_case_(x, cfg)
u = unpack_x_(cfg, x);
if strcmpi(field_or_(cfg, 'wagner_mode', 'quasi'), 'quasi') ...
        && ~field_or_(cfg, 'induced_ready', false)
    % 先用零诱导迎角读取桨流/构型对应的当地迎角，再解静态诱导固定点。
    cfg_probe = cfg;
    cfg_probe.induced_ready = true;
    cfg_probe.zero_induced = true;
    cfg_probe.sample_mode = 'first';
    [~, ~, ~, ~, alpha_base] = run_case_(x, cfg_probe);
    cfg_eval = cfg;
    cfg_eval.induced_ready = true;
    cfg_eval.sample_mode = 'first';
    cfg_eval.aero_ic = aero_states_from_alpha_(alpha_base, ...
        u.aileron_deg, u.elevator_deg, u.rudder_deg);
    [res, accel, CL_qs, CL_wagner, alpha_base] = run_case_(x, cfg_eval);
    return
end
T = cfg.T;
tail_fraction = cfg.tail_fraction;
if strcmpi(field_or_(cfg, 'wagner_mode', 'quasi'), 'quasi')
    T = min(T, field_or_(cfg, 'quasi_simulation_time_s', 0.05));
    tail_fraction = 0.5;
end
alpha = deg2rad(u.alpha_deg);
if isempty(cfg.Velocity_b0)
    V0 = cfg.U * [cos(alpha); 0; sin(alpha)];
else
    V0 = cfg.Velocity_b0(:);
end
if cfg.theta_equals_alpha
    att0 = [deg2rad(cfg.phi0_deg); alpha; deg2rad(cfg.psi0_deg)];
else
    att0 = [deg2rad(cfg.phi0_deg); deg2rad(cfg.theta0_deg); deg2rad(cfg.psi0_deg)];
end
t = [0; T];
tilt = cfg.tilt_cmd_rad(:).';
dtilt = cfg.dot_tilt_cmd_radps(:).';

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries([u.rpm.'; u.rpm.'], t), 'Throttle');
ds = ds.addElement(timeseries([tilt; tilt], t), 'Tilt_cmd');
ds = ds.addElement(timeseries([dtilt; dtilt], t), 'dotTilt_cmd');
ds = ds.addElement(timeseries(u.aileron_deg * ones(2, 1), t), 'Aileron_deg');
ds = ds.addElement(timeseries(u.elevator_deg * ones(2, 1), t), 'Elevator_deg');
ds = ds.addElement(timeseries(u.rudder_deg * ones(2, 1), t), 'Rudder_deg');

in = Simulink.SimulationInput(cfg.model);
in = in.setExternalInput(ds);
in = in.setVariable('BladePitch_deg', cfg.blade_pitch_deg);
in = in.setVariable('x0_structure', zeros(size(cfg.As, 1), 1));
% 刚体方案固定零弹性变形，不让结构瞬态混入刚体残差。
in = in.setVariable('As', eye(size(cfg.As)));
in = in.setVariable('Bs', zeros(size(cfg.Bs)));
in = in.setModelParameter('StopTime', num2str(T), ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
in = in.setBlockParameter([cfg.model '/Flexible Aircraft Dynamics/Integrator'], ...
    'InitialCondition', mat2str(V0, 16));
in = in.setBlockParameter([cfg.model '/Flexible Aircraft Dynamics/Integrator1'], ...
    'InitialCondition', mat2str(cfg.Omega_b0(:), 16));
in = in.setBlockParameter([cfg.model '/Flexible Aircraft Dynamics/Integrator2'], ...
    'InitialCondition', mat2str(cfg.Position0(:), 16));
in = in.setBlockParameter([cfg.model '/Flexible Aircraft Dynamics/Integrator3'], ...
    'InitialCondition', mat2str(att0, 16));

if field_or_(cfg, 'freeze_rigid', false)
    % 配平目标只评估给定状态下的力/力矩，让诱导迎角延迟稳定，但不允许刚体状态漂移。
    blocks = {'Integrator', 'Integrator1', 'Integrator2', 'Integrator3'};
    values = {V0, cfg.Omega_b0(:), cfg.Position0(:), att0};
    for k = 1:numel(blocks)
        path = [cfg.model '/Flexible Aircraft Dynamics/' blocks{k}];
        band = 1e-12 * ones(size(values{k}));
        in = in.setBlockParameter(path, 'LimitOutput', 'on');
        in = in.setBlockParameter(path, 'UpperSaturationLimit', ...
            mat2str(values{k} + band, 16));
        in = in.setBlockParameter(path, 'LowerSaturationLimit', ...
            mat2str(values{k} - band, 16));
    end
end

in = configure_wagner_(in, cfg, u);

out = sim(in);
dyn = out.yout.getElement('Out_Dynamics').Values;
aero_out = out.yout.getElement('Out_Aero').Values;
acc = channels_by_time_(dyn.Acc_b.Data, 3);
rate = channels_by_time_(dyn.Rate_b.Data, 3);
n = size(acc, 2);
if strcmpi(field_or_(cfg, 'sample_mode', 'tail'), 'first')
    i0 = 1;
    i1 = 1;
else
    i0 = max(1, floor((1 - tail_fraction) * n));
    i1 = n;
end
accel = [mean(acc(1, i0:i1)); ...
         mean(acc(3, i0:i1)); ...
         mean(rate(2, i0:i1))];
res = accel ./ [9.80665; 9.80665; 9.80665 / 2.56];
CL_qs.wing_L = window_mean_(aero_out.CL_strip_qs_LW.Data, i0, i1, 20);
CL_qs.wing_R = window_mean_(aero_out.CL_strip_qs_RW.Data, i0, i1, 20);
CL_qs.v_tail_L = window_mean_(aero_out.CL_strip_qs_LT.Data, i0, i1, 5);
CL_qs.v_tail_R = window_mean_(aero_out.CL_strip_qs_RT.Data, i0, i1, 5);
CL_wagner.wing_L = window_mean_(aero_out.CL_strip_c_LW.Data, i0, i1, 20);
CL_wagner.wing_R = window_mean_(aero_out.CL_strip_c_RW.Data, i0, i1, 20);
CL_wagner.v_tail_L = window_mean_(aero_out.CL_strip_c_LT.Data, i0, i1, 5);
CL_wagner.v_tail_R = window_mean_(aero_out.CL_strip_c_RT.Data, i0, i1, 5);
alpha_base.wing_L = window_mean_(aero_out.a_eff_c_LW.Data, i0, i1, 20);
alpha_base.wing_R = window_mean_(aero_out.a_eff_c_RW.Data, i0, i1, 20);
alpha_base.v_tail_L = window_mean_(aero_out.a_eff_c_LT.Data, i0, i1, 5);
alpha_base.v_tail_R = window_mean_(aero_out.a_eff_c_RT.Data, i0, i1, 5);
end


function algorithm = fmincon_algorithm_(cfg)
algorithm = lower(char(string(field_or_(cfg, 'fmincon_algorithm', 'sqp'))));
if ~ismember(algorithm, {'sqp', 'interior-point'})
    error('trim_fmincon_algorithm 只能是 ''sqp'' 或 ''interior-point''。');
end
end


function in = configure_wagner_(in, cfg, u)
names = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
blocks = {'Left_Wing', 'Right_Wing', 'Left_Tail', 'Right_Tail'};

if isfield(cfg, 'aero_ic')
    aero = cfg.aero_ic;
else
    aero = aero_states_(u.alpha_deg, u.aileron_deg, ...
        u.elevator_deg, u.rudder_deg);
end

if field_or_(cfg, 'zero_induced', false)
    for k = 1:numel(names)
        aero.(names{k}).alpha_du0(:) = 0;
    end
end

for k = 1:numel(names)
    name = names{k};
    path = [cfg.model '/Strip Aerodynamics/' blocks{k}];
    in = in.setBlockParameter([path '/Wagner_SS'], 'X0', ...
        mat2str(aero.(name).x0_wagner, 16));
    in = in.setBlockParameter([path '/Delay_alpha_du'], ...
        'InitialCondition', mat2str(aero.(name).alpha_du0, 16));
    if strcmpi(field_or_(cfg, 'wagner_mode', 'quasi'), 'quasi')
        A = evalin('base', ['Ad_' name]);
        B = evalin('base', ['Bd_' name]);
        C = evalin('base', ['Cd_' name]);
        D = evalin('base', ['Dd_' name]);
        prefix = ['trim_qs_' name '_'];
        in = in.setVariable([prefix 'A'], zeros(size(A)));
        in = in.setVariable([prefix 'B'], zeros(size(B)));
        in = in.setVariable([prefix 'C'], zeros(size(C)));
        in = in.setVariable([prefix 'D'], eye(size(D)));
        wagner_path = [path '/Wagner_SS'];
        in = in.setBlockParameter(wagner_path, 'A', [prefix 'A']);
        in = in.setBlockParameter(wagner_path, 'B', [prefix 'B']);
        in = in.setBlockParameter(wagner_path, 'C', [prefix 'C']);
        in = in.setBlockParameter(wagner_path, 'D', [prefix 'D']);
    end
end
end


function states = steady_aero_states_(CL_qs)
names = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
for k = 1:numel(names)
    name = names{k};
    A = evalin('base', ['Ad_' name]);
    B = evalin('base', ['Bd_' name]);
    G = evalin('base', ['G_du_' name]);
    u0 = CL_qs.(name);
    states.(name).x0_wagner = (eye(size(A)) - A) \ (B * u0);
    states.(name).alpha_du0 = G * u0;
end
end


function value = window_mean_(data, i0, i1, n_channel)
y = channels_by_time_(data, n_channel);
i0 = min(i0, size(y, 2));
i1 = min(i1, size(y, 2));
value = mean(y(:, i0:i1), 2);
end


function [x0, xmin, xmax, dx] = pack_x_(cfg)
x0 = [];
xmin = [];
xmax = [];
dx = [];
if cfg.free_alpha
    x0(end+1, 1) = cfg.alpha0_deg;
    xmin(end+1, 1) = cfg.alpha_limit_deg(1);
    xmax(end+1, 1) = cfg.alpha_limit_deg(2);
    dx(end+1, 1) = field_or_(cfg, 'dx_alpha_deg', 0.05);
end
groups = rpm_groups_(cfg.rpm_trim_index);
free_g = rpm_group_free_(cfg, numel(groups));
for i = 1:numel(groups)
    if ~free_g(i)
        continue
    end
    x0(end+1, 1) = mean(cfg.rpm_cmd(groups{i}));
    xmin(end+1, 1) = cfg.rpm_limit(1);
    xmax(end+1, 1) = cfg.rpm_limit(2);
    dx(end+1, 1) = field_or_(cfg, 'dx_rpm', 10);
end
if cfg.free_elevator
    x0(end+1, 1) = cfg.elevator0_deg;
    xmin(end+1, 1) = cfg.elevator_limit_deg(1);
    xmax(end+1, 1) = cfg.elevator_limit_deg(2);
    dx(end+1, 1) = field_or_(cfg, 'dx_surface_deg', 0.05);
end
if cfg.free_aileron
    x0(end+1, 1) = cfg.aileron0_deg;
    xmin(end+1, 1) = cfg.aileron_limit_deg(1);
    xmax(end+1, 1) = cfg.aileron_limit_deg(2);
    dx(end+1, 1) = field_or_(cfg, 'dx_surface_deg', 0.05);
end
if cfg.free_rudder
    x0(end+1, 1) = cfg.rudder0_deg;
    xmin(end+1, 1) = cfg.rudder_limit_deg(1);
    xmax(end+1, 1) = cfg.rudder_limit_deg(2);
    dx(end+1, 1) = field_or_(cfg, 'dx_surface_deg', 0.05);
end
if isempty(x0)
    error('trim_cruise_rigid: 至少把 alpha / RPM 组 / 升降舵 中的一项设为可调。');
end
end


function u = unpack_x_(cfg, x)
u.alpha_deg = cfg.alpha0_deg;
u.rpm = cfg.rpm_cmd(:);
u.elevator_deg = cfg.elevator0_deg;
u.aileron_deg = cfg.aileron0_deg;
u.rudder_deg = cfg.rudder0_deg;
k = 1;
if cfg.free_alpha
    u.alpha_deg = x(k); k = k + 1;
end
groups = rpm_groups_(cfg.rpm_trim_index);
free_g = rpm_group_free_(cfg, numel(groups));
for i = 1:numel(groups)
    if ~free_g(i)
        continue
    end
    u.rpm(groups{i}) = x(k); k = k + 1;
end
if cfg.free_elevator
    u.elevator_deg = x(k); k = k + 1;
end
if cfg.free_aileron
    u.aileron_deg = x(k); k = k + 1;
end
if cfg.free_rudder
    u.rudder_deg = x(k);
end
end


function states = aero_states_(alpha_deg, aileron_deg, elevator_deg, rudder_deg)
alpha = deg2rad(alpha_deg);
cant = evalin('base', 'v_tail_cant_rad');
[a_wr, a_wl, a_tr, a_tl] = strip_local_alphas_( ...
    [cos(alpha); 0; sin(alpha)], cant);
local_alpha.wing_R = a_wr;
local_alpha.wing_L = a_wl;
local_alpha.v_tail_R = a_tr;
local_alpha.v_tail_L = a_tl;
states = aero_states_from_alpha_(local_alpha, ...
    aileron_deg, elevator_deg, rudder_deg);
end


function states = aero_states_from_alpha_(local_alpha, ...
        aileron_deg, elevator_deg, rudder_deg)

names = {'wing_R', 'wing_L', 'v_tail_R', 'v_tail_L'};
polar.alpha_deg = evalin('base', 'polar_alpha_deg');
polar.CL = evalin('base', 'polar_CL');
polar.CD = evalin('base', 'polar_CD');
polar.CM = evalin('base', 'polar_CM');
tables = control_surfaces.load_tables();
cmd = [aileron_deg; elevator_deg; rudder_deg; 0];

for k = 1:4
    name = names{k};
    A = evalin('base', ['Ad_' name]);
    B = evalin('base', ['Bd_' name]);
    G = evalin('base', ['G_du_' name]);
    n = size(B, 2);
    Mix = evalin('base', ['ctrl_Mix_' name]);
    family = evalin('base', ['ctrl_family_' name]);
    delta = control_surfaces.mix(cmd, Mix);

    alpha_local = local_alpha.(name);
    if isscalar(alpha_local)
        alpha_local = alpha_local * ones(n, 1);
    else
        alpha_local = alpha_local(:);
    end
    CL0 = control_surfaces.lookup( ...
        rad2deg(alpha_local), delta, family, tables, polar);
    % 静态诱导迎角固定点：CL = polar(alpha_local - G*CL)。
    for j = 1:20
        CL_new = control_surfaces.lookup( ...
            rad2deg(alpha_local - G * CL0), delta, family, tables, polar);
        CL0 = 0.5 * (CL0 + CL_new);
    end
    states.(name).x0_wagner = (eye(size(A)) - A) \ (B * CL0);
    states.(name).alpha_du0 = G * CL0;
end
end


function [a_wr, a_wl, a_tr, a_tl] = strip_local_alphas_(V_b, cant_rad)
% 四个气动面当地迎角；结构系 R_sb=diag(-1,1,-1)。
V_s = [-V_b(1); V_b(2); -V_b(3)];
a_wr = atan2(-V_s(3), -V_s(1));
a_wl = a_wr;
cr = cos(cant_rad);
sr = sin(cant_rad);
Vtr = [V_s(1); cr*V_s(2) + sr*V_s(3); -sr*V_s(2) + cr*V_s(3)];
Vtl = [V_s(1); cr*V_s(2) - sr*V_s(3);  sr*V_s(2) + cr*V_s(3)];
a_tr = atan2(-Vtr(3), -Vtr(1));
a_tl = atan2(-Vtl(3), -Vtl(1));
end


function y = channels_by_time_(data, n_channel)
y = squeeze(data);
if size(y, 1) ~= n_channel && size(y, 2) == n_channel
    y = y.';
end
if size(y, 1) ~= n_channel
    error('输出信号维度不符合预期：需要 %d 个通道。', n_channel);
end
end


function s = rpm_print_(u, cfg)
s = mat2str(rpm_group_values_(u.rpm, cfg.rpm_trim_index).', 4);
end


function groups = rpm_groups_(idx)
if isempty(idx)
    groups = {};
    return
end
if iscell(idx)
    groups = idx(:)';
else
    groups = {idx(:).'};
end
used = false(8, 1);
for i = 1:numel(groups)
    g = groups{i}(:)';
    if isempty(g)
        error('rpm_trim_index 第 %d 组是空的。', i);
    end
    if any(g < 1 | g > 8 | g ~= floor(g))
        error('rpm_trim_index 第 %d 组必须是 1..8 的发动机编号。', i);
    end
    if any(used(g))
        error('rpm_trim_index 各组发动机编号不能重复。');
    end
    used(g) = true;
    groups{i} = g;
end
end


function v = rpm_group_values_(rpm, idx)
groups = rpm_groups_(idx);
if isempty(groups)
    v = mean(rpm(:));
    return
end
v = zeros(numel(groups), 1);
for i = 1:numel(groups)
    v(i) = mean(rpm(groups{i}));
end
end


function v = rpm_group_free_(cfg, n)
if n == 0
    v = false(1, 0);
    return
end
if ~isfield(cfg, 'rpm_group_free') || isempty(cfg.rpm_group_free)
    v = true(1, n);
    return
end
v = logical(cfg.rpm_group_free(:)).';
if isscalar(v)
    v = repmat(v, 1, n);
end
if numel(v) ~= n
    error('rpm_group_free 长度必须等于 rpm_trim_index 组数（%d）。', n);
end
end


function [same, reason] = case_matches_(result, cfg, mode)
same = false;
if ~isfield(result, 'ok') || ~result.ok
    reason = '保存结果未收敛';
    return
end
if ~isfield(result, 'case_signature')
    reason = '旧文件没有完整工况签名';
    return
end

current = case_signature_(cfg, mode);
saved = result.case_signature;
names = fieldnames(current);
different = strings(0, 1);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(saved, name) || ~isequaln(saved.(name), current.(name))
        different(end+1, 1) = string(name); %#ok<AGROW>
    end
end
if isempty(different)
    same = true;
    reason = '';
else
    reason = ['工况不一致: ' char(strjoin(different, ', '))];
end
end


function s = case_signature_(cfg, mode)
s.version = 2;
s.mode = char(mode);
s.model = cfg.model;
s.model_file = model_file_stamp_(cfg.model);
s.flight_condition = [cfg.U_mps, field_or_(cfg, 'altitude_m', 0), ...
    field_or_(cfg, 'rho', NaN), field_or_(cfg, 'g', 9.80665)];
s.propeller = [cfg.blade_pitch_deg; cfg.tilt_deg(:); cfg.dot_tilt_degps(:)];
s.rpm_cmd = cfg.rpm_cmd(:);
s.rpm_groups = rpm_groups_(cfg.rpm_trim_index);
s.rpm_group_free = rpm_group_free_(cfg, numel(s.rpm_groups));
s.free_flags = logical([cfg.free_alpha, cfg.free_elevator, ...
    cfg.free_aileron, cfg.free_rudder]);
s.initial_controls = [cfg.alpha0_deg, cfg.elevator0_deg, ...
    cfg.aileron0_deg, cfg.rudder0_deg];
s.limits = [cfg.alpha_limit_deg(:); cfg.rpm_limit(:); ...
    cfg.elevator_limit_deg(:); cfg.aileron_limit_deg(:); ...
    cfg.rudder_limit_deg(:)];
s.attitude_rule = [logical(cfg.theta_equals_alpha), cfg.phi0_deg, ...
    cfg.theta0_deg, cfg.psi0_deg];
s.velocity_b0 = cfg.Velocity_b0(:);
s.omega_b0 = cfg.Omega_b0(:);
s.numerics = [field_or_(cfg, 'Ts', NaN), ...
    field_or_(cfg, 'quasi_simulation_time_s', 0.05), cfg.simulation_time_s, ...
    cfg.tail_fraction, cfg.trim_tolerance];
s.fmincon_algorithm = fmincon_algorithm_(cfg);
s.model_configuration = {field_or_(cfg, 'struct_dir', ''), ...
    field_or_(cfg, 'mac_m', NaN), field_or_(cfg, 'v_tail_cant_deg', NaN), ...
    field_or_(cfg, 'zeta', NaN), field_or_(cfg, 'polar_file', ''), ...
    field_or_(cfg, 'flap_polar_file', ''), field_or_(cfg, 'wagner_a0', NaN), ...
    field_or_(cfg, 'ctrl_flap_kappa', []), field_or_(cfg, 'prop_table_dir', '')};
s.structure = structure_stamp_();
end


function stamp = model_file_stamp_(model)
root = fileparts(fileparts(mfilename('fullpath')));
info = dir(fullfile(root, [model '.slx']));
if isempty(info)
    stamp = [];
else
    stamp = [info.bytes, info.datenum];
end
end


function stamp = structure_stamp_()
if ~evalin('base', 'exist(''As'',''var'') && exist(''Bs'',''var'')')
    stamp = [];
    return
end
A = evalin('base', 'As');
B = evalin('base', 'Bs');
stamp = [size(A), size(B), norm(A, 'fro'), norm(B, 'fro'), ...
    sum(A(:)), sum(B(:))];
end


function apply_result_to_workspace_(result)
assignin('base', 'trim_Velocity_b0', result.Velocity_b0(:));
assignin('base', 'trim_Omega_b0', result.Omega_b0(:));
assignin('base', 'trim_Position0', result.Position0(:));
assignin('base', 'trim_Attitude_rad0', result.Attitude_rad0(:));
assignin('base', 'trim_RPM_cmd', result.rpm_cmd(:));
assignin('base', 'trim_Tilt_cmd_rad', result.tilt_cmd_rad(:));
assignin('base', 'trim_dotTilt_cmd_radps', result.dot_tilt_cmd_radps(:));
assignin('base', 'trim_Aileron_deg', result.aileron_deg);
assignin('base', 'trim_Elevator_deg', result.elevator_deg);
assignin('base', 'trim_Rudder_deg', result.rudder_deg);
if isfield(result, 'x0_structure')
    assignin('base', 'x0_structure', result.x0_structure);
end
if isfield(result, 'aero_ic')
    aero = result.aero_ic;
    names = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
    for k = 1:numel(names)
        nm = names{k};
        assignin('base', ['x0_wagner_' nm], aero.(nm).x0_wagner);
        assignin('base', ['alpha_du0_' nm], aero.(nm).alpha_du0);
    end
end
if evalin('base', 'exist(''sim_time'',''var'')')
    sim_time = evalin('base', 'sim_time');
    t = [0; sim_time];
    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(timeseries([result.rpm_cmd(:).'; result.rpm_cmd(:).'], t), 'Throttle');
    ds = ds.addElement(timeseries([result.tilt_cmd_rad(:).'; result.tilt_cmd_rad(:).'], t), 'Tilt_cmd');
    ds = ds.addElement(timeseries([result.dot_tilt_cmd_radps(:).'; result.dot_tilt_cmd_radps(:).'], t), 'dotTilt_cmd');
    ds = ds.addElement(timeseries(result.aileron_deg * ones(2, 1), t), 'Aileron_deg');
    ds = ds.addElement(timeseries(result.elevator_deg * ones(2, 1), t), 'Elevator_deg');
    ds = ds.addElement(timeseries(result.rudder_deg * ones(2, 1), t), 'Rudder_deg');
    assignin('base', 'trim_input_dataset', ds);
end
if evalin('base', 'exist(''trim_case'',''var'')')
    c = evalin('base', 'trim_case');
    c.alpha0_deg = result.alpha_deg;
    c.rpm_cmd = result.rpm_cmd(:);
    c.elevator0_deg = result.elevator_deg;
    c.aileron0_deg = result.aileron_deg;
    c.rudder0_deg = result.rudder_deg;
    assignin('base', 'trim_case', c);
end
fprintf(['已写回工作区：trim_Velocity_b0 / 姿态 / Dataset / Wagner IC' ...
    ' / trim_case（α、RPM、舵）。可直接仿真。\n']);
end


function v = field_or_(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end


function c = case_from_base_()
if ~evalin('base', 'exist(''trim_case'',''var'')')
    error('缺少 trim_case。请先运行 initial_main_V1_1（工况在该文件开头自定义块）。');
end
c = evalin('base', 'trim_case');
end
