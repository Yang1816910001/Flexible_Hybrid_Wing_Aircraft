function result = trim_flexible_direct_findop(validation_time_s, custom)
%TRIM_FLEXIBLE_DIRECT_FINDOP Joint rigid-body and flexible-mode trim.
%
% The rigid states, structural modal states and longitudinal controls are
% solved in one findop call. Wagner and downwash are bypassed.
% The model is configured in memory only; the .slx file is not saved.

if nargin < 1
    validation_time_s = 0.3;
end
if nargin < 2
    custom = struct();
end

%% ================= 1. 工况参数自定义 =================
% 这些量定义要配平的飞行状态和固定构型，求解过程中不会改变。
condition.U_mps              = 50.0;  % [m/s] 机体系纵向速度基准；须与 initial_main_V1_1 的 U 一致
condition.altitude_m         = 0.0;   % [m]
condition.rho_kgpm3          = 1.225; % []=按高度用 ISA 计算，也可直接填密度
condition.blade_pitch_deg    = 34.0;  % [deg] 共用桨距；表节点 9.9/13/22/31/34/40

% 顺序均为 E1...E8；也可只填一个数统一设置。静态配平时角速度通常为零。
tilt_deg=0;
condition.tilt_deg           = [0 tilt_deg tilt_deg 0 0 tilt_deg tilt_deg 0]; % [deg]
condition.dot_tilt_degps     = [0 0 0 0 0 0 0 0]; % [deg/s]

% 悬停桨转速，顺序为 [E1 E4 E5 E8]；可填一个数或 4 个数。
% trim_hover_rpm=false 时为固定工况，true 时也作为默认求解初值。
condition.hover_rpm          = 0;
condition.aileron_deg        = 0;
condition.rudder_deg         = 0;

%% ================= 2. 初始化参数自定义 =================
% 这些量只是联合求解的起点，不是最终工况约束。
initial.alpha_deg            = 0.3945; % [deg] 攻角/俯仰角初值
initial.elevator_deg         = -3.0;    % [deg] 升降舵初值
% 巡航桨初值，顺序为 [E2 E3 E6 E7]；可填一个数、4 个数或 []。
% [] 表示沿用刚体配平结果中的对应转速。
initial.cruise_rpm           = [];
% 悬停桨配平初值，顺序为 [E1 E4 E5 E8]；[]=沿用 condition.hover_rpm。
% 仅在 solver.trim_hover_rpm=true 时使用。
initial.hover_rpm            = [];
% [] 表示由当前刚体气动力估算静态模态初值；也可直接填 n_mode 个 eta。
initial.modal_eta            = [];
initial.modal_seed_scale     = 0.25;

%% ================= 3. 求解与模型开关 =================
solver.alpha_min_deg         = -8.0;
solver.alpha_max_deg         = 12.0;
solver.elevator_min_deg      = -20.0;
solver.elevator_max_deg      = 20.0;
% false：悬停桨 RPM 固定为 condition.hover_rpm；true：加入配平待求量。
solver.trim_hover_rpm        = false;
solver.hover_rpm_min         = 0;
solver.hover_rpm_max         = 2000;
% 巡航桨上下限顺序为 [E2 E3 E6 E7]；可填一个数或 4 个数。
% 下限 [] 时根据速度、倾转角和 J_max 自动计算，避免超出桨表范围。
solver.cruise_rpm_min        = [];
solver.cruise_rpm_max        = 3000;
solver.J_max                 = 2.4;
solver.prop_diameter_m       = 3.0;
solver.max_iterations        = 200;
solver.constraint_tol        = 1.0e-8;
solver.optimality_tol        = 1.0e-8;
solver.trim_tolerance        = 2.0e-5;

feature.enable_downwash  = false;
feature.enable_wagner    = false;
feature.bypass_delays    = true;
%% ==========================================================

% 可选批处理入口：custom.condition / custom.initial / custom.solver /
% custom.feature 中给出的字段，会覆盖上面的默认设置。
condition = merge_settings_(condition, field_or_(custom, 'condition', struct()));
initial = merge_settings_(initial, field_or_(custom, 'initial', struct()));
solver = merge_settings_(solver, field_or_(custom, 'solver', struct()));
feature = merge_settings_(feature, field_or_(custom, 'feature', struct()));

mdl = 'aerodynamic_model_V1_1_Coupling';
root_dir = fileparts(fileparts(mfilename('fullpath')));
cd(root_dir);
addpath(root_dir);
addpath(fullfile(root_dir, 'trim'));

fprintf('\n=== Direct rigid + flexible joint trim ===\n');
fprintf('Model      : %s\n', mdl);
fprintf('Condition  : U = %.3f m/s, altitude = %.1f m\n', ...
    condition.U_mps, condition.altitude_m);
fprintf('Blade pitch: %.3f deg\n', condition.blade_pitch_deg);
fprintf('Trim hover : %d\n', solver.trim_hover_rpm);
fprintf('Wagner     : OFF\n');
fprintf('Downwash   : OFF\n\n');

% 工作区由 initial_main_V1_1 建立；不再依赖 TRIM 刚体 findop。
if evalin('base', "~exist('As','var')")
    evalin('base', 'initial_main_V1_1');
end
if evalin('base', "~exist('trim_result','var') || ~isstruct(trim_result)")
    seed_trim = struct();
    seed_trim.ok = true;
    seed_trim.rpm_cmd = evalin('base', 'trim_case.rpm_cmd(:)');
    assignin('base', 'trim_result', seed_trim);
end
load_system(mdl);

As0 = evalin('base', 'As');
Bs0 = evalin('base', 'Bs');
Cs0 = evalin('base', 'Cs');
Ds0 = evalin('base', 'Ds');
n_mode = size(Bs0, 2);

hover = [1 4 5 8];
cruise = [2 3 6 7];
condition.tilt_deg = vector_value_(condition.tilt_deg, 8, 'condition.tilt_deg');
condition.dot_tilt_degps = vector_value_( ...
    condition.dot_tilt_degps, 8, 'condition.dot_tilt_degps');
condition.hover_rpm = vector_value_(condition.hover_rpm, 4, 'condition.hover_rpm');
if any(condition.hover_rpm < 0)
    error('condition.hover_rpm cannot contain negative values.');
end
if isempty(condition.rho_kgpm3)
    condition.rho_kgpm3 = isa_density_(condition.altitude_m);
end
if ~isscalar(condition.rho_kgpm3) || ~isfinite(condition.rho_kgpm3) || ...
        condition.rho_kgpm3 <= 0
    error('condition.rho_kgpm3 must be a positive scalar.');
end

rigid = evalin('base', 'trim_result');
rpm0 = rigid.rpm_cmd(:);
rpm0(hover) = condition.hover_rpm;
if solver.trim_hover_rpm && ~isempty(initial.hover_rpm)
    rpm0(hover) = vector_value_(initial.hover_rpm, 4, 'initial.hover_rpm');
end
if ~isempty(initial.cruise_rpm)
    rpm0(cruise) = vector_value_( ...
        initial.cruise_rpm, 4, 'initial.cruise_rpm');
end

tilt_rad = deg2rad(condition.tilt_deg);
dot_tilt_radps = deg2rad(condition.dot_tilt_degps);
if isempty(solver.cruise_rpm_min)
    axial_speed = abs(condition.U_mps*cosd(condition.tilt_deg(cruise)));
    cruise_rpm_min = ceil(60*axial_speed / ...
        (solver.J_max*solver.prop_diameter_m));
else
    cruise_rpm_min = vector_value_( ...
        solver.cruise_rpm_min, 4, 'solver.cruise_rpm_min');
end
cruise_rpm_max = vector_value_( ...
    solver.cruise_rpm_max, 4, 'solver.cruise_rpm_max');
hover_rpm_min = vector_value_( ...
    solver.hover_rpm_min, 4, 'solver.hover_rpm_min');
hover_rpm_max = vector_value_( ...
    solver.hover_rpm_max, 4, 'solver.hover_rpm_max');
if any(cruise_rpm_min < 0) || any(cruise_rpm_max < cruise_rpm_min)
    error('Cruise RPM limits must satisfy 0 <= min <= max.');
end
if any(hover_rpm_min < 0) || any(hover_rpm_max < hover_rpm_min)
    error('Hover RPM limits must satisfy 0 <= min <= max.');
end
rpm0(cruise) = min(max(rpm0(cruise), cruise_rpm_min), cruise_rpm_max);
rpm0(hover) = min(max(rpm0(hover), hover_rpm_min), hover_rpm_max);

% 模型参数和初值必须写入基础工作区，模型不会读取函数局部变量。
assignin('base', 'BladePitch_deg', condition.blade_pitch_deg);
assignin('base', 'rho', condition.rho_kgpm3);

seed = struct();
seed.velocity_b = [condition.U_mps; 0; ...
    condition.U_mps*tand(initial.alpha_deg)];
seed.omega_b = zeros(3, 1);
seed.position = [0; 0; -condition.altitude_m];
seed.attitude_rad = [0; deg2rad(initial.alpha_deg); 0];
seed.rpm = rpm0;
seed.tilt_cmd_rad = tilt_rad;
seed.dot_tilt_cmd_radps = dot_tilt_radps;
seed.aileron_deg = condition.aileron_deg;
seed.elevator_deg = initial.elevator_deg;
seed.rudder_deg = condition.rudder_deg;
seed.input_dataset = input_dataset_(seed);
publish_common_initial_conditions_(seed, zeros(n_mode, 1), n_mode);
set_param(mdl, 'LoadExternalInput', 'on', ...
    'ExternalInput', 'trim_input_dataset');

% Sample the rigid modal load with structure, Wagner and downwash bypassed.
configure_model_(mdl, false, Cs0, Ds0, feature);
% A linear static modal estimate is used only as the nonlinear solver seed.
if isempty(initial.modal_eta)
    Q0 = sample_rigid_modal_load_(mdl, seed.input_dataset, n_mode);
    x_static0 = (eye(size(As0)) - As0) \ (Bs0 * Q0);
    eta_seed = initial.modal_seed_scale * x_static0(1:n_mode);
else
    eta_seed = vector_value_(initial.modal_eta, n_mode, 'initial.modal_eta');
end

configure_model_(mdl, true, Cs0, Ds0, feature);
spec = operspec(mdl);

i_vel = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator');
i_omega = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator1');
i_pos = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator2');
i_att = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator3');
i_structure = state_index_(spec, '/Flexible Aircraft Dynamics/Structure_SS');

U = condition.U_mps;
spec.States(i_vel).x = [U; 0; U*tand(initial.alpha_deg)];
spec.States(i_vel).Known = [true; true; false];
spec.States(i_vel).SteadyState = true(3, 1);
spec.States(i_vel).Min = [-inf; 0; U*tand(solver.alpha_min_deg)];
spec.States(i_vel).Max = [ inf; 0; U*tand(solver.alpha_max_deg)];

spec.States(i_omega).x = zeros(3, 1);
spec.States(i_omega).Known = true(3, 1);
spec.States(i_omega).SteadyState = true(3, 1);

spec.States(i_pos).x = [0; 0; -condition.altitude_m];
spec.States(i_pos).Known = true(3, 1);
spec.States(i_pos).SteadyState = [false; false; true];

spec.States(i_att).x = [0; deg2rad(initial.alpha_deg); 0];
spec.States(i_att).Known = [true; false; true];
spec.States(i_att).SteadyState = true(3, 1);
spec.States(i_att).Min = [0; deg2rad(solver.alpha_min_deg); 0];
spec.States(i_att).Max = [0; deg2rad(solver.alpha_max_deg); 0];

% Key difference from the outer-loop method: modal states are unknown and
% their state derivatives are constrained to zero in this same solve.
spec.States(i_structure).x = [eta_seed; zeros(n_mode, 1)];
spec.States(i_structure).Known = false(2*n_mode, 1);
spec.States(i_structure).SteadyState = true(2*n_mode, 1);

for i = 1:numel(spec.States)
    block = spec.States(i).Block;
    if contains(block, '/Downwash_Delay') || contains(block, '/Delay_alpha_du')
        spec.States(i).x = zeros(size(spec.States(i).x));
        spec.States(i).Known = true(size(spec.States(i).Known));
        spec.States(i).SteadyState = false(size(spec.States(i).SteadyState));
    elseif contains(block, '/Wagner_SS')
        spec.States(i).Known = false(size(spec.States(i).Known));
        spec.States(i).SteadyState = true(size(spec.States(i).SteadyState));
    end
end

i_rpm = input_index_(spec, '/Throttle');
i_tilt = input_index_(spec, '/Tilt_cmd');
i_dot_tilt = input_index_(spec, '/dotTilt_cmd');
i_aileron = input_index_(spec, '/Aileron_deg');
i_elevator = input_index_(spec, '/Elevator_deg');
i_rudder = input_index_(spec, '/Rudder_deg');

rpm_known = true(8, 1);
rpm_known(cruise) = false;
if solver.trim_hover_rpm
    rpm_known(hover) = false;
end
rpm_min = zeros(8, 1);
rpm_min(cruise) = cruise_rpm_min;
rpm_min(hover) = hover_rpm_min;
rpm_max = zeros(8, 1);
if solver.trim_hover_rpm
    rpm_max(hover) = hover_rpm_max;
else
    rpm_min(hover) = condition.hover_rpm;
    rpm_max(hover) = condition.hover_rpm;
end
rpm_max(cruise) = cruise_rpm_max;

spec.Inputs(i_rpm).u = rpm0;
spec.Inputs(i_rpm).Known = rpm_known;
spec.Inputs(i_rpm).Min = rpm_min;
spec.Inputs(i_rpm).Max = rpm_max;
set_fixed_input_(spec.Inputs(i_tilt), tilt_rad);
set_fixed_input_(spec.Inputs(i_dot_tilt), dot_tilt_radps);
set_fixed_input_(spec.Inputs(i_aileron), condition.aileron_deg);
set_fixed_input_(spec.Inputs(i_rudder), condition.rudder_deg);

spec.Inputs(i_elevator).u = initial.elevator_deg;
spec.Inputs(i_elevator).Known = false;
spec.Inputs(i_elevator).Min = solver.elevator_min_deg;
spec.Inputs(i_elevator).Max = solver.elevator_max_deg;

for i = 1:numel(spec.Outputs)
    spec.Outputs(i).Known = false(size(spec.Outputs(i).Known));
end

opt = findopOptions('DisplayReport', 'off');
opt.OptimizerType = 'lsqnonlin';
opt.OptimizationOptions.MaxIterations = solver.max_iterations;
opt.OptimizationOptions.ConstraintTolerance = solver.constraint_tol;
opt.OptimizationOptions.OptimalityTolerance = solver.optimality_tol;

[op, report] = findop(mdl, spec, opt);

velocity = op.States(i_vel).x(:);
omega = op.States(i_omega).x(:);
position = op.States(i_pos).x(:);
attitude = op.States(i_att).x(:);
x_structure = op.States(i_structure).x(:);
eta = x_structure(1:n_mode);

d_velocity = state_derivative_(op, report, i_vel);
d_omega = state_derivative_(op, report, i_omega);
d_position = state_derivative_(op, report, i_pos);
d_structure = state_derivative_(op, report, i_structure);
residual_vector = [d_velocity; d_omega; d_position(3); d_structure];
max_residual = max(abs(residual_vector));

point = struct();
point.velocity_b = velocity;
point.omega_b = omega;
point.position = position;
point.attitude_rad = attitude;
point.alpha_deg = atan2d(velocity(3), velocity(1));
point.theta_deg = rad2deg(attitude(2));
point.rpm = op.Inputs(i_rpm).u(:);
point.tilt_cmd_rad = tilt_rad;
point.dot_tilt_cmd_radps = dot_tilt_radps;
point.aileron_deg = condition.aileron_deg;
point.elevator_deg = op.Inputs(i_elevator).u;
point.rudder_deg = condition.rudder_deg;
point.input_dataset = input_dataset_(point);

publish_initial_conditions_(mdl, op, point, eta, n_mode);
configure_model_(mdl, true, Cs0, Ds0, feature);
validation = validate_(mdl, validation_time_s, eta, n_mode);

result = struct();
result.model = mdl;
result.scheme = 'direct_joint_findop';
result.condition = condition;
result.initial = initial;
result.solver = solver;
result.feature = feature;
result.modal_seed = eta_seed;
result.eta = eta;
result.modal_load_Q = validation.modal_load_Q_mean;
result.alpha_deg = point.alpha_deg;
result.theta_deg = point.theta_deg;
result.elevator_deg = point.elevator_deg;
result.rpm = point.rpm;
result.blade_pitch_deg = condition.blade_pitch_deg;
result.tilt_cmd_rad = point.tilt_cmd_rad;
result.dot_tilt_cmd_radps = point.dot_tilt_cmd_radps;
result.aileron_deg = point.aileron_deg;
result.rudder_deg = point.rudder_deg;
result.residual_vector = residual_vector;
result.max_residual = max_residual;
result.operating_point = op;
result.operating_report = report;
result.validation = validation;
result.converged = max_residual <= solver.trim_tolerance;
result.ok = result.converged && validation.ok;
result.workspace_ready = result.ok;

assignin('base', 'aeroelastic_direct_result', result);
assignin('base', 'trim_operating_point', op);
assignin('base', 'wagner_restore_source', 'flexible_direct');

fprintf('\n=== Direct joint trim result ===\n');
fprintf('Converged         : %d\n', result.converged);
fprintf('Max residual      : %.3e\n', max_residual);
fprintf('Alpha / theta     : %.6f / %.6f deg\n', point.alpha_deg, point.theta_deg);
fprintf('Elevator          : %.6f deg\n', point.elevator_deg);
fprintf('RPM               : %s\n', mat2str(point.rpm(:).', 7));
fprintf('Tilt              : %s deg\n', mat2str(condition.tilt_deg(:).', 5));
fprintf('|eta|             : %.6e\n', norm(eta));
fprintf('Validation dV     : %s m/s\n', mat2str(validation.delta_velocity_b(:).', 5));
fprintf('Validation dOmega : %s rad/s\n', mat2str(validation.delta_omega_b(:).', 5));
fprintf('Validation dEta   : %.3e\n', validation.delta_eta_norm);
fprintf('Ready to simulate : %d\n', result.ok);
if result.ok
    fprintf('Next command      : restore_wagner_restored\n');
end

if ~result.ok
    warning('Direct joint trim did not pass both convergence and hold checks.');
end
end


function Q_mean = sample_rigid_modal_load_(mdl, input_data, n_mode)
sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setExternalInput(input_data);
sim_in = sim_in.setModelParameter('StopTime', '0.03', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
out = sim(sim_in);
dyn = out.yout.getElement('Out_Dynamics').Values;
Q = channels_by_time_(dyn.Q.Data, numel(dyn.Q.Time), n_mode);
Q_mean = mean(Q(:, max(1, floor(size(Q, 2)/2)):end), 2);
end


function configure_model_(mdl, structure_active, Cs0, Ds0, feature)
structure = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
if structure_active
    set_param(structure, 'C', 'Cs', 'D', 'Ds');
else
    set_param(structure, 'C', 'zeros(size(Cs))', 'D', 'zeros(size(Ds))');
end
assignin('base', 'Cs', Cs0);
assignin('base', 'Ds', Ds0);

blocks = find_system(mdl, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Name', 'Wagner_SS');
for i = 1:numel(blocks)
    if feature.enable_wagner
        set_param(blocks{i}, 'Commented', 'off');
    else
        set_param(blocks{i}, 'Commented', 'through');
    end
end

blocks = find_system(mdl, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Name', 'From_adu');
for i = 1:numel(blocks)
    if feature.enable_downwash
        set_param(blocks{i}, 'Commented', 'off');
    else
        set_param(blocks{i}, 'Commented', 'on');
    end
end

if feature.bypass_delays
    set_param([mdl '/Strip Aerodynamics/Delay_Dynamics'], 'Commented', 'through');
    set_param([mdl '/Propulsion Dynamics/Delay_Dynamic'], 'Commented', 'through');
end
set_param([mdl '/Strip Aerodynamics/cant_Tail'], 'Value', '-v_tail_cant_rad');
set_param(mdl, 'SimulationCommand', 'update');
end


function publish_initial_conditions_(mdl, op, point, eta, n_mode)
publish_common_initial_conditions_(point, eta, n_mode);
assignin('base', 'trim_state_struct', getstatestruct(op));

set_param(mdl, 'LoadInitialState', 'off');
set_param(mdl, 'LoadExternalInput', 'on');
set_param(mdl, 'ExternalInput', 'trim_input_dataset');
end


function publish_common_initial_conditions_(point, eta, n_mode)
assignin('base', 'trim_Velocity_b0', point.velocity_b);
assignin('base', 'trim_Omega_b0', point.omega_b);
assignin('base', 'trim_Position0', point.position);
assignin('base', 'trim_Attitude_rad0', point.attitude_rad);
assignin('base', 'trim_RPM_cmd', point.rpm);
assignin('base', 'trim_Tilt_cmd_rad', point.tilt_cmd_rad);
assignin('base', 'trim_dotTilt_cmd_radps', point.dot_tilt_cmd_radps);
assignin('base', 'trim_Aileron_deg', point.aileron_deg);
assignin('base', 'trim_Elevator_deg', point.elevator_deg);
assignin('base', 'trim_Rudder_deg', point.rudder_deg);
assignin('base', 'x0_structure', [eta(:); zeros(n_mode, 1)]);
assignin('base', 'trim_input_dataset', point.input_dataset);

assignin('base', 'alpha_du0_wing_L', zeros(20, 1));
assignin('base', 'alpha_du0_wing_R', zeros(20, 1));
assignin('base', 'alpha_du0_v_tail_L', zeros(5, 1));
assignin('base', 'alpha_du0_v_tail_R', zeros(5, 1));
end


function validation = validate_(mdl, stop_time, eta0, n_mode)
sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setExternalInput(evalin('base', 'trim_input_dataset'));
sim_in = sim_in.setModelParameter('StopTime', num2str(stop_time, 16), ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');
out = sim(sim_in);
dyn = out.yout.getElement('Out_Dynamics').Values;
V = channels_by_time_(dyn.Velocity_b.Data, numel(dyn.Velocity_b.Time), 3);
W = channels_by_time_(dyn.Omega_b.Data, numel(dyn.Omega_b.Time), 3);
eta = channels_by_time_(dyn.elastic_displacement.Data, ...
    numel(dyn.elastic_displacement.Time), n_mode);
Q = channels_by_time_(dyn.Q.Data, numel(dyn.Q.Time), n_mode);
tail = max(1, floor(0.5*size(Q, 2))):size(Q, 2);

validation.stop_time_s = stop_time;
validation.delta_velocity_b = V(:, end) - V(:, 1);
validation.delta_omega_b = W(:, end) - W(:, 1);
validation.delta_eta = eta(:, end) - eta0(:);
validation.delta_eta_norm = norm(validation.delta_eta);
validation.modal_load_Q_mean = mean(Q(:, tail), 2);
validation.ok = max(abs(validation.delta_velocity_b)) < 0.10 && ...
    max(abs(validation.delta_omega_b)) < 0.01 && ...
    validation.delta_eta_norm < 0.05;
end


function ds = input_dataset_(point)
t = [0; 1.0e6];
names = {'Throttle', 'Tilt_cmd', 'dotTilt_cmd', 'Aileron', 'Elevator', 'Rudder'};
values = {point.rpm(:), point.tilt_cmd_rad(:), ...
    point.dot_tilt_cmd_radps(:), ...
    point.aileron_deg, point.elevator_deg, point.rudder_deg};
ds = Simulink.SimulationData.Dataset;
for i = 1:numel(names)
    sig = Simulink.SimulationData.Signal;
    sig.Name = names{i};
    value = values{i};
    sig.Values = timeseries([value(:).'; value(:).'], t);
    ds = ds.addElement(sig, names{i});
end
end


function value = vector_value_(value, count, name)
if isempty(value)
    error('%s cannot be empty here.', name);
end
value = value(:);
if isscalar(value)
    value = repmat(value, count, 1);
elseif numel(value) ~= count
    error('%s must contain either 1 or %d values.', name, count);
end
if any(~isfinite(value))
    error('%s must contain finite values.', name);
end
end


function rho = isa_density_(altitude_m)
% Standard-atmosphere density in the troposphere.
if ~isscalar(altitude_m) || ~isfinite(altitude_m) || ...
        altitude_m < -500 || altitude_m > 11000
    error(['Automatic ISA density currently supports a scalar altitude ' ...
        'from -500 m to 11000 m. Set condition.rho_kgpm3 manually otherwise.']);
end
T0 = 288.15;
p0 = 101325;
L = 0.0065;
R = 287.05287;
g0 = 9.80665;
T = T0 - L*altitude_m;
p = p0*(T/T0)^(g0/(R*L));
rho = p/(R*T);
end


function value = field_or_(source, name, fallback)
if isstruct(source) && isfield(source, name)
    value = source.(name);
else
    value = fallback;
end
end


function base = merge_settings_(base, override)
if isempty(override)
    return
end
if ~isstruct(override) || ~isscalar(override)
    error('Each custom settings group must be a scalar struct.');
end
names = fieldnames(override);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(base, name)
        error('Unknown custom setting: %s.', name);
    end
    base.(name) = override.(name);
end
end


function i = state_index_(object, suffix)
blocks = string({object.States.Block});
i = find(endsWith(blocks, suffix), 1);
if isempty(i)
    error('State block ending in "%s" was not found.', suffix);
end
end


function i = input_index_(spec, suffix)
blocks = string({spec.Inputs.Block});
i = find(endsWith(blocks, suffix), 1);
if isempty(i)
    error('Root input ending in "%s" was not found.', suffix);
end
end


function set_fixed_input_(input_spec, value)
input_spec.u = value;
input_spec.Known = true(size(value));
end


function dx = state_derivative_(op, report, i)
if isprop(op.States(i), 'dx')
    dx = op.States(i).dx(:);
elseif isprop(report, 'States') && isprop(report.States(i), 'dx')
    dx = report.States(i).dx(:);
elseif isprop(report, 'States') && isprop(report.States(i), 'Residual')
    dx = report.States(i).Residual(:);
else
    error('The operating-point report does not contain state derivatives.');
end
end


function X = channels_by_time_(data, n_time, n_channel)
data = squeeze(data);
if isvector(data)
    X = reshape(data, n_channel, []);
elseif size(data, 1) == n_time && size(data, 2) == n_channel
    X = data.';
elseif size(data, 1) == n_channel
    X = data;
elseif size(data, 2) == n_channel
    X = data.';
else
    error('Cannot reshape signal data of size %s.', mat2str(size(data)));
end
end
