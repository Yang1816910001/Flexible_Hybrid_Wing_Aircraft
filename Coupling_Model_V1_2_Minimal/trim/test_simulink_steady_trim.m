function result = test_simulink_steady_trim( ...
    validation_time_s, alpha_deg, bypass_dynamics_delay, fix_gravity_sign)
%TEST_SIMULINK_STEADY_TRIM 用 Simulink findop 独立验证刚体纵向配平。
%
% result = test_simulink_steady_trim
% result = test_simulink_steady_trim(5)
% result = test_simulink_steady_trim(5, 0.3945, true, false)
%
% 只加载 aerodynamic_model_V1_1_TRIM，不保存模型文件。
% 成功后写回工作区，并把当前会话中的模型准备为可直接运行状态。

if nargin < 1
    validation_time_s = 1.0;
end
if nargin < 2
    alpha_deg = 0.3945;
end
if nargin < 3
    bypass_dynamics_delay = true;
end
if nargin < 4
    fix_gravity_sign = false;
end

root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
addpath(fullfile(root, 'trim'));

% 初始化只负责加载模型所需参数；待测植物线在下面明确指定为 TRIM。
assignin('base', 'trim_mode_override', 'rigid');
evalin('base', 'initial_main_V1_1');
evalin('base', 'clear trim_mode_override');

mdl = 'aerodynamic_model_V1_1_TRIM';
loaded_here = ~bdIsLoaded(mdl);
if loaded_here
    load_system(mdl);
end
old_load_input = get_param(mdl, 'LoadExternalInput');
old_load_state = get_param(mdl, 'LoadInitialState');
gravity_block = [mdl '/Flexible Aircraft Dynamics/Const_Wmass_g'];
old_gravity_value = get_param(gravity_block, 'Value');
delay_blocks = { ...
    [mdl '/Strip Aerodynamics/Delay_Dynamics']; ...
    [mdl '/Propulsion Dynamics/Delay_Dynamic']};
old_delay_comments = cellfun(@(b) get_param(b, 'Commented'), ...
    delay_blocks, 'UniformOutput', false);
cleanup = onCleanup(@() cleanup_model_( ...
    mdl, loaded_here, old_load_input, old_load_state, ...
    gravity_block, old_gravity_value, delay_blocks, old_delay_comments));

% 新工作点搜索时不能再加载旧 report5。
set_param(mdl, 'LoadExternalInput', 'off', 'LoadInitialState', 'off');
if fix_gravity_sign
    % 诊断性临时改动：不保存模型，退出函数时恢复原值。
    set_param(gravity_block, 'Value', 'W_mass_g');
end
if bypass_dynamics_delay
    % 两个总线 Unit Delay 没出现在 operspec 状态中，findop 无法把其零初值配稳。
    % 仅在本次诊断中旁路，退出函数时恢复且不保存模型。
    cellfun(@(b) set_param(b, 'Commented', 'through'), delay_blocks);
end

U = evalin('base', 'trim_case.U_mps');
elevator_deg = -4.0;          % 升降舵初值，findop 可调
cruise_rpm = 850;             % E2/E3/E6/E7 初值，findop 可调
prop_diameter_m = 3.0;
prop_J_max = 2.4;
cruise_rpm_min = ceil(60 * U / (prop_J_max * prop_diameter_m));

spec = operspec(mdl);

% 状态顺序由块路径识别，不依赖工作点管理器界面中的显示顺序。
i_vel = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator');
i_omega = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator1');
i_pos = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator2');
i_att = state_index_(spec, '/Flexible Aircraft Dynamics/Integrator3');
i_structure = state_index_(spec, '/Flexible Aircraft Dynamics/Structure_SS');

alpha = deg2rad(alpha_deg);
spec.States(i_vel).x = U * [cos(alpha); 0; sin(alpha)];
spec.States(i_vel).Known = true(3, 1);
spec.States(i_vel).SteadyState = true(3, 1);

spec.States(i_omega).x = zeros(3, 1);
spec.States(i_omega).Known = true(3, 1);
spec.States(i_omega).SteadyState = true(3, 1);

% 前飞时位置导数不为零，位置绝不能勾选 Steady State。
spec.States(i_pos).x = zeros(3, 1);
spec.States(i_pos).Known = true(3, 1);
spec.States(i_pos).SteadyState = false(3, 1);

% 平直飞行：phi=psi=0，theta=alpha，且角速度为零。
spec.States(i_att).x = [0; alpha; 0];
spec.States(i_att).Known = true(3, 1);
spec.States(i_att).SteadyState = true(3, 1);

% 刚体方案中结构输出已经冻结；不要再约束其内部模态必须稳态。
spec.States(i_structure).x = zeros(size(spec.States(i_structure).x));
spec.States(i_structure).Known = true(size(spec.States(i_structure).x));
spec.States(i_structure).SteadyState = false(size(spec.States(i_structure).x));

% 当前四组 du 延迟没有进入有效攻角通道，也不参与刚体稳态方程。
for k = 1:numel(spec.States)
    if contains(spec.States(k).Block, '/Delay_alpha_du')
        spec.States(k).Known = true(size(spec.States(k).x));
        spec.States(k).SteadyState = false(size(spec.States(k).x));
    end
end

% 根输入：仅 E2/E3/E6/E7 和升降舵可调，其余全部固定。
i_rpm = input_index_(spec, '/Throttle');
i_tilt = input_index_(spec, '/Tilt_cmd');
i_dot_tilt = input_index_(spec, '/dotTilt_cmd');
i_aileron = input_index_(spec, '/Aileron_deg');
i_elevator = input_index_(spec, '/Elevator_deg');
i_rudder = input_index_(spec, '/Rudder_deg');

rpm0 = [0; cruise_rpm; cruise_rpm; 0; 0; cruise_rpm; cruise_rpm; 0];
spec.Inputs(i_rpm).u = rpm0;
spec.Inputs(i_rpm).Known = logical([1; 0; 0; 1; 1; 0; 0; 1]);
spec.Inputs(i_rpm).Min = [0; cruise_rpm_min; cruise_rpm_min; 0; 0; ...
    cruise_rpm_min; cruise_rpm_min; 0];
spec.Inputs(i_rpm).Max = 2000 * ones(8, 1);

set_fixed_input_(spec.Inputs(i_tilt), zeros(8, 1));
set_fixed_input_(spec.Inputs(i_dot_tilt), zeros(8, 1));
set_fixed_input_(spec.Inputs(i_aileron), 0);
set_fixed_input_(spec.Inputs(i_rudder), 0);

spec.Inputs(i_elevator).u = elevator_deg;
spec.Inputs(i_elevator).Known = false;
spec.Inputs(i_elevator).Min = -20;
spec.Inputs(i_elevator).Max = 20;

fprintf('\nfindop: U=%.3f m/s, alpha=%.3f deg, RPM0=%g, elevator0=%.3f deg\n', ...
    U, alpha_deg, cruise_rpm, elevator_deg);
fprintf('gravity constant = %s (saved model uses %s)\n', ...
    get_param(gravity_block, 'Value'), old_gravity_value);
fprintf('Dynamics bus delays bypassed = %d\n', bypass_dynamics_delay);
options = findopOptions('DisplayReport', 'off');
if isprop(options, 'OptimizerType')
    options.OptimizerType = 'lsqnonlin';
end
[op, report] = findop(mdl, spec, options);

velocity_dot = state_derivative_(op, report, i_vel);
omega_dot = state_derivative_(op, report, i_omega);
rpm = op.Inputs(i_rpm).u(:);
elevator = op.Inputs(i_elevator).u;
residual = [velocity_dot; omega_dot];

fprintf('\n=== findop result ===\n');
fprintf('alpha       = %.6f deg (fixed flight condition)\n', alpha_deg);
fprintf('elevator    = %.6f deg\n', elevator);
fprintf('RPM E1..E8 = %s\n', mat2str(rpm.', 7));
fprintf('Vdot        = %s m/s^2\n', mat2str(velocity_dot.', 6));
fprintf('omegadot    = %s rad/s^2\n', mat2str(omega_dot.', 6));
fprintf('max residual= %.6g\n', max(abs(residual)));

% 将 findop 工作点和根输入直接用于短时仿真，验证不是“只在求解器里成立”。
input_data = root_input_dataset_(rpm, elevator, validation_time_s);
sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setInitialState(getstatestruct(op));
sim_in = sim_in.setExternalInput(input_data);
sim_in = sim_in.setModelParameter( ...
    'StopTime', num2str(validation_time_s, 16), ...
    'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Dataset');

sim_out = sim(sim_in);
xout = sim_out.xout;
velocity_ts = state_values_(xout, '/Flexible Aircraft Dynamics/Integrator');
omega_ts = state_values_(xout, '/Flexible Aircraft Dynamics/Integrator1');

V0 = row_sample_(velocity_ts.Data, 1);
Vf = row_sample_(velocity_ts.Data, numel(velocity_ts.Time));
W0 = row_sample_(omega_ts.Data, 1);
Wf = row_sample_(omega_ts.Data, numel(omega_ts.Time));

fprintf('\n=== %.3f s simulation check ===\n', validation_time_s);
fprintf('Delta Velocity_b = %s m/s\n', mat2str(Vf - V0, 6));
fprintf('Delta Omega_b    = %s rad/s\n', mat2str(Wf - W0, 6));

result = struct();
result.model = mdl;
result.gravity_constant = get_param(gravity_block, 'Value');
result.dynamics_delays_bypassed = bypass_dynamics_delay;
result.alpha_deg = alpha_deg;
result.elevator_deg = elevator;
result.rpm_cmd = rpm;
result.velocity_dot = velocity_dot;
result.omega_dot = omega_dot;
result.max_residual = max(abs(residual));
result.delta_velocity = (Vf - V0).';
result.delta_omega = (Wf - W0).';
result.operating_point = op;
result.report = report;
result.Velocity_b0 = op.States(i_vel).x(:);
result.Omega_b0 = op.States(i_omega).x(:);
result.Position0 = op.States(i_pos).x(:);
result.Attitude_rad0 = op.States(i_att).x(:);

trim_tolerance = evalin('base', 'trim_case.trim_tolerance');
result.ok = result.max_residual <= trim_tolerance;
result.workspace_ready = false;
if result.ok
    if evalin('base', 'exist(''sim_time'', ''var'')')
        run_time_s = evalin('base', 'sim_time');
    else
        run_time_s = validation_time_s;
    end
    run_input_data = root_input_dataset_(rpm, elevator, run_time_s);
    apply_result_to_workspace_(result, run_input_data);

    % 先执行清理器，恢复诊断期间的临时设置；随后按配平结果重新配置当前会话。
    delete(cleanup);
    load_system(mdl);
    set_param(gravity_block, 'Value', result.gravity_constant);
    if bypass_dynamics_delay
        cellfun(@(b) set_param(b, 'Commented', 'through'), delay_blocks);
    end
    set_param(mdl, ...
        'LoadInitialState', 'off', ...
        'LoadExternalInput', 'on', ...
        'ExternalInput', 'trim_input_dataset', ...
        'StopTime', num2str(run_time_s, 16));

    result.workspace_ready = true;
    assignin('base', 'trim_result', result);
    fprintf(['\n已写入工作区并准备好模型：trim_Velocity_b0 / trim_Omega_b0 / ' ...
        'trim_Position0 / trim_Attitude_rad0 / trim_RPM_cmd / 舵面 / ' ...
        'trim_input_dataset / trim_operating_point / trim_state_struct / trim_result\n']);
    fprintf('现在可直接运行 %s；运行前不要再次执行 initial_main_V1_1。\n', mdl);
    fprintf('当前模型改动只在内存中，未保存 .slx。\n');
else
    warning('残差 %.3e 超过容差 %.3e，未用该结果覆盖配平工作区变量。', ...
        result.max_residual, trim_tolerance);
end
end


function index = state_index_(spec, suffix)
blocks = string({spec.States.Block});
index = find(endsWith(blocks, suffix), 1);
if isempty(index)
    error('找不到状态块: %s', suffix);
end
end


function index = input_index_(spec, suffix)
blocks = string({spec.Inputs.Block});
index = find(endsWith(blocks, suffix), 1);
if isempty(index)
    error('找不到根输入: %s', suffix);
end
end


function set_fixed_input_(input_spec, value)
input_spec.u = value;
input_spec.Known = true(size(value));
end


function dx = state_derivative_(op, report, index)
if isprop(op.States(index), 'dx')
    dx = op.States(index).dx(:);
elseif isprop(report, 'States') && isprop(report.States(index), 'dx')
    dx = report.States(index).dx(:);
elseif isprop(report, 'States') && isprop(report.States(index), 'Residual')
    dx = report.States(index).Residual(:);
else
    error('当前 MATLAB 版本未在已知属性中提供状态导数。');
end
end


function data = root_input_dataset_(rpm, elevator, stop_time)
t = [0; stop_time];
data = Simulink.SimulationData.Dataset;
data = data.addElement(timeseries([rpm.'; rpm.'], t), 'Throttle');
data = data.addElement(timeseries(zeros(2, 8), t), 'Tilt_cmd');
data = data.addElement(timeseries(zeros(2, 8), t), 'dotTilt_cmd');
data = data.addElement(timeseries(zeros(2, 1), t), 'Aileron_deg');
data = data.addElement(timeseries(elevator * ones(2, 1), t), 'Elevator_deg');
data = data.addElement(timeseries(zeros(2, 1), t), 'Rudder_deg');
end


function apply_result_to_workspace_(result, input_data)
assignin('base', 'trim_Velocity_b0', result.Velocity_b0);
assignin('base', 'trim_Omega_b0', result.Omega_b0);
assignin('base', 'trim_Position0', result.Position0);
assignin('base', 'trim_Attitude_rad0', result.Attitude_rad0);
assignin('base', 'trim_RPM_cmd', result.rpm_cmd(:));
assignin('base', 'trim_Tilt_cmd_rad', zeros(8, 1));
assignin('base', 'trim_dotTilt_cmd_radps', zeros(8, 1));
assignin('base', 'trim_Aileron_deg', 0);
assignin('base', 'trim_Elevator_deg', result.elevator_deg);
assignin('base', 'trim_Rudder_deg', 0);
assignin('base', 'trim_input_dataset', input_data);
assignin('base', 'trim_operating_point', result.operating_point);
assignin('base', 'trim_state_struct', getstatestruct(result.operating_point));

if evalin('base', 'exist(''trim_case'', ''var'')')
    trim_case = evalin('base', 'trim_case');
    trim_case.model = result.model;
    trim_case.alpha0_deg = result.alpha_deg;
    trim_case.elevator0_deg = result.elevator_deg;
    trim_case.aileron0_deg = 0;
    trim_case.rudder0_deg = 0;
    trim_case.rpm_cmd = result.rpm_cmd(:);
    trim_case.Velocity_b0 = result.Velocity_b0;
    trim_case.Omega_b0 = result.Omega_b0;
    trim_case.Position0 = result.Position0;
    trim_case.theta0_deg = rad2deg(result.Attitude_rad0(2));
    assignin('base', 'trim_case', trim_case);
end
end


function values = state_values_(dataset, suffix)
for k = 1:dataset.numElements
    element = dataset.getElement(k);
    if endsWith(string(element.BlockPath.getBlock(1)), suffix)
        values = element.Values;
        return
    end
end
error('仿真状态结果中找不到: %s', suffix);
end


function row = row_sample_(data, index)
data = squeeze(data);
if size(data, 1) >= index && size(data, 2) == 3
    row = data(index, :);
elseif size(data, 2) >= index && size(data, 1) == 3
    row = data(:, index).';
else
    error('无法识别状态数据维度: %s', mat2str(size(data)));
end
end


function cleanup_model_(mdl, loaded_here, old_load_input, old_load_state, ...
    gravity_block, old_gravity_value, delay_blocks, old_delay_comments)
if ~bdIsLoaded(mdl)
    return
end
set_param(mdl, 'LoadExternalInput', old_load_input, ...
    'LoadInitialState', old_load_state);
set_param(gravity_block, 'Value', old_gravity_value);
for k = 1:numel(delay_blocks)
    set_param(delay_blocks{k}, 'Commented', old_delay_comments{k});
end
if loaded_here
    close_system(mdl, 0);
end
end
