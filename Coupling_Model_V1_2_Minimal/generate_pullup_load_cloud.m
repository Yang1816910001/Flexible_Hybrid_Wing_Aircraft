function result = generate_pullup_load_cloud(custom)
%GENERATE_PULLUP_LOAD_CLOUD  单频正弦升降舵载荷仿真与数据整理。
%
%   result = generate_pullup_load_cloud
%   result = generate_pullup_load_cloud(custom)
%
% 不修改、不保存 aerodynamic_model_V1_2_Wrench.slx。函数完成：
%   1) 45 m/s刚体+柔性联合配平；
%   2) 平滑包络的正弦升降舵输入；
%   3) 记录刚体、模态、节点气动/推进/重力/惯性载荷及变形后位置；
%   4) 从每个翼站的外侧节点载荷重构截面扳手；
%   5) 输出云图、MAT结果、合理性检查表。
%
% 注意：Out_Load当前只包含少数固定截面，不能直接形成展向云图；
% Out_T_engine只输出装配后的W_m。函数会临时记录模型内部的逐发动机
% 6分量扳手、节点位置和分项载荷，退出时恢复记录设置，不保存模型。

if nargin < 1
    custom = struct();
end

root_dir = fileparts(mfilename('fullpath'));
cd(root_dir);
addpath(root_dir, fullfile(root_dir, 'lib'), ...
    fullfile(root_dir, 'trim'), fullfile(root_dir, 'Library'));

mdl = 'aerodynamic_model_V1_2_Wrench';

% 配平和Wagner恢复都会临时改动模型块参数。进入函数时先保存原状态，
% 无论正常结束还是中途报错，都在退出时恢复；若模型原本未加载则关闭它。
model_was_loaded = bdIsLoaded(mdl);
load_system(mdl);
model_state = capture_temporary_model_state_(mdl, model_was_loaded);
model_cleanup = onCleanup(@() restore_temporary_model_state_(mdl, model_state));

% 工况、初始化和正弦输入统一放在独立设置函数中。
cfg = sine_load_case_settings();
condition = cfg.condition;
initial = cfg.initial;
solver = cfg.solver;
feature = cfg.feature;
maneuver = cfg.maneuver;
output = cfg.output;
checks = cfg.checks;

condition = merge_settings_(condition, field_or_(custom, 'condition', struct()));
initial = merge_settings_(initial, field_or_(custom, 'initial', struct()));
solver = merge_settings_(solver, field_or_(custom, 'solver', struct()));
feature = merge_settings_(feature, field_or_(custom, 'feature', struct()));
maneuver = merge_settings_(maneuver, field_or_(custom, 'maneuver', struct()));
output = merge_settings_(output, field_or_(custom, 'output', struct()));
checks = merge_settings_(checks, field_or_(custom, 'checks', struct()));
validate_settings_(condition, maneuver, output);

if ~isfolder(output.directory)
    mkdir(output.directory);
end

fprintf('\n=== %.1f m/s cruise elevator-sine load case ===\n', condition.U_mps);
fprintf('Model     : %s\n', mdl);
fprintf('Condition : U=%.3f m/s, altitude=%.1f m, blade pitch=%.2f deg\n', ...
    condition.U_mps, condition.altitude_m, condition.blade_pitch_deg);
fprintf('Elevator  : sine A=%+.2f deg, f=%.3f Hz over %.2f--%.2f s\n\n', ...
    maneuver.elevator_amplitude_deg, maneuver.sine_frequency_hz, ...
    maneuver.sine_start_s, maneuver.sine_end_s);

trim_feature = struct('enable_downwash', feature.enable_downwash, ...
    'enable_wagner', false, 'bypass_delays', feature.bypass_delays);
trim_custom = struct('condition', condition, 'initial', initial, ...
    'solver', solver, 'feature', trim_feature);
reuse_trim = logical(field_or_(custom, 'reuse_trim', false));
if reuse_trim && evalin('base', "exist('aeroelastic_direct_result','var')")
    trim_result = evalin('base', 'aeroelastic_direct_result');
    if ~trim_matches_condition_(trim_result, condition, trim_feature, mdl)
        error('generate_pullup_load_cloud:TrimReuseMismatch', ...
            '请求复用的配平点与当前固定工况不一致。');
    end
    fprintf('Trim       : reuse current validated operating point\n');
else
    trim_result = trim_flexible_direct_findop(0.3, trim_custom, mdl);
end
if ~isfield(trim_result, 'ok') || ~trim_result.ok
    error('generate_pullup_load_cloud:TrimFailed', ...
        '45 m/s联合配平或短时保持性验证未通过，停止载荷计算。');
end

wagner_result = struct('enabled', false, 'ok', true);
if feature.restore_wagner
    wagner_result = restore_wagner_from_flexible_direct(0.3, mdl);
    wagner_result.enabled = true;
    if ~wagner_result.ok
        error('generate_pullup_load_cloud:WagnerFailed', ...
            'Wagner恢复验证未通过，停止正弦输入仿真。');
    end
else
    % reuse_trim时模型可能已被上一工况恢复为入口状态，因此这里必须显式旁路，
    % 不能依赖配平函数或上一组扫频留下的块状态。
    set_wagner_enabled_(mdl, false);
end

old_dirty = get_param(mdl, 'Dirty');
log_specs = { ...
    [mdl '/Propulsion Dynamics/Mux1'],        1, 'cloud_engine_wrench_b'; ...
    [mdl '/Load Estimator/Sum1'],             1, 'cloud_node_position_b'; ...
    [mdl '/Load Estimator/MATLAB Function2'], 1, 'cloud_node_accel_b'; ...
    [mdl '/Load Estimator/MATLAB Function2'], 2, 'cloud_node_inertia_b'; ...
    [mdl '/Load Estimator/Product5'],         1, 'cloud_gravity_wrench_s'; ...
    [mdl '/Load Estimator/Product6'],         1, 'cloud_total_wrench_b'};
log_state = enable_logging_(log_specs);
log_cleanup = onCleanup(@() restore_logging_(mdl, log_state, old_dirty));

[input_dataset, input_history] = maneuver_inputs_(trim_result, condition, maneuver);
sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setExternalInput(input_dataset);
sim_in = sim_in.setModelParameter( ...
    'StopTime', num2str(maneuver.stop_time_s, 16), ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
sim_out = sim(sim_in);
clear log_cleanup;

%% 读取状态、输出总线和临时记录信号
dyn = output_bus_(sim_out, 'Out_Dynamics');
aero = output_bus_(sim_out, 'Out_Aero');
prop = output_bus_(sim_out, 'Out_T_engine');

t = (0:output.sample_time_s:maneuver.stop_time_s).';
if t(end) < maneuver.stop_time_s
    t(end+1, 1) = maneuver.stop_time_s;
end

struct_dir = evalin('base', 'struct_dir');
if evalin('base', "exist('nodes','var')")
    nodes = evalin('base', 'nodes');
else
    % initial_main有意不把nodes表留在基础工作区；按同一结构目录只读重载。
    nodes = readtable(fullfile(struct_dir, 'nodes.csv'));
end
Phi = evalin('base', 'Phi');
m_total_kg = evalin('base', 'm_total_kg');
if evalin('base', "exist('g','var')")
    g = evalin('base', 'g');
else
    g = 9.80665;
end
n_node = max(nodes.node_id);
n_mode = size(Phi, 2);

V_b = sample_ts_(bus_ts_(dyn, 'Velocity_b'), t, 3);
Omega_b = sample_ts_(bus_ts_(dyn, 'Omega_b'), t, 3);
Position = sample_ts_(bus_ts_(dyn, 'Position'), t, 3);
alpha_rad = sample_ts_(bus_ts_(dyn, 'Alpha'), t, 1);
attitude_rad = sample_ts_(bus_ts_(dyn, 'Attitude_rad'), t, 3);
F_cg = sample_ts_(bus_ts_(dyn, 'F_cg'), t, 3);
eta = sample_ts_(bus_ts_(dyn, 'elastic_displacement'), t, n_mode);
eta_dot = sample_ts_(bus_ts_(dyn, 'elastic_m_velocity'), t, n_mode);
eta_ddot = sample_ts_(bus_ts_(dyn, 'elastic_acc'), t, n_mode);

state = struct();
state.time_s = t;
state.velocity_b_mps = V_b;
state.omega_b_radps = Omega_b;
state.position_m = Position;
state.attitude_rad = attitude_rad;
state.attitude_deg = rad2deg(attitude_rad);
state.airspeed_mps = sqrt(sum(V_b.^2, 1));
state.alpha_deg = rad2deg(alpha_rad);
state.theta_deg = rad2deg(attitude_rad(2, :));
state.q_radps = Omega_b(2, :);
state.nz = -F_cg(3, :) / (m_total_kg * g);
state.eta = eta;
state.eta_dot = eta_dot;
state.eta_ddot = eta_ddot;

aero_s_flat = sample_ts_(bus_ts_(aero, 'W_aero_S'), t, 6*n_node);
prop_s_flat = sample_ts_(bus_ts_(prop, 'W_m'), t, 6*n_node);
engine_flat = sample_ts_(logged_ts_(sim_out, 'cloud_engine_wrench_b'), t, 48);
position_flat = sample_ts_(logged_ts_(sim_out, 'cloud_node_position_b'), t, 3*n_node);
accel_flat = sample_ts_(logged_ts_(sim_out, 'cloud_node_accel_b'), t, 3*n_node);
inertia_flat = sample_ts_(logged_ts_(sim_out, 'cloud_node_inertia_b'), t, 6*n_node);
gravity_s_flat = sample_ts_(logged_ts_(sim_out, 'cloud_gravity_wrench_s'), t, 6*n_node);
total_b_flat = sample_ts_(logged_ts_(sim_out, 'cloud_total_wrench_b'), t, 6*n_node);

n_time = numel(t);
T_sb6 = diag([-1, 1, -1, -1, 1, -1]);
aero_b = zeros(6, n_node, n_time);
prop_b = zeros(6, n_node, n_time);
gravity_b = zeros(6, n_node, n_time);
inertia_b = zeros(6, n_node, n_time);
total_b = zeros(6, n_node, n_time);
position_b = zeros(3, n_node, n_time);
accel_b = zeros(3, n_node, n_time);
engine_wrench_b = zeros(6, 8, n_time);
for k = 1:n_time
    aero_b(:, :, k) = T_sb6 * reshape(aero_s_flat(:, k), 6, n_node);
    prop_b(:, :, k) = T_sb6 * reshape(prop_s_flat(:, k), 6, n_node);
    gravity_b(:, :, k) = T_sb6 * reshape(gravity_s_flat(:, k), 6, n_node);
    inertia_b(:, :, k) = reshape(inertia_flat(:, k), 6, n_node);
    total_b(:, :, k) = reshape(total_b_flat(:, k), 6, n_node);
    position_b(:, :, k) = reshape(position_flat(:, k), 3, n_node);
    accel_b(:, :, k) = reshape(accel_flat(:, k), 3, n_node);
    engine_wrench_b(:, :, k) = reshape(engine_flat(:, k), 6, 8);
end

engine_force = engine_wrench_b(1:3, :, :);
engine_moment = engine_wrench_b(4:6, :, :);
engine_thrust = squeeze(sqrt(sum(engine_force.^2, 1)));
engine_axis = engine_force ./ max(reshape(engine_thrust, 1, 8, n_time), eps);
engine_reaction_torque = squeeze(sum(engine_moment .* engine_axis, 1));

node_loads = struct('aero', aero_b, 'prop', prop_b, ...
    'gravity', gravity_b, 'inertia', inertia_b, 'total', total_b);
wing = build_wing_sections_(nodes, position_b, node_loads);
tail = build_tail_sections_(nodes, position_b, node_loads);

assessment = assess_model_(state, wing, node_loads, accel_b, maneuver, checks);
fprintf('\n=== Model reasonableness checks ===\n');
disp(assessment.table);
fprintf('%s\n', assessment.summary);

result = struct();
result.model = mdl;
result.config = struct('condition', condition, 'initial', initial, ...
    'solver', solver, 'feature', feature, 'maneuver', maneuver, ...
    'output', output, 'checks', checks);
result.trim = trim_summary_(trim_result);
result.wagner = wagner_result;
result.input = input_history;
result.state = state;
result.nodes = struct('node_id', nodes.node_id, 'name', string(nodes.name), ...
    'component', string(nodes.component), ...
    'undeformed_position_b_m', evalin('base', 'r_cg2node_b'), ...
    'position_b_m', position_b, 'acceleration_b_mps2', accel_b, ...
    'aero_wrench_b', aero_b, 'prop_wrench_b', prop_b, ...
    'gravity_wrench_b', gravity_b, 'inertia_wrench_b', inertia_b, ...
    'total_wrench_b', total_b);
result.engines = struct('name', "E"+(1:8), 'wrench_b', engine_wrench_b, ...
    'force_b_N', engine_force, 'moment_b_Nm', engine_moment, ...
    'thrust_N', engine_thrust, 'reaction_torque_Nm', engine_reaction_torque);
result.wing = wing;
result.tail = tail;
result.assessment = assessment;
result.figure_files = strings(0, 1);
if output.plot_single_case
    result.figure_files = plot_sine_node_load_clouds(result, output);
end

writetable(assessment.table, fullfile(output.directory, ...
    'model_reasonableness_checks.csv'));
if output.save_mat
    save(fullfile(output.directory, 'sine_load_cloud_results.mat'), ...
        'result', '-v7.3');
end
assignin('base', 'sine_load_cloud_result', result);

fprintf('Results written to: %s\n', output.directory);
fprintf('Workspace variable : sine_load_cloud_result\n');
end


function [ds, history] = maneuver_inputs_(trim_result, condition, m)
t = (0:m.input_sample_time_s:m.stop_time_s).';
events = [0; m.sine_start_s; m.sine_start_s+m.transition_time_s; ...
    m.sine_end_s-m.transition_time_s; m.sine_end_s; m.stop_time_s];
t = unique([t; events]);

shape = zeros(size(t));
active = t >= m.sine_start_s & t <= m.sine_end_s;
envelope = zeros(size(t));
envelope(active) = 1;
i = t >= m.sine_start_s & t < m.sine_start_s+m.transition_time_s;
envelope(i) = cosine_step_((t(i)-m.sine_start_s) / m.transition_time_s);
i = t > m.sine_end_s-m.transition_time_s & t <= m.sine_end_s;
envelope(i) = 1-cosine_step_((t(i)-m.sine_end_s+m.transition_time_s) / ...
    m.transition_time_s);
shape(active) = sin(2*pi*m.sine_frequency_hz* ...
    (t(active)-m.sine_start_s)).*envelope(active);

rpm = trim_result.rpm(:);
cruise = [2 3 6 7];
rpm(cruise) = rpm(cruise) * m.cruise_rpm_scale;
tilt = trim_result.tilt_cmd_rad(:);
dot_tilt = trim_result.dot_tilt_cmd_radps(:);
aileron = trim_result.aileron_deg;
rudder = trim_result.rudder_deg;
elevator = trim_result.elevator_deg + m.elevator_amplitude_deg * shape;

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(repmat(rpm.', numel(t), 1), t), 'Throttle');
ds = ds.addElement(timeseries(repmat(tilt.', numel(t), 1), t), 'Tilt_cmd');
ds = ds.addElement(timeseries(repmat(dot_tilt.', numel(t), 1), t), 'dotTilt_cmd');
ds = ds.addElement(timeseries(aileron*ones(numel(t), 1), t), 'Aileron_deg');
ds = ds.addElement(timeseries(elevator, t), 'Elevator_deg');
ds = ds.addElement(timeseries(rudder*ones(numel(t), 1), t), 'Rudder_deg');

history = struct('time_s', t, 'rpm', repmat(rpm, 1, numel(t)), ...
    'tilt_rad', repmat(tilt, 1, numel(t)), ...
    'dot_tilt_radps', repmat(dot_tilt, 1, numel(t)), ...
    'aileron_deg', aileron*ones(1, numel(t)), ...
    'elevator_deg', elevator.', 'elevator_sine_shape', shape.', ...
    'rudder_deg', rudder*ones(1, numel(t)), ...
    'blade_pitch_deg', condition.blade_pitch_deg);
end


function y = cosine_step_(x)
y = 0.5*(1-cos(pi*min(max(x, 0), 1)));
end


function wing = build_wing_sections_(nodes, position_b, loads)
names = string(nodes.name(:));
undeformed_position_b = evalin('base', 'r_cg2node_b');
is_right = startsWith(names, "WR") & strlength(names) == 4;
is_left = startsWith(names, "WL") & strlength(names) == 4;

right_id = nodes.node_id(is_right);
left_id = nodes.node_id(is_left);
[~, order] = sort(abs(undeformed_position_b(2, right_id)), 'ascend');
right_id = right_id(order);
[~, order] = sort(abs(undeformed_position_b(2, left_id)), 'ascend');
left_id = left_id(order);

right_load_id = unique(evalin('base', 'surface_nodes.wing_R.node_id(:)'));
left_load_id = unique(evalin('base', 'surface_nodes.wing_L.node_id(:)'));
wing.right = side_sections_(right_id, right_load_id, ...
    undeformed_position_b, position_b, loads, wing_frame_(+1), ...
    abs(undeformed_position_b(2, right_id)));
wing.left = side_sections_(left_id, left_load_id, ...
    undeformed_position_b, position_b, loads, wing_frame_(-1), ...
    abs(undeformed_position_b(2, left_id)));
end


function tail = build_tail_sections_(nodes, position_b, loads)
names = string(nodes.name(:));
undeformed_position_b = evalin('base', 'r_cg2node_b');
right_id = sort(nodes.node_id(startsWith(names, "VTR")));
left_id = sort(nodes.node_id(startsWith(names, "VTL")));
right_load_id = unique(evalin('base', 'surface_nodes.v_tail_R.node_id(:)'));
left_load_id = unique(evalin('base', 'surface_nodes.v_tail_L.node_id(:)'));

R_right = lifting_surface_frame_(undeformed_position_b(:, right_id));
R_left = lifting_surface_frame_(undeformed_position_b(:, left_id));
s_right = surface_station_distance_(undeformed_position_b(:, right_id));
s_left = surface_station_distance_(undeformed_position_b(:, left_id));
tail.right = side_sections_(right_id, right_load_id, ...
    undeformed_position_b, position_b, loads, R_right, s_right);
tail.left = side_sections_(left_id, left_load_id, ...
    undeformed_position_b, position_b, loads, R_left, s_left);
end


function side = side_sections_(station_id, load_id, ...
        undeformed_position_b, position_b, loads, R_lb, span_coordinate)
n_station = numel(station_id);
n_time = size(position_b, 3);
source_names = fieldnames(loads);

span_coordinate = span_coordinate(:).';
span_position = (span_coordinate-span_coordinate(1)) / ...
    max(span_coordinate(end)-span_coordinate(1), eps);
origin_b = undeformed_position_b(:, station_id(1));
load_s = R_lb(1, :)*(undeformed_position_b(:, load_id)-origin_b);

side.node_id = station_id(:);
side.span_position_m = span_coordinate(:);
side.normalized_span = span_position(:);
side.frame_b_to_local = R_lb;
side.frame_description = ["x: outboard span/torsion", ...
    "y: chordwise/bending", "z: surface normal"];
side.wrench = struct;

for s = 1:numel(source_names)
    source = source_names{s};
    side.wrench.(source) = zeros(6, n_station, n_time);
end

for k = 1:n_station
    cut_id = station_id(k);
    cut_s = R_lb(1, :)*(undeformed_position_b(:, cut_id)-origin_b);
    outboard = load_s >= cut_s-1e-10;
    selected_id = load_id(outboard);
    for it = 1:n_time
        cut_position = position_b(:, cut_id, it);
        node_position = position_b(:, selected_id, it);
        for s = 1:numel(source_names)
            source = source_names{s};
            side.wrench.(source)(:, k, it) = section_wrench_(...
                loads.(source)(:, selected_id, it), node_position, ...
                cut_position, R_lb);
        end
    end
end
end


function distance = surface_station_distance_(position)
distance = [0, cumsum(vecnorm(diff(position, 1, 2), 2, 1))];
end


function R_lb = lifting_surface_frame_(position)
e_span_b = position(:, end)-position(:, 1);
e_span_b = e_span_b/max(norm(e_span_b), eps);
body_x = [1; 0; 0];
e_chord_b = body_x-e_span_b*(e_span_b.'*body_x);
e_chord_b = e_chord_b/max(norm(e_chord_b), eps);
e_normal_b = cross(e_span_b, e_chord_b);
if e_normal_b(3) > 0
    e_chord_b = -e_chord_b;
    e_normal_b = -e_normal_b;
end
R_lb = [e_span_b.'; e_chord_b.'; e_normal_b.'];
end


function wrench_local = section_wrench_(node_wrench_b, node_position_b, ...
        cut_position_b, R_lb)
force_b = node_wrench_b(1:3, :);
moment_b = node_wrench_b(4:6, :);
arm_b = node_position_b-cut_position_b;
transported_moment_b = cross(arm_b, force_b, 1)+moment_b;
wrench_local = [R_lb*sum(force_b, 2); ...
    R_lb*sum(transported_moment_b, 2)];
end


function R_lb = wing_frame_(side_sign)
e_span_b = [0; side_sign; 0];
e_normal_b = [0; 0; -1];
e_chord_b = cross(e_normal_b, e_span_b);
R_lb = [e_span_b.'; e_chord_b.'; e_normal_b.'];
end


function assessment = assess_model_(state, wing, loads, node_acceleration_b, ...
        maneuver, checks)
t = state.time_s(:).';
MR = squeeze(wing.right.wrench.total(5, :, :));
ML = squeeze(wing.left.wrench.total(5, :, :));
AR = squeeze(wing.right.wrench.aero(5, 1, :)).';

finite_ok = all(isfinite([state.airspeed_mps(:); state.alpha_deg(:); ...
    state.theta_deg(:); MR(:); ML(:)]));
symmetry_error = rms_(MR-ML) / max(rms_(0.5*(MR+ML)), eps);

root_history = 0.5*(abs(MR(1, :))+abs(ML(1, :)));
[~, peak_index] = max(root_history);
tip_root_ratio = max(abs([MR(end, peak_index), ML(end, peak_index)])) / ...
    max(mean(abs([MR(1, peak_index), ML(1, peak_index)])), eps);

profile = 0.5*(abs(MR(:, peak_index))+abs(ML(:, peak_index)));
monotonic_fraction = mean(diff(profile) <= ...
    checks.monotonic_relative_tolerance*max(profile(1), eps));

baseline = t < maneuver.sine_start_s;
sine_active = t >= maneuver.sine_start_s+maneuver.transition_time_s & ...
    t <= maneuver.sine_end_s-maneuver.transition_time_s;
aero_growth = mean(abs(AR(sine_active))) / max(mean(abs(AR(baseline))), eps);

m_node = evalin('base', 'm_node(:)');
expected_inertia_force = -node_acceleration_b .* reshape(m_node, 1, [], 1);
inertia_acceleration_error = max(abs(expected_inertia_force(:) - ...
    reshape(loads.inertia(1:3, :, :), [], 1))) / ...
    max(max(abs(expected_inertia_force(:))), 1);

modal_response = sqrt(sum((state.eta-state.eta(:, 1)).^2, 1) + ...
    0.01*sum(state.eta_dot.^2, 1));
early_recovery = t >= maneuver.sine_end_s & ...
    t <= maneuver.sine_end_s+checks.decay_window_s;
late_recovery = t >= maneuver.stop_time_s-checks.decay_window_s;
decay_ratio = rms_(modal_response(late_recovery)) / ...
    max(rms_(modal_response(early_recovery)), eps);

dy = diff(wing.right.span_position_m(:));
dM = abs(diff(MR(:, peak_index)));
relative_jump = dM / max(max(profile), eps);
engine_or_joint = abs(wing.right.span_position_m(1:end-1)-3.5) < 0.15 | ...
    abs(wing.right.span_position_m(1:end-1)-7.625) < 0.15 | dy > 0.8;
relative_jump(engine_or_joint) = 0;
max_adjacent_jump = max(relative_jump, [], 'omitnan');

component_sum = loads.aero+loads.prop+loads.gravity+loads.inertia;
closure_error = max(abs(component_sum(:)-loads.total(:))) / ...
    max(max(abs(loads.total(:))), 1);

name = ["simulation_finite"; "left_right_symmetry"; ...
    "tip_load_near_zero"; "load_grows_toward_root"; ...
    "aero_load_changes_during_sine"; "inertia_matches_node_acceleration"; ...
    "modal_response_decays"; "no_unexplained_section_jump"; ...
    "component_sum_closes"];
value = [double(finite_ok); symmetry_error; tip_root_ratio; ...
    monotonic_fraction; aero_growth; inertia_acceleration_error; ...
    decay_ratio; max_adjacent_jump; closure_error];
limit = [1; checks.symmetry_relative_error_max; checks.tip_root_ratio_max; ...
    checks.monotonic_fraction_min; checks.aero_growth_ratio_min; ...
    checks.inertia_acceleration_relative_error_max; checks.decay_ratio_max; ...
    checks.adjacent_jump_ratio_max; checks.closure_relative_error_max];
relation = ["=="; "<="; "<="; ">="; ">="; "<="; "<="; "<="; "<="];
pass = [finite_ok; symmetry_error <= limit(2); tip_root_ratio <= limit(3); ...
    monotonic_fraction >= limit(4); aero_growth >= limit(5); ...
    inertia_acceleration_error <= limit(6); decay_ratio <= limit(7); ...
    max_adjacent_jump <= limit(8); closure_error <= limit(9)];
status = repmat("FAIL", size(pass));
status(pass) = "PASS";
description = [
    "仿真和关键输出均为有限值";
    "左右翼同号当地弯矩的 RMS 相对误差";
    "峰值时刻翼尖/翼根弯矩比";
    "峰值时刻弯矩向翼尖不增大的截面比例";
    "正弦输入段/基准段气动翼根弯矩比";
    "节点惯性力与 -m 倍节点加速度的相对误差";
    "回舵后末段/初段模态响应 RMS 比";
    "排除发动机/接头后的最大相邻截面跳变量";
    "四类节点载荷之和与模型总载荷的相对误差"];

assessment.table = table(name, value, relation, limit, status, description);
assessment.peak_index = peak_index;
assessment.peak_time_s = t(peak_index);
assessment.pass = all(pass);
assessment.summary = sprintf('%d/%d checks passed', nnz(pass), numel(pass));
end


function y = rms_(x)
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = sqrt(mean(x.^2));
end
end


function files = make_figures_(state, input_history, position_b, nodes, ...
        elements, wing, assessment, maneuver, output)
visibility = on_off_(output.figure_visible);
fprintf('\n=== Generating load figures: Visible=%s ===\n', visibility);
files = strings(11, 1);
file_index = 0;
full_window = [state.time_s(1), state.time_s(end)];
maneuver_window = output.maneuver_window_s;

cloud_specs = { ...
    5, 1e-3, '挥舞弯矩', 'kN m', '01_flapwise_bending_cloud.png', ...
        '01b_flapwise_bending_cloud_maneuver.png'; ...
    3, 1e-3, '垂向剪力', 'kN', '02_vertical_shear_cloud.png', ...
        '02b_vertical_shear_cloud_maneuver.png'; ...
    4, 1e-3, '翼梁扭矩', 'kN m', '03_torsion_cloud.png', ...
        '03b_torsion_cloud_maneuver.png'};
for i = 1:size(cloud_specs, 1)
    file_index = file_index+1;
    files(file_index) = cloud_pair_(state.time_s, wing, cloud_specs{i, 1}, ...
        cloud_specs{i, 2}, cloud_specs{i, 3}, cloud_specs{i, 4}, ...
        cloud_specs{i, 5}, full_window, '全时域', maneuver, output, visibility);
    file_index = file_index+1;
    files(file_index) = cloud_pair_(state.time_s, wing, cloud_specs{i, 1}, ...
        cloud_specs{i, 2}, cloud_specs{i, 3}, cloud_specs{i, 4}, ...
        cloud_specs{i, 6}, maneuver_window, 'doublet窗口', maneuver, output, visibility);
end

file_index = file_index+1;
files(file_index) = difference_figure_(state.time_s, wing, maneuver, ...
    output, visibility);

maneuver_peak = peak_index_in_window_(state.time_s, wing, maneuver_window);
file_index = file_index+1;
files(file_index) = peak_3d_figure_(position_b, nodes, elements, wing, ...
    assessment.peak_index, assessment.peak_time_s, '全时域峰值', ...
    '05_peak_3d_structural_load.png', output, visibility);
file_index = file_index+1;
files(file_index) = peak_3d_figure_(position_b, nodes, elements, wing, ...
    maneuver_peak, state.time_s(maneuver_peak), 'doublet窗口峰值', ...
    '05b_peak_3d_structural_load_maneuver.png', output, visibility);
file_index = file_index+1;
files(file_index) = root_sources_figure_(state.time_s, wing, maneuver, ...
    output, visibility);
file_index = file_index+1;
files(file_index) = state_figure_(state, input_history, maneuver, output, visibility);
fprintf('Figure generation complete: %d figures.\n', file_index);
if output.figure_visible
    fprintf('Figure windows are kept open in MATLAB.\n');
end
end


function file = cloud_pair_(t, wing, component, scale, quantity, unit, ...
        filename, time_window, view_name, maneuver, output, visibility)
mask = time_mask_(t, time_window);
t_plot = t(mask);
x_plot = linspace(0, 1, output.span_plot_points);
fig = figure('Name', quantity, 'Visible', visibility, 'Color', 'w', ...
    'Position', [100 100 1180 560]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
sides = {'left', 'right'};
side_title = {'左机翼', '右机翼'};
all_values = [];
for i = 1:2
    value = scale*squeeze(wing.(sides{i}).wrench.total(component, :, mask)).';
    all_values = [all_values; value(:)]; %#ok<AGROW>
end
color_limit = max(abs(all_values), [], 'omitnan');
if isempty(color_limit) || ~isfinite(color_limit) || color_limit <= 0
    color_limit = 1;
end
ax = gobjects(2, 1);
for i = 1:2
    ax(i) = nexttile(tl);
    value = scale*squeeze(wing.(sides{i}).wrench.total(component, :, mask)).';
    value_plot = span_interp_(wing.(sides{i}).normalized_span, value, x_plot);
    surf(ax(i), x_plot, t_plot, value_plot, 'EdgeColor', 'none');
    view(ax(i), 2); axis(ax(i), 'tight');
    clim(ax(i), [-color_limit color_limit]); hold(ax(i), 'on');
    add_span_guides_(ax(i), wing.(sides{i}), output);
    add_event_lines_(ax(i), maneuver, output, time_window, 'y', true);
    title(ax(i), side_title{i});
    xlabel(ax(i), '归一化展向位置（翼根 \rightarrow 翼尖）');
    ylabel(ax(i), '时间 / s');
end
colormap(fig, diverging_colormap_(256));
cb = colorbar(ax(2));
cb.Layout.Tile = 'east';
ylabel(cb, unit);
title(tl, sprintf('%s云图—%s（色标为窗口内极值：±%.4g %s）', ...
    quantity, view_name, color_limit, unit));
file = export_figure_(fig, output.directory, filename, output.export_png);
end


function file = difference_figure_(t, wing, maneuver, output, visibility)
MR = 1e-3*squeeze(wing.right.wrench.total(5, :, :)).';
ML = 1e-3*squeeze(wing.left.wrench.total(5, :, :)).';
x_plot = linspace(0, 1, output.span_plot_points);
fig = figure('Name', '左右机翼弯矩差值', 'Visible', visibility, ...
    'Color', 'w', 'Position', [60 100 1450 520]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

windows = {[t(1), t(end)], output.maneuver_window_s};
window_titles = {'绝对差—全时域', '绝对差—doublet窗口'};
for i = 1:2
    mask = time_mask_(t, windows{i});
    delta = MR(mask, :)-ML(mask, :);
    value = span_interp_(wing.right.normalized_span, delta, x_plot);
    limit = max(abs(value(:)), [], 'omitnan');
    if ~isfinite(limit) || limit <= 0, limit = 1; end
    ax = nexttile(tl);
    surf(ax, x_plot, t(mask), value, 'EdgeColor', 'none');
    view(ax, 2); axis(ax, 'tight'); clim(ax, [-limit limit]); hold(ax, 'on');
    add_span_guides_(ax, wing.right, output);
    add_event_lines_(ax, maneuver, output, windows{i}, 'y', true);
    title(ax, sprintf('%s，±%.3g kN m', window_titles{i}, limit));
    xlabel(ax, '翼根 \rightarrow 翼尖'); ylabel(ax, '时间 / s'); colorbar(ax);
end

mask = time_mask_(t, output.maneuver_window_s);
denominator = 0.5*(abs(MR(mask, :))+abs(ML(mask, :)));
threshold = max(0.05*max(denominator(:), [], 'omitnan'), eps);
relative = 100*(MR(mask, :)-ML(mask, :))./max(denominator, threshold);
relative(denominator < threshold) = NaN;
relative_plot = span_interp_(wing.right.normalized_span, relative, x_plot);
relative_limit = max(abs(relative_plot(:)), [], 'omitnan');
if ~isfinite(relative_limit) || relative_limit <= 0, relative_limit = 1; end
ax = nexttile(tl);
surf(ax, x_plot, t(mask), relative_plot, 'EdgeColor', 'none');
view(ax, 2); axis(ax, 'tight'); clim(ax, [-relative_limit relative_limit]); hold(ax, 'on');
add_span_guides_(ax, wing.right, output);
add_event_lines_(ax, maneuver, output, output.maneuver_window_s, 'y', true);
title(ax, sprintf('相对差—doublet窗口，±%.3g%%', relative_limit));
xlabel(ax, '翼根 \rightarrow 翼尖'); ylabel(ax, '时间 / s'); colorbar(ax);
colormap(fig, diverging_colormap_(256));
title(tl, '右机翼 - 左机翼 挥舞弯矩差值（低于峰值5%的位置不计算百分比）');
file = export_figure_(fig, output.directory, ...
    '04_left_right_bending_difference.png', output.export_png);
end


function k = peak_index_in_window_(t, wing, time_window)
MR = squeeze(wing.right.wrench.total(5, 1, :));
ML = squeeze(wing.left.wrench.total(5, 1, :));
root_history = 0.5*(abs(MR)+abs(ML));
indices = find(time_mask_(t, time_window));
[~, local_index] = max(root_history(indices));
k = indices(local_index);
end


function file = peak_3d_figure_(position_b, nodes, elements, wing, ...
        k, peak_time_s, peak_name, filename, output, visibility)
ids = unique([wing.left.node_id; wing.right.node_id]);
moment = zeros(size(ids));
for i = 1:numel(ids)
    ir = find(wing.right.node_id == ids(i), 1);
    il = find(wing.left.node_id == ids(i), 1);
    if ~isempty(ir)
        moment(i) = wing.right.wrench.total(5, ir, k);
    elseif ~isempty(il)
        moment(i) = wing.left.wrench.total(5, il, k);
    end
end

fig = figure('Name', peak_name, 'Visible', visibility, ...
    'Color', 'w', 'Position', [100 100 1000 720]);
hold on; draw_structure_(position_b(:, :, k), elements);
all_id = nodes.node_id(:);
scatter3(position_b(1, all_id, k), position_b(2, all_id, k), ...
    position_b(3, all_id, k), 12, [0.68 0.68 0.68], 'filled');
size_value = 35+100*abs(moment)/max(max(abs(moment)), eps);
scatter3(position_b(1, ids, k), position_b(2, ids, k), ...
    position_b(3, ids, k), size_value, 1e-3*moment, 'filled');
limit = max(abs(1e-3*moment), [], 'omitnan');
if ~isfinite(limit) || limit <= 0, limit = 1; end
axis equal; grid on; view(42, 24); clim([-limit limit]);
cb = colorbar; ylabel(cb, '挥舞弯矩 / kN m');
colormap(fig, diverging_colormap_(256));
xlabel('机体系 x / m'); ylabel('机体系 y / m'); zlabel('机体系 z / m');
title(sprintf('%s：t = %.3f s（颜色保留弯矩正负，点大小表示绝对值）', ...
    peak_name, peak_time_s));
file = export_figure_(fig, output.directory, filename, output.export_png);
end


function file = root_sources_figure_(t, wing, maneuver, output, visibility)
fig = figure('Name', '翼根分项载荷', 'Visible', visibility, 'Color', 'w', ...
    'Position', [60 60 1320 760]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sides = {'left', 'right'};
titles = {'左翼根', '右翼根'};
sources = {'aero', 'prop', 'gravity', 'inertia', 'total'};
labels = {'气动', '推进', '重力', '惯性', '总计'};
colors = [0.00 0.35 0.70; 0.90 0.45 0.05; 0.55 0.55 0.55; ...
    0.48 0.22 0.70; 0.10 0.10 0.10];
baseline = t < maneuver.sine_start_s;
ax = gobjects(2, 2);
for i = 1:2
    for column = 1:2
        ax(i, column) = nexttile(tl, (i-1)*2+column);
        hold(ax(i, column), 'on');
        for j = 1:numel(sources)
            value = 1e-3*squeeze(wing.(sides{i}).wrench.(sources{j})(5, 1, :));
            if column == 2
                value = value-mean(value(baseline));
            end
            width = 1.15;
            if strcmp(sources{j}, 'total'), width = 1.8; end
            plot(ax(i, column), t, value, 'Color', colors(j, :), ...
                'LineWidth', width);
        end
        add_event_lines_(ax(i, column), maneuver, output, ...
            [t(1), t(end)], 'x', false);
        grid(ax(i, column), 'on'); ylabel(ax(i, column), '挥舞弯矩 / kN m');
        if column == 1
            title(ax(i, column), sprintf('%s—绝对载荷', titles{i}));
        else
            title(ax(i, column), sprintf('%s—相对配平点增量', titles{i}));
        end
    end
end
linkaxes(ax(:, 1), 'y'); linkaxes(ax(:, 2), 'y');
xlabel(ax(2, 1), '时间 / s'); xlabel(ax(2, 2), '时间 / s');
legend(ax(1, 1), labels, 'Location', 'best', 'NumColumns', 5);
title(tl, '翼根载荷来源：绝对值与相对配平点增量');
file = export_figure_(fig, output.directory, ...
    '06_root_load_sources.png', output.export_png);
end


function file = state_figure_(state, input_history, maneuver, output, visibility)
fig = figure('Name', 'Doublet状态响应', 'Visible', visibility, 'Color', 'w', ...
    'Position', [40 40 1220 900]);
tl = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_all = gobjects(8, 1);
ax = nexttile(tl); ax_all(1)=ax; plot(ax, state.time_s, state.airspeed_mps, 'LineWidth', 1.2);
ylabel(ax, '空速 / m s^{-1}'); grid(ax, 'on');
ax = nexttile(tl); ax_all(2)=ax; plot(ax, state.time_s, state.alpha_deg, ...
    state.time_s, state.theta_deg, 'LineWidth', 1.2);
ylabel(ax, '角度 / deg'); legend(ax, {'攻角', '俯仰角'}); grid(ax, 'on');
ax = nexttile(tl); ax_all(3)=ax; plot(ax, state.time_s, state.q_radps, 'LineWidth', 1.2);
yline(ax, 0, ':', 'Color', [0.4 0.4 0.4]);
ylabel(ax, 'q / rad s^{-1}'); grid(ax, 'on');
ax = nexttile(tl); ax_all(4)=ax; plot(ax, state.time_s, state.nz, 'LineWidth', 1.2);
yline(ax, 1, ':', '1g', 'Color', [0.4 0.4 0.4]);
ylabel(ax, 'n_z / g'); grid(ax, 'on');
ax = nexttile(tl); ax_all(5)=ax; plot(ax, state.time_s, vecnorm(state.eta, 2, 1), 'LineWidth', 1.1);
ylabel(ax, '||\eta||'); grid(ax, 'on');
ax = nexttile(tl); ax_all(6)=ax; plot(ax, state.time_s, vecnorm(state.eta_dot, 2, 1), 'LineWidth', 1.1);
ylabel(ax, '||d\eta/dt||'); grid(ax, 'on');
ax = nexttile(tl); ax_all(7)=ax; plot(ax, state.time_s, vecnorm(state.eta_ddot, 2, 1), 'LineWidth', 1.1);
ylabel(ax, '||d^2\eta/dt^2||'); grid(ax, 'on');
ax = nexttile(tl); ax_all(8)=ax; plot(ax, input_history.time_s, ...
    input_history.elevator_deg, 'LineWidth', 1.2);
ylabel(ax, '升降舵 / deg'); grid(ax, 'on');
for i = 1:numel(ax_all)
    add_event_lines_(ax_all(i), maneuver, output, ...
        [state.time_s(1), state.time_s(end)], 'x', i == numel(ax_all));
end
xlabel(tl, '时间 / s');
title(tl, sprintf('%.1f m/s 巡航升降舵doublet状态与输入', state.airspeed_mps(1)));
file = export_figure_(fig, output.directory, '07_state_response.png', output.export_png);
end


function mask = time_mask_(t, time_window)
mask = t >= time_window(1) & t <= time_window(2);
if ~any(mask)
    error('generate_pullup_load_cloud:EmptyPlotWindow', ...
        '绘图时间窗口[%g, %g] s内没有采样点。', time_window(1), time_window(2));
end
end


function value_plot = span_interp_(span, value, x_plot)
value_plot = interp1(span(:), value.', x_plot(:), 'linear').';
end


function add_span_guides_(ax, side, output)
if ~output.show_station_lines
    return
end
for x = side.normalized_span(:).'
    xline(ax, x, ':', 'Color', [0.40 0.40 0.40], 'LineWidth', 0.35, ...
        'HandleVisibility', 'off');
end
y0 = side.span_position_m(1);
y1 = side.span_position_m(end);
features = [3.5, 7.625];
labels = {'发动机站', '外翼接头'};
for i = 1:numel(features)
    x = (features(i)-y0)/max(y1-y0, eps);
    if x >= 0 && x <= 1
        xline(ax, x, '--', labels{i}, 'Color', [0.12 0.12 0.12], ...
            'LineWidth', 0.8, 'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off');
    end
end
end


function add_event_lines_(ax, maneuver, output, time_window, orientation, show_labels)
if ~output.show_event_lines
    return
end
times = [maneuver.sine_start_s, maneuver.sine_end_s];
labels = {'正弦开始', '正弦结束'};
styles = {'--', '--'};
for i = 1:numel(times)
    if times(i) >= time_window(1) && times(i) <= time_window(2)
        label = '';
        if show_labels
            label = labels{i};
        end
        if strcmp(orientation, 'x')
            xline(ax, times(i), styles{i}, label, ...
                'Color', [0.18 0.18 0.18], 'LineWidth', 0.75, ...
                'LabelHorizontalAlignment', 'left', ...
                'LabelVerticalAlignment', 'middle', ...
                'HandleVisibility', 'off');
        else
            yline(ax, times(i), styles{i}, label, ...
                'Color', [0.18 0.18 0.18], 'LineWidth', 0.75, ...
                'LabelHorizontalAlignment', 'left', ...
                'LabelVerticalAlignment', 'middle', ...
                'HandleVisibility', 'off');
        end
    end
end
end


function map = diverging_colormap_(n)
anchor_x = [0, 0.5, 1];
anchor_rgb = [0.10 0.30 0.62; 0.98 0.98 0.97; 0.88 0.35 0.08];
map = interp1(anchor_x, anchor_rgb, linspace(0, 1, n), 'linear');
end


function draw_structure_(position, elements)
n_element = height(elements);
x = nan(3*n_element, 1);
y = nan(3*n_element, 1);
z = nan(3*n_element, 1);
for i = 1:n_element
    a = elements.node_i(i);
    b = elements.node_j(i);
    j = 3*i-2;
    x(j:j+1) = position(1, [a b]);
    y(j:j+1) = position(2, [a b]);
    z(j:j+1) = position(3, [a b]);
end
plot3(x, y, z, '-', 'Color', [0.72 0.72 0.72], 'LineWidth', 0.65, ...
    'HandleVisibility', 'off');
end


function file = export_figure_(fig, directory, filename, do_export)
file = string(fullfile(directory, filename));
if strcmp(fig.Visible, 'on')
    figure(fig);
end
drawnow;
if do_export
    exportgraphics(fig, file, 'Resolution', 180);
    fprintf('  [figure] %s\n', file);
else
    file = "";
    fprintf('  [figure] %s (not exported)\n', string(fig.Name));
end
if strcmp(fig.Visible, 'off')
    close(fig);
else
    figure(fig);
    drawnow;
end
end


function summary = trim_summary_(trim_result)
keep = {'ok', 'condition', 'alpha_deg', 'elevator_deg', 'aileron_deg', ...
    'rudder_deg', 'rpm', 'tilt_cmd_rad', 'dot_tilt_cmd_radps', ...
    'eta', 'eta_dot', 'eta_ddot', 'residual', 'validation'};
summary = struct;
for i = 1:numel(keep)
    if isfield(trim_result, keep{i})
        summary.(keep{i}) = trim_result.(keep{i});
    end
end
end


function value = output_bus_(sim_out, name)
try
    yout = sim_out.get('yout');
catch
    yout = sim_out.yout;
end
if isa(yout, 'Simulink.SimulationData.Dataset')
    element = yout.getElement(name);
    if isempty(element)
        error('generate_pullup_load_cloud:MissingOutputBus', ...
            '输出数据集中没有找到 %s。', name);
    end
    value = element.Values;
elseif isstruct(yout) && isfield(yout, name)
    value = yout.(name);
else
    error('generate_pullup_load_cloud:UnsupportedOutputFormat', ...
        '无法从 yout 中读取输出总线 %s。', name);
end
end


function ts = bus_ts_(bus_value, field_name)
if isa(bus_value, 'Simulink.SimulationData.Dataset')
    element = bus_value.getElement(field_name);
    if isempty(element)
        error('generate_pullup_load_cloud:MissingBusField', ...
            '输出总线中没有字段 %s。', field_name);
    end
    ts = element.Values;
elseif isstruct(bus_value) && isfield(bus_value, field_name)
    ts = bus_value.(field_name);
else
    try
        ts = bus_value.(field_name);
    catch
        error('generate_pullup_load_cloud:MissingBusField', ...
            '输出总线中没有字段 %s。', field_name);
    end
end
end


function ts = logged_ts_(sim_out, name)
try
    logsout = sim_out.get('logsout');
catch
    logsout = sim_out.logsout;
end
if ~isa(logsout, 'Simulink.SimulationData.Dataset')
    error('generate_pullup_load_cloud:MissingSignalLog', ...
        'logsout不是Dataset，无法读取临时记录信号 %s。', name);
end
element = logsout.getElement(name);
if isempty(element)
    error('generate_pullup_load_cloud:MissingSignalLog', ...
        '没有记录到信号 %s，请确认模型模块名称未改变。', name);
end
ts = element.Values;
end


function sampled = sample_ts_(ts, target_time, expected_channels)
if ~isa(ts, 'timeseries')
    try
        ts = timeseries(ts.Data, ts.Time);
    catch
        error('generate_pullup_load_cloud:UnsupportedTimeseries', ...
            '信号不是可识别的 timeseries。');
    end
end
raw = squeeze(ts.Data);
raw = channels_by_time_(raw, numel(ts.Time), expected_channels);
sampled = interp1(double(ts.Time(:)), double(raw.'), ...
    double(target_time(:)), 'linear', 'extrap').';
if size(sampled, 1) ~= expected_channels
    error('generate_pullup_load_cloud:ChannelCount', ...
        '信号期望%d通道，实际读取到%d通道。', ...
        expected_channels, size(sampled, 1));
end
end


function value = channels_by_time_(value, n_time, expected_channels)
if isvector(value)
    if expected_channels == 1
        value = reshape(value, 1, []);
    elseif n_time == 1
        value = reshape(value, [], 1);
    elseif numel(value) == n_time*expected_channels
        value = reshape(value, expected_channels, n_time);
    else
        error('generate_pullup_load_cloud:SignalShape', ...
            '不能把长度%d的向量解释为%d通道、%d时刻。', ...
            numel(value), expected_channels, n_time);
    end
elseif size(value, 2) == n_time && size(value, 1) == expected_channels
    return
elseif size(value, 1) == n_time && size(value, 2) == expected_channels
    value = value.';
elseif numel(value) == n_time*expected_channels
    value = reshape(value, expected_channels, n_time);
else
    error('generate_pullup_load_cloud:SignalShape', ...
        '信号尺寸%s不能解释为%d通道、%d时刻。', ...
        mat2str(size(value)), expected_channels, n_time);
end
end


function state = enable_logging_(specs)
state = repmat(struct('port', [], 'data_logging', '', 'name_mode', '', ...
    'name', ''), size(specs, 1), 1);
for i = 1:size(specs, 1)
    block = specs{i, 1};
    port_number = specs{i, 2};
    signal_name = specs{i, 3};
    if getSimulinkBlockHandle(block) == -1
        error('generate_pullup_load_cloud:MissingBlock', ...
            '模型中找不到临时记录模块：%s', block);
    end
    ports = get_param(block, 'PortHandles');
    if port_number > numel(ports.Outport)
        error('generate_pullup_load_cloud:MissingPort', ...
            '%s没有第%d个输出端口。', block, port_number);
    end
    h = ports.Outport(port_number);
    state(i).port = h;
    state(i).data_logging = get_param(h, 'DataLogging');
    state(i).name_mode = get_param(h, 'DataLoggingNameMode');
    state(i).name = get_param(h, 'DataLoggingName');
    set_param(h, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
        'DataLoggingName', signal_name);
end
fprintf('Temporary signal logging enabled for %d internal signals; model is not saved.\n', ...
    size(specs, 1));
end


function state = capture_temporary_model_state_(mdl, was_loaded)
% 保存本函数及其配平/恢复子函数会改动的模型配置。
state.was_loaded = was_loaded;
state.dirty = get_param(mdl, 'Dirty');
state.blocks = repmat(struct('path', '', 'parameters', {{}}, 'values', {{}}), 0, 1);

wagner_blocks = find_system(mdl, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'IncludeCommented', 'on', 'Name', 'Wagner_SS');
for i = 1:numel(wagner_blocks)
    state.blocks(end+1) = block_state_(wagner_blocks{i}, {'Commented'});
end

downwash_blocks = find_system(mdl, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'IncludeCommented', 'on', 'Name', 'From_adu');
for i = 1:numel(downwash_blocks)
    state.blocks(end+1) = block_state_(downwash_blocks{i}, {'Commented'});
end

fixed_blocks = { ...
    [mdl '/Flexible Aircraft Dynamics/Structure_SS'], {'C', 'D'}; ...
    [mdl '/Strip Aerodynamics/Delay_Dynamics'],       {'Commented'}; ...
    [mdl '/Propulsion Dynamics/Delay_Dynamic'],       {'Commented'}; ...
    [mdl '/Strip Aerodynamics/cant_Tail'],            {'Value'}};
for i = 1:size(fixed_blocks, 1)
    if getSimulinkBlockHandle(fixed_blocks{i, 1}) ~= -1
        state.blocks(end+1) = block_state_( ...
            fixed_blocks{i, 1}, fixed_blocks{i, 2});
    end
end

state.model_parameters = {'LoadInitialState', 'LoadExternalInput', ...
    'ExternalInput', 'StopTime'};
state.model_values = cell(size(state.model_parameters));
for i = 1:numel(state.model_parameters)
    state.model_values{i} = get_param(mdl, state.model_parameters{i});
end
end


function state = block_state_(block, parameters)
state = struct('path', block, 'parameters', {parameters}, ...
    'values', {cell(size(parameters))});
for i = 1:numel(parameters)
    state.values{i} = get_param(block, parameters{i});
end
end


function restore_temporary_model_state_(mdl, state)
% 清理函数不得遮蔽原始仿真异常，所以逐项容错恢复。
if ~bdIsLoaded(mdl)
    return
end
for i = 1:numel(state.blocks)
    for j = 1:numel(state.blocks(i).parameters)
        try
            set_param(state.blocks(i).path, state.blocks(i).parameters{j}, ...
                state.blocks(i).values{j});
        catch
        end
    end
end
for i = 1:numel(state.model_parameters)
    try
        set_param(mdl, state.model_parameters{i}, state.model_values{i});
    catch
    end
end
try
    set_param(mdl, 'Dirty', state.dirty);
catch
end
if ~state.was_loaded
    try
        close_system(mdl, 0);
    catch
    end
end
end


function set_wagner_enabled_(mdl, enabled)
blocks = find_system(mdl, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'IncludeCommented', 'on', 'Name', 'Wagner_SS');
for i = 1:numel(blocks)
    if enabled
        set_param(blocks{i}, 'Commented', 'off');
    else
        set_param(blocks{i}, 'Commented', 'through');
    end
end
end


function restore_logging_(mdl, state, old_dirty)
for i = 1:numel(state)
    try
        set_param(state(i).port, 'DataLogging', state(i).data_logging, ...
            'DataLoggingNameMode', state(i).name_mode, ...
            'DataLoggingName', state(i).name);
    catch
        % 模型被用户关闭时无需阻止清理继续进行。
    end
end
try
    if strcmp(old_dirty, 'off')
        set_param(mdl, 'Dirty', 'off');
    end
catch
end
end


function validate_settings_(condition, maneuver, output)
validateattributes(condition.U_mps, {'numeric'}, {'scalar', 'positive', 'finite'});
validateattributes(condition.altitude_m, {'numeric'}, {'scalar', 'finite'});
validateattributes(condition.tilt_deg, {'numeric'}, {'vector', 'numel', 8, 'finite'});
validateattributes(condition.dot_tilt_degps, {'numeric'}, ...
    {'vector', 'numel', 8, 'finite'});
if ~(0 <= maneuver.sine_start_s && ...
        maneuver.sine_start_s < maneuver.sine_end_s && ...
        maneuver.sine_end_s < maneuver.stop_time_s)
    error('generate_pullup_load_cloud:BadManeuverTimes', ...
        '正弦时序必须满足 0 <= sine_start < sine_end < stop_time。');
end
validateattributes(maneuver.sine_frequency_hz, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(maneuver.transition_time_s, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
if 2*maneuver.transition_time_s >= ...
        maneuver.sine_end_s-maneuver.sine_start_s
    error('generate_pullup_load_cloud:BadTransitionTime', ...
        'transition_time_s必须小于正弦输入持续时间的一半。');
end
validateattributes(maneuver.elevator_amplitude_deg, {'numeric'}, ...
    {'scalar', 'finite'});
if maneuver.elevator_amplitude_deg == 0
    error('generate_pullup_load_cloud:ZeroSineAmplitude', ...
        'elevator_amplitude_deg不能为零。');
end
validateattributes(maneuver.input_sample_time_s, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(output.sample_time_s, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(output.maneuver_window_s, {'numeric'}, ...
    {'vector', 'numel', 2, 'finite'});
if output.maneuver_window_s(1) < 0 || ...
        output.maneuver_window_s(1) >= output.maneuver_window_s(2) || ...
        output.maneuver_window_s(2) > maneuver.stop_time_s
    error('generate_pullup_load_cloud:BadPlotWindow', ...
        '正弦绘图窗口必须位于[0, stop_time]内，且起点小于终点。');
end
validateattributes(output.span_plot_points, {'numeric'}, ...
    {'scalar', 'integer', '>=', 2, 'finite'});
validateattributes(output.deformation_scale, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(output.export_animation, {'logical', 'numeric'}, {'scalar'});
validateattributes(output.animation_window_s, {'numeric'}, ...
    {'vector', 'numel', 2, 'finite'});
if output.animation_window_s(1) < 0 || ...
        output.animation_window_s(1) >= output.animation_window_s(2) || ...
        output.animation_window_s(2) > maneuver.stop_time_s
    error('generate_pullup_load_cloud:BadAnimationWindow', ...
        '动画时间窗口必须位于[0, stop_time]内，且起点小于终点。');
end
validateattributes(output.animation_sample_time_s, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(output.animation_fps, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(output.show_station_lines, {'logical', 'numeric'}, {'scalar'});
validateattributes(output.show_event_lines, {'logical', 'numeric'}, {'scalar'});
validateattributes(output.plot_single_case, {'logical', 'numeric'}, {'scalar'});
end


function base = merge_settings_(base, override)
if isempty(override)
    return
end
if ~isstruct(override) || ~isscalar(override)
    error('generate_pullup_load_cloud:SettingsType', ...
        '每组自定义设置必须是标量struct。');
end
names = fieldnames(override);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(base, name)
        error('generate_pullup_load_cloud:UnknownSetting', ...
            '未知自定义字段：%s', name);
    end
    base.(name) = override.(name);
end
end


function ok = trim_matches_condition_(result, condition, trim_feature, mdl)
ok = isstruct(result) && isfield(result, 'ok') && result.ok && ...
    isfield(result, 'condition') && isfield(result, 'feature') && ...
    isfield(result, 'model') && ...
    strcmp(result.model, mdl);
if ~ok, return; end
c = result.condition;
scalar_fields = {'U_mps', 'altitude_m', 'blade_pitch_deg', ...
    'aileron_deg', 'rudder_deg'};
for i = 1:numel(scalar_fields)
    name = scalar_fields{i};
    if ~isfield(c, name) || abs(c.(name)-condition.(name)) > 1e-10
        ok = false;
        return
    end
end
vector_fields = {'tilt_deg', 'dot_tilt_degps', 'hover_rpm'};
for i = 1:numel(vector_fields)
    name = vector_fields{i};
    if ~isfield(c, name)
        ok = false;
        return
    end
    old_value = c.(name);
    new_value = condition.(name);
    if isscalar(old_value) && ~isscalar(new_value)
        old_value = repmat(old_value, size(new_value));
    elseif isscalar(new_value) && ~isscalar(old_value)
        new_value = repmat(new_value, size(old_value));
    end
    if numel(old_value) ~= numel(new_value) || ...
            any(abs(old_value(:)-new_value(:)) > 1e-10)
        ok = false;
        return
    end
end

feature_fields = {'enable_downwash', 'enable_wagner', 'bypass_delays'};
for i = 1:numel(feature_fields)
    name = feature_fields{i};
    if ~isfield(result.feature, name) || ~isfield(trim_feature, name) || ...
            logical(result.feature.(name)) ~= logical(trim_feature.(name))
        ok = false;
        return
    end
end
end


function value = field_or_(s, name, fallback)
if isfield(s, name)
    value = s.(name);
else
    value = fallback;
end
end


function value = on_off_(flag)
if flag
    value = 'on';
else
    value = 'off';
end
end
