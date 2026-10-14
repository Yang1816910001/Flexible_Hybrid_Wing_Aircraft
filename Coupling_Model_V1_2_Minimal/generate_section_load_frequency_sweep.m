function sweep_result = generate_section_load_frequency_sweep(custom)
%GENERATE_SECTION_LOAD_FREQUENCY_SWEEP 四个根部截面的升降舵扫频载荷云图。
%
%   sweep_result = generate_section_load_frequency_sweep()
%   sweep_result = generate_section_load_frequency_sweep(custom)
%
% 四个截面依次为：左主翼根、右主翼根、左V尾根、右V尾根。
% 每个频率使用相同的稳定正弦周期数；载荷先扣除配平基准，再在最后
% 若干完整周期内计算极值、RMS和频率—相位云图。
%
% 本函数不修改、不保存Simulink模型。每个频率子目录保存与单一工况
% sine_load_cloud_45mps 相同的图片、动画、检查表和完整MAT结果；顶层
% MAT文件另外保存便于频率间比较的紧凑扫频汇总。

if nargin < 1
    custom = struct();
end

root_dir = fileparts(mfilename('fullpath'));
cd(root_dir);
addpath(root_dir, fullfile(root_dir, 'trim'), ...
    fullfile(root_dir, 'lib'), fullfile(root_dir, 'Library'));

base = sine_load_case_settings();

%% ================= 1. 固定工况 =================
% 可直接修改这里，也可用custom.condition覆盖。
condition = base.condition;
condition.U_mps           = 45.0;
condition.altitude_m      = 100.0;
condition.blade_pitch_deg = 34.0;
condition.tilt_deg        = zeros(8, 1);
condition.dot_tilt_degps  = zeros(8, 1);
condition.hover_rpm       = 0;

%% ================= 2. 配平初始化与求解 =================
initial = base.initial;
solver = base.solver;
feature = base.feature;

%% ================= 3. 扫频控制输入 =================
% 这是有限周期载荷扫频，不等同于无限时长的稳态FRF。
sweep.frequencies_hz =linspace(0.8,5,8);
%[0.30 0.40 0.50 0.60 0.70 0.80];
% [ ...
%     0.30 0.40 0.50 0.60 0.70 0.80 ...
%     1.00 1.10 1.15 1.20 1.25 1.30 1.40 ...
%     1.60 2.00 2.50 2.80];
sweep.elevator_amplitude_deg = 1.0; % [deg] 配平值上的正弦增量幅值
sweep.trim_hold_time_s       = 2.0; % [s] 输入前配平保持段
sweep.ramp_cycles            = 0.5; % 输入首尾余弦包络各占周期数
sweep.total_cycles           = 3;   % 包络中间的恒幅周期数
sweep.measure_cycles         = 1;   % 最后若干完整恒幅周期用于统计
sweep.recovery_cycles        = 1;   % 回舵后的恢复周期数
sweep.phase_deg              = linspace(0, 360, 181);

%% ================= 4. 输出设置 =================
output = base.output;
output.figure_visible = true;
output.export_png     = true;
output.save_mat       = true;
output.directory = fullfile(root_dir, 'outputs', ...
    'section_load_frequency_sweep_45mps');

% 允许外部批处理覆盖上述设置。
condition = merge_settings_(condition, field_or_(custom, 'condition', struct()));
initial = merge_settings_(initial, field_or_(custom, 'initial', struct()));
solver = merge_settings_(solver, field_or_(custom, 'solver', struct()));
feature = merge_settings_(feature, field_or_(custom, 'feature', struct()));
sweep = merge_settings_(sweep, field_or_(custom, 'sweep', struct()));
output = merge_settings_(output, field_or_(custom, 'output', struct()));
maneuver_base = merge_settings_(base.maneuver, ...
    field_or_(custom, 'maneuver', struct()));
checks = merge_settings_(base.checks, field_or_(custom, 'checks', struct()));
reuse_first_trim = logical(field_or_(custom, 'reuse_trim', false));

validate_settings_(sweep, output);
frequencies_hz = sort(unique(sweep.frequencies_hz(:)));
phase_deg = sweep.phase_deg(:).';

if ~isfolder(output.directory)
    mkdir(output.directory);
end
clear_previous_numbered_visuals_(output.directory);

section_names = ["左主翼根"; "右主翼根"; "左V尾根"; "右V尾根"];
component_names = ["Fx"; "Fy"; "Fz"; "Mx"; "My"; "Mz"];
component_units = ["N"; "N"; "N"; "N m"; "N m"; "N m"];
n_frequency = numel(frequencies_hz);
n_phase = numel(phase_deg);
n_section = numel(section_names);
n_component = numel(component_names);

% 四组完整展向截面。现有 peak_abs 仍保留四个根部截面，下面的
% section_groups 记录每个频率下所有截面的配平基准和峰值增量。
section_group_keys = ["wing_left"; "wing_right"; "tail_left"; "tail_right"];
section_group_labels = ["左主翼"; "右主翼"; "左V尾"; "右V尾"];
section_groups = struct();

peak_abs = nan(n_frequency, n_component, n_section);
positive_peak = nan(size(peak_abs));
negative_peak = nan(size(peak_abs));
rms_load = nan(size(peak_abs));
phase_wrench = nan(n_frequency, n_phase, n_component, n_section);
active_duration_s = nan(n_frequency, 1);
total_duration_s = nan(n_frequency, 1);
measure_start_s = nan(n_frequency, 1);
measure_end_s = nan(n_frequency, 1);
trim_summary = struct();
rigid_velocity_peak_mps = nan(n_frequency, 3);
rigid_omega_peak_radps = nan(n_frequency, 3);
rigid_position_peak_m = nan(n_frequency, 3);
rigid_attitude_peak_rad = nan(n_frequency, 3);

fprintf('\n=== Four-section elevator frequency sweep ===\n');
fprintf('Condition : U=%.3f m/s, altitude=%.1f m, blade pitch=%.2f deg\n', ...
    condition.U_mps, condition.altitude_m, condition.blade_pitch_deg);
fprintf('Sections  : left/right wing root and left/right V-tail root\n');
fprintf('Amplitude : %+.3f deg; constant-amplitude cycles=%d; measured=%d\n', ...
    sweep.elevator_amplitude_deg, sweep.total_cycles, ...
    sweep.measure_cycles);

for i = 1:n_frequency
    frequency_hz = frequencies_hz(i);
    period_s = 1 / frequency_hz;
    transition_time_s = sweep.ramp_cycles * period_s;
    sine_start_s = sweep.trim_hold_time_s;
    sine_end_s = sine_start_s + ...
        (sweep.total_cycles + 2*sweep.ramp_cycles) * period_s;
    stop_time_s = sine_end_s + sweep.recovery_cycles * period_s;

    % 选择包络恒幅区内最后若干个、与输入相位对齐的完整周期。
    plateau_end_s = sine_end_s - transition_time_s;
    last_cycle_number = floor((plateau_end_s-sine_start_s) / period_s + 1e-10);
    measurement_end_s = sine_start_s + last_cycle_number * period_s;
    measurement_start_s = measurement_end_s - ...
        sweep.measure_cycles * period_s;

    fprintf('\n[%d/%d] f=%.3f Hz, simulation %.3f s, measure %.3f--%.3f s\n', ...
        i, n_frequency, frequency_hz, stop_time_s, ...
        measurement_start_s, measurement_end_s);

    maneuver = maneuver_base;
    maneuver.stop_time_s = stop_time_s;
    maneuver.sine_start_s = sine_start_s;
    maneuver.sine_end_s = sine_end_s;
    maneuver.sine_frequency_hz = frequency_hz;
    maneuver.elevator_amplitude_deg = sweep.elevator_amplitude_deg;
    maneuver.transition_time_s = transition_time_s;

    case_output = output;
    case_output.figure_visible = false;
    % 每个频率都按完整单一工况交付。export_png/save_mat/
    % plot_single_case/export_animation 沿用顶部 output 设置，因此默认
    % 内容与 outputs/sine_load_cloud_45mps 一致。
    case_output.export_png = output.export_png;
    case_output.save_mat = output.save_mat;
    case_output.plot_single_case = output.plot_single_case;
    case_output.export_animation = output.export_animation;
    case_output.maneuver_window_s = [sine_start_s, sine_end_s];
    case_output.animation_window_s = [0, stop_time_s];
    case_output.directory = fullfile(output.directory, ...
        sprintf('case_%02d_%0.3f_Hz', i, frequency_hz));

    case_custom = struct('condition', condition, 'initial', initial, ...
        'solver', solver, 'feature', feature, 'maneuver', maneuver, ...
        'output', case_output, 'checks', checks, ...
        'reuse_trim', i > 1 || reuse_first_trim);

    result = generate_pullup_load_cloud(case_custom);
    if i == 1
        trim_summary = result.trim;
    end

    t = result.state.time_s(:);
    section_wrench = extract_four_sections_(result);
    baseline = t < sine_start_s;
    measure = t >= measurement_start_s-1e-10 & ...
        t <= measurement_end_s+1e-10;
    if ~any(baseline) || nnz(measure) < 3
        error('generate_section_load_frequency_sweep:BadWindow', ...
            'Frequency %.3f Hz has insufficient baseline or measurement samples.', ...
            frequency_hz);
    end

    baseline_wrench = mean(section_wrench(:, :, baseline), 3);
    delta_wrench = section_wrench - baseline_wrench;
    measured_wrench = delta_wrench(:, :, measure);

    [rigid_velocity_peak_mps(i, :), rigid_omega_peak_radps(i, :), ...
        rigid_position_peak_m(i, :), rigid_attitude_peak_rad(i, :)] = ...
        rigid_state_peaks_(result.state, baseline, measure);

    % 保存四组表面的全部截面，而不只保存根部截面。
    case_groups = extract_section_groups_(result);
    for g = 1:numel(section_group_keys)
        key = char(section_group_keys(g));
        group = case_groups.(key);
        group_wrench = group.wrench;
        group_trim = mean(group_wrench(:, :, baseline), 3);
        group_delta = group_wrench-group_trim;
        group_measured = group_delta(:, :, measure);

        if i == 1
            n_station = size(group_wrench, 2);
            section_groups.(key) = struct( ...
                'label', section_group_labels(g), ...
                'node_id', group.node_id(:), ...
                'normalized_span', group.normalized_span(:), ...
                'trim_load', nan(n_frequency, n_component, n_station), ...
                'peak_abs', nan(n_frequency, n_component, n_station), ...
                'positive_peak', nan(n_frequency, n_component, n_station), ...
                'negative_peak', nan(n_frequency, n_component, n_station), ...
                'rms', nan(n_frequency, n_component, n_station));
        else
            expected = section_groups.(key).node_id;
            if ~isequal(group.node_id(:), expected)
                error('generate_section_load_frequency_sweep:SectionChanged', ...
                    'The section definition of %s changed between cases.', ...
                    section_group_labels(g));
            end
        end

        section_groups.(key).trim_load(i, :, :) = reshape(group_trim, ...
            1, n_component, []);
        section_groups.(key).peak_abs(i, :, :) = reshape( ...
            max(abs(group_measured), [], 3), 1, n_component, []);
        section_groups.(key).positive_peak(i, :, :) = reshape( ...
            max(group_measured, [], 3), 1, n_component, []);
        section_groups.(key).negative_peak(i, :, :) = reshape( ...
            min(group_measured, [], 3), 1, n_component, []);
        section_groups.(key).rms(i, :, :) = reshape( ...
            sqrt(mean(group_measured.^2, 3)), 1, n_component, []);
    end

    peak_abs(i, :, :) = reshape(max(abs(measured_wrench), [], 3), ...
        1, n_component, n_section);
    positive_peak(i, :, :) = reshape(max(measured_wrench, [], 3), ...
        1, n_component, n_section);
    negative_peak(i, :, :) = reshape(min(measured_wrench, [], 3), ...
        1, n_component, n_section);
    rms_load(i, :, :) = reshape(sqrt(mean(measured_wrench.^2, 3)), ...
        1, n_component, n_section);
    phase_wrench(i, :, :, :) = phase_average_(t, delta_wrench, ...
        frequency_hz, sine_start_s, measurement_end_s, ...
        sweep.measure_cycles, phase_deg);

    active_duration_s(i) = sine_end_s-sine_start_s;
    total_duration_s(i) = stop_time_s;
    measure_start_s(i) = measurement_start_s;
    measure_end_s(i) = measurement_end_s;

    clear result section_wrench delta_wrench measured_wrench case_groups
end

%% 左右对称性：基于窗口内绝对峰值
primary_components = [3 5 4]; % Fz, My, Mx
wing_symmetry_error = nan(n_frequency, numel(primary_components));
tail_symmetry_error = nan(size(wing_symmetry_error));
for j = 1:numel(primary_components)
    component = primary_components(j);
    wing_left = peak_abs(:, component, 1);
    wing_right = peak_abs(:, component, 2);
    tail_left = peak_abs(:, component, 3);
    tail_right = peak_abs(:, component, 4);
    wing_symmetry_error(:, j) = abs(wing_left-wing_right) ./ ...
        max(max(wing_left, wing_right), eps);
    tail_symmetry_error(:, j) = abs(tail_left-tail_right) ./ ...
        max(max(tail_left, tail_right), eps);
end

case_table = table(frequencies_hz, active_duration_s, total_duration_s, ...
    measure_start_s, measure_end_s, ...
    'VariableNames', {'Frequency_Hz', 'ActiveDuration_s', ...
    'SimulationDuration_s', 'MeasureStart_s', 'MeasureEnd_s'});
summary_table = summary_table_(frequencies_hz, section_names, ...
    peak_abs, positive_peak, negative_peak, rms_load);
all_section_peak_table = all_section_peak_table_( ...
    frequencies_hz, section_group_keys, section_groups);
symmetry_table = table(frequencies_hz, ...
    wing_symmetry_error(:, 1), wing_symmetry_error(:, 2), ...
    wing_symmetry_error(:, 3), tail_symmetry_error(:, 1), ...
    tail_symmetry_error(:, 2), tail_symmetry_error(:, 3), ...
    'VariableNames', {'Frequency_Hz', 'Wing_Fz', 'Wing_My', ...
    'Wing_Mx', 'VTail_Fz', 'VTail_My', 'VTail_Mx'});
rigid_state_peak = struct( ...
    'velocity_b_mps', rigid_velocity_peak_mps, ...
    'omega_b_radps', rigid_omega_peak_radps, ...
    'position_m', rigid_position_peak_m, ...
    'attitude_rad', rigid_attitude_peak_rad);
rigid_state_table = table(frequencies_hz, ...
    rigid_velocity_peak_mps(:, 1), rigid_velocity_peak_mps(:, 2), ...
    rigid_velocity_peak_mps(:, 3), ...
    rad2deg(rigid_omega_peak_radps(:, 1)), ...
    rad2deg(rigid_omega_peak_radps(:, 2)), ...
    rad2deg(rigid_omega_peak_radps(:, 3)), ...
    rigid_position_peak_m(:, 1), rigid_position_peak_m(:, 2), ...
    rigid_position_peak_m(:, 3), ...
    rad2deg(rigid_attitude_peak_rad(:, 1)), ...
    rad2deg(rigid_attitude_peak_rad(:, 2)), ...
    rad2deg(rigid_attitude_peak_rad(:, 3)), ...
    'VariableNames', {'Frequency_Hz', 'DeltaU_Peak_mps', ...
    'DeltaV_Peak_mps', 'DeltaW_Peak_mps', 'DeltaP_Peak_degps', ...
    'DeltaQ_Peak_degps', 'DeltaR_Peak_degps', 'DeltaX_Peak_m', ...
    'DeltaY_Peak_m', 'DeltaZ_Peak_m', 'DeltaPhi_Peak_deg', ...
    'DeltaTheta_Peak_deg', 'DeltaPsi_Peak_deg'});

sweep_result = struct();
sweep_result.config = struct('condition', condition, 'initial', initial, ...
    'solver', solver, 'feature', feature, 'sweep', sweep, 'output', output);
sweep_result.trim = trim_summary;
sweep_result.frequencies_hz = frequencies_hz;
sweep_result.phase_deg = phase_deg;
sweep_result.section_names = section_names;
sweep_result.component_names = component_names;
sweep_result.component_units = component_units;
sweep_result.peak_abs = peak_abs;
sweep_result.positive_peak = positive_peak;
sweep_result.negative_peak = negative_peak;
sweep_result.rms = rms_load;
sweep_result.phase_wrench = phase_wrench;
sweep_result.section_groups = section_groups;
sweep_result.wing_symmetry_error = wing_symmetry_error;
sweep_result.tail_symmetry_error = tail_symmetry_error;
sweep_result.case_table = case_table;
sweep_result.summary_table = summary_table;
sweep_result.all_section_peak_table = all_section_peak_table;
sweep_result.symmetry_table = symmetry_table;
sweep_result.rigid_state_peak = rigid_state_peak;
sweep_result.rigid_state_table = rigid_state_table;
sweep_result.figure_files = plot_sweep_(sweep_result, output);

writetable(case_table, fullfile(output.directory, ...
    'section_frequency_case_definition.csv'));
writetable(summary_table, fullfile(output.directory, ...
    'section_frequency_load_summary.csv'));
writetable(all_section_peak_table, fullfile(output.directory, ...
    'all_section_frequency_peak_summary.csv'));
writetable(symmetry_table, fullfile(output.directory, ...
    'section_frequency_symmetry.csv'));
writetable(rigid_state_table, fullfile(output.directory, ...
    'rigid_body_state_frequency_summary.csv'));
if output.save_mat
    save(fullfile(output.directory, ...
        'section_load_frequency_sweep_results.mat'), ...
        'sweep_result', '-v7.3');
end
assignin('base', 'section_load_frequency_sweep', sweep_result);

fprintf('\nSweep completed.\n');
fprintf('Results   : %s\n', output.directory);
fprintf('Workspace : section_load_frequency_sweep\n');
end


function section_wrench = extract_four_sections_(result)
n_time = numel(result.state.time_s);
section_wrench = zeros(6, 4, n_time);
section_wrench(:, 1, :) = result.wing.left.wrench.total(:, 1, :);
section_wrench(:, 2, :) = result.wing.right.wrench.total(:, 1, :);
section_wrench(:, 3, :) = result.tail.left.wrench.total(:, 1, :);
section_wrench(:, 4, :) = result.tail.right.wrench.total(:, 1, :);
end


function [velocity_peak, omega_peak, position_peak, attitude_peak] = ...
        rigid_state_peaks_(state, baseline, measure)
velocity_peak = state_peak_(state.velocity_b_mps, baseline, measure);
omega_peak = state_peak_(state.omega_b_radps, baseline, measure);
position_peak = position_residual_peak_( ...
    state.time_s, state.position_m, baseline, measure);
attitude_peak = state_peak_(state.attitude_rad, baseline, measure);
end


function peak = state_peak_(value, baseline, measure)
reference = mean(value(:, baseline), 2);
delta = value(:, measure)-reference;
peak = max(abs(delta), [], 2).';
end


function peak = position_residual_peak_(t, position, baseline, measure)
% 平飞位置本来沿稳态轨迹增长；扣除输入前线性轨迹，避免不同仿真时长
% 被误判为位置频响。
t = t(:).';
reference = zeros(size(position));
for component = 1:size(position, 1)
    coefficient = polyfit(t(baseline), position(component, baseline), 1);
    reference(component, :) = polyval(coefficient, t);
end
delta = position(:, measure)-reference(:, measure);
peak = max(abs(delta), [], 2).';
end


function groups = extract_section_groups_(result)
% Return every spanwise cut in the four lifting-surface groups.
groups = struct();
groups.wing_left = section_group_(result.wing.left);
groups.wing_right = section_group_(result.wing.right);
groups.tail_left = section_group_(result.tail.left);
groups.tail_right = section_group_(result.tail.right);
end


function group = section_group_(side)
group.node_id = side.node_id(:);
group.normalized_span = side.normalized_span(:);
group.wrench = side.wrench.total;
end


function phase_value = phase_average_(t, value, frequency_hz, ...
        input_start_s, measurement_end_s, n_cycles, phase_deg)
% 返回尺寸 [Nphase, 6, 4]，相位零点与升降舵正弦命令一致。
period_s = 1/frequency_hz;
n_phase = numel(phase_deg);
phase_value = zeros(n_phase, size(value, 1), size(value, 2));

last_cycle = round((measurement_end_s-input_start_s)/period_s);
first_cycle = last_cycle-n_cycles;
for cycle = first_cycle:last_cycle-1
    query_time = input_start_s + ...
        (cycle + phase_deg(:)/360) * period_s;
    for component = 1:size(value, 1)
        for section = 1:size(value, 2)
            history = squeeze(value(component, section, :));
            phase_value(:, component, section) = ...
                phase_value(:, component, section) + ...
                interp1(t, history, query_time, 'linear');
        end
    end
end
phase_value = phase_value/n_cycles;
end


function table_out = summary_table_(frequencies_hz, section_names, ...
        peak_abs, positive_peak, negative_peak, rms_load)
n_frequency = numel(frequencies_hz);
n_section = numel(section_names);
frequency = kron(frequencies_hz(:), ones(n_section, 1));
section = repmat(section_names(:), n_frequency, 1);

Fz_peak_kN = flatten_metric_(peak_abs, 3, 1e-3);
Fz_positive_kN = flatten_metric_(positive_peak, 3, 1e-3);
Fz_negative_kN = flatten_metric_(negative_peak, 3, 1e-3);
Fz_rms_kN = flatten_metric_(rms_load, 3, 1e-3);
My_peak_kNm = flatten_metric_(peak_abs, 5, 1e-3);
My_positive_kNm = flatten_metric_(positive_peak, 5, 1e-3);
My_negative_kNm = flatten_metric_(negative_peak, 5, 1e-3);
My_rms_kNm = flatten_metric_(rms_load, 5, 1e-3);
Mx_peak_kNm = flatten_metric_(peak_abs, 4, 1e-3);
Mx_positive_kNm = flatten_metric_(positive_peak, 4, 1e-3);
Mx_negative_kNm = flatten_metric_(negative_peak, 4, 1e-3);
Mx_rms_kNm = flatten_metric_(rms_load, 4, 1e-3);

table_out = table(frequency, section, Fz_peak_kN, Fz_positive_kN, ...
    Fz_negative_kN, Fz_rms_kN, My_peak_kNm, My_positive_kNm, ...
    My_negative_kNm, My_rms_kNm, Mx_peak_kNm, Mx_positive_kNm, ...
    Mx_negative_kNm, Mx_rms_kNm, ...
    'VariableNames', {'Frequency_Hz', 'Section', 'FzPeak_kN', ...
    'FzPositive_kN', 'FzNegative_kN', 'FzRMS_kN', 'MyPeak_kNm', ...
    'MyPositive_kNm', 'MyNegative_kNm', 'MyRMS_kNm', 'MxPeak_kNm', ...
    'MxPositive_kNm', 'MxNegative_kNm', 'MxRMS_kNm'});
end


function value = flatten_metric_(array, component, scale)
matrix = squeeze(array(:, component, :));
value = scale*reshape(matrix.', [], 1);
end


function table_out = all_section_peak_table_(frequencies_hz, keys, groups)
% One row per frequency and physical section. Loads are local-section axes.
table_out = table();
for g = 1:numel(keys)
    key = char(keys(g));
    group = groups.(key);
    n_frequency = numel(frequencies_hz);
    n_station = numel(group.node_id);

    frequency = kron(frequencies_hz(:), ones(n_station, 1));
    surface = repmat(string(group.label), n_frequency*n_station, 1);
    node_id = repmat(group.node_id(:), n_frequency, 1);
    normalized_span = repmat(group.normalized_span(:), n_frequency, 1);

    trim_fx = flatten_group_(group.trim_load, 1, 1e-3);
    trim_fy = flatten_group_(group.trim_load, 2, 1e-3);
    trim_fz = flatten_group_(group.trim_load, 3, 1e-3);
    trim_mx = flatten_group_(group.trim_load, 4, 1e-3);
    trim_my = flatten_group_(group.trim_load, 5, 1e-3);
    trim_mz = flatten_group_(group.trim_load, 6, 1e-3);
    peak_fx = flatten_group_(group.peak_abs, 1, 1e-3);
    peak_fy = flatten_group_(group.peak_abs, 2, 1e-3);
    peak_fz = flatten_group_(group.peak_abs, 3, 1e-3);
    peak_mx = flatten_group_(group.peak_abs, 4, 1e-3);
    peak_my = flatten_group_(group.peak_abs, 5, 1e-3);
    peak_mz = flatten_group_(group.peak_abs, 6, 1e-3);

    part = table(frequency, surface, node_id, normalized_span, ...
        trim_fx, trim_fy, trim_fz, trim_mx, trim_my, trim_mz, ...
        peak_fx, peak_fy, peak_fz, peak_mx, peak_my, peak_mz, ...
        'VariableNames', {'Frequency_Hz', 'Surface', 'NodeID', ...
        'NormalizedSpan', 'TrimFx_kN', 'TrimFy_kN', 'TrimFz_kN', ...
        'TrimMx_kNm', 'TrimMy_kNm', 'TrimMz_kNm', ...
        'DeltaFxPeak_kN', 'DeltaFyPeak_kN', 'DeltaFzPeak_kN', ...
        'DeltaMxPeak_kNm', 'DeltaMyPeak_kNm', 'DeltaMzPeak_kNm'});
    if isempty(table_out)
        table_out = part;
    else
        table_out = [table_out; part]; %#ok<AGROW>
    end
end
end


function value = flatten_group_(array, component, scale)
matrix = squeeze(array(:, component, :));
if isvector(matrix)
    matrix = reshape(matrix, size(array, 1), []);
end
value = scale*reshape(matrix.', [], 1);
end


function files = plot_sweep_(result, output)
visibility = on_off_(output.figure_visible);
files = strings(11, 1);
files(1) = peak_section_cloud_(result, output, visibility);
files(2) = phase_cloud_(result, 3, 'Fz', 'kN', 1e-3, ...
    '02_frequency_phase_Fz_cloud.png', output, visibility);
files(3) = phase_cloud_(result, 5, 'My', 'kN m', 1e-3, ...
    '03_frequency_phase_My_cloud.png', output, visibility);
files(4) = phase_cloud_(result, 4, 'Mx', 'kN m', 1e-3, ...
    '04_frequency_phase_Mx_cloud.png', output, visibility);
files(5) = peak_response_figure_(result, output, visibility);
files(6) = symmetry_figure_(result, output, visibility);
files(7) = sweep_definition_figure_(result, output, visibility);
files(8) = all_section_peak_cloud_(result, 3, 'Fz', 'kN', 1e-3, ...
    '08_all_sections_Fz_peak_cloud.png', output, visibility);
files(9) = all_section_peak_cloud_(result, 5, 'My', 'kN m', 1e-3, ...
    '09_all_sections_My_peak_cloud.png', output, visibility);
files(10) = all_section_peak_cloud_(result, 4, 'Mx', 'kN m', 1e-3, ...
    '10_all_sections_Mx_peak_cloud.png', output, visibility);
files(11) = rigid_state_frequency_figure_(result, output, visibility);
end


function file = rigid_state_frequency_figure_(result, output, visibility)
f = result.frequencies_hz;
s = result.rigid_state_peak;
fig = figure('Name', '刚体积分状态频率响应', 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 35 1320 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl);
plot(ax, f, s.velocity_b_mps, '-o', 'LineWidth', 1.2, 'MarkerSize', 4);
xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '峰值增量 / m s^{-1}');
title(ax, '速度：max|[u,v,w]-trim|');
legend(ax, {'u', 'v', 'w'}, 'Location', 'best'); grid(ax, 'on');

ax = nexttile(tl);
plot(ax, f, rad2deg(s.omega_b_radps), '-o', ...
    'LineWidth', 1.2, 'MarkerSize', 4);
xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '峰值增量 / deg s^{-1}');
title(ax, '角速度：max|[p,q,r]-trim|');
legend(ax, {'p', 'q', 'r'}, 'Location', 'best'); grid(ax, 'on');

ax = nexttile(tl);
plot(ax, f, s.position_m, '-o', 'LineWidth', 1.2, 'MarkerSize', 4);
xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '峰值增量 / m');
title(ax, '位置：相对稳态直线轨迹的峰值偏离');
legend(ax, {'x', 'y', 'z'}, 'Location', 'best'); grid(ax, 'on');

ax = nexttile(tl);
plot(ax, f, rad2deg(s.attitude_rad), '-o', ...
    'LineWidth', 1.2, 'MarkerSize', 4);
xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '峰值增量 / deg');
title(ax, '姿态：max|[\phi,\theta,\psi]-trim|');
legend(ax, {'\phi', '\theta', '\psi'}, 'Location', 'best'); grid(ax, 'on');

title(tl, '刚体积分状态相对配平点的扫频峰值响应');
file = export_figure_(fig, output, ...
    '11_rigid_body_state_frequency_response.png');
end


function file = peak_section_cloud_(result, output, visibility)
fig = figure('Name', '频率—截面峰值载荷', 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 50 1320 460]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
components = [3 5 4];
titles = {'法向剪力 Fz', '挥舞弯矩 My', '扭矩 Mx'};
units = {'kN', 'kN m', 'kN m'};

for j = 1:3
    ax = nexttile(tl);
    value = 1e-3*squeeze(result.peak_abs(:, components(j), :));
    draw_cloud_(ax, 1:4, result.frequencies_hz, value);
    axis(ax, 'tight'); colormap(ax, parula(256));
    xticks(ax, 1:4); xticklabels(ax, result.section_names);
    xtickangle(ax, 20);
    xlabel(ax, '载荷截面'); ylabel(ax, '升降舵频率 / Hz');
    title(ax, titles{j});
    cb = colorbar(ax); ylabel(cb, ['窗口峰值增量 / ' units{j}]);
end
title(tl, '四截面扫频窗口内绝对峰值（已扣除配平基准）');
file = export_figure_(fig, output, '01_frequency_section_peak_cloud.png');
end


function file = phase_cloud_(result, component, quantity, unit, scale, ...
        filename, output, visibility)
fig = figure('Name', ['频率—相位 ' quantity], 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 30 1320 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

all_value = scale*squeeze(result.phase_wrench(:, :, component, :));
limit = max(abs(all_value(:)));
if ~isfinite(limit) || limit <= 0, limit = 1; end
for section = 1:4
    ax = nexttile(tl);
    value = scale*squeeze(result.phase_wrench(:, :, component, section));
    draw_cloud_(ax, result.phase_deg, result.frequencies_hz, value);
    axis(ax, 'tight');
    clim(ax, [-limit limit]); colormap(ax, diverging_colormap_(256));
    xlabel(ax, '升降舵输入相位 / deg');
    ylabel(ax, '升降舵频率 / Hz');
    title(ax, result.section_names(section));
    cb = colorbar(ax); ylabel(cb, [quantity '增量 / ' unit]);
end
title(tl, ['四截面频率—相位' quantity '云图（末段周期平均）']);
file = export_figure_(fig, output, filename);
end


function file = peak_response_figure_(result, output, visibility)
fig = figure('Name', '截面载荷峰值频响', 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 50 1250 850]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
components = [3 5 4];
titles = {'法向剪力 Fz', '挥舞弯矩 My', '扭矩 Mx'};
units = {'kN', 'kN m', 'kN m'};
colors = lines(4);
for j = 1:3
    ax = nexttile(tl); hold(ax, 'on');
    value = 1e-3*squeeze(result.peak_abs(:, components(j), :));
    for section = 1:4
        plot(ax, result.frequencies_hz, value(:, section), '-o', ...
            'Color', colors(section, :), 'LineWidth', 1.25, ...
            'MarkerSize', 4);
    end
    xlabel(ax, '升降舵频率 / Hz');
    ylabel(ax, ['窗口峰值 / ' units{j}]);
    title(ax, titles{j}); grid(ax, 'on'); axis(ax, 'tight');
    if j == 1
        legend(ax, result.section_names, 'Location', 'best', ...
            'NumColumns', 2);
    end
end
title(tl, '四截面载荷峰值频率响应');
file = export_figure_(fig, output, '05_section_peak_frequency_response.png');
end


function file = symmetry_figure_(result, output, visibility)
fig = figure('Name', '左右截面对称性', 'Visible', visibility, ...
    'Color', 'w', 'Position', [50 100 1250 520]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
names = {'Fz', 'My', 'Mx'};
for panel = 1:2
    ax = nexttile(tl); hold(ax, 'on');
    if panel == 1
        value = result.wing_symmetry_error;
        panel_title = '左右主翼根';
    else
        value = result.tail_symmetry_error;
        panel_title = '左右V尾根';
    end
    plot(ax, result.frequencies_hz, value, '-o', 'LineWidth', 1.2, ...
        'MarkerSize', 4);
    xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '峰值相对差异');
    title(ax, panel_title); grid(ax, 'on'); axis(ax, 'tight');
    legend(ax, names, 'Location', 'best');
end
title(tl, '左右截面窗口峰值对称性');
file = export_figure_(fig, output, '06_left_right_symmetry_error.png');
end


function file = sweep_definition_figure_(result, output, visibility)
fig = figure('Name', '扫频输入定义', 'Visible', visibility, ...
    'Color', 'w', 'Position', [70 100 1200 500]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl);
phase = result.phase_deg;
amplitude = result.config.sweep.elevator_amplitude_deg;
plot(ax, phase, amplitude*sin(deg2rad(phase)), 'LineWidth', 1.6);
xlabel(ax, '输入相位 / deg'); ylabel(ax, '升降舵增量 / deg');
title(ax, '单周期控制命令'); grid(ax, 'on'); axis(ax, 'tight');

ax = nexttile(tl); hold(ax, 'on');
plot(ax, result.case_table.Frequency_Hz, ...
    result.case_table.ActiveDuration_s, '-o', 'LineWidth', 1.3);
plot(ax, result.case_table.Frequency_Hz, ...
    result.case_table.SimulationDuration_s, '-s', 'LineWidth', 1.3);
xlabel(ax, '升降舵频率 / Hz'); ylabel(ax, '时间 / s');
title(ax, '各频率输入与仿真时长'); grid(ax, 'on'); axis(ax, 'tight');
legend(ax, {'正弦输入时长', '仿真总时长'}, 'Location', 'best');

title(tl, '升降舵扫频控制输入定义');
file = export_figure_(fig, output, '07_sweep_control_definition.png');
end


function file = all_section_peak_cloud_(result, component, quantity, unit, ...
        scale, filename, output, visibility)
% Frequency-span cloud for every cut in all four surface groups.
keys = ["wing_left"; "wing_right"; "tail_left"; "tail_right"];
fig = figure('Name', ['全部截面峰值 ' quantity], 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 30 1320 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for g = 1:numel(keys)
    group = result.section_groups.(char(keys(g)));
    ax = nexttile(tl);
    value = scale*squeeze(group.peak_abs(:, component, :));
    if isvector(value)
        value = reshape(value, numel(result.frequencies_hz), []);
    end
    draw_cloud_(ax, group.normalized_span, result.frequencies_hz, value);
    axis(ax, 'tight'); colormap(ax, parula(256));
    xlabel(ax, '归一化展向位置（根部0，尖部1）');
    ylabel(ax, '升降舵频率 / Hz');
    title(ax, group.label);
    cb = colorbar(ax);
    ylabel(cb, [quantity ' 相对配平峰值 / ' unit]);
end
title(tl, ['全部截面频率—展向 ' quantity ...
    ' 峰值云图（max|L(t)-L_{trim}|）']);
file = export_figure_(fig, output, filename);
end


function draw_cloud_(ax, x, y, value)
% surf requires at least a 2-by-2 Z matrix; imagesc also supports one case.
if numel(x) >= 2 && numel(y) >= 2
    surf(ax, x, y, value, 'EdgeColor', 'none');
    view(ax, 2);
else
    imagesc(ax, x, y, value);
    set(ax, 'YDir', 'normal');
end
end


function file = export_figure_(fig, output, filename)
file = string(fullfile(output.directory, filename));
drawnow;
if output.export_png
    exportgraphics(fig, file, 'Resolution', 180);
else
    file = "";
end
if strcmp(fig.Visible, 'off')
    close(fig);
end
end


function map = diverging_colormap_(n)
if nargin < 1, n = 256; end
n1 = floor(n/2);
n2 = n-n1;
blue = [0.12 0.35 0.70];
white = [1 1 1];
red = [0.75 0.16 0.18];
map = [linspace(blue(1), white(1), n1).', ...
       linspace(blue(2), white(2), n1).', ...
       linspace(blue(3), white(3), n1).'; ...
       linspace(white(1), red(1), n2).', ...
       linspace(white(2), red(2), n2).', ...
       linspace(white(3), red(3), n2).'];
end


function validate_settings_(sweep, output)
validateattributes(sweep.frequencies_hz, {'numeric'}, ...
    {'vector', 'nonempty', 'positive', 'finite'});
validateattributes(sweep.elevator_amplitude_deg, {'numeric'}, ...
    {'scalar', 'finite', 'nonzero'});
validateattributes(sweep.trim_hold_time_s, {'numeric'}, ...
    {'scalar', 'nonnegative', 'finite'});
validateattributes(sweep.ramp_cycles, {'numeric'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(sweep.total_cycles, {'numeric'}, ...
    {'scalar', 'integer', 'positive', 'finite'});
validateattributes(sweep.measure_cycles, {'numeric'}, ...
    {'scalar', 'integer', 'positive', 'finite'});
if sweep.measure_cycles > sweep.total_cycles
    error('generate_section_load_frequency_sweep:BadMeasureCycles', ...
        'measure_cycles cannot exceed total_cycles.');
end
validateattributes(sweep.recovery_cycles, {'numeric'}, ...
    {'scalar', 'nonnegative', 'finite'});
validateattributes(sweep.phase_deg, {'numeric'}, ...
    {'vector', 'nonempty', 'finite', 'increasing'});
if sweep.phase_deg(1) ~= 0 || sweep.phase_deg(end) ~= 360
    error('generate_section_load_frequency_sweep:BadPhaseGrid', ...
        'phase_deg must start at 0 and end at 360 deg.');
end
if ~(ischar(output.directory) || isstring(output.directory))
    error('generate_section_load_frequency_sweep:BadOutputDirectory', ...
        'output.directory must be text.');
end
end


function merged = merge_settings_(base, override)
merged = base;
if isempty(override), return; end
names = fieldnames(override);
for i = 1:numel(names)
    merged.(names{i}) = override.(names{i});
end
end


function value = field_or_(s, name, fallback)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = fallback;
end
end


function value = on_off_(flag)
if flag, value = 'on'; else, value = 'off'; end
end


function clear_previous_numbered_visuals_(directory)
entries = [dir(fullfile(directory, '*.png')); ...
    dir(fullfile(directory, '*.mp4'))];
for i = 1:numel(entries)
    if ~isempty(regexp(entries(i).name, '^\d{2}_', 'once'))
        delete(fullfile(directory, entries(i).name));
    end
end
end
