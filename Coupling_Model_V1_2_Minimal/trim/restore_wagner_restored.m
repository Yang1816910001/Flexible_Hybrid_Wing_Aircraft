function result = restore_wagner_restored(validation_time_s, mdl)
%RESTORE_WAGNER_RESTORED 为 RESTORED 模型生成稳态 Wagner 初值并验证。
%
% 用法：
%   mdl = 'aerodynamic_model_V1_2_Wrench';
%   柔性联合配平：先运行 trim_flexible_direct_findop(..., mdl)，
%                 再运行 restore_wagner_restored(..., mdl)；
%   刚体配平：把下方 trim_source 改为 'rigid'，再运行本函数。
%
% 成功后 mdl 指定的模型在当前 MATLAB 会话中可直接运行。
% 本函数不保存 slx 文件。

if nargin < 1
    validation_time_s = 0.3;
end
if nargin < 2 || isempty(mdl)
    mdl = 'aerodynamic_model_V1_2_Wrench';
end
mdl = char(string(mdl));

%% ========== 配平来源开关：选择要恢复哪一个配平点 ==========
trim_source = 'flexible_direct';
% 'flexible_direct'：保留联合柔性配平的 eta，调用新恢复脚本（当前推荐）
% 'rigid'          ：使用原刚体配平点，并冻结结构模态
%% ==========================================================

switch trim_source
    case 'flexible_direct'
        fprintf('Wagner restoration source: flexible_direct\n');
        result = restore_wagner_from_flexible_direct(validation_time_s, mdl);
        return
    case 'rigid'
        fprintf('Wagner restoration source: rigid\n');
    otherwise
        error('Unknown trim_source: %s', trim_source);
end

%% ========== 恢复顺序开关：先只验证 Wagner ==========
freeze_structure = true;       % true：C/D 临时置零，只保留刚体 + Wagner
enable_downwash = false;       % 第一步建议 false；Wagner 验证后再改 true
bypass_bus_delays = true;      % findop 不管理这两个总线 Unit Delay
correct_vtail_sign = true;     % RESTORED 仍是旧 V 尾左右 cant 顺序
%% ===================================================

root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
addpath(fullfile(root, 'trim'));

if ~evalin('base', 'exist(''trim_result'', ''var'')')
    fprintf('未找到 trim_result，先执行刚体配平。\n');
    rigid = test_simulink_steady_trim(1);
else
    rigid = evalin('base', 'trim_result');
end
if ~isfield(rigid, 'ok') || ~rigid.ok
    error('刚体配平结果无效，请先运行 test_simulink_steady_trim。');
end

required = {'Ad_wing_L', 'Bd_wing_L', 'Cd_wing_L', 'Dd_wing_L', ...
    'Ad_wing_R', 'Bd_wing_R', 'Ad_v_tail_L', 'Bd_v_tail_L', ...
    'Ad_v_tail_R', 'Bd_v_tail_R', 'ctrl_Mix_wing_L', 'polar_alpha_deg'};
for k = 1:numel(required)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', required{k}))
        error('工作区缺少 %s；请先运行 test_simulink_steady_trim。', required{k});
    end
end

load_system(mdl);

% 刚体状态和根输入沿用已验证的 TRIM 工作点。
assignin('base', 'trim_Velocity_b0', rigid.Velocity_b0(:));
assignin('base', 'trim_Omega_b0', rigid.Omega_b0(:));
assignin('base', 'trim_Position0', rigid.Position0(:));
assignin('base', 'trim_Attitude_rad0', rigid.Attitude_rad0(:));
assignin('base', 'trim_RPM_cmd', rigid.rpm_cmd(:));
assignin('base', 'trim_Tilt_cmd_rad', zeros(8, 1));
assignin('base', 'trim_dotTilt_cmd_radps', zeros(8, 1));
assignin('base', 'trim_Aileron_deg', 0);
assignin('base', 'trim_Elevator_deg', rigid.elevator_deg);
assignin('base', 'trim_Rudder_deg', 0);

% 四个面分别计算准定常 CL、下洗稳态值和 Wagner 稳态状态。
aero_ic = steady_aero_states_(rigid, enable_downwash);
names = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
for k = 1:numel(names)
    name = names{k};
    assignin('base', ['x0_wagner_' name], aero_ic.(name).x0_wagner);
    assignin('base', ['alpha_du0_' name], aero_ic.(name).alpha_du0);
end
assignin('base', 'restored_aero_ic', aero_ic);

run_time_s = max(validation_time_s, evalin('base', 'sim_time'));
input_data = root_input_dataset_(rigid.rpm_cmd, rigid.elevator_deg, run_time_s);
assignin('base', 'trim_input_dataset', input_data);

% RESTORED 根端口与 Dataset 一一对应；不用旧 report5 初态。
set_param(mdl, ...
    'LoadInitialState', 'off', ...
    'LoadExternalInput', 'on', ...
    'ExternalInput', 'trim_input_dataset', ...
    'StopTime', num2str(run_time_s, 16));

surfaces = {'Left_Wing', 'Right_Wing', 'Left_Tail', 'Right_Tail'};
for k = 1:numel(surfaces)
    base = [mdl '/Strip Aerodynamics/' surfaces{k}];
    set_param([base '/Wagner_SS'], 'Commented', 'off');
    if enable_downwash
        set_param([base '/From_adu'], 'Commented', 'off'); %#ok<UNRCH>
    else
        set_param([base '/From_adu'], 'Commented', 'on');
    end
end

if freeze_structure
    structure = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
    set_param(structure, 'C', 'zeros(size(Cs))', 'D', 'zeros(size(Ds))');
    assignin('base', 'x0_structure', zeros(size(evalin('base', 'As'), 1), 1));
end

if bypass_bus_delays
    set_param([mdl '/Strip Aerodynamics/Delay_Dynamics'], 'Commented', 'through');
    set_param([mdl '/Propulsion Dynamics/Delay_Dynamic'], 'Commented', 'through');
end
if correct_vtail_sign
    % RESTORED: +cant 接右尾、-cant 接左尾；用负常量后与已修正 TRIM 一致。
    set_param([mdl '/Strip Aerodynamics/cant_Tail'], ...
        'Value', '-v_tail_cant_rad');
end

% 用 RESTORED 模型端口实际仿真，检查首拍 Wagner 匹配和状态保持性。
sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setModelParameter( ...
    'StopTime', num2str(validation_time_s, 16), ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', ...
    'SaveState', 'on', 'StateSaveName', 'xout');
sim_out = sim(sim_in);

dyn = sim_out.yout.getElement('Out_Dynamics').Values;
aero_out = sim_out.yout.getElement('Out_Aero').Values;
velocity = channels_by_time_(dyn.Velocity_b.Data, 3);
omega = channels_by_time_(dyn.Omega_b.Data, 3);

cl_error = struct();
cl_error.wing_L = first_error_(aero_out.CL_strip_c_LW.Data, ...
    aero_out.CL_strip_qs_LW.Data, 20);
cl_error.wing_R = first_error_(aero_out.CL_strip_c_RW.Data, ...
    aero_out.CL_strip_qs_RW.Data, 20);
cl_error.v_tail_L = first_error_(aero_out.CL_strip_c_LT.Data, ...
    aero_out.CL_strip_qs_LT.Data, 5);
cl_error.v_tail_R = first_error_(aero_out.CL_strip_c_RT.Data, ...
    aero_out.CL_strip_qs_RT.Data, 5);

result = struct();
result.model = mdl;
result.freeze_structure = freeze_structure;
result.enable_downwash = enable_downwash;
result.bypass_bus_delays = bypass_bus_delays;
result.correct_vtail_sign = correct_vtail_sign;
result.aero_ic = aero_ic;
result.wagner_first_sample_error = cl_error;
result.max_wagner_error = max(structfun(@max, cl_error));
result.delta_velocity = velocity(:, end) - velocity(:, 1);
result.delta_omega = omega(:, end) - omega(:, 1);
result.hold_ok = max(abs(result.delta_velocity)) < 0.1 && ...
    max(abs(result.delta_omega)) < 0.01;
result.ok = result.max_wagner_error < 1e-8 && result.hold_ok;

assignin('base', 'wagner_result', result);
fprintf('\n=== RESTORED Wagner initialization ===\n');
fprintf('wing state sizes = [%d %d], tail state sizes = [%d %d]\n', ...
    numel(aero_ic.wing_L.x0_wagner), numel(aero_ic.wing_R.x0_wagner), ...
    numel(aero_ic.v_tail_L.x0_wagner), numel(aero_ic.v_tail_R.x0_wagner));
fprintf('max |CL_Wagner(0)-CL_qs(0)| = %.3e\n', result.max_wagner_error);
fprintf('Delta Velocity_b(%g s) = %s m/s\n', validation_time_s, ...
    mat2str(result.delta_velocity.', 6));
fprintf('Delta Omega_b(%g s) = %s rad/s\n', validation_time_s, ...
    mat2str(result.delta_omega.', 6));
fprintf(['已写入 x0_wagner_* / alpha_du0_* / restored_aero_ic / ' ...
    'trim_input_dataset / wagner_result。\n']);
fprintf('当前会话可直接运行 %s；不要再次运行 initial_main_V1_1。\n', mdl);
fprintf('模型只在内存中配置，未保存 slx。\n');
end


function states = steady_aero_states_(rigid, enable_downwash)
cant = evalin('base', 'v_tail_cant_rad');
[a_wr, a_wl, a_tr, a_tl] = strip_local_alphas_( ...
    rigid.Velocity_b0(:), cant);
local_alpha = struct('wing_R', a_wr, 'wing_L', a_wl, ...
    'v_tail_R', a_tr, 'v_tail_L', a_tl);

names = {'wing_L', 'wing_R', 'v_tail_L', 'v_tail_R'};
polar.alpha_deg = evalin('base', 'polar_alpha_deg');
polar.CL = evalin('base', 'polar_CL');
polar.CD = evalin('base', 'polar_CD');
polar.CM = evalin('base', 'polar_CM');
tables = control_surfaces.load_tables();
cmd = [0; rigid.elevator_deg; 0; 0];

for k = 1:numel(names)
    name = names{k};
    A = evalin('base', ['Ad_' name]);
    B = evalin('base', ['Bd_' name]);
    C = evalin('base', ['Cd_' name]);
    D = evalin('base', ['Dd_' name]);
    G = evalin('base', ['G_du_' name]);
    n = size(B, 2);
    Mix = evalin('base', ['ctrl_Mix_' name]);
    family = evalin('base', ['ctrl_family_' name]);
    delta = control_surfaces.mix(cmd, Mix);
    alpha_local = local_alpha.(name) * ones(n, 1);

    CL = control_surfaces.lookup( ...
        rad2deg(alpha_local), delta, family, tables, polar);
    if enable_downwash
        for iteration = 1:200
            CL_new = control_surfaces.lookup( ...
                rad2deg(alpha_local - G * CL), delta, family, tables, polar);
            CL_next = 0.85 * CL + 0.15 * CL_new;
            if max(abs(CL_next - CL)) < 1e-10
                CL = CL_next;
                break
            end
            CL = CL_next;
        end
        alpha_du = G * CL;
    else
        alpha_du = zeros(n, 1);
    end

    x0 = (eye(size(A)) - A) \ (B * CL);
    CL_wagner = C * x0 + D * CL;
    states.(name) = struct( ...
        'CL_qs', CL, ...
        'CL_wagner0', CL_wagner, ...
        'x0_wagner', x0, ...
        'alpha_du0', alpha_du, ...
        'state_residual', max(abs(A * x0 + B * CL - x0)), ...
        'output_error', max(abs(CL_wagner - CL)));
end
end


function [a_wr, a_wl, a_tr, a_tl] = strip_local_alphas_(V_b, cant_rad)
V_s = [-V_b(1); V_b(2); -V_b(3)];
a_wr = atan2(-V_s(3), -V_s(1));
a_wl = a_wr;
cr = cos(cant_rad);
sr = sin(cant_rad);
Vtr = [V_s(1); cr * V_s(2) + sr * V_s(3); ...
    -sr * V_s(2) + cr * V_s(3)];
Vtl = [V_s(1); cr * V_s(2) - sr * V_s(3); ...
     sr * V_s(2) + cr * V_s(3)];
a_tr = atan2(-Vtr(3), -Vtr(1));
a_tl = atan2(-Vtl(3), -Vtl(1));
end


function data = root_input_dataset_(rpm, elevator, stop_time)
t = [0; stop_time];
rpm = rpm(:);
data = Simulink.SimulationData.Dataset;
data = data.addElement(timeseries([rpm.'; rpm.'], t), 'Throttle');
data = data.addElement(timeseries(zeros(2, 8), t), 'Tilt_cmd');
data = data.addElement(timeseries(zeros(2, 8), t), 'dotTilt_cmd');
data = data.addElement(timeseries(zeros(2, 1), t), 'Aileron_deg');
data = data.addElement(timeseries(elevator * ones(2, 1), t), 'Elevator_deg');
data = data.addElement(timeseries(zeros(2, 1), t), 'Rudder_deg');
end


function error_value = first_error_(wagner_data, qs_data, n_channel)
wagner = channels_by_time_(wagner_data, n_channel);
quasi = channels_by_time_(qs_data, n_channel);
error_value = abs(wagner(:, 1) - quasi(:, 1));
end


function y = channels_by_time_(data, n_channel)
y = squeeze(data);
if isvector(y)
    y = y(:);
end
if size(y, 1) == n_channel
    return
end
if size(y, 2) == n_channel
    y = y.';
    return
end
error('无法把数据维度 %s 整理为 %d 个通道。', ...
    mat2str(size(y)), n_channel);
end
