function cfg = sine_load_case_settings()
%SINE_LOAD_CASE_SETTINGS 单频正弦升降舵载荷算例的统一设置。
% 只保存工况、初始化、求解、控制输入和绘图设置，不修改模型。

root_dir = fileparts(mfilename('fullpath'));

%% 1. 固定工况
cfg.condition.U_mps           = 45.0;
cfg.condition.altitude_m      = 100.0;
cfg.condition.rho_kgpm3       = [];
cfg.condition.blade_pitch_deg = 34.0;
cfg.condition.tilt_deg        = zeros(8, 1);
cfg.condition.dot_tilt_degps  = zeros(8, 1);
cfg.condition.hover_rpm       = 0;
cfg.condition.aileron_deg     = 0;
cfg.condition.rudder_deg      = 0;

%% 2. 配平初始化
cfg.initial.alpha_deg        = 2.0;
cfg.initial.elevator_deg     = -3.0;
cfg.initial.cruise_rpm       = [];
cfg.initial.hover_rpm        = [];
cfg.initial.modal_eta        = [];
cfg.initial.modal_seed_scale = 0.25;

%% 3. 配平求解
cfg.solver.alpha_min_deg      = -8.0;
cfg.solver.alpha_max_deg      = 12.0;
cfg.solver.elevator_min_deg   = -20.0;
cfg.solver.elevator_max_deg   = 20.0;
cfg.solver.trim_hover_rpm     = false;
cfg.solver.hover_rpm_min      = 0;
cfg.solver.hover_rpm_max      = 2000;
cfg.solver.cruise_rpm_min     = [];
cfg.solver.cruise_rpm_max     = 3000;
cfg.solver.J_max              = 2.4;
cfg.solver.prop_diameter_m    = 3.0;
cfg.solver.max_iterations     = 200;
cfg.solver.constraint_tol     = 1.0e-8;
cfg.solver.optimality_tol     = 1.0e-8;
cfg.solver.trim_tolerance     = 2.0e-6;

cfg.feature.enable_downwash = false;
cfg.feature.restore_wagner  = true;
cfg.feature.bypass_delays   = true;

%% 4. 正弦升降舵输入
% delta_e = amplitude*sin(2*pi*f*(t-start))*平滑包络。
cfg.maneuver.stop_time_s            = 7.0;
cfg.maneuver.sine_start_s           = 2;
cfg.maneuver.sine_end_s             = 4;
cfg.maneuver.sine_frequency_hz      = 3;
cfg.maneuver.elevator_amplitude_deg = -2.0;
cfg.maneuver.transition_time_s      = 0.20;
cfg.maneuver.cruise_rpm_scale       = 1.0;
cfg.maneuver.input_sample_time_s    = 0.005;

%% 5. 输出与扫频
cfg.output.sample_time_s       = 0.005;
cfg.output.figure_visible      = true;
cfg.output.export_png          = true;
cfg.output.save_mat            = true;
cfg.output.plot_single_case    = true;
cfg.output.maneuver_window_s   = [1.5, 4.8];
cfg.output.span_plot_points    = 161;
cfg.output.deformation_scale   = 1.0; % 3D形变显示比例，1表示真实比例
cfg.output.export_animation    = true;
cfg.output.animation_window_s  = [0, cfg.maneuver.stop_time_s];
cfg.output.animation_sample_time_s = 0.05;
cfg.output.animation_fps       = 20;
cfg.output.show_station_lines  = true;
cfg.output.show_event_lines    = true;
cfg.output.directory = fullfile(root_dir, 'outputs', 'sine_load_cloud_45mps');

cfg.sweep.frequencies_hz = [0.50 0.75 1.00 1.25 1.50 2.00];
cfg.sweep.output_directory = fullfile(root_dir, 'outputs', ...
    'elevator_frequency_sweep_45mps');

%% 6. 合理性判据
cfg.checks.symmetry_relative_error_max = 0.10;
cfg.checks.tip_root_ratio_max = 0.10;
cfg.checks.monotonic_fraction_min = 0.75;
cfg.checks.monotonic_relative_tolerance = 0.02;
cfg.checks.aero_growth_ratio_min = 1.02;
cfg.checks.inertia_acceleration_relative_error_max = 1.0e-8;
cfg.checks.decay_ratio_max = 0.95;
cfg.checks.decay_window_s = 0.75;
cfg.checks.adjacent_jump_ratio_max = 0.35;
cfg.checks.closure_relative_error_max = 1.0e-6;
end
