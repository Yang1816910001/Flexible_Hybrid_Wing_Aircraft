function files = plot_sine_node_load_clouds(result, output)
%PLOT_SINE_NODE_LOAD_CLOUDS 单频正弦工况的形变、命令和截面载荷图。
% 不输出节点法向形变；输出三维形变、控制命令及载荷来源分解。

if nargin < 2 || isempty(output)
    output = result.config.output;
end
if ~isfolder(output.directory)
    mkdir(output.directory);
end
clear_previous_numbered_visuals_(output.directory);

t = result.state.time_s(:);
mask = t >= output.maneuver_window_s(1) & ...
    t <= output.maneuver_window_s(2);
if ~any(mask)
    error('plot_sine_node_load_clouds:EmptyWindow', ...
        '绘图时间窗口内没有采样点。');
end
t_plot = t(mask);
x_plot = linspace(0, 1, output.span_plot_points);
visibility = on_off_(output.figure_visible);
files = strings(9, 1);

fprintf('\n=== Plotting deformation, commands and section loads ===\n');
files(1) = moment_figure_(result.wing, '主翼', t_plot, mask, x_plot, ...
    result.config.maneuver, output, visibility, ...
    '01_main_wing_bending_torsion_cloud.png');
files(2) = moment_figure_(result.tail, 'V尾', t_plot, mask, x_plot, ...
    result.config.maneuver, output, visibility, ...
    '02_vtail_bending_torsion_cloud.png');
files(3) = control_command_figure_(result, output, visibility);
files(4) = load_surface_3d_figure_(result.wing, '主翼', ...
    '04_main_wing_load_surfaces_3d.png', t_plot, mask, x_plot, ...
    output, visibility);
files(5) = load_surface_3d_figure_(result.tail, 'V尾', ...
    '05_vtail_load_surfaces_3d.png', t_plot, mask, x_plot, ...
    output, visibility);
files(6) = root_source_time_figure_(result.wing, '主翼', ...
    '06_wing_root_load_sources_time_history.png', t_plot, mask, ...
    result.config.maneuver, output, visibility);
files(7) = deformation_animation_(result, output);
files(8) = root_source_time_figure_(result.tail, 'V尾', ...
    '08_vtail_root_load_sources_time_history.png', t_plot, mask, ...
    result.config.maneuver, output, visibility);
files(9) = rigid_body_state_figure_(result, output, visibility);
end


function file = deformation_3d_figure_(result, mask, output, visibility)
% 峰值时刻按节点三维形变合量选取，不输出节点法向形变量。
sides = {'left', 'right'};
indices = find(mask);
peak_metric = zeros(numel(indices), 1);

for i = 1:2
    side = result.wing.(sides{i});
    ids = side.node_id(:).';
    dr_dynamic = result.nodes.position_b_m(:, ids, :)- ...
        result.nodes.position_b_m(:, ids, 1);
    side_peak = max(squeeze(vecnorm(dr_dynamic(:, :, mask), 2, 1)), ...
        [], 1, 'omitnan').';
    peak_metric = max(peak_metric, side_peak);
end
[~, local_peak] = max(peak_metric);
k = indices(local_peak);
peak_time_s = result.state.time_s(k);

fig = figure('Name', '主翼三维变形构型', 'Visible', visibility, ...
    'Color', 'w', 'Position', [80 60 1120 760]);
ax = axes(fig); hold(ax, 'on');
legend_handles = gobjects(3, 1);
shown_all = zeros(3, 0);
for i = 1:2
    side = result.wing.(sides{i});
    ids = side.node_id(:).';
    base = result.nodes.undeformed_position_b_m(:, ids);
    trimmed = result.nodes.position_b_m(:, ids, 1);
    current = result.nodes.position_b_m(:, ids, k);
    shown = base+output.deformation_scale*(current-base);
    shown_all = [shown_all, base, trimmed, shown]; %#ok<AGROW>

    % 每条细线连接配平位置与当前显示位置，直观表示节点形变矢量。
    plot3(ax, [trimmed(1, :); shown(1, :)], ...
        [trimmed(2, :); shown(2, :)], [trimmed(3, :); shown(3, :)], ...
        ':', 'Color', [0.70 0.70 0.70], 'LineWidth', 0.7, ...
        'HandleVisibility', 'off');

    if i == 1
        legend_handles(1) = plot3(ax, base(1, :), base(2, :), base(3, :), ...
            '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.3);
        legend_handles(2) = plot3(ax, trimmed(1, :), trimmed(2, :), ...
            trimmed(3, :), '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.2);
        legend_handles(3) = plot3(ax, shown(1, :), shown(2, :), shown(3, :), ...
            '-', 'Color', [0.05 0.28 0.55], 'LineWidth', 1.8);
    else
        plot3(ax, base(1, :), base(2, :), base(3, :), '--', ...
            'Color', [0.55 0.55 0.55], 'LineWidth', 1.3, ...
            'HandleVisibility', 'off');
        plot3(ax, trimmed(1, :), trimmed(2, :), trimmed(3, :), '-', ...
            'Color', [0.15 0.15 0.15], 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
        plot3(ax, shown(1, :), shown(2, :), shown(3, :), '-', ...
            'Color', [0.05 0.28 0.55], 'LineWidth', 1.8, ...
            'HandleVisibility', 'off');
    end
    scatter3(ax, shown(1, :), shown(2, :), shown(3, :), 52, ...
        [0.13 0.42 0.72], 'filled', 'MarkerEdgeColor', [0.15 0.15 0.15], ...
        'HandleVisibility', 'off');
end

% 梁节点在未变形时几乎是一条展向直线，不能使用axis equal，否则
% MATLAB会为了零弦向宽度制造大量空白。这里固定显示框比例，数值坐标不变。
span_range = max(shown_all(2, :))-min(shown_all(2, :));
x_center = 0.5*(max(shown_all(1, :))+min(shown_all(1, :)));
x_half = max(0.06*span_range, ...
    0.65*(max(shown_all(1, :))-min(shown_all(1, :))));
z_center = 0.5*(max(shown_all(3, :))+min(shown_all(3, :)));
z_half = max(0.04*span_range, ...
    0.65*(max(shown_all(3, :))-min(shown_all(3, :))));
xlim(ax, x_center+[-x_half x_half]);
ylim(ax, [min(shown_all(2, :))-0.03*span_range, ...
    max(shown_all(2, :))+0.03*span_range]);
zlim(ax, z_center+[-z_half z_half]);
pbaspect(ax, [0.65 2.4 0.95]);
grid(ax, 'on'); box(ax, 'on'); view(ax, 38, 22);
xlabel(ax, '机体系 x_b / m'); ylabel(ax, '机体系 y_b / m');
zlabel(ax, '机体系 z_b / m');
legend(ax, legend_handles, {'未变形构型', '配平初始构型', '峰值时刻构型'}, ...
    'Location', 'best');
title(ax, sprintf(['主翼三维变形构型，t = %.3f s，显示比例 = %.3g 倍\n' ...
    '峰值按节点三维动态形变合量选取'], ...
    peak_time_s, output.deformation_scale));
file = export_figure_(fig, output, '01_main_wing_peak_deformed_shape.png');
end


function file = moment_figure_(surface, surface_name, t, mask, x_plot, ...
        maneuver, output, visibility, filename)
fig = figure('Name', surface_name+"弯矩与扭矩", 'Visible', visibility, ...
    'Color', 'w', 'Position', [50 50 1250 880]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sides = {'left', 'right'};
side_titles = {'左侧', '右侧'};
my = cell(2, 1);
mx = cell(2, 1);
for i = 1:2
    my{i} = 1e-3*squeeze(surface.(sides{i}).wrench.total(5, :, mask)).';
    mx{i} = 1e-3*squeeze(surface.(sides{i}).wrench.total(4, :, mask)).';
end
my_limit = shared_abs_limit_(my);
mx_limit = shared_abs_limit_(mx);

for i = 1:2
    side = surface.(sides{i});
    ax = nexttile(tl, i);
    draw_cloud_(ax, side.normalized_span, my{i}, x_plot, t, ...
        [-my_limit my_limit], diverging_colormap_(256));
    title(ax, side_titles{i}+"：当地y向弯矩 My");
    xlabel(ax, '时间 / s'); ylabel(ax, '归一化展向位置');
    colorbar_label_(ax, 'kN m');
    add_event_lines_(ax, maneuver, output);

    ax = nexttile(tl, i+2);
    draw_cloud_(ax, side.normalized_span, mx{i}, x_plot, t, ...
        [-mx_limit mx_limit], diverging_colormap_(256));
    title(ax, side_titles{i}+"：当地x向扭矩 Mx");
    xlabel(ax, '时间 / s'); ylabel(ax, '归一化展向位置');
    colorbar_label_(ax, 'kN m');
    add_event_lines_(ax, maneuver, output);
end
title(tl, surface_name+"截面载荷云图");
file = export_figure_(fig, output, filename);
end


function file = control_command_figure_(result, output, visibility)
input = result.input;
t = input.time_s(:);
fig = figure('Name', '控制命令', 'Visible', visibility, ...
    'Color', 'w', 'Position', [60 50 1250 850]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, input.elevator_deg(:), 'LineWidth', 1.6);
plot(ax, t, input.aileron_deg(:), 'LineWidth', 1.2);
plot(ax, t, input.rudder_deg(:), 'LineWidth', 1.2);
xlabel(ax, '时间 / s'); ylabel(ax, '舵面命令 / deg');
legend(ax, {'升降舵', '副翼', '方向舵'}, 'Location', 'best');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, input.rpm.', 'LineWidth', 1.1);
xlabel(ax, '时间 / s'); ylabel(ax, '发动机命令 / rpm');
legend(ax, compose('E%d', 1:size(input.rpm, 1)), ...
    'Location', 'eastoutside', 'NumColumns', 2);
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, rad2deg(input.tilt_rad.'), 'LineWidth', 1.1);
xlabel(ax, '时间 / s'); ylabel(ax, '倾转角命令 / deg');
legend(ax, compose('E%d', 1:size(input.tilt_rad, 1)), ...
    'Location', 'eastoutside', 'NumColumns', 2);
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, input.blade_pitch_deg*ones(size(t)), 'LineWidth', 1.5);
xlabel(ax, '时间 / s'); ylabel(ax, '桨距角命令 / deg');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

title(tl, '控制命令时间历程');
file = export_figure_(fig, output, '03_control_commands.png');
end


function file = rigid_body_state_figure_(result, output, visibility)
% 四个刚体积分器的完整状态：速度、角速度、位置和欧拉姿态。
state = result.state;
t = state.time_s(:);
fig = figure('Name', '刚体积分状态', 'Visible', visibility, ...
    'Color', 'w', 'Position', [40 35 1320 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, state.velocity_b_mps.', 'LineWidth', 1.2);
xlabel(ax, '时间 / s'); ylabel(ax, '机体系速度 / m s^{-1}');
title(ax, '速度积分器：u, v, w');
legend(ax, {'u', 'v', 'w'}, 'Location', 'best');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, rad2deg(state.omega_b_radps).', 'LineWidth', 1.2);
xlabel(ax, '时间 / s'); ylabel(ax, '机体系角速度 / deg s^{-1}');
title(ax, '角速度积分器：p, q, r');
legend(ax, {'p', 'q', 'r'}, 'Location', 'best');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, state.position_m.', 'LineWidth', 1.2);
xlabel(ax, '时间 / s'); ylabel(ax, '位置 / m');
title(ax, '位置积分器：x, y, z');
legend(ax, {'x', 'y', 'z'}, 'Location', 'best');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

ax = nexttile(tl); hold(ax, 'on');
plot(ax, t, state.attitude_deg.', 'LineWidth', 1.2);
xlabel(ax, '时间 / s'); ylabel(ax, '欧拉角 / deg');
title(ax, '姿态积分器：\phi, \theta, \psi');
legend(ax, {'\phi', '\theta', '\psi'}, 'Location', 'best');
grid(ax, 'on'); add_event_xlines_(ax, result.config.maneuver, output);

title(tl, '刚体运动积分状态时间历程');
file = export_figure_(fig, output, '09_rigid_body_integrator_states.png');
end


function file = load_surface_3d_figure_(surface, surface_name, filename, ...
        t, mask, x_plot, output, visibility)
surface_name = string(surface_name);
fig = figure('Name', surface_name+"截面载荷三维曲面", 'Visible', visibility, ...
    'Color', 'w', 'Position', [30 30 1400 950]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sides = {'left', 'right'};
side_titles = {'左主翼', '右主翼'};
components = [5, 4];
quantity = {'My弯矩', 'Mx扭矩'};

for row = 1:2
    values = cell(2, 1);
    for column = 1:2
        values{column} = 1e-3*squeeze(surface.(sides{column}).wrench.total( ...
            components(row), :, mask)).';
    end
    limit = shared_abs_limit_(values);
    for column = 1:2
        side = surface.(sides{column});
        value_plot = interp1(side.normalized_span(:), values{column}.', ...
            x_plot(:), 'linear').';
        ax = nexttile(tl, (row-1)*2+column);
        surf(ax, t, x_plot, value_plot.', value_plot.', 'EdgeColor', 'none');
        axis(ax, 'tight'); view(ax, 38, 28); grid(ax, 'on'); box(ax, 'on');
        clim(ax, [-limit limit]); colormap(ax, diverging_colormap_(256));
        xlabel(ax, '时间 / s'); ylabel(ax, '归一化展向位置');
        zlabel(ax, string(quantity{row})+" / kN m");
        title(ax, string(side_titles{column})+"："+string(quantity{row}));
        colorbar_label_(ax, 'kN m');
    end
end
title(tl, surface_name+"总截面载荷三维曲面（时间为横轴）");
file = export_figure_(fig, output, filename);
end


function file = root_source_time_figure_(surface, surface_name, filename, ...
        t, mask, maneuver, output, visibility)
surface_name = string(surface_name);
fig = figure('Name', surface_name+"根部载荷来源", 'Visible', visibility, ...
    'Color', 'w', 'Position', [50 40 1320 900]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sides = {'left', 'right'};
side_titles = {"左"+surface_name+"根", "右"+surface_name+"根"};
source_names = {'aero', 'inertia', 'gravity', 'prop', 'total'};
source_titles = {'气动', '惯性', '重力', '推进', '总载荷'};
colors = [0.00 0.45 0.74; 0.49 0.18 0.56; 0.47 0.67 0.19; ...
    0.93 0.50 0.13; 0.10 0.10 0.10];
components = [5, 4];
quantity = {'My弯矩', 'Mx扭矩'};

for row = 1:2
    for column = 1:2
        ax = nexttile(tl, (row-1)*2+column); hold(ax, 'on');
        for source_index = 1:numel(source_names)
            side_wrench = surface.(sides{column}).wrench;
            source_wrench = side_wrench.(source_names{source_index});
            value = 1e-3*squeeze(source_wrench(components(row), 1, mask));
            width = 1.25;
            if strcmp(source_names{source_index}, 'total'), width = 2.2; end
            plot(ax, t, value(:), 'Color', colors(source_index, :), ...
                'LineWidth', width);
        end
        yline(ax, 0, ':', 'Color', [0.45 0.45 0.45], ...
            'HandleVisibility', 'off');
        add_event_lines_(ax, maneuver, output);
        xlabel(ax, '时间 / s'); ylabel(ax, string(quantity{row})+" / kN m");
        title(ax, string(side_titles{column})+"："+string(quantity{row}));
        grid(ax, 'on'); axis(ax, 'tight');
        if row == 1 && column == 1
            legend(ax, source_titles, 'Location', 'best');
        end
    end
end
title(tl, surface_name+"根部各载荷来源时间历程（当地坐标系，时间为横轴）");
file = export_figure_(fig, output, filename);
end


function file = deformation_animation_(result, output)
file = string(fullfile(output.directory, ...
    '07_main_wing_deformation_animation.mp4'));
if ~output.export_animation
    file = "";
    return
end

t = result.state.time_s(:);
frame_time = (output.animation_window_s(1): ...
    output.animation_sample_time_s:output.animation_window_s(2)).';
if frame_time(end) < output.animation_window_s(2)
    frame_time(end+1, 1) = output.animation_window_s(2);
end
frame_index = round(interp1(t, (1:numel(t)).', frame_time, ...
    'nearest', 'extrap'));
frame_index = unique(frame_index, 'stable');
frame_index = frame_index(frame_index >= 1 & frame_index <= numel(t));

sides = {'left', 'right'};
base = cell(2, 1);
trimmed = cell(2, 1);
shown_series = cell(2, 1);
all_min = [inf; inf; inf];
all_max = [-inf; -inf; -inf];
for i = 1:2
    ids = result.wing.(sides{i}).node_id(:).';
    base{i} = result.nodes.undeformed_position_b_m(:, ids);
    trimmed{i} = result.nodes.position_b_m(:, ids, 1);
    current = result.nodes.position_b_m(:, ids, frame_index);
    shown_series{i} = base{i}+output.deformation_scale*(current-base{i});
    points = reshape(shown_series{i}, 3, []);
    all_min = min(all_min, min([base{i}, trimmed{i}, points], [], 2));
    all_max = max(all_max, max([base{i}, trimmed{i}, points], [], 2));
end

fig = figure('Name', '主翼形变动画', 'Visible', 'on', ...
    'Color', 'w', 'Position', [80 80 960 600]);
ax = axes(fig); hold(ax, 'on');
current_line = gobjects(2, 1);
connector_line = gobjects(2, 1);
legend_handles = gobjects(3, 1);
for i = 1:2
    shown = shown_series{i}(:, :, 1);
    [cx, cy, cz] = connector_coordinates_(trimmed{i}, shown);
    connector_line(i) = plot3(ax, cx, cy, cz, ':', ...
        'Color', [0.68 0.68 0.68], 'LineWidth', 0.7, ...
        'HandleVisibility', 'off');
    if i == 1
        legend_handles(1) = plot3(ax, base{i}(1, :), base{i}(2, :), ...
            base{i}(3, :), '--', 'Color', [0.55 0.55 0.55], ...
            'LineWidth', 1.2);
        legend_handles(2) = plot3(ax, trimmed{i}(1, :), trimmed{i}(2, :), ...
            trimmed{i}(3, :), '-', 'Color', [0.12 0.12 0.12], ...
            'LineWidth', 1.2);
        current_line(i) = plot3(ax, shown(1, :), shown(2, :), shown(3, :), ...
            '-o', 'Color', [0.05 0.32 0.65], 'MarkerFaceColor', ...
            [0.22 0.55 0.85], 'LineWidth', 1.8, 'MarkerSize', 5);
        legend_handles(3) = current_line(i);
    else
        plot3(ax, base{i}(1, :), base{i}(2, :), base{i}(3, :), '--', ...
            'Color', [0.55 0.55 0.55], 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
        plot3(ax, trimmed{i}(1, :), trimmed{i}(2, :), trimmed{i}(3, :), '-', ...
            'Color', [0.12 0.12 0.12], 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
        current_line(i) = plot3(ax, shown(1, :), shown(2, :), shown(3, :), ...
            '-o', 'Color', [0.05 0.32 0.65], 'MarkerFaceColor', ...
            [0.22 0.55 0.85], 'LineWidth', 1.8, 'MarkerSize', 5, ...
            'HandleVisibility', 'off');
    end
end

span_range = max(all_max(2)-all_min(2), eps);
x_center = 0.5*(all_max(1)+all_min(1));
z_center = 0.5*(all_max(3)+all_min(3));
x_half = max(0.06*span_range, 0.60*(all_max(1)-all_min(1)));
z_half = max(0.04*span_range, 0.60*(all_max(3)-all_min(3)));
xlim(ax, x_center+[-x_half x_half]);
ylim(ax, [all_min(2)-0.03*span_range, all_max(2)+0.03*span_range]);
zlim(ax, z_center+[-z_half z_half]);
pbaspect(ax, [0.65 2.4 0.95]); view(ax, 38, 22);
grid(ax, 'on'); box(ax, 'on');
xlabel(ax, '机体系 x_b / m'); ylabel(ax, '机体系 y_b / m');
zlabel(ax, '机体系 z_b / m');
legend(ax, legend_handles, {'未变形构型', '配平初始构型', '当前构型'}, ...
    'Location', 'best');

writer = VideoWriter(char(file), 'MPEG-4');
writer.FrameRate = output.animation_fps;
open(writer);
fprintf('  [animation] %s (%d frames)\n', file, numel(frame_index));
video_frame_size = [];
try
    report_step = max(1, round(numel(frame_index)/10));
    for frame_number = 1:numel(frame_index)
        for i = 1:2
            shown = shown_series{i}(:, :, frame_number);
            set(current_line(i), 'XData', shown(1, :), ...
                'YData', shown(2, :), 'ZData', shown(3, :));
            [cx, cy, cz] = connector_coordinates_(trimmed{i}, shown);
            set(connector_line(i), 'XData', cx, 'YData', cy, 'ZData', cz);
        end
        title(ax, sprintf('主翼三维形变，t = %.3f s，显示比例 = %.3g 倍', ...
            t(frame_index(frame_number)), output.deformation_scale));
        drawnow;
        rgb = print(fig, '-RGBImage', '-r100');
        [rgb, video_frame_size] = fixed_video_frame_( ...
            rgb, video_frame_size);
        writeVideo(writer, rgb);
        if mod(frame_number, report_step) == 0 || ...
                frame_number == numel(frame_index)
            fprintf('    frame %d/%d\n', frame_number, numel(frame_index));
        end
    end
    close(writer);
catch error_info
    close(writer);
    if isfile(file)
        delete(file);
    end
    if isgraphics(fig), close(fig); end
    rethrow(error_info);
end
if ~output.figure_visible && isgraphics(fig), close(fig); end
end


function [frame, target_size] = fixed_video_frame_(frame, target_size)
% VideoWriter requires every frame to have exactly the same dimensions.
% print(...,'-RGBImage') can vary by one or two pixels as text extents are
% updated.  Fix the first frame to an even H.264 size, then centre-pad or
% centre-crop every subsequent frame to that same size.
if isempty(target_size)
    target_size = 2*ceil([size(frame, 1), size(frame, 2)]/2);
end

fixed = uint8(255*ones(target_size(1), target_size(2), 3));
copy_h = min(size(frame, 1), target_size(1));
copy_w = min(size(frame, 2), target_size(2));
source_row = floor((size(frame, 1)-copy_h)/2)+(1:copy_h);
source_col = floor((size(frame, 2)-copy_w)/2)+(1:copy_w);
target_row = floor((target_size(1)-copy_h)/2)+(1:copy_h);
target_col = floor((target_size(2)-copy_w)/2)+(1:copy_w);
fixed(target_row, target_col, :) = frame(source_row, source_col, :);
frame = fixed;
end


function [x, y, z] = connector_coordinates_(start_position, end_position)
n = size(start_position, 2);
x = reshape([start_position(1, :); end_position(1, :); nan(1, n)], 1, []);
y = reshape([start_position(2, :); end_position(2, :); nan(1, n)], 1, []);
z = reshape([start_position(3, :); end_position(3, :); nan(1, n)], 1, []);
end


function draw_cloud_(ax, span, value, x_plot, t, limits, map)
value_plot = interp1(span(:), value.', x_plot(:), 'linear').';
surf(ax, t, x_plot, value_plot.', 'EdgeColor', 'none');
view(ax, 2); axis(ax, 'tight'); clim(ax, limits);
colormap(ax, map); hold(ax, 'on');
end


function add_event_lines_(ax, maneuver, output)
if ~output.show_event_lines
    return
end
xline(ax, maneuver.sine_start_s, '--', '正弦开始', ...
    'Color', [0.15 0.15 0.15], 'HandleVisibility', 'off');
xline(ax, maneuver.sine_end_s, '--', '正弦结束', ...
    'Color', [0.15 0.15 0.15], 'HandleVisibility', 'off');
end


function add_event_xlines_(ax, maneuver, output)
if ~output.show_event_lines
    return
end
xline(ax, maneuver.sine_start_s, '--', '正弦开始', ...
    'Color', [0.15 0.15 0.15], 'HandleVisibility', 'off');
xline(ax, maneuver.sine_end_s, '--', '正弦结束', ...
    'Color', [0.15 0.15 0.15], 'HandleVisibility', 'off');
end


function colorbar_label_(ax, text_value)
cb = colorbar(ax);
ylabel(cb, text_value);
end


function value = shared_abs_limit_(cells)
value = 0;
for i = 1:numel(cells)
    value = max(value, max(abs(cells{i}(:)), [], 'omitnan'));
end
if ~isfinite(value) || value <= 0, value = 1; end
end


function map = diverging_colormap_(n)
map = interp1([0 0.5 1], ...
    [0.10 0.30 0.62; 0.98 0.98 0.97; 0.88 0.35 0.08], ...
    linspace(0, 1, n), 'linear');
end


function file = export_figure_(fig, output, filename)
file = string(fullfile(output.directory, filename));
if strcmp(fig.Visible, 'on'), figure(fig); end
drawnow;
if output.export_png
    % 对多子图和三维曲面，print比exportgraphics占用更稳定。
    print(fig, char(file), '-dpng', '-r150');
    fprintf('  [figure] %s\n', file);
else
    file = "";
end
if strcmp(fig.Visible, 'off'), close(fig); end
end


function value = on_off_(flag)
if flag, value = 'on'; else, value = 'off'; end
end


function clear_previous_numbered_visuals_(directory)
% 仅清理本流程按“序号_名称”生成的图片和动画，保留MAT、CSV等结果。
entries = [dir(fullfile(directory, '*.png')); ...
    dir(fullfile(directory, '*.mp4'))];
removed = 0;
for i = 1:numel(entries)
    if ~isempty(regexp(entries(i).name, '^\d{2}_', 'once'))
        delete(fullfile(directory, entries(i).name));
        removed = removed+1;
    end
end
if removed > 0
    fprintf('Removed %d previous visualization files from: %s\n', ...
        removed, directory);
end
end
