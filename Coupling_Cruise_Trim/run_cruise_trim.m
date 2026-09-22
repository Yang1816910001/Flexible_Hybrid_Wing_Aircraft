%% 巡航配平入口：准定常柔性联合配平 → 恢复 Wagner
%  植物：aerodynamic_model_V1_1_Coupling.slx
%  同一 MATLAB 会话内顺序执行，中途不要再跑 initial_main_V1_1。
%
%  改工况：
%    initial_main_V1_1.m              里的 U / 桨距 / RPM 初值
%    trim/trim_flexible_direct_findop.m 顶部的 condition.*
%  两处空速必须相同。

pack_root = fileparts(mfilename('fullpath'));
cd(pack_root);
addpath(pack_root);
addpath(fullfile(pack_root, 'trim'));
addpath(fullfile(pack_root, 'lib'));
addpath(fullfile(pack_root, 'thrust'));
addpath(fullfile(pack_root, 'Library'));

fprintf('\n=== Coupling cruise trim pack ===\n');
fprintf('Root : %s\n', pack_root);
fprintf('Plant: aerodynamic_model_V1_1_Coupling\n\n');

direct = trim_flexible_direct_findop(0.3);
if ~isfield(direct, 'converged') || ~direct.converged
    if isfield(direct, 'max_residual')
        error(['trim_flexible_direct_findop 未收敛（max residual %.3e）。' ...
            '请检查空速/桨距/悬停 RPM 后再跑 restore。'], direct.max_residual);
    end
    error('trim_flexible_direct_findop 未收敛。请检查空速/桨距/悬停 RPM 后再跑 restore。');
end

wagner = restore_wagner_restored(0.3);

fprintf('\n=== Pack pipeline finished ===\n');
fprintf('Direct trim ok : %d\n', logical(direct.ok));
fprintf('Wagner ready   : %d\n', logical(wagner.ok));
fprintf('Next           : 不要再跑 initial_main_V1_1，直接仿真当前已加载的 Coupling 模型。\n');
