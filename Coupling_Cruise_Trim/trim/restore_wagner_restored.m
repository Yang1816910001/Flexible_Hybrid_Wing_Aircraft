function result = restore_wagner_restored(validation_time_s)
%RESTORE_WAGNER_RESTORED 在柔性联合配平点上恢复 Wagner 状态。
%
% 用法（同一 MATLAB 会话）：
%   trim_flexible_direct_findop
%   restore_wagner_restored
%
% 植物线：aerodynamic_model_V1_1_Coupling.slx
% 成功后模型留在内存中可直接仿真，不保存 .slx。

if nargin < 1
    validation_time_s = 0.3;
end

fprintf('Wagner restoration source: flexible_direct\n');
result = restore_wagner_from_flexible_direct(validation_time_s);
end
