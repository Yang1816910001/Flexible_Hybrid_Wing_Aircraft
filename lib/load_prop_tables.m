function [cfd2d, cfd3d] = load_prop_tables(tableDir)
%LOAD_PROP_TABLES  读 CFD 规则表和物理常数到 base（不建表）
%
%   load_prop_tables
%   load_prop_tables(tableDir)
%   [cfd2d, cfd3d] = load_prop_tables
%
% 要 CFD_HOVER_2D.mat、CFD_DATA_3D.mat。默认先看本 lib，再看
% ../Prop_dimentionless/results（需先在那边跑 main）。

    if nargin < 1 || isempty(tableDir)
        tableDir = local_table_dir_();
    end
    tableDir = char(tableDir);

    cfd2d = unpack_2d_(fullfile(tableDir, 'CFD_HOVER_2D.mat'));
    cfd3d = unpack_3d_(fullfile(tableDir, 'CFD_DATA_3D.mat'));
end

function d = local_table_dir_()
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    candidates = {
        here
        fullfile(root, 'results')
        fullfile(root, '..', 'Prop_dimentionless', 'results')
        };
    need = {'CFD_HOVER_2D.mat', 'CFD_DATA_3D.mat'};
    for i = 1:numel(candidates)
        d = candidates{i};
        if isfile(fullfile(d, need{1})) && isfile(fullfile(d, need{2}))
            return
        end
    end
    error('load_prop_tables:NoTable', [ ...
        '找不到 CFD_HOVER_2D.mat / CFD_DATA_3D.mat。\n' ...
        '在 Prop_dimentionless 跑 main，或把这两个 mat 放到:\n  %s\n  %s'], ...
        here, candidates{3});
end

function L = unpack_2d_(f)
    if ~isfile(f)
        error('load_prop_tables:NoHover', '没有举升表: %s', f);
    end
    S = load(f, 'cfd2d');
    L = S.cfd2d;
    assignin('base', 'cfd2d', L);
    assignin('base', 'cfd2d_ja', L.ja);
    assignin('base', 'cfd2d_jl', L.jl);
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    for k = 1:6
        assignin('base', ['cfd2d_P1_' coeff{k}], L.prop1.(coeff{k}));
        assignin('base', ['cfd2d_P5_' coeff{k}], L.prop5.(coeff{k}));
    end
    assignin('base', 'cfd2d_P1_valid', double(L.prop1.valid));
    assignin('base', 'cfd2d_P5_valid', double(L.prop5.valid));
    assign_phys_('cfd2d', L);
    fprintf('cfd2d  %s  Ja=%d  Jl=%d  D=%.3g  kT=%.4f  kQ=%.4f\n', ...
        f, numel(L.ja), numel(L.jl), L.D, L.kT, L.kQ);
end

function L = unpack_3d_(f)
    if ~isfile(f)
        error('load_prop_tables:NoTilt', '没有倾转表: %s', f);
    end
    S = load(f, 'cfd3d');
    L = S.cfd3d;
    assignin('base', 'cfd3d', L);
    assignin('base', 'cfd3d_ja', L.ja);
    assignin('base', 'cfd3d_jl', L.jl);
    assignin('base', 'cfd3d_pitch', L.pitch);
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    for k = 1:6
        assignin('base', ['cfd3d_P2_' coeff{k}], L.prop2.(coeff{k}));
        assignin('base', ['cfd3d_P6_' coeff{k}], L.prop6.(coeff{k}));
    end
    assignin('base', 'cfd3d_P2_valid', double(L.prop2.valid));
    assignin('base', 'cfd3d_P6_valid', double(L.prop6.valid));
    assign_phys_('cfd3d', L);
    fprintf('cfd3d  %s  Ja=%d  Jl=%d  pitch=%d  D=%.3g  kT=%.4f  kQ=%.4f\n', ...
        f, numel(L.ja), numel(L.jl), numel(L.pitch), L.D, L.kT, L.kQ);
end

function assign_phys_(prefix, L)
    names = {'D','rho','kT','kQ','aInf','cantDeg'};
    for i = 1:numel(names)
        n = names{i};
        if isfield(L, n)
            assignin('base', [prefix '_' n], L.(n));
        end
    end
end
