classdef control_surfaces
%CONTROL_SURFACES  片条舵面掩膜、da/de/dr/df 混合、带舵 polar 查表
%
%  当地 δ>0 = 后缘朝该条 +n（机翼后缘向下；V 尾朝本侧面法向）。
%  Mix 列序 [da, de, dr, df]，度：
%    da: 两片副翼 右 −、左 +  → 正 da 右翼下（Q_roll<0）
%    de: 尾翼两片同向 + → 低头（Q_pitch<0）
%    dr: 右尾 +、左 − → Q_yaw<0
%    df: 左右襟翼同向 +（后缘下）
%
%  站位（内台 y=±3.5≈WR07，外台 y=±7.625≈WR16）：
%    襟翼     翼根→内台  CBR/CBL + WR/WL01–07
%    副翼内   内台→中缝  WR/WL08–12
%    副翼外   中缝→外台  WR/WL13–16
%    翼尖     WR/WL17–19  无舵
%    尾内     VTR/VTL01–02
%    尾外     VTR/VTL03–05
%  机翼舵弦 25%（aileron 表），V 尾 30%（ruddervator 表）。

    methods (Static)
        function s = attach(s)
            names = string(s.node_name(:));
            n = numel(names);
            ctrl = repmat("none", n, 1);
            fam = repmat("clean", n, 1);

            flap = control_surfaces.match_(names, "WR", 1:7) ...
                | control_surfaces.match_(names, "WL", 1:7) ...
                | (names == "CBR01") | (names == "CBL01");
            ail_in  = control_surfaces.match_(names, "WR", 8:12) ...
                | control_surfaces.match_(names, "WL", 8:12);
            ail_out = control_surfaces.match_(names, "WR", 13:16) ...
                | control_surfaces.match_(names, "WL", 13:16);
            tail_in  = control_surfaces.match_(names, "VTR", 1:2) ...
                | control_surfaces.match_(names, "VTL", 1:2);
            tail_out = control_surfaces.match_(names, "VTR", 3:5) ...
                | control_surfaces.match_(names, "VTL", 3:5);
            ail = ail_in | ail_out;
            rud = tail_in | tail_out;

            ctrl(flap) = "flap";
            ctrl(ail_in) = "aileron_in";
            ctrl(ail_out) = "aileron_out";
            ctrl(tail_in) = "tail_in";
            ctrl(tail_out) = "tail_out";
            fam(flap | ail) = "aileron";
            fam(rud) = "ruddervator";

            Mix = zeros(n, 4);
            Mix(ail & startsWith(names, "WR"), 1) = -1;
            Mix(ail & startsWith(names, "WL"), 1) = 1;
            Mix(rud, 2) = 1;
            Mix(rud & startsWith(names, "VTR"), 3) = 1;
            Mix(rud & startsWith(names, "VTL"), 3) = -1;
            Mix(flap, 4) = 1;

            s.ctrl = ctrl;
            s.flap_family = fam;
            s.Mix = Mix;
            s.map_aileron = double(ail);
            s.map_tail = double(rud);
            s.ctrl_xh_c = nan(n, 1);
            s.ctrl_xh_c(flap | ail) = 0.75;
            s.ctrl_xh_c(rud) = 0.70;
            s.delta_lim_deg = 20;
            s.ctrl_note = [ ...
                'flap CBR/CBL+WR/WL01-07; aileron_in 08-12; aileron_out 13-16; ' ...
                'tip 17-19 none; tail_in VT*01-02; tail_out 03-05. ' ...
                'delta_strip = Mix*[da;de;dr;df] deg. +delta = TE toward +n.'];
        end

        function d = mix(varargin)
        %MIX  δ = Mix * [da;de;dr;df]
        %  mix(cmd, Mix) 或 mix(da,de,dr,Mix) / mix(da,de,dr,df,Mix)
            if nargin == 2
                cmd = varargin{1}(:);
                Mix = varargin{2};
            elseif nargin == 4
                cmd = [varargin{1}; varargin{2}; varargin{3}];
                Mix = varargin{4};
            elseif nargin == 5
                cmd = [varargin{1}; varargin{2}; varargin{3}; varargin{4}];
                Mix = varargin{5};
            else
                error('control_surfaces:Mix', 'mix(cmd,Mix) or mix(da,de,dr[,df],Mix)');
            end
            Mix = double(Mix);
            ncol = size(Mix, 2);
            if numel(cmd) < ncol
                cmd(end+1:ncol, 1) = 0;
            else
                cmd = cmd(1:ncol);
            end
            d = Mix * cmd;
            lim = 20;
            d = max(min(d, lim), -lim);
        end

        function T = load_tables(fname)
            if nargin < 1 || isempty(fname)
                fname = fullfile(control_surfaces.root_(), 'polars', 'flap_polars.mat');
            end
            T = struct('ok', false, 'file', fname, 'kappa', 1);
            if ~isfile(fname)
                return
            end
            S = load(fname);
            T.ok = true;
            T.aileron = S.aileron;
            T.ruddervator = S.ruddervator;
            if isfield(S, 'kappa')
                T.kappa = S.kappa;
            end
            T.clean = struct();
            if isfield(S, 'clean')
                T.clean = S.clean;
            end
        end

        function [CL, CD, CM] = lookup(alpha_deg, delta_deg, family, tables, clean)
            alpha_deg = alpha_deg(:);
            n = numel(alpha_deg);
            if isscalar(delta_deg)
                delta_deg = delta_deg * ones(n, 1);
            else
                delta_deg = delta_deg(:);
            end
            if n ~= numel(delta_deg)
                error('control_surfaces:Size', 'alpha/delta length mismatch');
            end
            family = string(family);
            if isscalar(family)
                family = repmat(family, n, 1);
            end
            CL = zeros(n, 1);
            CD = zeros(n, 1);
            CM = zeros(n, 1);
            if nargin < 5 || isempty(clean)
                clean = control_surfaces.clean_from_ws_(tables);
            end
            have = isstruct(tables) && isfield(tables, 'ok') && tables.ok;
            kap = 1;
            if have && isfield(tables, 'kappa')
                kap = tables.kappa;
            end
            for i = 1:n
                [cl0, cd0, cm0] = control_surfaces.interp_clean_(clean, alpha_deg(i));
                fam = lower(strtrim(family(i)));
                if ~have || fam == "clean" || fam == "none" || abs(delta_deg(i)) < 1e-12
                    CL(i) = cl0; CD(i) = cd0; CM(i) = cm0;
                    continue
                end
                if fam == "aileron" || fam == "flap" || startsWith(fam, "aileron")
                    tab = tables.aileron;
                elseif fam == "ruddervator" || startsWith(fam, "tail")
                    tab = tables.ruddervator;
                else
                    CL(i) = cl0; CD(i) = cd0; CM(i) = cm0;
                    continue
                end
                [dcl, dcd, dcm] = control_surfaces.delta_coeff_(tab, alpha_deg(i), delta_deg(i));
                CL(i) = cl0 + kap * dcl;
                CD(i) = cd0 + dcd;
                CM(i) = cm0 + kap * dcm;
            end
        end

        function export_workspace(s, tables)
            ws = 'base';
            Mix = s.Mix;
            assignin(ws, 'ctrl_Mix', Mix);
            assignin(ws, 'ctrl_da_deg', 0);
            assignin(ws, 'ctrl_de_deg', 0);
            assignin(ws, 'ctrl_dr_deg', 0);
            assignin(ws, 'ctrl_df_deg', 0);
            assignin(ws, 'ctrl_delta_lim_deg', s.delta_lim_deg);
            assignin(ws, 'ctrl_family', s.flap_family);
            assignin(ws, 'ctrl_name', s.ctrl);
            names = s.surface_names;
            aero = s.by_surface;
            for k = 1:numel(names)
                nm = names(k);
                assignin(ws, ['ctrl_Mix_' char(nm)], aero.(nm).Mix);
                assignin(ws, ['ctrl_family_' char(nm)], aero.(nm).flap_family);
            end
            % 指令 → 对应升力面片条：有舵=1，无舵=0（列向量，δ = map * u）
            assignin(ws, 'ctrl_map_aileron_R', aero.wing_R.map_aileron(:));
            assignin(ws, 'ctrl_map_aileron_L', aero.wing_L.map_aileron(:));
            assignin(ws, 'ctrl_map_tail_R', aero.v_tail_R.map_tail(:));
            assignin(ws, 'ctrl_map_tail_L', aero.v_tail_L.map_tail(:));
            kap = 1;
            if nargin >= 2 && isstruct(tables) && isfield(tables, 'ok') && tables.ok
                assignin(ws, 'polar_flap', tables);
                control_surfaces.export_lut_(ws, tables.aileron, 'polar_aileron');
                control_surfaces.export_lut_(ws, tables.ruddervator, 'polar_ruddervator');
                if isfield(tables, 'kappa')
                    kap = tables.kappa;
                end
            end
            assignin(ws, 'ctrl_flap_kappa', kap);
        end

        function export_polar_luts(tables)
        %EXPORT_POLAR_LUTS  只写 polar_aileron_* / polar_ruddervator_*（无 Mix / 指令）
            if nargin < 1 || ~isstruct(tables) || ~isfield(tables, 'ok') || ~tables.ok
                warning('control_surfaces:NoFlapLut', ...
                    'flap polars missing; V1_1 dCL LUT will not resolve');
                return
            end
            ws = 'base';
            control_surfaces.export_lut_(ws, tables.aileron, 'polar_aileron');
            control_surfaces.export_lut_(ws, tables.ruddervator, 'polar_ruddervator');
        end
    end

    methods (Static, Access = private)
        function m = match_(names, prefix, ids)
            m = false(size(names));
            for k = ids
                m = m | (names == (prefix + sprintf('%02d', k)));
            end
        end

        function r = root_()
            r = fileparts(fileparts(mfilename('fullpath')));
        end

        function clean = clean_from_ws_(tables)
            clean = struct();
            if isstruct(tables) && isfield(tables, 'clean') && isfield(tables.clean, 'alpha_deg')
                clean = tables.clean;
                return
            end
            if evalin('base', 'exist(''polar_alpha_deg'',''var'')')
                clean.alpha_deg = evalin('base', 'polar_alpha_deg');
                clean.CL = evalin('base', 'polar_CL');
                clean.CD = evalin('base', 'polar_CD');
                clean.CM = evalin('base', 'polar_CM');
            end
        end

        function [CL, CD, CM] = interp_clean_(clean, a)
            if ~isfield(clean, 'alpha_deg') || isempty(clean.alpha_deg)
                CL = NaN; CD = NaN; CM = NaN;
                return
            end
            CL = interp1(clean.alpha_deg(:), clean.CL(:), a, 'linear', 'extrap');
            CD = interp1(clean.alpha_deg(:), clean.CD(:), a, 'linear', 'extrap');
            CM = interp1(clean.alpha_deg(:), clean.CM(:), a, 'linear', 'extrap');
        end

        function [dcl, dcd, dcm] = delta_coeff_(tab, a, d)
            aa = tab.alpha_deg(:);
            dd = tab.delta_deg(:);
            a = min(max(a, aa(1)), aa(end));
            d = min(max(d, dd(1)), dd(end));
            cl = interp2(dd, aa, tab.CL, d, a, 'linear');
            cd = interp2(dd, aa, tab.CD, d, a, 'linear');
            cm = interp2(dd, aa, tab.CM, d, a, 'linear');
            cl0 = interp2(dd, aa, tab.CL, 0, a, 'linear');
            cd0 = interp2(dd, aa, tab.CD, 0, a, 'linear');
            cm0 = interp2(dd, aa, tab.CM, 0, a, 'linear');
            if ~isfinite(cl) || ~isfinite(cl0)
                dcl = 0; dcd = 0; dcm = 0;
                return
            end
            dcl = cl - cl0;
            dcd = cd - cd0;
            dcm = cm - cm0;
            if ~isfinite(dcd), dcd = 0; end
            if ~isfinite(dcm), dcm = 0; end
        end

        function export_lut_(ws, tab, prefix)
            assignin(ws, [prefix '_alpha_deg'], tab.alpha_deg(:)');
            assignin(ws, [prefix '_delta_deg'], tab.delta_deg(:)');
            assignin(ws, [prefix '_CL'], tab.CL);
            assignin(ws, [prefix '_CD'], tab.CD);
            assignin(ws, [prefix '_CM'], tab.CM);
            [~, j0] = min(abs(tab.delta_deg(:)));
            cl0 = tab.CL(:, j0);
            cd0 = tab.CD(:, j0);
            cm0 = tab.CM(:, j0);
            assignin(ws, [prefix '_dCL'], tab.CL - cl0);
            assignin(ws, [prefix '_dCD'], tab.CD - cd0);
            assignin(ws, [prefix '_dCM'], tab.CM - cm0);
        end
    end
end
