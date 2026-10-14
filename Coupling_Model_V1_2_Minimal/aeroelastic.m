classdef aeroelastic
%AEROELASTIC  片条气弹：映射、状态空间（入口见 initial_main）
%
% Φ 前 6 列为刚体（surge/sway/heave/roll/pitch/yaw），其后为弹性。
% 入口：setup_paths（含 Library/*.slx）, load_strips, load_maps, wagner_ss, structure_ss。

    methods (Static)
        function p = proj_root
            p = fileparts(mfilename('fullpath'));
        end

        function setup_paths
        %SETUP_PATHS  把可用的运行目录加进 MATLAB 路径
            r = aeroelastic.proj_root;
            addpath(r);
            addpath(fullfile(r, 'lib'));
            addpath(fullfile(r, 'thrust'));
            addpath(fullfile(r, 'trim'));
            tests = fullfile(r, 'tests');
            if isfolder(tests)
                addpath(tests);
            end
            viz = fullfile(r, 'viz');
            if isfolder(viz)
                addpath(viz);
            end
            libdir = fullfile(r, 'Library');
            if isfolder(libdir)
                addpath(libdir);
            end
            aeroelastic.load_libraries();
        end

        function names = load_libraries
        %LOAD_LIBRARIES  load_system(Library/*.slx)，解析 Prop_gryo 等链接
            names = {};
            if isempty(which('load_system'))
                return
            end
            libdir = fullfile(aeroelastic.proj_root, 'Library');
            if ~isfolder(libdir)
                return
            end
            files = dir(fullfile(libdir, '*.slx'));
            names = cell(numel(files), 1);
            loaded = {};
            for i = 1:numel(files)
                [~, names{i}] = fileparts(files(i).name);
                slx = fullfile(libdir, files(i).name);
                try
                    if ~bdIsLoaded(names{i})
                        load_system(slx);
                    end
                    loaded{end+1} = names{i}; %#ok<AGROW>
                catch ME
                    warning('aeroelastic:Library', 'could not load %s: %s', ...
                        files(i).name, ME.message);
                end
            end
            if ~isempty(loaded)
                fprintf('Library loaded: %s\n', strjoin(loaded, ', '));
            end
        end

        function s = load_strips(results_dir, varargin)
        %LOAD_STRIPS  主翼 + V 尾片条，气动力在 c/4
            if nargin < 1 || isempty(results_dir)
                results_dir = fullfile(aeroelastic.proj_root, 'Library');
            end
            p = inputParser;
            % mac_m is retained as the main-wing mean aerodynamic chord for
            % backward compatibility.  The V-tail has its own chord because
            % using the wing chord there over-predicts tail area and loads.
            addParameter(p, 'mac_m', 2.56, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'v_tail_mac_m', 1.20, ...
                @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'v_tail_cant_deg', 42, @(x) isnumeric(x) && isscalar(x));
            % keep_rigid=false (default): Phi / Tq / T_m2* / T_m2body 只有弹性阶；
            % 刚体由 Newton–Euler 走，不再进模态列。
            addParameter(p, 'keep_rigid', false, @(x) islogical(x) && isscalar(x));
            parse(p, varargin{:});
            wing_mac = p.Results.mac_m;
            v_tail_mac = p.Results.v_tail_mac_m;
            cant = deg2rad(p.Results.v_tail_cant_deg);
            keep_rigid = p.Results.keep_rigid;

            node_file = fullfile(results_dir, 'nodes.csv');
            phi_file = fullfile(results_dir, 'modal_matrix_15.csv');
            freq_file = fullfile(results_dir, 'frequencies.csv');
            elem_file = fullfile(results_dir, 'elements.csv');
            if ~isfile(node_file) || ~isfile(phi_file) || ~isfile(freq_file) || ~isfile(elem_file)
                error('aeroelastic:MissingResults', ...
                    'need nodes, elements, modal_matrix_15, frequencies in %s', results_dir);
            end

            T = readtable(node_file);
            E = readtable(elem_file);
            names = string(T.name);
            xyz = [T.x_m, T.y_m, T.z_m];
            nid = T.node_id;

            is_strip = startsWith(names, "WR") | startsWith(names, "WL") ...
                | startsWith(names, "CBR") | startsWith(names, "CBL") ...
                | startsWith(names, "VTR") | startsWith(names, "VTL");

            s.nodes = T;
            s.strip_node_ids = nid(is_strip);
            s.node_name = names(is_strip);
            s.xyz = xyz(is_strip, :);
            s.y_strip = s.xyz(:, 2);
            n_strip = numel(s.strip_node_ids);

            s.surface = strings(n_strip, 1);
            s.surface(startsWith(s.node_name, "WR") | startsWith(s.node_name, "CBR")) = "wing_R";
            s.surface(startsWith(s.node_name, "WL") | startsWith(s.node_name, "CBL")) = "wing_L";
            s.surface(startsWith(s.node_name, "VTR")) = "v_tail_R";
            s.surface(startsWith(s.node_name, "VTL")) = "v_tail_L";

            e_chord = [1, 0, 0];
            n_hat = zeros(n_strip, 3);
            iw = s.surface == "wing_R" | s.surface == "wing_L";
            ir = s.surface == "v_tail_R";
            il = s.surface == "v_tail_L";
            s.c_strip = zeros(n_strip, 1);
            s.c_strip(iw) = wing_mac;
            s.c_strip(ir | il) = v_tail_mac;
            n_hat(iw, :) = repmat([0, 0, 1], nnz(iw), 1);
            n_hat(ir, :) = repmat([0, -sin(cant), cos(cant)], nnz(ir), 1);
            n_hat(il, :) = repmat([0,  sin(cant), cos(cant)], nnz(il), 1);
            e_pitch = zeros(n_strip, 3);
            r_ac = zeros(n_strip, 3);
            for i = 1:n_strip
                pv = cross(n_hat(i, :), e_chord);
                nv = norm(pv);
                if nv < 1e-12
                    pv = [0, 1, 0];
                else
                    pv = pv / nv;
                end
                e_pitch(i, :) = pv;
                r_ac(i, :) = -0.25 * s.c_strip(i) * e_chord;
            end
            s.n_hat = n_hat;
            s.e_pitch = e_pitch;
            s.e_chord = e_chord;
            s.r_ac = r_ac;

            s.ds = zeros(n_strip, 1);
            etype = string(E.type);
            erole = string(E.role);
            lift_elem = ismember(etype, ["center_wing_box", "wing_R", "wing_L", "v_tail_R", "v_tail_L"]) ...
                & (erole == "primary" | erole == "center_wing_box");
            id2k = zeros(max(nid), 1);
            id2k(s.strip_node_ids) = 1:n_strip;
            for e = find(lift_elem).'
                a = E.node_i(e);
                b = E.node_j(e);
                L = E.length_m(e);
                ka = 0; kb = 0;
                if a <= numel(id2k), ka = id2k(a); end
                if b <= numel(id2k), kb = id2k(b); end
                if ka > 0 && kb > 0
                    s.ds(ka) = s.ds(ka) + 0.5 * L;
                    s.ds(kb) = s.ds(kb) + 0.5 * L;
                elseif ka > 0
                    s.ds(ka) = s.ds(ka) + L;
                elseif kb > 0
                    s.ds(kb) = s.ds(kb) + L;
                end
            end
            s.S_strip = s.c_strip .* s.ds;

            s.Phi = readmatrix(phi_file);
            F = readtable(freq_file);
            omega_e = F.omega_rad_s(:);
            freq_e = F.freq_hz(:);
            family_e = string(F.family);
            if ismember('name', F.Properties.VariableNames)
                name_e = string(F.name);
            else
                name_e = "E-mode" + (1:numel(omega_e)).';
            end
            n_nodes = height(T);
            ndof = size(s.Phi, 1);
            if ndof ~= 6 * n_nodes
                error('aeroelastic:PhiSize', 'Phi rows ~= 6*n_nodes');
            end

            s.n_elastic = size(s.Phi, 2);
            s.n_rigid = 0;
            [Phi_r, rb] = rigid_body_modes_(T);
            s.cg_rigid_m = rb.cg_m;  % CG always; rigid Phi only if keep_rigid
            if keep_rigid
                s.Phi = [Phi_r, s.Phi];
                s.omega = [zeros(6, 1); omega_e];
                s.freq_hz = [zeros(6, 1); freq_e];
                s.family = [repmat("rigid", 6, 1); family_e];
                s.mode_name = [rb.names; name_e];
                s.n_rigid = 6;
            else
                s.omega = omega_e;
                s.freq_hz = freq_e;
                s.family = family_e;
                s.mode_name = name_e;
            end
            n_modes = size(s.Phi, 2);

            ndn = 6;
            s.T_m2h = zeros(n_strip, n_modes);
            s.T_m2b = zeros(n_strip, n_modes);
            s.Tq = zeros(n_modes, n_strip);
            s.Tq_D = zeros(n_modes, n_strip);
            s.Tq_CM = zeros(n_modes, n_strip);
            e_drag = e_chord;
            s.e_drag = e_drag;
            for i = 1:n_strip
                base = (s.strip_node_ids(i) - 1) * ndn;
                Phi_u = s.Phi(base + (1:3), :);
                Phi_th = s.Phi(base + (4:6), :);
                s.T_m2h(i, :) = n_hat(i, :) * Phi_u;
                s.T_m2b(i, :) = e_pitch(i, :) * Phi_th;
                rxn = cross(r_ac(i, :), n_hat(i, :));
                rxd = cross(r_ac(i, :), e_drag);
                s.Tq(:, i) = (n_hat(i, :) * Phi_u + rxn * Phi_th).';
                s.Tq_D(:, i) = (e_drag * Phi_u + rxd * Phi_th).';
                s.Tq_CM(:, i) = (e_pitch(i, :) * Phi_th).';
            end

            s.G_du = zeros(n_strip, n_strip);
            wR = s.surface == "wing_R";
            wL = s.surface == "wing_L";
            vtR = s.surface == "v_tail_R";
            vtL = s.surface == "v_tail_L";
            b_wR = max(s.xyz(wR, 2));
            b_wL = -min(s.xyz(wL, 2));
            b_vt = 0.1875 * 18.0;
            s.G_du = s.G_du + surface_induced_(s.S_strip, wR, b_wR);
            s.G_du = s.G_du + surface_induced_(s.S_strip, wL, b_wL);
            s.G_du = s.G_du + surface_induced_(s.S_strip, vtR, b_vt);
            s.G_du = s.G_du + surface_induced_(s.S_strip, vtL, b_vt);
            s.surface_span_m = struct('wing_R', b_wR, 'wing_L', b_wL, ...
                'v_tail_R', b_vt, 'v_tail_L', b_vt);

            s.n_strip = n_strip;
            s.n_modes = n_modes;
            s.results_dir = results_dir;
            s.mac_m = wing_mac;  % legacy alias: main-wing MAC
            s.wing_mac_m = wing_mac;
            s.v_tail_mac_m = v_tail_mac;
            s.surface_names = ["wing_R", "wing_L", "v_tail_R", "v_tail_L"];
            s = control_surfaces.attach(s);
            s.by_surface = aeroelastic.pack_surfaces(s);
            if keep_rigid
                s.note = [ ...
                    'Phi columns: 6 rigid (surge/sway/heave/roll/pitch/yaw) then elastic. ' ...
                    'Strips follow beam stations. AC = node + [-0.25c,0,0]. ' ...
                    'Q = Tq*L + Tq_D*(q S CD) + Tq_CM*(q S c CM).'];
            else
                s.note = [ ...
                    'Phi columns: elastic only (no rigid modes; n_rigid=0). ' ...
                    'Strips follow beam stations. AC = node + [-0.25c,0,0]. ' ...
                    'Q = Tq*L + Tq_D*(q S CD) + Tq_CM*(q S c CM).'];
            end
        end

        function by = pack_surfaces(s)
        %PACK_SURFACES  片条映射拆成 4 个面（T_m2b/h、Tq*、G_du）
            names = ["wing_R", "wing_L", "v_tail_R", "v_tail_L"];
            by = struct();
            for k = 1:numel(names)
                name = names(k);
                idx = s.surface == name;
                ii = find(idx);
                b = struct();
                b.name = name;
                b.idx = ii;
                b.n_strip = numel(ii);
                b.node_name = s.node_name(idx);
                b.strip_node_ids = s.strip_node_ids(idx);
                b.xyz = s.xyz(idx, :);
                b.n_hat = s.n_hat(idx, :);
                b.e_pitch = s.e_pitch(idx, :);
                b.ds = s.ds(idx);
                b.c_strip = s.c_strip(idx);
                b.S_strip = s.S_strip(idx);
                b.T_m2b = s.T_m2b(idx, :);
                b.T_m2h = s.T_m2h(idx, :);
                if isfield(s, 'cg_rigid_m') && ~isempty(s.cg_rigid_m)
                    cg = s.cg_rigid_m(:).';
                else
                    cg = mean(s.xyz, 1);
                end
                p_ac = s.xyz(idx, :) + s.r_ac(idx, :);
                b.r_ac_s = (p_ac - cg).';
                b.n_hat_s = s.n_hat(idx, :).';
                b.Tq = s.Tq(:, idx);
                b.Tq_D = s.Tq_D(:, idx);
                b.Tq_CM = s.Tq_CM(:, idx);
                b.G_du = s.G_du(idx, idx);
                if isfield(s, 'ctrl')
                    b.ctrl = s.ctrl(idx);
                    b.flap_family = s.flap_family(idx);
                    b.Mix = s.Mix(idx, :);
                    b.map_aileron = s.map_aileron(idx);
                    b.map_tail = s.map_tail(idx);
                    b.ctrl_xh_c = s.ctrl_xh_c(idx);
                end
                by.(name) = b;
            end
            by.surface_names = names;
        end

        function out = wing_aileron_to_W(s)
        %WING_AILERON_TO_W  片条 6 维气动扳手（AC）→ 结构节点 W
        %
        %  四面顺序与 Mux 一致：wing_L, wing_R, tail_L, tail_R。
        %  每个面内部与 W_pack 一致：先该面全部 Fx，再全部 Fy、Fz、Mx、My、Mz。
        %  不是按片条把六个分量放在一起。副翼不拆开。
        %  力臂 r_ac 折到梁站：M_node = M + r_ac × F。
        %
        %  W = T_aero2W * f            T 为 ndof×(6n)
        %  Q = Tq_mass * W
            ndn = 6;
            ndof = size(s.Phi, 1);
            names = string(s.node_name(:));
            groups = { ...
                'wing_L', s.surface(:) == "wing_L"
                'wing_R', s.surface(:) == "wing_R"
                'tail_L', s.surface(:) == "v_tail_L"
                'tail_R', s.surface(:) == "v_tail_R"
                };
            n_g = zeros(1, 4);
            idx = cell(1, 4);
            for k = 1:4
                idx{k} = find(groups{k, 2});
                n_g(k) = numel(idx{k});
            end
            n = sum(n_g);
            T = zeros(ndof, ndn * n);
            T_L = zeros(ndof, n);
            T_D = zeros(ndof, n);
            T_M = zeros(ndof, n);
            col_name = strings(n, 1);
            col_group = strings(n, 1);
            strip_i = zeros(n, 1);
            col = 0;
            for k = 1:4
                for j = 1:n_g(k)
                    i = idx{k}(j);
                    col = col + 1;
                    nid = s.strip_node_ids(i);
                    rows = (nid - 1) * ndn + (1:ndn);
                    n_hat = s.n_hat(i, :).';
                    if size(s.e_drag, 1) == 1
                        e_d = s.e_drag(:);
                    else
                        e_d = s.e_drag(i, :).';
                    end
                    e_th = s.e_pitch(i, :).';
                    r = s.r_ac(i, :).';
                    rx = [0, -r(3), r(2); r(3), 0, -r(1); -r(2), r(1), 0];
                    Ad = [eye(3), zeros(3); rx, eye(3)];
                    T(rows, (col - 1) * ndn + (1:ndn)) = Ad;
                    T_L(rows, col) = Ad * [n_hat; zeros(3, 1)];
                    T_D(rows, col) = Ad * [e_d; zeros(3, 1)];
                    T_M(rows, col) = Ad * [zeros(3, 1); e_th];
                    strip_i(col) = i;
                    col_name(col) = names(i);
                    col_group(col) = groups{k, 1};
                end
            end
            c0 = [0, cumsum(n_g)];
            out = struct();
            out.T = T;
            out.T_L = T_L;
            out.T_D = T_D;
            out.T_M = T_M;
            out.n = n_g;
            out.group = string({groups{:, 1}});
            out.idx = idx;
            out.strip_i = strip_i;
            out.node_name = col_name;
            out.col_group = col_group;
            out.ndof = ndof;
            out.n_strip = n;
            gn = out.group;
            out.cols = struct();
            out.cols6 = struct();
            for k = 1:4
                out.cols.(gn(k)) = (c0(k) + 1):c0(k + 1);
                out.cols6.(gn(k)) = (c0(k) * ndn + 1):(c0(k + 1) * ndn);
            end
            out.note = [ ...
                'W=T_aero2W*f, f=6n x 1 structural AC wrenches [Fx Fy Fz Mx My Mz] per strip. ' ...
                'Order wing_L, wing_R, tail_L, tail_R (whole surfaces, left then right). Q=Tq_mass*W.'];
        end

        function maps = load_maps(Phi, nodes)
        %LOAD_MAPS  推进 / 质量扳手 → Q（Tq_prop、Tq_mass）
            if istable(nodes)
                node_id = nodes.node_id;
                node_name = string(nodes.name);
                xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
            else
                error('aeroelastic:Nodes', 'nodes must be a table from nodes.csv');
            end
            n_nodes = numel(node_id);
            ndn = 6;
            ndof = size(Phi, 1);
            n_modes = size(Phi, 2);
            if ndof ~= ndn * n_nodes
                error('aeroelastic:Size', 'Phi rows (%d) ~= 6*n_nodes (%d)', ndof, ndn*n_nodes);
            end

            eng_name = ["TL2_F"; "TL1_F"; "TR1_F"; "TR2_F"; ...
                        "TL2_R"; "TL1_R"; "TR1_R"; "TR2_R"];
            n_prop = numel(eng_name);
            prop_node_ids = zeros(n_prop, 1);
            prop_xyz = zeros(n_prop, 3);
            for i = 1:n_prop
                k = find(node_name == eng_name(i), 1);
                if isempty(k)
                    error('aeroelastic:Engine', 'missing node %s', eng_name(i));
                end
                prop_node_ids(i) = node_id(k);
                prop_xyz(i, :) = xyz(k, :);
            end

            maps = struct();
            maps.frame = 'structural  x aft, y right, z up';
            maps.wrench_order = '[Fx,Fy,Fz,Mx,My,Mz]';
            maps.n_modes = n_modes;
            maps.n_nodes = n_nodes;
            maps.ndof = ndof;
            maps.n_prop = n_prop;
            maps.n_prop_wrench = n_prop * ndn;
            maps.prop_id = "E" + (1:n_prop).';
            maps.prop_node_name = eng_name;
            maps.prop_node_ids = prop_node_ids;
            maps.prop_xyz_m = prop_xyz;
            maps.T_prop2W = scatter_nodes_to_W_(ndof, prop_node_ids);  % ndof×48, W=T_prop2W*W_prop
            maps.Tq_prop = phi_nodes_to_tq_(Phi, prop_node_ids);       % n_modes×48, Q=Tq_prop*W_prop
            maps.Tq_mass = Phi.';
            maps.note = [ ...
                'Gravity is W_mass_g via Tq_mass. ' ...
                'W_prop is 48x1 structural wrenches at E1..E8 (TL2_F..TR2_R). ' ...
                'W=T_prop2W*W_prop, Qp=Tq_prop*W_prop=Phi''*W.'];
            maps.engine_geom = aeroelastic.engine_geom(nodes, maps);
        end

        function g = engine_geom(nodes, maps)
        %ENGINE_GEOM  8 发结构系推力轴：倾转 0=前飞(-x)，pi/2=沿短梁向上（Fz>0）
            if nargin < 2 || isempty(maps)
                error('aeroelastic:EngineGeom', 'pass maps from load_maps');
            end
            names = maps.prop_node_name;
            kink = replace(names, ["_F","_R"], ["_FK","_RK"]);
            xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
            node_name = string(nodes.name);
            n = numel(names);
            e_stub = zeros(3, n);
            e_hover = zeros(3, n);
            e_cruise = zeros(3, n);
            is_tilt = false(n, 1);
            p_kink = zeros(3, n);
            p_eng = maps.prop_xyz_m.';
            for i = 1:n
                ke = find(node_name == names(i), 1);
                kk = find(node_name == kink(i), 1);
                if isempty(kk)
                    error('aeroelastic:Kink', 'missing kink %s', kink(i));
                end
                p_kink(:, i) = xyz(kk, :).';
                d = xyz(ke, :).' - xyz(kk, :).';
                nrm = norm(d);
                if nrm < 1e-12
                    d = [0; 0; 1];
                    nrm = 1;
                end
                e_stub(:, i) = d / nrm;
                % 悬停沿短梁，取结构 Fz>0（向上）。后内台短梁朝下，用 -e_stub，
                % 不要只 abs(z)：那样 y 不翻，轴不在桅杆上。
                sz = sign(e_stub(3, i));
                if sz == 0
                    sz = 1;
                end
                e_hover(:, i) = e_stub(:, i) * sz;
                is_tilt(i) = contains(names(i), "L1_") || contains(names(i), "R1_");
                if is_tilt(i)
                    e_cruise(:, i) = [-1; 0; 0];
                else
                    e_cruise(:, i) = e_hover(:, i);
                end
            end
            Mtot = sum(nodes.mass_kg);
            cg = (nodes.mass_kg.' * xyz / Mtot).';
            g = struct();
            g.prop_id = maps.prop_id;
            g.prop_node_name = names;
            g.kind = strings(n, 1);
            g.kind(is_tilt) = "tilt";
            g.kind(~is_tilt) = "lift";
            g.is_tilt = is_tilt;
            g.e_stub = e_stub;
            g.e_cruise = e_cruise;
            g.e_hover = e_hover;
            g.p_eng_m = p_eng;
            g.p_kink_m = p_kink;
            g.cg_m = cg;
            g.R_sb = diag([-1, 1, -1]);
            g.r_cg2kink_m = p_kink - cg;                    % 3×8 结构系
            g.r_cg2kink_b = g.R_sb * g.r_cg2kink_m;         % 3×8 机体系
            g.e_hinge_s = zeros(3, n);
            for i = 1:n
                if ~is_tilt(i)
                    continue
                end
                kh = cross(e_cruise(:, i), e_hover(:, i));
                nk = norm(kh);
                if nk > 1e-12
                    g.e_hinge_s(:, i) = kh / nk;
                end
            end
            g.note = [ ...
                'E1 TL2_F lift L-out-F, E2 TL1_F tilt L-in-F, E3 TR1_F tilt R-in-F, E4 TR2_F lift R-out-F, ' ...
                'E5 TL2_R lift L-out-R, E6 TL1_R tilt L-in-R, E7 TR1_R tilt R-in-R, E8 TR2_R lift R-out-R. ' ...
                'tilt=0 cruise, pi/2 hover (mesh). Stub rotates about kink, e_hinge=e_cruise x e_hover. ' ...
                'W_prop F at mesh engine node, M=(p(theta)-p0)x F. Do not add r_cg x F.'];
        end

        function out = gravity(nodes, varargin)
        %GRAVITY  节点质量 → 结构系重力扳手 W_mass（进 Tq_mass）
            p = inputParser;
            addParameter(p, 'g', 9.80665, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'g_struct', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 3));
            parse(p, varargin{:});
            r = p.Results;

            nid = nodes.node_id(:);
            m = nodes.mass_kg(:);
            xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
            n = numel(nid);
            ndof = 6 * max(nid);

            g_struct = r.g_struct(:);
            if isempty(g_struct)
                g_struct = [0; 0; -r.g];
            end

            Mtot = sum(m);
            if Mtot <= 0
                error('aeroelastic:Mass', 'total nodal mass <= 0');
            end
            cg = (m.' * xyz / Mtot).';

            W_mass = zeros(ndof, 1);
            F_g_total = zeros(3, 1);
            I_cg_m = zeros(3);
            for k = 1:n
                F_g = m(k) * g_struct;
                F_g_total = F_g_total + F_g;
                rows = (nid(k) - 1) * 6 + (1:6);
                W_mass(rows(1:3)) = F_g;
                rk = xyz(k, :).' - cg;
                I_cg_m = I_cg_m + m(k) * ((rk.' * rk) * eye(3) - rk * rk.');
            end
            R_sb = diag([-1, 1, -1]);
            I_cg_b = R_sb * I_cg_m * R_sb.';
            I_cg_m_inv = inv(I_cg_m);
            I_cg_b_inv = R_sb * I_cg_m_inv * R_sb.';

            out = struct();
            out.g = r.g;
            out.g_struct = g_struct;
            out.m_total_kg = Mtot;
            out.cg_m = cg;
            out.I_cg_m = I_cg_m;
            out.I_cg_b = I_cg_b;
            out.I_cg_m_inv = I_cg_m_inv;
            out.I_cg_b_inv = I_cg_b_inv;
            out.W_mass = W_mass;
            out.F_g_total = F_g_total;
            out.note = 'Gravity -> Tq_mass. I_cg_* from nodal point masses about CG.';
        end

        function out = node_arms(nodes, varargin)
        %NODE_ARMS  质心 → 全部节点：r 与把节点扳手合到质心的 T
        %
        %  列号 = node_id，与 W((id-1)*6+(1:6)) 对齐。
        %  [F;M]_cg_s = T_W2cg_s * W_s
        %  [F;M]_cg_b = T_W2cg_b * W_s   （W 仍是结构系，合力在机体系）
        %  合力：F = Σ F_i，M = Σ (r_i × F_i + M_i)。不是 6 维去叉乘。
            p = inputParser;
            addParameter(p, 'R_sb', diag([-1, 1, -1]), @(x) isnumeric(x) && isequal(size(x), [3 3]));
            parse(p, varargin{:});
            R_sb = p.Results.R_sb;
            nid = nodes.node_id(:);
            xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
            m = nodes.mass_kg(:);
            Mtot = sum(m);
            cg = (m.' * xyz / Mtot).';
            n_id = max(nid);
            ndof = 6 * n_id;
            r_m = zeros(3, n_id);
            for k = 1:numel(nid)
                r_m(:, nid(k)) = xyz(k, :).' - cg;
            end
            r_b = R_sb * r_m;
            T_s = zeros(6, ndof);
            for k = 1:numel(nid)
                id = nid(k);
                cols = (id - 1) * 6 + (1:6);
                rv = r_m(:, id);
                rx = [0, -rv(3), rv(2); rv(3), 0, -rv(1); -rv(2), rv(1), 0];
                T_s(:, cols) = [eye(3), zeros(3); rx, eye(3)];
            end
            Rb6 = blkdiag(R_sb, R_sb);
            out = struct();
            out.cg_m = cg;
            out.n_nodes = n_id;
            out.ndof = ndof;
            out.R_sb = R_sb;
            out.r_cg2node_m = r_m;
            out.r_cg2node_b = r_b;
            out.T_W2cg_s = T_s;
            out.T_W2cg_b = Rb6 * T_s;
            out.note = [ ...
                'r_cg2node_* is 3 x n_nodes, col i = node_id i. ' ...
                'W_cg = T_W2cg_* * W, W is ndof x 1 structural [F M] per node.'];
        end

        function maps = node_kinematics(Phi, nodes)
        %NODE_KINEMATICS  η̇ → 节点速度 / CG 体轴 6DOF
            if istable(nodes)
                node_id = nodes.node_id(:);
                xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
            else
                error('aeroelastic:Nodes', 'nodes must be a table from nodes.csv');
            end
            n_nodes = numel(node_id);
            ndn = 6;
            ndof = size(Phi, 1);
            n_modes = size(Phi, 2);
            if ndof ~= ndn * n_nodes
                error('aeroelastic:Size', 'Phi rows (%d) ~= 6*n_nodes (%d)', ndof, ndn * n_nodes);
            end

            T_m2v = zeros(3 * n_nodes, n_modes);
            for k = 1:n_nodes
                base6 = (node_id(k) - 1) * ndn;
                base3 = (node_id(k) - 1) * 3;
                Phi_u = Phi(base6 + (1:3), :);
                T_m2v(base3 + (1:3), :) = Phi_u;
            end

            m = nodes.mass_kg(:);
            Mtot = sum(m);
            cg = (m.' * xyz / Mtot).';
            T_m2v_cg = zeros(3, n_modes);
            T_H = zeros(3, n_modes);
            Icg = zeros(3);
            for k = 1:n_nodes
                base3 = (node_id(k) - 1) * 3 + (1:3);
                rk = xyz(k, :).' - cg;
                Tk = T_m2v(base3, :);
                T_m2v_cg = T_m2v_cg + m(k) * Tk;
                rx = [0, -rk(3), rk(2); rk(3), 0, -rk(1); -rk(2), rk(1), 0];
                T_H = T_H + m(k) * (rx * Tk);
                Icg = Icg + m(k) * ((rk.' * rk) * eye(3) - rk * rk.');
            end
            T_m2v_cg = T_m2v_cg / Mtot;
            T_m2w_cg = Icg \ T_H;
            R_sb = diag([-1, 1, -1]);
            T_m2vb = R_sb * T_m2v_cg;
            T_m2wb = R_sb * T_m2w_cg;

            maps = struct();
            maps.frame = 'structural  x aft, y right, z up';
            maps.n_nodes = n_nodes;
            maps.n_modes = n_modes;
            maps.cg_m = cg;
            maps.R_sb = R_sb;
            maps.T_m2v = T_m2v;
            maps.T_m2body = [T_m2vb; T_m2wb];
            maps.note = [ ...
                'Body (x fwd, y right, z down): nu_body=[u;v;w;p;q;r]=T_m2body*eta_dot. ' ...
                'R_sb=diag(-1,1,-1).'];
        end

        function p = polar(fname)
        %POLAR  读 XFOIL NACA 2412 polar
            if nargin < 1 || isempty(fname)
                fname = fullfile(aeroelastic.proj_root, ...
                    'polars', 'naca2412_Re6p81e6_Ncrit9.pol');
            end
            if ~isfile(fname)
                error('aeroelastic:MissingPolar', 'polar file not found: %s', fname);
            end
            fid = fopen(fname, 'r');
            if fid < 0
                error('aeroelastic:OpenPolar', 'cannot open %s', fname);
            end
            cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

            p.file = fname;
            p.airfoil = 'NACA 2412';
            p.Mach = NaN;
            p.Re = NaN;
            p.Ncrit = 9;
            header_done = false;
            while true
                line = fgetl(fid);
                if ~ischar(line)
                    error('aeroelastic:PolarFormat', 'no data table in %s', fname);
                end
                if contains(line, 'Calculated polar for:')
                    p.airfoil = strtrim(erase(line, 'Calculated polar for:'));
                end
                if contains(line, 'Mach =')
                    tok = regexp(line, ...
                        'Mach\s*=\s*([0-9.]+)\s+Re\s*=\s*([0-9.]+)\s*e\s*([0-9]+)\s+Ncrit\s*=\s*([0-9.]+)', ...
                        'tokens', 'once');
                    if ~isempty(tok)
                        p.Mach = str2double(tok{1});
                        p.Re = str2double(tok{2}) * 10^str2double(tok{3});
                        p.Ncrit = str2double(tok{4});
                    end
                end
                if contains(line, '------')
                    header_done = true;
                    break
                end
            end
            if ~header_done
                error('aeroelastic:PolarFormat', 'no ------ separator in %s', fname);
            end
            C = textscan(fid, '%f%f%f%f%f%f%f');
            A = [C{:}];
            if isempty(A) || size(A, 2) < 5
                error('aeroelastic:PolarEmpty', 'no polar rows in %s', fname);
            end
            [~, iord] = sort(A(:, 1));
            A = A(iord, :);
            [p.alpha_deg, iu] = unique(A(:, 1), 'stable');
            p.CL  = A(iu, 2);
            p.CD  = A(iu, 3);
            p.CDp = A(iu, 4);
            p.CM  = A(iu, 5);
            if size(A, 2) >= 7
                p.Top_Xtr = A(iu, 6);
                p.Bot_Xtr = A(iu, 7);
            end
            p.alpha_rad = p.alpha_deg * pi / 180;
            p.n = numel(p.alpha_deg);
            p.fit_alpha_deg = [-4, 4];
            mask = p.alpha_deg >= p.fit_alpha_deg(1) & p.alpha_deg <= p.fit_alpha_deg(2);
            if nnz(mask) < 4
                error('aeroelastic:PolarFit', 'too few points in linear alpha range');
            end
            coef = polyfit(p.alpha_rad(mask), p.CL(mask), 1);
            p.a0 = coef(1);
            p.CL0 = coef(2);
            p.alpha0_rad = -p.CL0 / p.a0;
            p.alpha0_deg = p.alpha0_rad * 180 / pi;
        end

        function [sys_c, sys_d, info] = wagner_ss(varargin)
        %WAGNER_SS  多片条解耦 Jones/Wagner
            p = inputParser;
            addParameter(p, 'a0', 2*pi, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'c', 2.0, @(x) isnumeric(x) && isvector(x) && all(x > 0));
            addParameter(p, 'c_root', [], @(x) isempty(x) || (isscalar(x) && x > 0));
            addParameter(p, 'c_tip', [], @(x) isempty(x) || (isscalar(x) && x > 0));
            addParameter(p, 'span', 8.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'n_strip', 8, @(x) isnumeric(x) && isscalar(x) && x >= 1);
            addParameter(p, 'U', 50, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'rho', 1.225, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'Ts', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'y_strip', [], @(x) isempty(x) || (isnumeric(x) && isvector(x)));
            addParameter(p, 'ds', [], @(x) isempty(x) || (isnumeric(x) && isvector(x) && all(x > 0)));
            addParameter(p, 'output', 'lift', @(s) any(strcmpi(char(s), {'lift','CL'})));
            addParameter(p, 'discretize_method', 'zoh', @(s) ischar(s) || isstring(s));
            parse(p, varargin{:});
            r = p.Results;

            n = round(r.n_strip);
            s = r.span / 2;

            if ~isempty(r.ds)
                dy = r.ds(:);
                n = numel(dy);
                if ~isempty(r.y_strip)
                    y = r.y_strip(:);
                    if numel(y) ~= n
                        error('aeroelastic:WagnerSize', 'y_strip and ds length mismatch');
                    end
                else
                    y = zeros(n, 1);
                end
            elseif ~isempty(r.y_strip)
                y_in = r.y_strip(:);
                n = numel(y_in);
                [y_sorted, ord] = sort(y_in);
                y_edge = zeros(n + 1, 1);
                y_edge(1) = -s;
                y_edge(end) = s;
                for i = 1:n-1
                    y_edge(i+1) = 0.5 * (y_sorted(i) + y_sorted(i+1));
                end
                dy_sorted = diff(y_edge);
                y = zeros(n, 1);
                dy = zeros(n, 1);
                y(ord) = y_sorted;
                dy(ord) = dy_sorted;
            else
                y_edge = linspace(-s, s, n + 1).';
                y = 0.5 * (y_edge(1:end-1) + y_edge(2:end));
                dy = diff(y_edge);
            end

            if ~isempty(r.c_root) && ~isempty(r.c_tip)
                eta = min(abs(y) / max(s, eps), 1);
                c_strip = r.c_root + (r.c_tip - r.c_root) .* eta;
            else
                c_in = r.c(:);
                if isscalar(c_in)
                    c_strip = c_in * ones(n, 1);
                else
                    if numel(c_in) ~= n
                        error('aeroelastic:BadChord', 'c must be scalar or length n_strip');
                    end
                    c_strip = c_in;
                end
            end

            A1 = 0.165;  eps1 = 0.0455;
            A2 = 0.335;  eps2 = 0.300;
            phi0 = 1 - A1 - A2;

            qbar = 0.5 * r.rho * r.U^2;
            nx = 2 * n;
            A = zeros(nx);
            B = zeros(nx, n);
            C = zeros(n, nx);
            D = zeros(n, n);
            k_strip = r.U ./ (c_strip / 2);
            out_cl = strcmpi(char(r.output), 'CL');
            for i = 1:n
                k = k_strip(i);
                ix = [2*i-1, 2*i];
                A(ix, ix) = diag([-eps1 * k, -eps2 * k]);
                B(ix, i) = [k; k];
                if out_cl
                    gain = r.a0;
                else
                    gain = qbar * c_strip(i) * dy(i) * r.a0;
                end
                C(i, ix) = gain * [A1 * eps1, A2 * eps2];
                D(i, i) = gain * phi0;
            end

            sys_c = ss(A, B, C, D);
            sys_d = c2d(sys_c, r.Ts, char(r.discretize_method));
            T_kin = [ones(n, 1), y / r.U, (c_strip / 2) / r.U];
            S_strip = c_strip .* dy;
            info = struct();
            info.a0 = r.a0;
            info.c_strip = c_strip;
            info.span = r.span;
            info.n_strip = n;
            info.U = r.U;
            info.rho = r.rho;
            info.qbar = qbar;
            info.Ts = r.Ts;
            info.y_strip = y;
            info.dy = dy;
            info.S_strip = S_strip;
            info.S = sum(S_strip);
            info.AR = r.span^2 / max(info.S, eps);
            info.w_CL = (S_strip / max(info.S, eps)).';
            info.b_half = c_strip / 2;
            info.k = k_strip;
            info.jones = struct('A1', A1, 'eps1', eps1, 'A2', A2, 'eps2', eps2, 'phi0', phi0);
            info.output = char(r.output);
            info.T_kin = T_kin;
            info.T_kin_u = {'alpha_rad'; 'p_radps'; 'q_radps'};
            info.A = A; info.B = B; info.C = C; info.D = D;
            info.Ad = sys_d.A; info.Bd = sys_d.B; info.Cd = sys_d.C; info.Dd = sys_d.D;
        end

        function ss4 = split_wagner(Ad, Bd, Cd, Dd, x0, qS, qSc, c2U, u0, aero)
        %SPLIT_WAGNER  整机 Wagner 按 4 个面切开并写入 base
            names = aero.surface_names;
            ss4 = struct();
            for k = 1:numel(names)
                nm = names(k);
                idx = aero.(nm).idx(:);
                st = reshape([2*idx - 1, 2*idx].', [], 1);
                p = struct();
                p.idx = idx;
                p.n_strip = numel(idx);
                p.Ad = Ad(st, st);
                p.Bd = Bd(st, idx);
                p.Cd = Cd(idx, st);
                p.Dd = Dd(idx, idx);
                p.x0_wagner = x0(st);
                p.qS_strip = qS(idx);
                p.qSc_strip = qSc(idx);
                p.c_over_2U = c2U(idx);
                p.alpha_du0 = aero.(nm).G_du * u0(idx);
                p.G_du = aero.(nm).G_du;
                p.T_m2b = aero.(nm).T_m2b;
                p.T_m2h = aero.(nm).T_m2h;
                p.r_ac_s = aero.(nm).r_ac_s;
                p.n_hat_s = aero.(nm).n_hat_s;
                p.Tq = aero.(nm).Tq;
                p.Tq_D = aero.(nm).Tq_D;
                p.Tq_CM = aero.(nm).Tq_CM;
                ss4.(nm) = p;
                export_wagner_surface_(nm, p);
            end
            ss4.surface_names = names;
        end

        function [sys_c, sys_d, info] = structure_ss(omega, varargin)
        %STRUCTURE_SS  质量归一模态，x=[η; η̇]，y=[η; η̇]
        % 刚体 ω=0：双积分、C=0。Rayleigh 只用弹性阶，避免刚体被 αI 拖住。
            p = inputParser;
            addRequired(p, 'omega', @(x) isnumeric(x) && isvector(x) && all(x >= 0));
            addParameter(p, 'zeta', 0.02, @(x) isnumeric(x) && isscalar(x) && x >= 0);
            addParameter(p, 'Ts', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'damping', 'rayleigh', @(s) any(strcmpi(char(s), {'rayleigh','modal'})));
            addParameter(p, 'discretize_method', 'zoh', @(s) ischar(s) || isstring(s));
            addParameter(p, 'n_rigid', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
            parse(p, omega, varargin{:});
            r = p.Results;
            om = omega(:);
            n = numel(om);
            if isempty(r.n_rigid)
                n_rigid = nnz(om < 1e-4);
            else
                n_rigid = r.n_rigid;
            end
            iel = (1:n) > n_rigid;
            Om2 = diag(om.^2);
            C_modal = zeros(n);
            alpha_r = NaN; beta_r = NaN;
            switch lower(char(r.damping))
                case 'rayleigh'
                    we = om(iel);
                    if numel(we) >= 2
                        wsum = we(1) + we(2);
                        alpha_r = 2 * r.zeta * we(1) * we(2) / wsum;
                        beta_r = 2 * r.zeta / wsum;
                        C_modal(iel, iel) = diag(alpha_r + beta_r * we.^2);
                    elseif numel(we) == 1
                        C_modal(iel, iel) = 2 * r.zeta * we;
                    end
                case 'modal'
                    C_modal = diag(2 * r.zeta * om);
            end
            A = [zeros(n), eye(n); -Om2, -C_modal];
            B = [zeros(n); eye(n)];
            C = eye(2 * n);
            D = zeros(2 * n, n);
            sys_c = ss(A, B, C, D);
            sys_d = c2d(sys_c, r.Ts, char(r.discretize_method));
            info = struct();
            info.n_modes = n;
            info.n_rigid = n_rigid;
            info.omega = om;
            info.zeta = r.zeta;
            info.damping = char(r.damping);
            info.alpha_rayleigh = alpha_r;
            info.beta_rayleigh = beta_r;
            info.C_modal = C_modal;
            info.Ts = r.Ts;
            info.A = A; info.B = B; info.C = C; info.D = D;
            info.Ad = sys_d.A; info.Bd = sys_d.B; info.Cd = sys_d.C; info.Dd = sys_d.D;
            info.output = '[eta; eta_dot]';
            info.input = 'Q (modal force, mass-normalized)';
        end

        function s = surface_load_nodes(nodes)
        %SURFACE_LOAD_NODES  左右翼、左右 V 尾载荷用的结构节点。
        %
        %  右翼：WR* 梁站 + CBR* 中央翼盒右半 + TR* 发动机臂
        %  左翼：WL* + CBL* + TL*
        %  右/左尾：只有 VTR* / VTL*，不含机身
        %  机身 F* 不进任何一个面。
        %
        %  node_id 与 nodes.csv 一致。dof 是 534 维向量里该节点 6 行，
        %  顺序 [ux;uy;uz;tx;ty;tz]，可直接切 Phi 和惯性载荷。
            names = string(nodes.name);
            nid = nodes.node_id(:);
            comp = string(nodes.component);
            s = struct();
            s.wing_R = pack_surface_nodes_(nid, names, ...
                comp == "wing_R" | startsWith(names, "CBR"));
            s.wing_L = pack_surface_nodes_(nid, names, ...
                comp == "wing_L" | startsWith(names, "CBL"));
            s.v_tail_R = pack_surface_nodes_(nid, names, ...
                comp == "v_tail_R" | startsWith(names, "VTR"));
            s.v_tail_L = pack_surface_nodes_(nid, names, ...
                comp == "v_tail_L" | startsWith(names, "VTL"));
            % 机翼连接处：切在 F07。外侧 = 左右中央翼盒 + 左右翼梁 + 发动机臂。
            s.wing_joint = pack_surface_nodes_(nid, names, ...
                comp == "wing_R" | comp == "wing_L" | startsWith(names, "CB"));
            s.wing_joint.cut_node = 7;
            s.wing_joint.cut_name = "F07";
            % 尾翼连接处：切在 F14。外侧 = 左右 V 尾，不含 F14 和机尾 F15。
            s.v_tail_joint = pack_surface_nodes_(nid, names, ...
                comp == "v_tail_R" | comp == "v_tail_L" | startsWith(names, "VT"));
            s.v_tail_joint.cut_node = 14;
            s.v_tail_joint.cut_name = "F14";
            % 尾舱：切在 F13，梁的外侧是 F14。再带上 F14 上挂的 V 尾和机尾 F15。
            s.tail_bay = pack_surface_nodes_(nid, names, ...
                names == "F14" | names == "F15" | startsWith(names, "VT"));
            s.tail_bay.cut_node = 13;
            s.tail_bay.cut_name = "F13";
            s.note = [ ...
                'Wing/tail surface sets exclude the fuselage. ' ...
                'wing_joint cut=F07; v_tail_joint cut=F14; tail_bay cut=F13. ' ...
                'Cut nodes are not inside the outboard load set.'];
        end

        function out = nodal_mass(nodes)
        %NODAL_MASS  每个节点一块 6×6 集中质量，下标 = node_id。
        %
        %  点质量：平动 m，三个转角惯量为 0。
        %  M(:,:,id) = diag(m, m, m, 0, 0, 0)
        %
        %  加速度 a 为 6×n，列 k 对应 node_id k：
        %    a(:,k) = [ax; ay; az; alphax; alphay; alphaz]
        %  坐标系与 a 相同。惯性载荷（达朗贝尔，与外力相加）
        %    W_in = -pagemtimes(M_node, reshape(a, 6, 1, []));
        %  W_in 为 6×1×n，前三维 -m*a，后三维 0。
        %  只有平动加速度（3×n）时，后三维补 0 再乘。
            nid = nodes.node_id(:);
            m = nodes.mass_kg(:);
            n_id = max(nid);
            M = zeros(6, 6, n_id);
            m_node = zeros(n_id, 1);
            for k = 1:numel(nid)
                id = nid(k);
                mk = m(k);
                M(:, :, id) = diag([mk, mk, mk, 0, 0, 0]);
                m_node(id) = mk;
            end
            out = struct();
            out.M = M;
            out.m_node = m_node;
            out.n_node = n_id;
            out.note = ['Point mass, rotary inertia 0. ' ...
                'Inertial wrench W_in = -M*a, same frame as a.'];
        end

    end
end

function b = pack_surface_nodes_(nid, names, mask)
    id = sort(nid(mask));
    [~, ord] = ismember(id, nid);
    n = numel(id);
    dof = zeros(6 * n, 1);
    for k = 1:n
        dof((k - 1) * 6 + (1:6)) = (id(k) - 1) * 6 + (1:6).';
    end
    b = struct('node_id', id, 'name', names(ord), 'dof', dof, 'n', n);
end

function [Phi_r, info] = rigid_body_modes_(nodes)
% 结构系刚体 6 模态，集中质量归一：平移 xyz，绕 CG 滚转/俯仰/偏航。
    nid = nodes.node_id(:);
    m = nodes.mass_kg(:);
    xyz = [nodes.x_m, nodes.y_m, nodes.z_m];
    n = numel(nid);
    ndof = 6 * max(nid);
    Mtot = sum(m);
    if Mtot <= 0
        error('aeroelastic:RigidMass', 'total nodal mass <= 0');
    end
    cg = (m.' * xyz / Mtot).';

    V = zeros(ndof, 6);
    for k = 1:n
        rows = (nid(k) - 1) * 6 + (1:6);
        r = xyz(k, :).' - cg;
        V(rows(1), 1) = 1;
        V(rows(2), 2) = 1;
        V(rows(3), 3) = 1;
        V(rows(1:3), 4) = cross([1; 0; 0], r);
        V(rows(4), 4) = 1;
        V(rows(1:3), 5) = cross([0; 1; 0], r);
        V(rows(5), 5) = 1;
        V(rows(1:3), 6) = cross([0; 0; 1], r);
        V(rows(6), 6) = 1;
    end

    Phi_r = zeros(ndof, 6);
    for j = 1:6
        v = V(:, j);
        for i = 1:j-1
            v = v - mass_dot_(Phi_r(:, i), v, nid, m) * Phi_r(:, i);
        end
        nv = sqrt(max(mass_dot_(v, v, nid, m), 0));
        if nv < 1e-12
            error('aeroelastic:RigidMode', 'rigid mode %d has zero mass', j);
        end
        Phi_r(:, j) = v / nv;
    end
    info = struct();
    info.cg_m = cg;
    info.m_total_kg = Mtot;
    info.names = ["R-surge"; "R-sway"; "R-heave"; "R-roll"; "R-pitch"; "R-yaw"];
end

function s = mass_dot_(a, b, nid, m)
    s = 0;
    for k = 1:numel(nid)
        rows = (nid(k) - 1) * 6 + (1:3);
        s = s + m(k) * (a(rows).' * b(rows));
    end
end

function G = surface_induced_(S_strip, idx, b_span)
    n = numel(S_strip);
    G = zeros(n);
    idx = idx(:);
    if ~any(idx), return; end
    S = sum(S_strip(idx));
    if S <= 0, return; end
    AR = b_span^2 / S;
    w = zeros(1, n);
    w(idx) = S_strip(idx).' / S;
    ones_s = zeros(n, 1);
    ones_s(idx) = 1;
    G = (1 / (pi * AR)) * (ones_s * w);
end

function T = scatter_nodes_to_W_(ndof, node_ids)
    ndn = 6;
    n = numel(node_ids);
    T = zeros(ndof, n * ndn);
    for i = 1:n
        rows = (node_ids(i) - 1) * ndn + (1:ndn);
        cols = (i - 1) * ndn + (1:ndn);
        T(rows, cols) = eye(ndn);
    end
end

function Tq = phi_nodes_to_tq_(Phi, node_ids)
    ndn = 6;
    n = numel(node_ids);
    n_modes = size(Phi, 2);
    Tq = zeros(n_modes, n * ndn);
    for i = 1:n
        rows = (node_ids(i) - 1) * ndn + (1:ndn);
        cols = (i - 1) * ndn + (1:ndn);
        Tq(:, cols) = Phi(rows, :).';
    end
end

function export_wagner_surface_(nm, p)
    suf = char(nm);
    ws = 'base';
    assignin(ws, ['Ad_' suf], p.Ad);
    assignin(ws, ['Bd_' suf], p.Bd);
    assignin(ws, ['Cd_' suf], p.Cd);
    assignin(ws, ['Dd_' suf], p.Dd);
    assignin(ws, ['x0_wagner_' suf], p.x0_wagner);
    assignin(ws, ['qS_strip_' suf], p.qS_strip);
    assignin(ws, ['qSc_strip_' suf], p.qSc_strip);
    assignin(ws, ['c_over_2U_' suf], p.c_over_2U);
    assignin(ws, ['alpha_du0_' suf], p.alpha_du0);
    assignin(ws, ['G_du_' suf], p.G_du);
    assignin(ws, ['T_m2b_' suf], p.T_m2b);
    assignin(ws, ['T_m2h_' suf], p.T_m2h);
    if isfield(p, 'r_ac_s')
        assignin(ws, ['r_ac_s_' suf], p.r_ac_s);
        assignin(ws, ['n_hat_s_' suf], p.n_hat_s);
    end
    assignin(ws, ['Tq_' suf], p.Tq);
    assignin(ws, ['Tq_D_' suf], p.Tq_D);
    assignin(ws, ['Tq_CM_' suf], p.Tq_CM);
end
