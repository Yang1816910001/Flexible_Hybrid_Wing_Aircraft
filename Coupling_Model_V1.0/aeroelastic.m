classdef aeroelastic
%AEROELASTIC  片条气弹：映射、状态空间（入口见 initial_main）
%
% Φ 前 6 列为刚体（surge/sway/heave/roll/pitch/yaw），其后为弹性。
% 日常：aeroelastic.setup_paths + load_strips / load_maps / wagner_ss / structure_ss。
% wire_* 只在重建 slx 时用，不要对现成 aerodynamic_model.slx 再跑。

    methods (Static)
        function p = proj_root
            p = fileparts(mfilename('fullpath'));
        end

        function setup_paths
        %SETUP_PATHS  把 thrust / trim / tests / viz 加进 MATLAB 路径
            r = aeroelastic.proj_root;
            addpath(r);
            addpath(fullfile(r, 'thrust'));
            addpath(fullfile(r, 'trim'));
            addpath(fullfile(r, 'tests'));
            addpath(fullfile(r, 'viz'));
        end

        function s = load_strips(results_dir, varargin)
        %LOAD_STRIPS  主翼 + V 尾片条，气动力在 c/4
            if nargin < 1 || isempty(results_dir)
                results_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
            end
            p = inputParser;
            addParameter(p, 'mac_m', 2.56, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'v_tail_cant_deg', 42, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'keep_rigid', true, @(x) islogical(x) && isscalar(x));
            parse(p, varargin{:});
            c_mac = p.Results.mac_m;
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

            s.c_strip = c_mac * ones(n_strip, 1);
            e_chord = [1, 0, 0];
            n_hat = zeros(n_strip, 3);
            iw = s.surface == "wing_R" | s.surface == "wing_L";
            ir = s.surface == "v_tail_R";
            il = s.surface == "v_tail_L";
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
            if keep_rigid
                [Phi_r, rb] = rigid_body_modes_(T);
                s.Phi = [Phi_r, s.Phi];
                s.omega = [zeros(6, 1); omega_e];
                s.freq_hz = [zeros(6, 1); freq_e];
                s.family = [repmat("rigid", 6, 1); family_e];
                s.mode_name = [rb.names; name_e];
                s.n_rigid = 6;
                s.cg_rigid_m = rb.cg_m;
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
            s.mac_m = c_mac;
            s.surface_names = ["wing_R", "wing_L", "v_tail_R", "v_tail_L"];
            s.by_surface = aeroelastic.pack_surfaces(s);
            s.note = [ ...
                'Phi columns: 6 rigid (surge/sway/heave/roll/pitch/yaw) then elastic. ' ...
                'Strips follow beam stations. AC = node + [-0.25c,0,0]. ' ...
                'Q = Tq*L + Tq_D*(q S CD) + Tq_CM*(q S c CM).'];
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
                b.Tq = s.Tq(:, idx);
                b.Tq_D = s.Tq_D(:, idx);
                b.Tq_CM = s.Tq_CM(:, idx);
                b.G_du = s.G_du(idx, idx);
                by.(name) = b;
            end
            by.surface_names = names;
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
            maps.Tq_prop = phi_nodes_to_tq_(Phi, prop_node_ids);
            maps.Tq_mass = Phi.';
            maps.note = [ ...
                'Gravity is W_mass_g via Tq_mass. Modal inertia W=T_etaddot2Winer*eta_ddot (not in Q). ' ...
                'W_prop is 48x1 structural wrenches at E1..E8 (TL2_F..TR2_R).'];
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
            g.R_sb = diag([-1, 1, -1]);
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
            for k = 1:n
                F_g = m(k) * g_struct;
                F_g_total = F_g_total + F_g;
                rows = (nid(k) - 1) * 6 + (1:6);
                W_mass(rows(1:3)) = F_g;
            end

            out = struct();
            out.g = r.g;
            out.g_struct = g_struct;
            out.m_total_kg = Mtot;
            out.cg_m = cg;
            out.W_mass = W_mass;
            out.F_g_total = F_g_total;
            out.note = 'Gravity -> Tq_mass. Modal point inertia from eta_ddot, not here.';
        end

        function maps = point_inertia(Phi, nodes)
        %POINT_INERTIA  W = T_etaddot2Winer * eta_ddot（节点惯性载荷恢复，不进 Q）
            nid = nodes.node_id(:);
            m = nodes.mass_kg(:);
            n_nodes = numel(nid);
            ndn = 6;
            n_modes = size(Phi, 2);
            ndof = size(Phi, 1);

            T_etaddot2Winer = zeros(ndof, n_modes);
            for k = 1:n_nodes
                base6 = (nid(k) - 1) * ndn;
                Phi_u = Phi(base6 + (1:3), :);
                T_etaddot2Winer(base6 + (1:3), :) = -m(k) * Phi_u;
            end

            maps = struct();
            maps.T_etaddot2Winer = T_etaddot2Winer;
            maps.n_nodes = n_nodes;
            maps.ndof = ndof;
            maps.n_modes = n_modes;
            maps.note = 'W = T_etaddot2Winer * eta_ddot';
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

        function wire()
        %WIRE  重力常值进 Tq_mass；按 η̈ 恢复节点惯性载荷（不进 Q）
            root = aeroelastic.proj_root;
            cd(root);
            mdl = 'aerodynamic_model';
            slx = fullfile(root, [mdl '.slx']);
            if ~isfile(slx)
                error('aeroelastic:MissingSlx', 'no %s', slx);
            end
            if bdIsLoaded(mdl)
                close_system(mdl, 0);
            end
            load_system(mdl);

            add_const_sum_(mdl, 'From_Wmass', 'Tq_mass', ...
                'W_mass_g', 'Const_Wmass_g', 'Sum_Wmass');

            try, delete_line(mdl, 'Sum_Winert/1', 'Tq_inertia/1'); end %#ok<TRYNC>
            try, delete_line(mdl, 'From_Winert/1', 'Sum_Winert/1'); end %#ok<TRYNC>
            try, delete_line(mdl, 'Const_Winert_rb/1', 'Sum_Winert/2'); end %#ok<TRYNC>
            try, delete_line(mdl, 'From_Winert/1', 'Tq_inertia/1'); end %#ok<TRYNC>
            safe_del_(mdl, 'Sum_Winert');
            safe_del_(mdl, 'Const_Winert_rb');
            addl_(mdl, 'From_Winert/1', 'Tq_inertia/1');

            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'From_Q_eom'))
                add_block('simulink/Signal Routing/From', [mdl '/From_Q_eom'], ...
                    'GotoTag', 'Q', 'TagVisibility', 'local', ...
                    'Position', [250 880 335 908]);
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'From_eta_eom'))
                add_block('simulink/Signal Routing/From', [mdl '/From_eta_eom'], ...
                    'GotoTag', 'elastic_displacement', 'TagVisibility', 'local', ...
                    'Position', [250 920 385 948]);
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'From_etad_eom'))
                add_block('simulink/Signal Routing/From', [mdl '/From_etad_eom'], ...
                    'GotoTag', 'elastic_velocity', 'TagVisibility', 'local', ...
                    'Position', [250 960 370 988]);
            end

            addg_(mdl, 'Gain_Om2', 'Om2_modal', [420 918 510 952], 'Matrix(K*u)');
            addg_(mdl, 'Gain_Cmodal', 'C_modal', [420 958 510 992], 'Matrix(K*u)');
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Sum_etaddot'))
                add_block('simulink/Math Operations/Sum', [mdl '/Sum_etaddot'], ...
                    'Inputs', '+--', 'IconShape', 'rectangular', ...
                    'Position', [540 875 575 995]);
            else
                set_param([mdl '/Sum_etaddot'], 'Inputs', '+--');
            end
            addg_(mdl, 'T_etaddot2Winer', 'T_etaddot2Winer', ...
                [610 910 740 960], 'Matrix(K*u)');
            addg_(mdl, 'T_m2a', 'T_m2v', [610 980 700 1014], 'Matrix(K*u)');

            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'W_inertia_modal'))
                add_block('simulink/Sinks/Out1', [mdl '/W_inertia_modal'], ...
                    'Position', [770 922 800 938], 'PortDimensions', 'ndof');
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'a_node_modal'))
                add_block('simulink/Sinks/Out1', [mdl '/a_node_modal'], ...
                    'Position', [770 992 800 1008], 'PortDimensions', '-1');
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Goto_Winer_m'))
                add_block('simulink/Signal Routing/Goto', [mdl '/Goto_Winer_m'], ...
                    'GotoTag', 'W_inertia_modal', 'TagVisibility', 'local', ...
                    'Position', [770 948 900 972]);
            end

            addl_(mdl, 'From_Q_eom/1', 'Sum_etaddot/1');
            addl_(mdl, 'From_eta_eom/1', 'Gain_Om2/1');
            addl_(mdl, 'Gain_Om2/1', 'Sum_etaddot/2');
            addl_(mdl, 'From_etad_eom/1', 'Gain_Cmodal/1');
            addl_(mdl, 'Gain_Cmodal/1', 'Sum_etaddot/3');
            addl_(mdl, 'Sum_etaddot/1', 'T_etaddot2Winer/1');
            addl_(mdl, 'Sum_etaddot/1', 'T_m2a/1');
            addl_(mdl, 'T_etaddot2Winer/1', 'W_inertia_modal/1');
            addl_(mdl, 'T_etaddot2Winer/1', 'Goto_Winer_m/1');
            addl_(mdl, 'T_m2a/1', 'a_node_modal/1');

            % 体轴运动: nu_body = [u;v;w;p;q;r] = T_m2body * eta_dot
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'From_etad_body'))
                add_block('simulink/Signal Routing/From', [mdl '/From_etad_body'], ...
                    'GotoTag', 'elastic_velocity', 'TagVisibility', 'local', ...
                    'Position', [250 1040 370 1068]);
            end
            addg_(mdl, 'T_m2body', 'T_m2body', [420 1035 530 1075], 'Matrix(K*u)');
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Demux_body'))
                add_block('simulink/Signal Routing/Demux', [mdl '/Demux_body'], ...
                    'Outputs', '[3 3]', ...
                    'Position', [560 1030 615 1080]);
            else
                set_param([mdl '/Demux_body'], 'Outputs', '[3 3]');
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'V_body'))
                add_block('simulink/Sinks/Out1', [mdl '/V_body'], ...
                    'Position', [770 1038 800 1054], 'PortDimensions', '3');
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'omega_body'))
                add_block('simulink/Sinks/Out1', [mdl '/omega_body'], ...
                    'Position', [770 1062 800 1078], 'PortDimensions', '3');
            end
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'nu_body'))
                add_block('simulink/Sinks/Out1', [mdl '/nu_body'], ...
                    'Position', [770 1090 800 1106], 'PortDimensions', '6');
            end
            addl_(mdl, 'From_etad_body/1', 'T_m2body/1');
            addl_(mdl, 'T_m2body/1', 'Demux_body/1');
            addl_(mdl, 'T_m2body/1', 'nu_body/1');
            addl_(mdl, 'Demux_body/1', 'V_body/1');
            addl_(mdl, 'Demux_body/2', 'omega_body/1');

            comment_offpath_(mdl, 'Aerodynamic_Mass_Inertial_Load');
            comment_offpath_(mdl, 'Aerodynamic_Grativity_Load');
            comment_offpath_(mdl, 'Propusion_Grativity_Load');

            save_system(mdl, slx);
            close_system(mdl, 0);
            fprintf('wired gravity + modal point inertia into %s\n', slx);
        end

        function wire_thrust()
        %WIRE_THRUST  重接 thrust_demo：8 发子系统（轴 / R_sb / DCM / αβ / W_prop）
            root = aeroelastic.proj_root;
            cd(root);
            addpath(fullfile(root, 'thrust'));
            mdl = 'thrust_demo';
            slx = fullfile(root, [mdl '.slx']);
            if ~isfile(slx)
                error('aeroelastic:MissingSlx', 'no %s', slx);
            end
            if bdIsLoaded(mdl)
                close_system(mdl, 0);
            end
            load_system(mdl);
            tf_name = 'Turbofan Engine System';
            keep = find_system(mdl, 'SearchDepth', 1, 'Name', tf_name);
            blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
            for i = 1:numel(blks)
                nm = get_param(blks{i}, 'Name');
                if strcmp(nm, mdl) || strcmp(nm, tf_name)
                    continue
                end
                try
                    lh = get_param(blks{i}, 'LineHandles');
                    for fn = fieldnames(lh)'
                        h = lh.(fn{1});
                        for k = 1:numel(h)
                            if h(k) > 0
                                try, delete_line(h(k)); end %#ok<TRYNC>
                            end
                        end
                    end
                    delete_block(blks{i});
                catch
                end
            end
            if isempty(keep)
                error('aeroelastic:Turbofan', 'Turbofan Engine System missing');
            end
            set_param([mdl '/' tf_name], 'Position', [280 40 490 120]);

            add_block('simulink/Sources/In1', [mdl '/Throttle'], ...
                'PortDimensions', '8', 'Position', [30 30 60 44]);
            add_block('simulink/Sources/In1', [mdl '/Mach'], ...
                'PortDimensions', '1', 'Position', [30 80 60 94]);
            add_block('simulink/Sources/In1', [mdl '/Altitude_m'], ...
                'PortDimensions', '1', 'Position', [30 120 60 134]);
            add_block('simulink/Sources/In1', [mdl '/Tilt_cmd'], ...
                'PortDimensions', '8', 'Position', [30 220 60 234]);
            add_block('simulink/Sources/In1', [mdl '/Tilt_rate'], ...
                'PortDimensions', '8', 'Position', [30 500 60 514]);
            add_block('simulink/Sources/In1', [mdl '/V_body'], ...
                'PortDimensions', '3', 'Position', [30 420 60 434]);
            add_block('simulink/Sources/In1', [mdl '/omega_body'], ...
                'PortDimensions', '3', 'Position', [30 460 60 474]);

            add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_thr'], ...
                'LowerLimit', '0', 'UpperLimit', '1', ...
                'Position', [90 26 140 48]);
            add_block('simulink/Sources/Constant', [mdl '/Thr_map'], ...
                'Value', '1', 'Position', [90 62 130 88]);
            add_block('simulink/Math Operations/Gain', [mdl '/m2ft'], ...
                'Gain', '3.280839895', 'Position', [90 114 150 146]);
            add_block('simulink/Math Operations/Product', [mdl '/T_i'], ...
                'Inputs', '2', 'Multiplication', 'Element-wise(.*)', ...
                'Position', [530 36 570 76]);

            add_const_mat_(mdl, 'e_cruise', 'e_thrust_cruise', [90 250 170 270]);
            add_const_mat_(mdl, 'e_hover', 'e_thrust_hover', [90 280 170 300]);
            add_const_mat_(mdl, 'is_tilt', 'eng_is_tilt', [90 310 170 330]);
            add_const_mat_(mdl, 'p_kink', 'p_kink_m', [90 340 170 360]);
            add_const_mat_(mdl, 'p_eng0', 'p_eng0_m', [90 370 170 390]);
            add_const_mat_(mdl, 'cg', 'cg_m', [90 400 170 420]);
            add_const_mat_(mdl, 'R_sb', 'R_sb', [90 430 170 450]);

            add_mlfcn_sub_(mdl, 'Stub_Pose', [220 190 400 360], ...
                sprintf(['function [e_s, p_eng, r_stub, e_hinge] = fcn(', ...
                'tilt, e_cruise, e_hover, p_kink, p_eng0, is_tilt)\n', ...
                '[e_s, p_eng, r_stub, e_hinge] = engine_stub_pose(', ...
                'tilt, e_cruise, e_hover, p_kink, p_eng0, is_tilt);\n']), ...
                {'tilt', '8'; 'e_cruise', '[3 8]'; 'e_hover', '[3 8]'; ...
                 'p_kink', '[3 8]'; 'p_eng0', '[3 8]'; 'is_tilt', '8'}, ...
                {'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'r_stub', '[3 8]'; 'e_hinge', '[3 8]'}, ...
                'yellow', 'engine_stub_pose', 'R(th-pi/2) about kink');
            add_mlfcn_sub_(mdl, 'Coord_Struct2Body', [450 250 660 400], ...
                sprintf(['function [e_b, r_eng_b, r_stub_b, e_hinge_b] = fcn(', ...
                'e_s, p_eng, p_kink, e_hinge, cg, R_sb)\n', ...
                '[e_b, r_eng_b, r_stub_b, e_hinge_b] = engine_struct_to_body(', ...
                'e_s, p_eng, p_kink, e_hinge, cg, R_sb);\n']), ...
                {'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'p_kink', '[3 8]'; ...
                 'e_hinge', '[3 8]'; 'cg', '[3 1]'; 'R_sb', '[3 3]'}, ...
                {'e_b', '[3 8]'; 'r_eng_b', '[3 8]'; 'r_stub_b', '[3 8]'; 'e_hinge_b', '[3 8]'}, ...
                'lightBlue', 'engine_struct_to_body', 'v_b = R_sb * v_s');
            add_mlfcn_sub_(mdl, 'Pack_Wrench', [450 70 660 180], ...
                sprintf(['function W_prop = fcn(T, e_s, p_eng, p_eng0)\n', ...
                'W_prop = engine_pack_wrenches(T, e_s, p_eng, p_eng0);\n']), ...
                {'T', '8'; 'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'p_eng0', '[3 8]'}, ...
                {'W_prop', '48'}, 'gray', ...
                'engine_pack_wrenches', 'F at node, M=(p-p0)xF');
            add_mlfcn_sub_(mdl, 'Point_Velocity', [700 330 920 460], ...
                sprintf(['function V_pt = fcn(V_b, omega_b, r_eng_b, r_stub_b, e_hinge_b, tilt_rate)\n', ...
                'V_pt = engine_point_velocity(V_b, omega_b, r_eng_b, r_stub_b, e_hinge_b, tilt_rate);\n']), ...
                {'V_b', '3'; 'omega_b', '3'; 'r_eng_b', '[3 8]'; ...
                 'r_stub_b', '[3 8]'; 'e_hinge_b', '[3 8]'; 'tilt_rate', '8'}, ...
                {'V_pt', '[3 8]'}, 'green', ...
                'engine_point_velocity', 'V+w x r + thdot*k x stub');
            add_mlfcn_sub_(mdl, 'Engine_DCM', [700 230 890 310], ...
                sprintf(['function C9 = fcn(e_b)\n', ...
                'C9 = engine_dcm_body_to_eng(e_b);\n']), ...
                {'e_b', '[3 8]'}, {'C9', '[9 8]'}, 'cyan', ...
                'engine_dcm_body_to_eng', 'C_eb = [ex ey ez]''');
            add_mlfcn_sub_(mdl, 'Aero_Angles', [960 250 1150 360], ...
                sprintf(['function [alpha, beta] = fcn(C9, V_pt)\n', ...
                '[alpha, beta] = engine_alpha_beta(C9, V_pt);\n']), ...
                {'C9', '[9 8]'; 'V_pt', '[3 8]'}, ...
                {'alpha', '8'; 'beta', '8'}, 'orange', ...
                'engine_alpha_beta', 'alpha=atan2(w,u)  beta=asin(v/|V|)');

            add_block('simulink/Math Operations/Reshape', [mdl '/e_s_col'], ...
                'OutputDimensionality', '1-D array', ...
                'Position', [700 175 760 205]);
            add_block('simulink/Sinks/Out1', [mdl '/W_prop'], ...
                'PortDimensions', 'n_prop_wrench', 'Position', [700 100 730 114]);
            add_block('simulink/Sinks/Out1', [mdl '/T_engine'], ...
                'PortDimensions', '8', 'Position', [700 40 730 54]);
            add_block('simulink/Sinks/Out1', [mdl '/e_thrust_s'], ...
                'PortDimensions', '24', 'Position', [800 182 830 196]);
            add_block('simulink/Sinks/Out1', [mdl '/alpha_eng'], ...
                'PortDimensions', '8', 'Position', [1210 266 1240 280]);
            add_block('simulink/Sinks/Out1', [mdl '/beta_eng'], ...
                'PortDimensions', '8', 'Position', [1210 310 1240 324]);

            addl_(mdl, 'Throttle/1', 'Sat_thr/1');
            addl_(mdl, 'Sat_thr/1', 'T_i/1');
            addl_(mdl, 'Thr_map/1', [tf_name '/1']);
            addl_(mdl, 'Mach/1', [tf_name '/2']);
            addl_(mdl, 'Altitude_m/1', 'm2ft/1');
            addl_(mdl, 'm2ft/1', [tf_name '/3']);
            addl_(mdl, [tf_name '/1'], 'T_i/2');
            addl_(mdl, 'T_i/1', 'T_engine/1');
            addl_(mdl, 'T_i/1', 'Pack_Wrench/1');
            addl_(mdl, 'Tilt_cmd/1', 'Stub_Pose/1');
            addl_(mdl, 'e_cruise/1', 'Stub_Pose/2');
            addl_(mdl, 'e_hover/1', 'Stub_Pose/3');
            addl_(mdl, 'p_kink/1', 'Stub_Pose/4');
            addl_(mdl, 'p_eng0/1', 'Stub_Pose/5');
            addl_(mdl, 'is_tilt/1', 'Stub_Pose/6');
            addl_(mdl, 'Stub_Pose/1', 'Pack_Wrench/2');
            addl_(mdl, 'Stub_Pose/2', 'Pack_Wrench/3');
            addl_(mdl, 'p_eng0/1', 'Pack_Wrench/4');
            addl_(mdl, 'Stub_Pose/1', 'Coord_Struct2Body/1');
            addl_(mdl, 'Stub_Pose/2', 'Coord_Struct2Body/2');
            addl_(mdl, 'p_kink/1', 'Coord_Struct2Body/3');
            addl_(mdl, 'Stub_Pose/4', 'Coord_Struct2Body/4');
            addl_(mdl, 'cg/1', 'Coord_Struct2Body/5');
            addl_(mdl, 'R_sb/1', 'Coord_Struct2Body/6');
            addl_(mdl, 'Stub_Pose/1', 'e_s_col/1');
            addl_(mdl, 'Coord_Struct2Body/1', 'Engine_DCM/1');
            addl_(mdl, 'Coord_Struct2Body/2', 'Point_Velocity/3');
            addl_(mdl, 'Coord_Struct2Body/3', 'Point_Velocity/4');
            addl_(mdl, 'Coord_Struct2Body/4', 'Point_Velocity/5');
            addl_(mdl, 'V_body/1', 'Point_Velocity/1');
            addl_(mdl, 'omega_body/1', 'Point_Velocity/2');
            addl_(mdl, 'Tilt_rate/1', 'Point_Velocity/6');
            addl_(mdl, 'Engine_DCM/1', 'Aero_Angles/1');
            addl_(mdl, 'Point_Velocity/1', 'Aero_Angles/2');
            addl_(mdl, 'Pack_Wrench/1', 'W_prop/1');
            addl_(mdl, 'e_s_col/1', 'e_thrust_s/1');
            addl_(mdl, 'Aero_Angles/1', 'alpha_eng/1');
            addl_(mdl, 'Aero_Angles/2', 'beta_eng/1');

            save_system(mdl, slx);
            close_system(mdl, 0);
            fprintf('rewired 8-engine thrust_demo (split modules) -> %s\n', slx);
        end

        function wire_aero_thrust()
        %WIRE_AERO_THRUST  更新 aerodynamic_model 里推进子系统为短梁绕折点
            root = aeroelastic.proj_root;
            cd(root);
            addpath(fullfile(root, 'thrust'));
            mdl = 'aerodynamic_model';
            slx = fullfile(root, [mdl '.slx']);
            if ~isfile(slx)
                error('aeroelastic:MissingSlx', 'no %s', slx);
            end
            if bdIsLoaded(mdl)
                close_system(mdl, 0);
            end
            load_system(mdl);
            p = [mdl '/Subsystem'];
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Subsystem'))
                error('aeroelastic:NoThrustSub', 'aerodynamic_model/Subsystem missing');
            end

            old = {'Thrust_Axes', 'Coord_Struct2Body', 'Pack_Wrench', ...
                'Point_Velocity', 'r_eng', 'Stub_Pose', 'p_kink', 'p_eng0', 'cg'};
            for i = 1:numel(old)
                safe_del_(p, old{i});
            end

            add_const_mat_(p, 'p_kink', 'p_kink_m', [500 400 580 420]);
            add_const_mat_(p, 'p_eng0', 'p_eng0_m', [500 430 580 450]);
            add_const_mat_(p, 'cg', 'cg_m', [500 460 580 480]);

            add_mlfcn_sub_(p, 'Stub_Pose', [605 250 800 400], ...
                sprintf(['function [e_s, p_eng, r_stub, e_hinge] = fcn(', ...
                'tilt, e_cruise, e_hover, p_kink, p_eng0, is_tilt)\n', ...
                '[e_s, p_eng, r_stub, e_hinge] = engine_stub_pose(', ...
                'tilt, e_cruise, e_hover, p_kink, p_eng0, is_tilt);\n']), ...
                {'tilt', '8'; 'e_cruise', '[3 8]'; 'e_hover', '[3 8]'; ...
                 'p_kink', '[3 8]'; 'p_eng0', '[3 8]'; 'is_tilt', '8'}, ...
                {'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'r_stub', '[3 8]'; 'e_hinge', '[3 8]'}, ...
                'yellow', 'engine_stub_pose', 'R(th-pi/2) about kink');
            add_mlfcn_sub_(p, 'Coord_Struct2Body', [820 360 1040 530], ...
                sprintf(['function [e_b, r_eng_b, r_stub_b, e_hinge_b] = fcn(', ...
                'e_s, p_eng, p_kink, e_hinge, cg, R_sb)\n', ...
                '[e_b, r_eng_b, r_stub_b, e_hinge_b] = engine_struct_to_body(', ...
                'e_s, p_eng, p_kink, e_hinge, cg, R_sb);\n']), ...
                {'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'p_kink', '[3 8]'; ...
                 'e_hinge', '[3 8]'; 'cg', '[3 1]'; 'R_sb', '[3 3]'}, ...
                {'e_b', '[3 8]'; 'r_eng_b', '[3 8]'; 'r_stub_b', '[3 8]'; 'e_hinge_b', '[3 8]'}, ...
                'lightBlue', 'engine_struct_to_body', 'v_b = R_sb * v_s');
            add_mlfcn_sub_(p, 'Pack_Wrench', [935 160 1125 280], ...
                sprintf(['function W_prop = fcn(T, e_s, p_eng, p_eng0)\n', ...
                'W_prop = engine_pack_wrenches(T, e_s, p_eng, p_eng0);\n']), ...
                {'T', '8'; 'e_s', '[3 8]'; 'p_eng', '[3 8]'; 'p_eng0', '[3 8]'}, ...
                {'W_prop', '48'}, 'gray', ...
                'engine_pack_wrenches', 'F at node, M=(p-p0)xF');
            add_mlfcn_sub_(p, 'Point_Velocity', [1115 540 1330 700], ...
                sprintf(['function V_pt = fcn(V_b, omega_b, r_eng_b, r_stub_b, e_hinge_b, tilt_rate)\n', ...
                'V_pt = engine_point_velocity(V_b, omega_b, r_eng_b, r_stub_b, e_hinge_b, tilt_rate);\n']), ...
                {'V_b', '3'; 'omega_b', '3'; 'r_eng_b', '[3 8]'; ...
                 'r_stub_b', '[3 8]'; 'e_hinge_b', '[3 8]'; 'tilt_rate', '8'}, ...
                {'V_pt', '[3 8]'}, 'green', ...
                'engine_point_velocity', 'V+w x r + thdot*k x stub');

            addl_(p, 'Manual Switch/1', 'Stub_Pose/1');
            addl_(p, 'e_cruise/1', 'Stub_Pose/2');
            addl_(p, 'e_hover/1', 'Stub_Pose/3');
            addl_(p, 'p_kink/1', 'Stub_Pose/4');
            addl_(p, 'p_eng0/1', 'Stub_Pose/5');
            addl_(p, 'is_tilt/1', 'Stub_Pose/6');
            addl_(p, 'T_i/1', 'Pack_Wrench/1');
            addl_(p, 'Stub_Pose/1', 'Pack_Wrench/2');
            addl_(p, 'Stub_Pose/2', 'Pack_Wrench/3');
            addl_(p, 'p_eng0/1', 'Pack_Wrench/4');
            addl_(p, 'Stub_Pose/1', 'Coord_Struct2Body/1');
            addl_(p, 'Stub_Pose/2', 'Coord_Struct2Body/2');
            addl_(p, 'p_kink/1', 'Coord_Struct2Body/3');
            addl_(p, 'Stub_Pose/4', 'Coord_Struct2Body/4');
            addl_(p, 'cg/1', 'Coord_Struct2Body/5');
            addl_(p, 'R_sb/1', 'Coord_Struct2Body/6');
            addl_(p, 'Stub_Pose/1', 'e_s_col/1');
            addl_(p, 'Coord_Struct2Body/1', 'engine_dcm_body_to_eng/1');
            addl_(p, 'Coord_Struct2Body/2', 'Point_Velocity/3');
            addl_(p, 'Coord_Struct2Body/3', 'Point_Velocity/4');
            addl_(p, 'Coord_Struct2Body/4', 'Point_Velocity/5');
            addl_(p, 'In Bus Element2/1', 'Point_Velocity/1');
            addl_(p, 'In Bus Element3/1', 'Point_Velocity/2');
            addl_(p, 'dot_Tilt_cmd/1', 'Point_Velocity/6');
            addl_(p, 'Point_Velocity/1', 'Aero_Angles/2');
            addl_(p, 'Pack_Wrench/1', 'Out Bus Element1/1');

            save_system(mdl, slx);
            close_system(mdl, 0);
            fprintf('updated aerodynamic_model/Subsystem stub tilt -> %s\n', slx);
        end

        function wire_flex_kin()
        %WIRE_FLEX_KIN  T_m2body* 与 Sum_etaddot 改接全模态 η/η̇/η̈(21)
            mdl = 'aerodynamic_model';
            slx = fullfile(aeroelastic.proj_root, [mdl '.slx']);
            if ~isfile(slx)
                error('aeroelastic:MissingSlx', 'no %s', slx);
            end
            if bdIsLoaded(mdl)
                close_system(mdl, 0);
            end
            load_system(mdl);
            p = [mdl '/Flexible Aircraft Dynamics'];
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Flexible Aircraft Dynamics'))
                error('aeroelastic:NoFlex', 'Flexible Aircraft Dynamics missing');
            end

            repl_input_(p, 'T_m2body', 'From_etad_body');
            repl_input_(p, 'T_m2body1', 'From_etad_body1');
            repl_input_(p, 'T_m2body2', 'From_etad_body2');
            repl_input_(p, 'Gain_Om2', 'From_eta_eom');
            repl_input_(p, 'Gain_Cmodal', 'From_etad_eom');

            addl_(p, 'Demux_modal/1', 'T_m2body1/1');
            addl_(p, 'Demux_modal/2', 'T_m2body/1');
            addl_(p, 'Demux_modal/1', 'Gain_Om2/1');
            addl_(p, 'Demux_modal/2', 'Gain_Cmodal/1');
            addl_(p, 'Sum_etaddot/1', 'T_m2body2/1');

            try, delete_line(p, 'T_m2body1/1', 'BusElementOut/1'); end %#ok<TRYNC>
            try, delete_line(p, 'T_m2body/1', 'BusElementOut3/1'); end %#ok<TRYNC>
            addl_(p, 'Demux/2', 'BusElementOut/1');
            addl_(p, 'Demux1/2', 'BusElementOut3/1');

            save_system(mdl, slx);
            close_system(mdl, 0);
            fprintf('wired full modal (21) kinematics in Flexible Aircraft Dynamics\n');
        end

        function wire_strip_aero()
        %WIRE_STRIP_AERO  Strip 四面：η/η̇ 拼成 21 维再进 T_m2b/T_m2h
            mdl = 'aerodynamic_model';
            slx = fullfile(aeroelastic.proj_root, [mdl '.slx']);
            if ~isfile(slx)
                error('aeroelastic:MissingSlx', 'no %s', slx);
            end
            if bdIsLoaded(mdl)
                close_system(mdl, 0);
            end
            load_system(mdl);
            p = [mdl '/Strip Aerodynamics'];
            if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Strip Aerodynamics'))
                error('aeroelastic:NoStrip', 'Strip Aerodynamics missing');
            end

            add_bus_in_(p, 'In Bus Element3', 'rigid_m_displacement', [30 180 60 194]);
            add_bus_in_(p, 'In Bus Element4', 'rigid_m_velocity', [30 240 60 254]);

            pos_cat = [100 100 105 180];
            add_cat_(p, 'Cat_eta', pos_cat);
            add_cat_(p, 'Cat_etad', [100 220 105 300]);

            addl_(p, 'In Bus Element3/1', 'Cat_eta/1');
            addl_(p, 'In Bus Element/1', 'Cat_eta/2');
            addl_(p, 'In Bus Element4/1', 'Cat_etad/1');
            addl_(p, 'In Bus Element1/1', 'Cat_etad/2');

            wings = {'Left_Wing', 'Right_Wing', 'Left_Tail', 'Right_Tail'};
            for i = 1:numel(wings)
                w = wings{i};
                try, delete_line(p, 'In Bus Element/1', [w '/2']); end %#ok<TRYNC>
                try, delete_line(p, 'In Bus Element1/1', [w '/3']); end %#ok<TRYNC>
                addl_(p, 'Cat_eta/1', [w '/2']);
                addl_(p, 'Cat_etad/1', [w '/3']);
            end

            save_system(mdl, slx);
            close_system(mdl, 0);
            fprintf('wired Cat_eta/Cat_etad (21) into Strip Aerodynamics\n');
        end

        function wire_aero_fix()
        %WIRE_AERO_FIX  结构↔片条维数修正（先 flex 再 strip）
            aeroelastic.wire_flex_kin();
            aeroelastic.wire_flex_bus();
            aeroelastic.wire_strip_aero();
        end

        function wire_flex_bus()
        %WIRE_FLEX_BUS  导出精简 DynamicsAndFlexibleStates（无 η̈ 诊断量）
            bus = make_dynamics_bus_();
            assignin('base', 'DynamicsAndFlexibleStates', bus);
            try
                ic = Simulink.Bus.createMATLABStruct('DynamicsAndFlexibleStates');
                assignin('base', 'DynamicsAndFlexibleStates_IC', ic);
            catch
            end
            fprintf(['DynamicsAndFlexibleStates exported (11 elems). ' ...
                'Propulsion altitude = Altitude_m [m].\n']);
        end
    end
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
    assignin(ws, ['Tq_' suf], p.Tq);
    assignin(ws, ['Tq_D_' suf], p.Tq_D);
    assignin(ws, ['Tq_CM_' suf], p.Tq_CM);
end

function add_const_sum_(mdl, from_name, gain_name, expr, cname, sname)
    gain_b = [mdl '/' gain_name];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', from_name)) ...
            || isempty(find_system(mdl, 'SearchDepth', 1, 'Name', gain_name))
        fprintf('  skip %s / %s (missing)\n', from_name, gain_name);
        return
    end
    try
        delete_line(mdl, [from_name '/1'], [gain_name '/1']);
    catch
    end
    pos_g = get_param(gain_b, 'Position');
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', cname))
        add_block('simulink/Sources/Constant', [mdl '/' cname], ...
            'Value', expr, 'VectorParams1D', 'on', ...
            'Position', [pos_g(1)-220, pos_g(2)+50, pos_g(1)-160, pos_g(2)+80]);
    else
        set_param([mdl '/' cname], 'Value', expr);
    end
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', sname))
        add_block('simulink/Math Operations/Sum', [mdl '/' sname], ...
            'Inputs', '++', 'IconShape', 'round', ...
            'Position', [pos_g(1)-80, pos_g(2)+8, pos_g(1)-50, pos_g(2)+38]);
    end
    addl_(mdl, [from_name '/1'], [sname '/1']);
    addl_(mdl, [cname '/1'], [sname '/2']);
    addl_(mdl, [sname '/1'], [gain_name '/1']);
end

function addg_(mdl, name, gain, pos, mult)
    b = [mdl '/' name];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
        add_block('simulink/Math Operations/Gain', b, 'Position', pos);
    end
    set_param(b, 'Gain', gain, 'Multiplication', mult);
end

function safe_del_(mdl, name)
    b = [mdl '/' name];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
        return
    end
    try
        lh = get_param(b, 'LineHandles');
        for fn = fieldnames(lh)'
            h = lh.(fn{1});
            for i = 1:numel(h)
                if h(i) > 0
                    try, delete_line(h(i)); end %#ok<TRYNC>
                end
            end
        end
        delete_block(b);
    catch
    end
end

function addl_(mdl, src, dst)
    try
        add_line(mdl, src, dst, 'autorouting', 'on');
    catch ME
        if ~contains(ME.message, 'already') && ~contains(ME.message, 'exist')
            fprintf('  skip line %s -> %s | %s\n', src, dst, strtrim(ME.message));
        end
    end
end

function repl_input_(subsys, dst_blk, src_blk)
    try, delete_line(subsys, [src_blk '/1'], [dst_blk '/1']); end %#ok<TRYNC>
end

function add_bus_in_(subsys, name, element, pos)
    blk = [subsys '/' name];
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
        set_param(blk, 'Element', element);
        return
    end
    ref = [subsys '/In Bus Element'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'In Bus Element'))
        add_block('simulink/Ports & Subsystems/In Bus Element', blk, ...
            'Element', element, 'Position', pos);
        return
    end
    add_block(ref, blk);
    set_param(blk, 'Name', name, 'Element', element, 'Position', pos);
end

function add_cat_(subsys, name, pos)
    blk = [subsys '/' name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
        try
            add_block('simulink/Math Operations/Vector Concatenate', blk, ...
                'NumInputs', '2', 'Mode', 'Vector', ...
                'ConcatenateDimension', '1', 'Position', pos);
        catch
            add_block('simulink/Signal Routing/Mux', blk, ...
                'Inputs', '[6 15]', 'DisplayOption', 'bar', 'Position', pos);
        end
    end
end

function add_reshape9_(subsys, name, src_blk, dst_blk, pos)
    blk = [subsys '/' name];
    try, delete_line(subsys, [src_blk '/1'], [dst_blk '/1']); end %#ok<TRYNC>
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
        add_block('simulink/Math Operations/Reshape', blk, ...
            'OutputDimensionality', '1-D array', ...
            'OutputDimensions', '9', ...
            'Position', pos);
    end
    addl_(subsys, [src_blk '/1'], [name '/1']);
    addl_(subsys, [name '/1'], [dst_blk '/1']);
end

function bus = make_dynamics_bus_(~)
    specs = { ...
        'rigid_m_displacement', 6; ...
        'rigid_m_velocity', 6; ...
        'elastic_m_velocity', 15; ...
        'Velocity_b', 3; ...
        'Omega_b', 3; ...
        'Alpha', 1; ...
        'Beta', 1; ...
        'mach', 1; ...
        'Q', 21; ...
        'elastic_displacement', 15};
    elems = Simulink.BusElement;
    for i = 1:size(specs, 1)
        e = Simulink.BusElement;
        e.Name = specs{i, 1};
        e.Dimensions = specs{i, 2};
        e.DataType = 'double';
        e.Complexity = 'real';
        e.DimensionsMode = 'Fixed';
        elems(i) = e;
    end
    bus = Simulink.Bus;
    bus.Elements = elems;
    bus.Description = 'Strip/Propulsion states (no eta_ddot diagnostics)';
end

function add_const_mat_(mdl, name, value, pos)
    add_block('simulink/Sources/Constant', [mdl '/' name], ...
        'Value', value, 'VectorParams1D', 'off', 'Position', pos);
end

function add_mlfcn_sub_(mdl, name, pos, script, ins, outs, color, fcn_name, desc)
    blk = [mdl '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', blk, 'Position', pos);
    if nargin >= 7 && ~isempty(color)
        set_param(blk, 'BackgroundColor', color);
    end
    if nargin >= 9 && ~isempty(desc)
        set_param(blk, 'Description', desc, 'AttributesFormatString', '%<Description>');
    end
    Simulink.SubSystem.deleteContents(blk);
    if nargin < 8 || isempty(fcn_name)
        fcn_name = 'fcn';
    end
    fcn = [blk '/' fcn_name];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
        'Position', [220 80 390 180]);
    drawnow;
    set_eml_script_(fcn, script);
    nI = size(ins, 1);
    nO = size(outs, 1);
    for i = 1:nI
        ip = [blk '/' ins{i, 1}];
        add_block('simulink/Sources/In1', ip, ...
            'PortDimensions', ins{i, 2}, ...
            'Position', [30 40+36*(i-1) 60 54+36*(i-1)]);
        add_line(blk, [ins{i, 1} '/1'], [fcn_name '/' num2str(i)], 'autorouting', 'on');
    end
    for i = 1:nO
        op = [blk '/' outs{i, 1}];
        add_block('simulink/Sinks/Out1', op, ...
            'PortDimensions', outs{i, 2}, ...
            'Position', [480 50+40*(i-1) 510 64+40*(i-1)]);
        add_line(blk, [fcn_name '/' num2str(i)], [outs{i, 1} '/1'], 'autorouting', 'on');
    end
end

function set_eml_script_(blk, script)
    try
        cfg = get_param(blk, 'MATLABFunctionConfiguration');
        cfg.FunctionScript = script;
        try
            cfg.SimulateUsing = 'InterpretedExecution';
        catch
        end
        return
    catch
    end
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
    if isempty(ch)
        error('aeroelastic:EMChart', 'no MATLAB Function chart at %s', blk);
    end
    ch(1).Script = script;
end

function comment_offpath_(mdl, name)
    b = [mdl '/' name];
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
        return
    end
    set_param(b, 'Commented', 'on');
end
