function verify_prop_gryo_test()
% Current test.slx: Prop1 hover, Prop2 tilt+gyro, Prop3 tilt no-gyro (45 deg)
    root = fileparts(fileparts(mfilename('fullpath')));
    pd = fullfile(root, '..', 'Prop_dimentionless');
    addpath(fullfile(pd, 'lib'));
    addpath(fullfile(root, 'lib'));

    load_prop_tables();
    Lh = evalin('base', 'cfd2d');
    Lt = evalin('base', 'cfd3d');

    V = 10; rpm = 500; n = rpm/60; nD = n*Lh.D;
    cant = -7; tilt = 45; pitch = 11;
    Vb = [10; 0; 0];
    C = angle2dcm(deg2rad(cant), deg2rad(tilt), 0, 'XYZ');

    [Fh, Mh] = recon_hover(V, 0, 0, cant, rpm, 1, Lh);
    [Ft, Mt] = recon_tilt(V, 0, 0, tilt, cant, rpm, pitch, 2, Lt);
    Fh = Fh(:); Mh = Mh(:); Ft = Ft(:); Mt = Mt(:);
    [~, ~, ph] = lift_inflow(V, 0, 0, n, Lh.D, cant);
    [~, ~, pt] = engine_inflow(V, 0, 0, tilt, cant, n, Lt.D);

    % Display numbers from current test.slx
    Wp1 = [734.8; -2.151; -49.7; -94.1; 146.8; -133.3];
    Wb1 = [49.59; 3.922; 734.8; 114.4; 162; -94.1];
    Wp2 = [550.7; -10.69; -49.3; 106.6; 70.52; 90.76];
    Wb2F = [354.5; -62.51; -419.8];
    Wb2M_g = [143.3; 68.58; -23.42];
    Wb2M_n = [139.6; 68.58; -19.72];
    Va2 = 7.282; Vl2 = 6.861;

    fprintf('\n=== DCM / inflow at tilt=45 cant=-7 Vb=[10;0;0] ===\n');
    fprintf('C*Vb      %s   ue=%.4f hypot(v,w)=%.4f\n', mat2str((C*Vb).',4), (C*Vb).'*[1;0;0], hypot((C*Vb).'*[0;1;0],(C*Vb).'*[0;0;1]));
    fprintf('C''*Vb     %s   ue=%.4f hypot(v,w)=%.4f\n', mat2str((C.'*Vb).',4), (C.'*Vb).'*[1;0;0], hypot((C.'*Vb).'*[0;1;0],(C.'*Vb).'*[0;0;1]));
    fprintf('Vb''*C     %s\n', mat2str(Vb.'*C, 4));
    fprintf('engine_inflow V_eng=%s  Va=%.4f Vlat=%.4f  Ja=%.4f Jl=%.4f\n', ...
        mat2str(pt.V_eng,4), pt.Va, pt.Vlat, pt.Va/nD, pt.Vlat/nD);
    fprintf('Display          Va=%.4f Vl=%.4f  Ja_disp=%.4f Jl_disp=%.4f\n', ...
        Va2, Vl2, Va2/nD, Vl2/nD);
    fprintf('lift_inflow hover V_eng=%s\n', mat2str(ph.V_eng,4));

    % LUT with Display Ja/Jl vs theory
    [Ft_disp, Mt_disp] = recon_JaJl(Va2/nD, Vl2/nD, n, pitch, 2, Lt);

    fprintf('\n=== Prop1 hover engine wrench vs Display Wp ===\n');
    cmp_('F_e', Fh, Wp1(1:3));
    cmp_('M_e', Mh, Wp1(4:6));
    Ch = angle2dcm(deg2rad(cant), pi/2, 0, 'XYZ');
    fprintf('Wb vs C*F_e  %s\n', rel_(Ch*Fh, Wb1(1:3)));
    fprintf('Wb vs C''*F_e %s\n', rel_(Ch.'*Fh, Wb1(1:3)));
    fprintf('hover C*F  %s  %s\n', mat2str((Ch*Fh).',4), rel_(Ch*Fh, Wb1(1:3)));
    fprintf('hover C''*F %s  %s\n', mat2str((Ch.'*Fh).',4), rel_(Ch.'*Fh, Wb1(1:3)));
    fprintf('hover C*M  %s  %s\n', mat2str((Ch*Mh).',4), rel_(Ch*Mh, Wb1(4:6)));
    fprintf('hover C''*M %s  %s\n', mat2str((Ch.'*Mh).',4), rel_(Ch.'*Mh, Wb1(4:6)));

    fprintf('\n=== Prop2/3 tilt=45 engine wrench vs Display Wp ===\n');
    cmp_('F theory (engine_inflow)', Ft, Wp2(1:3));
    cmp_('M theory', Mt, Wp2(4:6));
    cmp_('F using Display Ja/Jl', Ft_disp, Wp2(1:3));
    cmp_('M using Display Ja/Jl', Mt_disp, Wp2(4:6));

    fprintf('\n=== Prop2 body force: C vs C'' vs Rz(beta)* ===\n');
    Ve = C*Vb;
    beta = asin(max(-1,min(1, Ve(2)/norm(Ve))));
    Rz = angle2dcm(0, 0, beta, 'XYZ');
    fprintf('beta from C*Vb = %.4f deg\n', rad2deg(beta));
    Ve2 = C.'*Vb;
    beta2 = asin(max(-1,min(1, Ve2(2)/norm(Ve2))));
    Rz2 = angle2dcm(0, 0, beta2, 'XYZ');
    fprintf('beta from C''*Vb = %.4f deg\n', rad2deg(beta2));
    fprintf('C *F     %s  %s\n', mat2str((C*Wp2(1:3)).',4), rel_(C*Wp2(1:3), Wb2F));
    fprintf('C''*F     %s  %s\n', mat2str((C.'*Wp2(1:3)).',4), rel_(C.'*Wp2(1:3), Wb2F));
    fprintf('C''*Rz*F  %s  %s\n', mat2str((C.'*Rz*Wp2(1:3)).',4), rel_(C.'*Rz*Wp2(1:3), Wb2F));
    fprintf('C''*Rz2*F %s  %s\n', mat2str((C.'*Rz2*Wp2(1:3)).',4), rel_(C.'*Rz2*Wp2(1:3), Wb2F));
    fprintf('Rz2*C''*F %s  %s\n', mat2str((Rz2*C.'*Wp2(1:3)).',4), rel_(Rz2*(C.'*Wp2(1:3)), Wb2F));

    fprintf('\n=== Prop2 body moment (no-gyro Display) ===\n');
    fprintf('C *M     %s  %s\n', mat2str((C*Wp2(4:6)).',4), rel_(C*Wp2(4:6), Wb2M_n));
    fprintf('C''*M     %s  %s\n', mat2str((C.'*Wp2(4:6)).',4), rel_(C.'*Wp2(4:6), Wb2M_n));
    fprintf('C''*Rz2*M %s  %s\n', mat2str((C.'*Rz2*Wp2(4:6)).',4), rel_(C.'*Rz2*Wp2(4:6), Wb2M_n));

    dM = Wb2M_g - Wb2M_n;
    fprintf('\n=== gyro delta (Prop2-Prop3) Display Wb_M ===\n');
    fprintf('dM = %s   |dM|=%.4f\n', mat2str(dM.',4), norm(dM));
    TA = C.'*[1;0;0];
    w = -2*pi*n;
    H = 1.0 * w * TA;
    fprintf('H=I*w*T_A (w=-2pi n) %s  |H|=%.4f\n', mat2str(H.',4), norm(H));
    kTscale = (1+Lh.kT*(pi*n*Lh.D/Lh.aInf)^2)/(1+Lt.kT*(pi*n*Lt.D/Lt.aInf)^2);
    fprintf('kT 2d=%.5g 3d=%.5g  scale 2d/3d at this Mtip = %.6f\n', ...
        Lh.kT, Lt.kT, kTscale);

    xls = fullfile(root, 'tests', 'Prop_gryo_verify.xlsx');
    write_xlsx_(xls, struct( ...
        'V', V, 'rpm', rpm, 'n', n, 'nD', nD, 'cant', cant, 'tilt', tilt, ...
        'pitch', pitch, 'Mtip', pi*n*Lh.D/Lh.aInf, 'kT2', Lh.kT, 'kT3', Lt.kT, ...
        'kQ2', Lh.kQ, 'kQ3', Lt.kQ, 'kTscale', kTscale, ...
        'Fh', Fh, 'Mh', Mh, 'Ft', Ft, 'Mt', Mt, ...
        'Ft_disp', Ft_disp(:), 'Mt_disp', Mt_disp(:), ...
        'Wp1', Wp1, 'Wb1', Wb1, 'Wp2', Wp2, 'Wb2F', Wb2F, ...
        'Wb2M_g', Wb2M_g, 'Wb2M_n', Wb2M_n, 'Va2', Va2, 'Vl2', Vl2, ...
        'C', C, 'Ch', Ch, 'pt', pt, 'ph', ph, 'Vb', Vb, ...
        'dM', dM, 'H', H, 'w', w, ...
        'beta_C', rad2deg(beta), 'beta_Ct', rad2deg(beta2)));
    fprintf('\nExcel: %s\n', xls);
end

function write_xlsx_(xls, S)
    if isfile(xls)
        delete(xls);
    end
    names6 = {'Fx'; 'Fy'; 'Fz'; 'Mx'; 'My'; 'Mz'};

    concl = {
        '项', '结论', '依据'
        '桨轴系查表+量纲化', '通过', 'Prop1 Wp 与 reconstruct 相对误差 <0.03%'
        '无陀螺块只砍陀螺', '通过', 'Prop2/Prop3 的 Wp、Va、Vl、Wb力 相同，仅 Wb 力矩不同'
        '倾转块 Wb=C''*We', '通过', 'Prop2/3 力、矩相对误差 <0.3%'
        '举升块 Wb 旋转', '未通过', 'Prop1 仍是 C*We，与倾转块 C'' 符号相反'
        '45deg 入流 Va/Vl', '未通过', '几何应得 7.071/7.071，Display 为 7.282/6.861'
        '零速率陀螺', '未通过', 'Omegab=0、dot_eta=0 时 |dM|≈5.23 N*m，应约为 0'
        '倾转马赫 kT', '窗口内可忽略', sprintf('cfd2d_kT/kQ 用在 3D 表上，本工况相对差 %.2e', S.kTscale-1)
        };
    writecell(concl, xls, 'Sheet', '结论');

    setup = {
        '量', '值', '备注'
        '模型', 'test.slx', '2026-09-07'
        '库', 'Library/Prop_gryo.slx', 'Prop1/2 链接；Prop3 断开链（无陀螺拷贝）'
        'V (m/s)', S.V, 'Constant [10 0 0]，1x3 行向量'
        'omega_b (rad/s)', '[0 0 0]', '1x3'
        'phi_cant (deg)', S.cant, 'Constant -7/180*pi，接到左侧 Prop1/2/3'
        'eta tilt (deg)', S.tilt, 'Prop2/3；Prop1 内部锁 90deg'
        'pitch (deg)', S.pitch, '倾转表桨距'
        'n (rps)', S.n, 'RPS = 500/60'
        'D (m)', 3, '库内 Constant'
        'nD (m/s)', S.nD, ''
        'M_tip', S.Mtip, 'Display Tip_mach=0.2308'
        'r_cg2kink', 'r_cg2kink_b(:,3)', 'E3=TR1_F 右发内台，接到左发块'
        'r_arm', 3, '标量，不是 3x1 短梁'
        'I_rotor', 1.0, '占位'
        'cfd2d kT / kQ', sprintf('%.5g / %.5g', S.kT2, S.kQ2), '举升表；倾转块马赫修正仍用这对'
        'cfd3d kT / kQ', sprintf('%.5g / %.5g', S.kT3, S.kQ3), '倾转表'
        };
    writecell(setup, xls, 'Sheet', '工况');

    h = [{'分量', 'Display Wp', 'MATLAB Fe/Me', '相对误差%', 'Display Wb', 'C*We', 'Ct*We'}; ...
        wrench_rows_(names6, S.Wp1, [S.Fh; S.Mh], S.Wb1, [S.Ch*S.Fh; S.Ch*S.Mh], [S.Ch.'*S.Fh; S.Ch.'*S.Mh])];
    writecell(h, xls, 'Sheet', 'Prop1_悬停');
    writecell({
        '相对误差相对 MATLAB 桨轴系回放。Wp 已对齐。'
        'Wb 与 C*We 对齐，与 Ct*We 差一个符号 → Prop1 机体系仍用 C 而非 C''。'
        'V_eng (lift_inflow) = [0 0 10]，Display Va≈0、Vl=10。'
        }, xls, 'Sheet', 'Prop1_悬停', 'Range', 'A10');

    t = [{'分量', 'Display Wp', 'engine_inflow 回放', '相对误差%_几何入流', ...
        'Display JaJl 回放', '相对误差%_Display入流'}; ...
        cmp6_rows_(names6, S.Wp2, [S.Ft; S.Mt], [S.Ft_disp; S.Mt_disp])];
    writecell(t, xls, 'Sheet', 'Prop2_倾转45');
    Ctv = S.C.' * S.Vb;
    writecell({
        '入流', 'Va', 'Vl', 'Ja', 'Jl'
        'engine_inflow / C*Vb', S.pt.Va, S.pt.Vlat, S.pt.Va/S.nD, S.pt.Vlat/S.nD
        'Display', S.Va2, S.Vl2, S.Va2/S.nD, S.Vl2/S.nD
        'Ct*Vb 的 ue / hypot(v,w)', Ctv(1), hypot(Ctv(2), Ctv(3)), [], []
        }, xls, 'Sheet', 'Prop2_倾转45', 'Range', 'A10');
    writecell({'用 Display 的 Ja/Jl 查表与 Wp 一致(<0.03%)，表对；几何入流差约 1.5%，进距比算错。'}, ...
        xls, 'Sheet', 'Prop2_倾转45', 'Range', 'A15');

    Wb_n = [S.Wb2F; S.Wb2M_n];
    Wb_g = [S.Wb2F; S.Wb2M_g];
    CWe = [S.C*S.Wp2(1:3); S.C*S.Wp2(4:6)];
    CtWe = [S.C.'*S.Wp2(1:3); S.C.'*S.Wp2(4:6)];
    dcm = [{'分量', 'Display Wb 无陀螺', 'Display Wb 有陀螺', 'C*Wp', '相对误差%_C', 'Ct*Wp', '相对误差%_Ct'}; ...
        dcm_rows_(names6, Wb_n, Wb_g, CWe, CtWe)];
    writecell(dcm, xls, 'Sheet', '机体系DCM');
    writecell({
        sprintf('beta from C*Vb = %.4f deg（应为 0，Rz 不生效）', S.beta_C)
        sprintf('beta from Ct*Vb = %.4f deg', S.beta_Ct)
        'Prop2/3 的 Wb 与 Ct*Wp 对齐。Prop1 仍是 C*Wp。'
        }, xls, 'Sheet', '机体系DCM', 'Range', 'A10');

    gyro = {
        '量', 'x', 'y', 'z', '模'
        'dM = Prop2-Prop3（Wb 力矩）', S.dM(1), S.dM(2), S.dM(3), norm(S.dM)
        'H = I*(-2*pi*n)*T_A，I=1', S.H(1), S.H(2), S.H(3), norm(S.H)
        'w = -2*pi*n (rad/s)', S.w, [], [], []
        'dM 等效 |Omega| ≈ |dM|/|H| (rad/s)', norm(S.dM)/norm(S.H), [], [], []
        };
    writecell(gyro, xls, 'Sheet', '陀螺残差');
    writecell({
        'Omegab=0、dot_eta=0 时陀螺应为 0。残差约 |H|*0.1 rad/s。'
        '无陀螺块把这一路去掉后，气动 Wp 不变，对比本身干净。'
        }, xls, 'Sheet', '陀螺残差', 'Range', 'A8');

    open = {
        '未覆盖项', '原因'
        '倾转块 cfd2d_kT/kQ', sprintf('本工况相对差 %.2e，被入流误差淹没', S.kTscale-1)
        '右发镜像 diag([1,-1,1,-1,1,-1])', '接的是左侧 Prop1/Prop2'
        'lift_inflow 的 Rx 外倾符号', 'alpha=beta=0 时 Ja/Jl 与外倾无关'
        'I=1 真实惯量', '占位，模对不代表惯量对'
        '体轴 z 向下 vs 桨表 z 向上', 'V 没有 z 分量'
        'r x F、点速度 Omega x r', 'Omegab=0'
        };
    writecell(open, xls, 'Sheet', '未覆盖');
end

function rows = wrench_rows_(names, Wp, We, Wb, CWe, CtWe)
    n = numel(names);
    rows = cell(n, 7);
    for i = 1:n
        rows{i,1} = names{i};
        rows{i,2} = Wp(i);
        rows{i,3} = We(i);
        rows{i,4} = 100*abs(Wp(i)-We(i))/max(abs(We(i)), 1);
        rows{i,5} = Wb(i);
        rows{i,6} = CWe(i);
        rows{i,7} = CtWe(i);
    end
end

function rows = cmp6_rows_(names, Wp, Wgeo, Wdisp)
    n = numel(names);
    rows = cell(n, 6);
    for i = 1:n
        rows{i,1} = names{i};
        rows{i,2} = Wp(i);
        rows{i,3} = Wgeo(i);
        rows{i,4} = 100*abs(Wp(i)-Wgeo(i))/max(abs(Wgeo(i)), 1);
        rows{i,5} = Wdisp(i);
        rows{i,6} = 100*abs(Wp(i)-Wdisp(i))/max(abs(Wdisp(i)), 1);
    end
end

function rows = dcm_rows_(names, Wb_n, Wb_g, CWe, CtWe)
    n = numel(names);
    rows = cell(n, 7);
    for i = 1:n
        rows{i,1} = names{i};
        rows{i,2} = Wb_n(i);
        rows{i,3} = Wb_g(i);
        rows{i,4} = CWe(i);
        rows{i,5} = 100*abs(CWe(i)-Wb_n(i))/max(abs(Wb_n(i)), 1);
        rows{i,6} = CtWe(i);
        rows{i,7} = 100*abs(CtWe(i)-Wb_n(i))/max(abs(Wb_n(i)), 1);
    end
end

function C = C90_(cant)
    C = angle2dcm(deg2rad(cant), pi/2, 0, 'XYZ');
end

function [F, M] = recon_hover(V, a, b, cant, rpm, prop, L)
    n = rpm/60;
    [Ja, Jl, inflow] = lift_inflow(V, a, b, n, L.D, cant);
    [Cef, Cem] = look2(L, prop, Ja, Jl);
    F = scale_(Cef, inflow.n, L, 'F');
    M = scale_(Cem, inflow.n, L, 'M');
end

function [F, M] = recon_tilt(V, a, b, tilt, cant, rpm, pitch, prop, L)
    n = rpm/60;
    [Ja, Jl, inflow] = engine_inflow(V, a, b, tilt, cant, n, L.D);
    [Cef, Cem] = look3(L, prop, Ja, Jl, pitch);
    F = scale_(Cef, inflow.n, L, 'F');
    M = scale_(Cem, inflow.n, L, 'M');
end

function [F, M] = recon_JaJl(Ja, Jl, n, pitch, prop, L)
    [Cef, Cem] = look3(L, prop, Ja, Jl, pitch);
    F = scale_(Cef, n, L, 'F');
    M = scale_(Cem, n, L, 'M');
end

function [Cef, Cem] = look2(L, prop, Ja, Jl)
    P = L.(sprintf('prop%d', prop));
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(1,6);
    for k=1:6
        G = griddedInterpolant({L.ja, L.jl}, P.(coeff{k}), 'linear', 'nearest');
        C(k) = G(Ja, Jl);
    end
    Cef = C(1:3); Cem = C(4:6);
end

function [Cef, Cem] = look3(L, prop, Ja, Jl, th)
    P = L.(sprintf('prop%d', prop));
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(1,6);
    for k=1:6
        G = griddedInterpolant({L.ja, L.jl, L.pitch}, P.(coeff{k}), 'linear', 'nearest');
        C(k) = G(Ja, Jl, th);
    end
    Cef = C(1:3); Cem = C(4:6);
end

function FM = scale_(Cinc, n, L, kind)
    Mtip2 = (pi*n*L.D/L.aInf)^2;
    if kind=='F'
        FM = Cinc*(1+L.kT*Mtip2)*(L.rho*n^2*L.D^4);
    else
        FM = Cinc*(1+L.kQ*Mtip2)*(L.rho*n^2*L.D^5);
    end
end

function cmp_(tag, a, b)
    a=a(:); b=b(:);
    fprintf('%-28s ml=%s\n%28s sl=%s  maxrel=%.3g%%\n', tag, mat2str(a.',4), '', mat2str(b.',4), ...
        100*max(abs(a-b)./max(abs(a),1)));
end

function s = rel_(a, b)
    a=a(:); b=b(:);
    s = sprintf('sl=%s  rel=%.3g%%', mat2str(b.',4), 100*max(abs(a-b)./max(abs(b),1)));
end
