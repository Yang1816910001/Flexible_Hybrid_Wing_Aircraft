function make_verify_cases()
%MAKE_VERIFY_CASES  print isolated Prop_gryo check cases
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'lib'));
    addpath(fullfile(root, '..', 'Prop_dimentionless', 'lib'));
    load_prop_tables();
    Lh = evalin('base', 'cfd2d');
    Lt = evalin('base', 'cfd3d');
    rpm = 500;
    n = rpm / 60;
    pitch = 11;

    cases = {
        'A'  'Prop1 hover LUT / n=rps'              10 0 0   -7  90  'hover'
        'B'  'Prop2 tilt=0 cruise inflow'           10 0 0   -7   0  'tilt'
        'C'  'Prop2 45deg cant=0  geometry'         10 0 0    0  45  'tilt'
        'D'  'Prop2 45deg cant=-7  Va/Vl vs C'      10 0 0   -7  45  'tilt'
        'E'  'Prop2 45deg cant=+7  same JaJl'       10 0 0    7  45  'tilt'
        'F'  'Prop1 hover V along +z  cant couples'  0 0 10  -7  90  'hover'
        'G'  'Prop2 45deg V along +z'                0 0 10  -7  45  'tilt'
        };

    fprintf('\n共用: n=500/60=%.6f rps  pitch=%g deg  Omega=[0;0;0]  r_arm=[0;0;0]  Vb列向量\n', n, pitch);
    fprintf('机体系按桨表: x前 y右 z上。Constant 全部写成 3x1。\n\n');

    for i = 1:size(cases, 1)
        id = cases{i,1};
        name = cases{i,2};
        Vb = [cases{i,3}; cases{i,4}; cases{i,5}];
        cant = cases{i,6};
        tilt = cases{i,7};
        kind = cases{i,8};
        V = norm(Vb);
        if V < 1e-12
            a = 0; b = 0;
        else
            % V_body = V [cos a cos b; sin b; -sin a cos b]
            a = atan2d(-Vb(3), Vb(1));
            spdxy = hypot(Vb(1), Vb(3));
            b = atan2d(Vb(2), spdxy);
        end
        C = angle2dcm(deg2rad(cant), deg2rad(tilt), 0, 'XYZ');
        VengC = C * Vb;
        if strcmp(kind, 'hover')
            [Ja, Jl, pack] = lift_inflow(V, a, b, n, Lh.D, cant);
            [Fe, Me] = recon_h(V, a, b, cant, rpm, 1, Lh);
        else
            [Ja, Jl, pack] = engine_inflow(V, a, b, tilt, cant, n, Lt.D);
            [Fe, Me] = recon_t(V, a, b, tilt, cant, rpm, pitch, 2, Lt);
        end
        Fe = Fe(:); Me = Me(:);
        Wb_C = [C*Fe; C*Me];
        Wb_Ct = [C.'*Fe; C.'*Me];
        Mtip = pi * n * Lh.D / Lh.aInf;
        fprintf('======== %s  %s ========\n', id, name);
        fprintf('  Vb=[%.4g; %.4g; %.4g]  cant=%g deg  eta=%g deg  (alpha=%.3g beta=%.3g)\n', ...
            Vb, cant, tilt, a, b);
        fprintf('  C*Vb     [%10.4f %10.4f %10.4f]  ue=%.4f  hypot=%.4f\n', VengC, VengC(1), hypot(VengC(2), VengC(3)));
        fprintf('  inflow   Va=%.4f  Vl=%.4f  Ja=%.6f  Jl=%.6f  Mtip=%.4f\n', pack.Va, pack.Vlat, Ja, Jl, Mtip);
        fprintf('  Wp Fe    [%10.4f %10.4f %10.4f]\n', Fe);
        fprintf('  Wp Me    [%10.4f %10.4f %10.4f]\n', Me);
        fprintf('  Wb C*    [%10.4f %10.4f %10.4f | %10.4f %10.4f %10.4f]\n', Wb_C);
        fprintf('  Wb C''*   [%10.4f %10.4f %10.4f | %10.4f %10.4f %10.4f]\n', Wb_Ct);
        fprintf('\n');
    end

    fprintf('======== H  零速率陀螺（Prop2 vs 无陀螺） ========\n');
    fprintf('  用 D 的输入，再加: Omegab=[0;0;0], dot_eta=0\n');
    fprintf('  期望: 两块 Wb 力矩差 = [0;0;0]  （现在窗口里是 [3.7; 0; -3.7]，那就是残差）\n\n');

    fprintf('======== I  有速率陀螺（只看差量） ========\n');
    I = 1.0;
    n = 500/60;
    w = -2*pi*n;
    cant = -7; tilt = 45;
    C = angle2dcm(deg2rad(cant), deg2rad(tilt), 0, 'XYZ');
    TA = C.' * [1; 0; 0];
    H = I * w * TA;
    Om = [0; 0.2; 0];
    dM = cross(H, Om);  % H x Omega_b if both body
    fprintf('  用 D，改 Omegab=[0; 0.2; 0], r_arm=[0;0;0], dot_eta=0, I=1\n');
    fprintf('  H = I*(-2*pi*n)*T_A = [%10.4f %10.4f %10.4f]  |H|=%.4f\n', H, norm(H));
    fprintf('  若 M_gyro = H x Omega_b (体轴): [%10.4f %10.4f %10.4f]\n', dM);
    fprintf('  若 M_gyro = Omega_b x H:       [%10.4f %10.4f %10.4f]\n', -dM);
    fprintf('  期望: Prop2-无陀螺 的 dM 等于其中一组；Wp 气动不变\n');
    fprintf('  r_arm=0 时 Va/Vl 应与 D 相同（点速度不该变）\n\n');

    fprintf('======== 0  r_arm 泄漏（先做） ========\n');
    fprintf('  用 D，只改 r_arm=[0;0;0] vs 标量 3，Omega=0\n');
    fprintf('  期望: Va/Vl/Wp 完全相同。若标量 3 变成 7.282/6.861，就是力臂串进了入流\n');
end

function [F, M] = recon_h(V, a, b, cant, rpm, prop, L)
    n = rpm/60;
    [Ja, Jl, inflow] = lift_inflow(V, a, b, n, L.D, cant);
    P = L.(sprintf('prop%d', prop));
    Cinc = look2(L, P, Ja, Jl);
    F = scale_(Cinc(1:3), inflow.n, L, 'F');
    M = scale_(Cinc(4:6), inflow.n, L, 'M');
end

function [F, M] = recon_t(V, a, b, tilt, cant, rpm, pitch, prop, L)
    n = rpm/60;
    [Ja, Jl, inflow] = engine_inflow(V, a, b, tilt, cant, n, L.D);
    P = L.(sprintf('prop%d', prop));
    Cinc = look3(L, P, Ja, Jl, pitch);
    F = scale_(Cinc(1:3), inflow.n, L, 'F');
    M = scale_(Cinc(4:6), inflow.n, L, 'M');
end

function C = look2(L, P, Ja, Jl)
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(1, 6);
    for k = 1:6
        G = griddedInterpolant({L.ja, L.jl}, P.(coeff{k}), 'linear', 'nearest');
        C(k) = G(Ja, Jl);
    end
end

function C = look3(L, P, Ja, Jl, th)
    coeff = {'CEF_X','CEF_Y','CEF_Z','CEM_X','CEM_Y','CEM_Z'};
    C = zeros(1, 6);
    for k = 1:6
        G = griddedInterpolant({L.ja, L.jl, L.pitch}, P.(coeff{k}), 'linear', 'nearest');
        C(k) = G(Ja, Jl, th);
    end
end

function FM = scale_(Cinc, n, L, kind)
    Mtip2 = (pi * n * L.D / L.aInf)^2;
    if kind == 'F'
        FM = Cinc * (1 + L.kT * Mtip2) * (L.rho * n^2 * L.D^4);
    else
        FM = Cinc * (1 + L.kQ * Mtip2) * (L.rho * n^2 * L.D^5);
    end
end
