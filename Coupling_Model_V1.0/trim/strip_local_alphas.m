function [a_wr, a_wl, a_tr, a_tl] = strip_local_alphas(V_b, cant_rad)
%STRIP_LOCAL_ALPHAS  各气动面当地迎角（rad），含 V 尾外倾
%
% 结构系：V_s = R_sb V_b，R_sb=diag(-1,1,-1)
% 当地：u = -e_c·V_s（指向前缘的弦向来流），w = -n·V_s
% α = atan2(w, u) —— 与主翼体轴 Alpha=atan2(w_b,u_b) 同号约定
%
% 主翼 n=[0,0,1]；右 V 尾 n=[0,-sinΓ,cosΓ]；左 V 尾 n=[0,sinΓ,cosΓ]

    if nargin < 2 || isempty(cant_rad)
        cant_rad = deg2rad(42);
    end
    V_b = V_b(:);
    if numel(V_b) ~= 3
        error('strip_local_alphas:Vb', 'V_b must be 3x1');
    end
    if norm(V_b) < 1e-6
        a_wr = 0; a_wl = 0; a_tr = 0; a_tl = 0;
        return
    end

    R_sb = diag([-1, 1, -1]);
    V_s = R_sb * V_b;
    e_c = [1; 0; 0];
    sG = sin(cant_rad);
    cG = cos(cant_rad);

    a_wr = local_(V_s, e_c, [0; 0; 1]);
    a_wl = local_(V_s, e_c, [0; 0; 1]);
    a_tr = local_(V_s, e_c, [0; -sG; cG]);
    a_tl = local_(V_s, e_c, [0;  sG; cG]);
end

function a = local_(V_s, e_c, n)
    u = -dot(e_c, V_s);
    w = -dot(n, V_s);
    a = atan2(w, u);
end
