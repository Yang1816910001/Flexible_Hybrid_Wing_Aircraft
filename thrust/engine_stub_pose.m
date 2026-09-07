function [e_s, p_eng, r_stub, e_hinge] = engine_stub_pose( ...
        tilt, e_cruise, e_hover, p_kink, p_eng0, is_tilt)
%ENGINE_STUB_POSE  短梁绕折点带着发动机转
%
% 网格位形 = 悬停（θ=π/2）：p_eng = p_eng0。
% θ=0 前飞：短梁转到 e_cruise。铰轴 e_hinge = e_cruise × e_hover。
% 举升台不转。推力沿当前短梁，符号与 e_hover 一致（结构 Fz>0）。

    n = size(e_cruise, 2);
    tilt = tilt(:);
    is_t = logical(is_tilt(:));
    e_s = e_hover;
    p_eng = p_eng0;
    e_hinge = zeros(3, n);
    r0 = p_eng0 - p_kink;
    for i = 1:n
        if ~is_t(i)
            continue
        end
        kh = cross(e_cruise(:, i), e_hover(:, i));
        nk = norm(kh);
        if nk < 1e-12
            e_s(:, i) = e_cruise(:, i);
            continue
        end
        kh = kh / nk;
        e_hinge(:, i) = kh;
        th = tilt(i);
        ei = engine_rodrigues(kh, th, e_cruise(:, i));
        ni = norm(ei);
        if ni < 1e-12
            ei = e_cruise(:, i);
        else
            ei = ei / ni;
        end
        e_s(:, i) = ei;
        p_eng(:, i) = p_kink(:, i) + engine_rodrigues(kh, th - pi/2, r0(:, i));
    end
    r_stub = p_eng - p_kink;
end
