function [alpha, beta] = strip_aero_angles(V) %#codegen
%STRIP_AERO_ANGLES  3×n 当地速度 → 每条 α、β（列向量）
%
%  V      3×n  每列一个点的速度（与 α 所用轴一致）
%  alpha  n×1  atan2(w, u)
%  beta   n×1  atan2(v, hypot(u,w))
%
%  列数跟 V 走，不要在 Simulink 符号表里写死 n。

    u = V(1, :);
    v = V(2, :);
    w = V(3, :);
    alpha = atan2(w, u).';
    beta = atan2(v, hypot(u, w)).';
end
