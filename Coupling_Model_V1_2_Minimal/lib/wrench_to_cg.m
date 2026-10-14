function [F_cg, M_cg] = wrench_to_cg(F, M, r) %#codegen
%WRENCH_TO_CG  节点力/力矩合到质心
%
%  [F_cg, M_cg] = wrench_to_cg(F, M, r)
%
%  F, M, r  3×n  每列一个节点，必须同一坐标系
%            r 用 r_cg2node_m（结构）或 r_cg2node_b（机体）
%  F_cg     3×1  Σ F
%  M_cg     3×1  Σ (r × F + M)
%
%  不要把 6 维扳手整段叉乘。力只求和；力矩才是 r×F 再加节点力矩。

    F_cg = sum(F, 2);
    M_cg = sum(cross3n(r, F) + M, 2);
end
