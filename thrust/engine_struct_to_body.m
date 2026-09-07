function [e_b, r_eng_b, r_stub_b, e_hinge_b] = engine_struct_to_body( ...
        e_s, p_eng, p_kink, e_hinge, cg, R_sb)
%ENGINE_STRUCT_TO_BODY  结构系 → 机体系（当前短梁构型）
%
% R_sb = diag(-1, 1, -1)。cg 为 3×1 结构系重心。

    cg = cg(:);
    e_b = R_sb * e_s;
    r_eng_b = R_sb * (p_eng - cg);
    r_stub_b = R_sb * (p_eng - p_kink);
    e_hinge_b = R_sb * e_hinge;
end
