function C9 = engine_dcm_body_to_eng(e_b)
%ENGINE_DCM_BODY_TO_ENG  机体 → 发动机 DCM，按列堆成 9×n
%
% C_eb = [ex ey ez].'，V_eng = C_eb * V_body
% C9(:,i) = C_eb(:)（列优先）

    [ex, ey, ez] = engine_body_triad(e_b);
    n = size(e_b, 2);
    C9 = zeros(9, n);
    for i = 1:n
        C_eb = [ex(:, i), ey(:, i), ez(:, i)].';
        C9(:, i) = C_eb(:);
    end
end
