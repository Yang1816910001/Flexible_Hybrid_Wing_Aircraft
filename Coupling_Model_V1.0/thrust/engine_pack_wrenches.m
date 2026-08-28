function W_prop = engine_pack_wrenches(T, e_s, p_eng, p_eng0)
%ENGINE_PACK_WRENCHES  结构系节点扳手 6n×1（折算到网格发动机节点）
%
% 力 F = T * e_s，加在未变形发动机节点上。
% 力矩 M = (p_eng(θ) − p_eng0) × F：短梁转走后作用点相对网格节点的力臂。
% 不要再加 r_cg × F（Φ 已在该节点）。悬停时 p=p0，M=0。

    n = size(e_s, 2);
    T = T(:).';
    F = e_s .* T;
    d = p_eng - p_eng0;
    M = [d(2, :).*F(3, :) - d(3, :).*F(2, :);
         d(3, :).*F(1, :) - d(1, :).*F(3, :);
         d(1, :).*F(2, :) - d(2, :).*F(1, :)];
    W_prop = zeros(6 * n, 1);
    for i = 1:n
        W_prop((i - 1) * 6 + (1:3)) = F(:, i);
        W_prop((i - 1) * 6 + (4:6)) = M(:, i);
    end
end
