function [ex, ey, ez] = engine_body_triad(e_b)
%ENGINE_BODY_TRIAD  机体系下发动机正交标架
%
% ex = 推力轴
% ey = z_b × ex（z_b = [0;0;1] 机体向下）；ex 与 z 平行时改用 x_b × ex
% ez = ex × ey
% 右手系，V_eng = [ex ey ez].' * V_body

    n = size(e_b, 2);
    nrm = max(sqrt(sum(e_b.^2, 1)), eps);
    ex = e_b ./ nrm;
    ey = zeros(3, n);
    z0 = [0; 0; 1];
    x0 = [1; 0; 0];
    for i = 1:n
        yi = cross(z0, ex(:, i));
        if norm(yi) < 1e-8
            yi = cross(x0, ex(:, i));
        end
        ey(:, i) = yi / max(norm(yi), eps);
    end
    ez = [ex(2, :).*ey(3, :) - ex(3, :).*ey(2, :);
          ex(3, :).*ey(1, :) - ex(1, :).*ey(3, :);
          ex(1, :).*ey(2, :) - ex(2, :).*ey(1, :)];
end
