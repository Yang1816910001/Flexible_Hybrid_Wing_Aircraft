function v_out = engine_rodrigues(k, ang, v)
%ENGINE_RODRIGUES  绕轴 k 转 ang，作用在列向量 v 上
%
% k 3×1 或 3×n，ang 标量或 n×1，v 3×n

    n = size(v, 2);
    if size(k, 2) == 1
        k = repmat(k, 1, n);
    end
    nrm = max(sqrt(sum(k.^2, 1)), eps);
    k = k ./ nrm;
    a = ang(:).';
    if numel(a) == 1
        a = repmat(a, 1, n);
    end
    c = cos(a);
    s = sin(a);
    kxv = [k(2, :).*v(3, :) - k(3, :).*v(2, :);
           k(3, :).*v(1, :) - k(1, :).*v(3, :);
           k(1, :).*v(2, :) - k(2, :).*v(1, :)];
    kdv = sum(k .* v, 1);
    v_out = v .* c + kxv .* s + k .* (kdv .* (1 - c));
end
