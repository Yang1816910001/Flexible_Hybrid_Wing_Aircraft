function [res, Q, pack] = hover_static_residual(x, ctx, mode)
%HOVER_STATIC_RESIDUAL  Residuals for hover static trim (tilt frozen at pi/2).
%
%  mode = 'shared' (default):
%    x = [T_all_N]
%    res = [Q(3)]                 % heave
%
%  mode = 'pitch2':
%    x = [T_front_N; T_aft_N]     % engines 1-4 / 5-8 (lift+tilt together per station)
%    res = [Q(3); Q(5)]           % heave, pitch

    if nargin < 3 || isempty(mode)
        mode = 'shared';
    end

    T = zeros(8, 1);
    switch lower(mode)
        case 'shared'
            T(:) = max(x(1), 0);
            res_idx = 3;
        case 'pitch2'
            T(1:4) = max(x(1), 0);
            T(5:8) = max(x(2), 0);
            res_idx = [3; 5];
        otherwise
            error('hover_static_residual:Mode', 'unknown mode %s', mode);
    end

    [Q, pack] = hover_static_Q(T, ctx);
    res = Q(res_idx);
end
